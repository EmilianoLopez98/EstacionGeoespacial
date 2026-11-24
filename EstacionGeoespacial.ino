#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <SD.h>
#include <TinyGPS++.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_AHTX0.h>
#include <RTClib.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ArduinoJson.h>
#include <math.h>
#include <esp_task_wdt.h>

// ================= PINES ETAPA 1 =================
#define LORA_SCK   18
#define LORA_MISO  19
#define LORA_MOSI  23
#define LORA_CS     5
#define LORA_RST   14
#define LORA_DIO0  26
#define SD_CS       4

#define GPS_RX     16
#define GPS_TX     17

#define BUZZER_PIN 25
#define LED_R      27
#define LED_G      32

// ================= PINES ETAPA 2 =================
// DRV8833 #1 (Motores Izquierdos: IN1+IN3 -> 13, IN2+IN4 -> 12)
#define MOT_IZQ_1  13
#define MOT_IZQ_2  12
// DRV8833 #2 (Motores Derechos: IN1+IN3 -> 15, IN2+IN4 -> 2)
#define MOT_DER_1  15
#define MOT_DER_2   2

// Ultrasonico HC-SR04 (Divisor 2k + 4k en ECHO hacia GPIO 34)
#define TRIG_PIN   33
#define ECHO_PIN   34

// Seguidor de linea TCRT5000
#define IR_IZQ     35
#define IR_CEN     36
#define IR_DER     39

// ACTIVADO: Divisor de voltaje instalado en GPIO 34
#define USAR_ULTRASONICO true

// ================= CONSTANTES DE TIEMPO Y CALIBRACION =================
const unsigned long INTERVALO_OLED       = 1000;
const unsigned long INTERVALO_TELEMETRIA = 3000;
const unsigned long INTERVALO_CONTROL    = 60;
const unsigned long INTERVALO_RAMPA      = 30;
const unsigned long TIMEOUT_MANUAL_MS    = 5000;

const int PASO_RAMPA_NORMAL = 20;
const int PASO_RAMPA_LINEA  = 70;
const int PASO_RAMPA_GIRO   = 120;  // Respuesta casi inmediata para vencer friccion lateral
const int VEL_MAX           = 235;  // Permite torque extra en giros sobre su eje
const int VEL_AVANCE        = 175;  // Velocidad crucero segura (~5.5V efectivos)
const int VEL_GIRO          = 225;  // Potencia alta para girar las 4 llantas fijas
const long DIST_OBSTACULO_CM = 25;

// CALIBRAR ESTOS TIEMPOS EN EL PISO REAL PARA EL MODO "CIRCUITO" Y GIROS:
const unsigned long TIEMPO_1M_MS       = 2200; // ms que tarda en avanzar 1 metro a VEL_AVANCE
const unsigned long TIEMPO_GIRO_90_MS  = 550;  // ms que tarda en girar 90 grados a VEL_GIRO
const unsigned long TIEMPO_GIRO_180_MS = 1100; // ms que tarda en girar 180 grados a VEL_GIRO

// Navegacion GPS y Mapa de Memoria (Grid)
const double RADIO_META_M             = 3.0;
const double DIST_MIN_RUMBO_M         = 2.0;
const unsigned long TIMEOUT_AVANCE_MS = 15000;
const unsigned long GPS_MAX_EDAD_MS   = 2000;

// ================= OBJETOS =================
Adafruit_SSD1306 display(128, 64, &Wire, -1, 100000UL, 100000UL);
Adafruit_BMP280 bmp;
Adafruit_AHTX0 aht;
RTC_DS1307 rtc;
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);

// ================= ESTADO DEL SISTEMA =================
enum EstadoEvasion { EV_AVANZAR, EV_PAUSA, EV_RETROCEDER, EV_GIRAR };

String modoActual = "MANUAL"; // MANUAL, LINEA, OBSTACULO, AUTO, CIRCUITO, TEST_MOTORES
String ultimaOrden = "Listo";
bool sdLista         = false;
bool loraListo       = false;
bool bmpOk           = false;
bool ahtOk           = false;
bool rtcOk           = false;
bool rtcSincronizado = false;
bool oledOk          = false;
bool forzarRefrescoOled = false;

String serialBuffer = "";

// Temporizadores no bloqueantes
unsigned long ultimoOledMs        = 0;
unsigned long ultimaTelemetriaMs  = 0;
unsigned long ultimoControlMs     = 0;
unsigned long ultimoRampaMs       = 0;
unsigned long ultimaOrdenManualMs = 0;

// Cache de sensores
float tempCache      = NAN;
float humCache       = NAN;
float presionCache   = NAN;
float alturaCache    = NAN;
long distanciaActual = 999;
bool objetoDetectadoPrev = false;

// Control de motores
int velActualIzq = 0, velActualDer = 0;
int velObjetivoIzq = 0, velObjetivoDer = 0;
int ultimoPwmIzq = -999, ultimoPwmDer = -999;
int pasoRampa = PASO_RAMPA_NORMAL;

// Buzzer no bloqueante
bool buzzerActivo = false;
bool enPausaBeep = false;
unsigned long timerBuzzer = 0;
unsigned long duracionBuzzer = 0;
int beepsRestantes = 0;

// Evasion no bloqueante
EstadoEvasion estadoEv = EV_AVANZAR;
unsigned long timerEv = 0;

// Modo CIRCUITO de prueba
int pasoCircuito = 0;
unsigned long timerCircuito = 0;

// Seguidor de linea
int ultimoLadoLinea = 0;

// Navegacion GPS + Memoria de Celdas Visitadas (Evita repetir caminos fallidos)
bool destinoActivo = false;
double targetLat = 0.0, targetLng = 0.0;
double origenLat = 0.0, origenLng = 0.0;
bool origenFijado = false;
double prevLat = 0.0, prevLng = 0.0;
unsigned long timerAutoGps = 0;
unsigned long duracionGiro = 0;
int estadoAuto = 0;

// Cuadricula de memoria 21x21 celdas (cada celda = 1.5m x 1.5m alrededor del inicio)
// Valor 0 = libre/no visitada, 1..10 = veces visitada (penalizacion), 255 = obstaculo/callejon
const int GRID_SIZE = 21;
const double CELDA_M = 1.5;
uint8_t mapaCosto[GRID_SIZE][GRID_SIZE];

/*
===========================================================================
  COMANDOS JSON (MONITOR SERIE 115200 baudios - "Nueva linea")
===========================================================================
--- PRUEBA DE GIROS EXACTOS Y REVERSA ---
{"cmd":"GIRO","grados":-90}    -> Gira 90 grados a la izquierda
{"cmd":"GIRO","grados":90}     -> Gira 90 grados a la derecha
{"cmd":"GIRO","grados":180}    -> Gira 180 grados
{"cmd":"MOV","dir":"ATRAS","vel":180}

--- EJECUTAR CIRCUITO DE PRUEBA (1m -> 90° izq -> 1m -> 180° -> STOP) ---
{"cmd":"MODO","val":"CIRCUITO"}

--- NAVEGACION CON MEMORIA DE CAMINOS ---
{"cmd":"GOTO","lat":19.2438,"lng":-103.7251}
===========================================================================
*/

// ================= LED ANODO COMUN (LOW ENCIENDE, HIGH APAGA) =================
void setLed(bool r, bool g) {
  digitalWrite(LED_R, !r);
  digitalWrite(LED_G, !g);
}

void actualizarLedEstado() {
  // Prioridad 1: Si el ultrasonico detecta objeto <= 25 cm -> AMARILLO (Rojo + Verde)
  if (USAR_ULTRASONICO && distanciaActual <= DIST_OBSTACULO_CM) {
    setLed(true, true);
    return;
  }
  // Estado normal: Verde si SD y LoRa OK, Rojo si falta alguno
  bool todoOk = sdLista && loraListo;
  setLed(!todoOk, todoOk);
}

void iniciarBeep(int duracionMs, int repeticiones = 1) {
  duracionBuzzer = duracionMs;
  beepsRestantes = repeticiones;
  enPausaBeep = false;
  buzzerActivo = true;
  timerBuzzer = millis();
  digitalWrite(BUZZER_PIN, HIGH);
}

void actualizarBuzzer() {
  if (!buzzerActivo) return;
  if (millis() - timerBuzzer >= duracionBuzzer) {
    if (!enPausaBeep) {
      digitalWrite(BUZZER_PIN, LOW);
      beepsRestantes--;
      if (beepsRestantes > 0) {
        enPausaBeep = true;
        timerBuzzer = millis();
      } else {
        buzzerActivo = false;
      }
    } else {
      enPausaBeep = false;
      timerBuzzer = millis();
      digitalWrite(BUZZER_PIN, HIGH);
    }
  }
}

String obtenerTimestamp() {
  if (!rtcOk) return "2026-00-00T00:00:00";
  DateTime now = rtc.now();
  char ts[25];
  snprintf(ts, sizeof(ts), "%04d-%02d-%02dT%02d:%02d:%02d",
           now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
  return String(ts);
}

bool gpsConFix() {
  return gps.location.isValid() && gps.location.age() < GPS_MAX_EDAD_MS;
}

bool modoValido(const String &m) {
  return m == "MANUAL" || m == "LINEA" || m == "OBSTACULO" || m == "AUTO" || m == "CIRCUITO";
}

double normalizarAngulo(double a) {
  while (a > 180.0) a -= 360.0;
  while (a < -180.0) a += 360.0;
  return a;
}

long medianaDe3(long a, long b, long c) {
  long t;
  if (a > b) { t = a; a = b; b = t; }
  if (b > c) { t = b; b = c; c = t; }
  if (a > b) { t = a; a = b; b = t; }
  return b;
}

// ================= MOTORES =================
void setMotoresObjetivo(int izq, int der) {
  velObjetivoIzq = constrain(izq, -VEL_MAX, VEL_MAX);
  velObjetivoDer = constrain(der, -VEL_MAX, VEL_MAX);
}

void aplicarCanalMotor(int pinAdelante, int pinAtras, int vel) {
  if (vel > 0) {
    analogWrite(pinAtras, 0);
    analogWrite(pinAdelante, vel);
  } else if (vel < 0) {
    analogWrite(pinAdelante, 0);
    analogWrite(pinAtras, -vel);
  } else {
    analogWrite(pinAdelante, 0);
    analogWrite(pinAtras, 0);
  }
}

void actualizarMotoresSuave() {
  if (velActualIzq < velObjetivoIzq) velActualIzq = min(velActualIzq + pasoRampa, velObjetivoIzq);
  else if (velActualIzq > velObjetivoIzq) velActualIzq = max(velActualIzq - pasoRampa, velObjetivoIzq);

  if (velActualDer < velObjetivoDer) velActualDer = min(velActualDer + pasoRampa, velObjetivoDer);
  else if (velActualDer > velObjetivoDer) velActualDer = max(velActualDer - pasoRampa, velObjetivoDer);

  if (velActualIzq != ultimoPwmIzq) {
    aplicarCanalMotor(MOT_IZQ_1, MOT_IZQ_2, velActualIzq);
    ultimoPwmIzq = velActualIzq;
  }
  if (velActualDer != ultimoPwmDer) {
    aplicarCanalMotor(MOT_DER_1, MOT_DER_2, velActualDer);
    ultimoPwmDer = velActualDer;
  }
}

void detenerInmediato() {
  velObjetivoIzq = 0; velObjetivoDer = 0;
  velActualIzq = 0;   velActualDer = 0;
  ultimoPwmIzq = 0;   ultimoPwmDer = 0;
  analogWrite(MOT_IZQ_1, 0); analogWrite(MOT_IZQ_2, 0);
  analogWrite(MOT_DER_1, 0); analogWrite(MOT_DER_2, 0);
}

void reiniciarMovimiento() {
  detenerInmediato();
  destinoActivo = false;
  estadoAuto = 0;
  estadoEv = EV_AVANZAR;
  pasoCircuito = 0;
  ultimoLadoLinea = 0;
}

// ================= SENSOR ULTRASONICO =================
long medirDistanciaCm() {
  if (!USAR_ULTRASONICO) return 999;
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long duracion = pulseIn(ECHO_PIN, HIGH, 15000);
  if (duracion == 0) return 999;
  long cm = (long)(duracion * 0.034 / 2);
  return (cm < 2) ? 999 : cm;
}

// ================= MATEMATICAS GPS Y MAPA DE MEMORIA =================
double calcularDistanciaM(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000.0;
  double dLat = radians(lat2 - lat1);
  double dLon = radians(lon2 - lon1);
  double a = sin(dLat / 2) * sin(dLat / 2) +
             cos(radians(lat1)) * cos(radians(lat2)) * sin(dLon / 2) * sin(dLon / 2);
  return R * 2 * atan2(sqrt(a), sqrt(1 - a));
}

double calcularRumboDeg(double lat1, double lon1, double lat2, double lon2) {
  double dLon = radians(lon2 - lon1);
  double y = sin(dLon) * cos(radians(lat2));
  double x = cos(radians(lat1)) * sin(radians(lat2)) -
             sin(radians(lat1)) * cos(radians(lat2)) * cos(dLon);
  double brng = degrees(atan2(y, x));
  return fmod((brng + 360.0), 360.0);
}

void limpiarMapaMemoria() {
  memset(mapaCosto, 0, sizeof(mapaCosto));
  origenFijado = false;
}

// Convierte coordenada GPS a celda (gx, gy) y registra penalizacion por visita o bloqueo
void registrarCeldaMapa(double lat, double lng, bool esBloqueo) {
  if (!origenFijado) {
    origenLat = lat;
    origenLng = lng;
    origenFijado = true;
  }
  double dy = (lat - origenLat) * 111320.0;
  double dx = (lng - origenLng) * 111320.0 * cos(radians(origenLat));
  int gx = constrain((int)round(dx / CELDA_M) + GRID_SIZE / 2, 0, GRID_SIZE - 1);
  int gy = constrain((int)round(dy / CELDA_M) + GRID_SIZE / 2, 0, GRID_SIZE - 1);

  if (esBloqueo) {
    mapaCosto[gx][gy] = 255; // Celda bloqueada: no volver a intentar por aqui
  } else if (mapaCosto[gx][gy] < 200) {
    mapaCosto[gx][gy] += 2;  // Penaliza caminos ya recorridos para preferir rutas nuevas
  }
}

uint8_t consultarCostoSiguienteCelda(double lat, double lng, double rumboDeg) {
  if (!origenFijado) return 0;
  double rad = radians(rumboDeg);
  double sigLat = lat + (CELDA_M * cos(rad)) / 111320.0;
  double sigLng = lng + (CELDA_M * sin(rad)) / (111320.0 * cos(radians(lat)));

  double dy = (sigLat - origenLat) * 111320.0;
  double dx = (sigLng - origenLng) * 111320.0 * cos(radians(origenLat));
  int gx = constrain((int)round(dx / CELDA_M) + GRID_SIZE / 2, 0, GRID_SIZE - 1);
  int gy = constrain((int)round(dy / CELDA_M) + GRID_SIZE / 2, 0, GRID_SIZE - 1);
  return mapaCosto[gx][gy];
}

// ================= DATALOGGER Y OLED =================
void inicializarArchivosSD() {
  if (!SD.exists("/ubicacion.csv")) {
    File f = SD.open("/ubicacion.csv", FILE_WRITE);
    if (f) { f.println("fecha_hora,latitud,longitud,satelites"); f.close(); }
  }
  if (!SD.exists("/clima.csv")) {
    File f = SD.open("/clima.csv", FILE_WRITE);
    if (f) { f.println("fecha_hora,temperatura_C,humedad_pct,presion_hPa,altura_m"); f.close(); }
  }
  if (!SD.exists("/eventos.csv")) {
    File f = SD.open("/eventos.csv", FILE_WRITE);
    if (f) { f.println("fecha_hora,modo,evento"); f.close(); }
  }
}

void registrarEvento(const String &evento) {
  if (!sdLista) return;
  File f = SD.open("/eventos.csv", FILE_APPEND);
  if (f) {
    f.println(obtenerTimestamp() + "," + modoActual + "," + evento);
    f.close();
  }
}

void actualizarLecturasSensores() {
  if (ahtOk) {
    sensors_event_t humEv, tempEv;
    aht.getEvent(&humEv, &tempEv);
    tempCache = tempEv.temperature;
    humCache = humEv.relative_humidity;
  }
  if (bmpOk) {
    tempCache = bmp.readTemperature();
    presionCache = bmp.readPressure() / 100.0F;
    alturaCache = bmp.readAltitude(1013.25);
  }
}

void dibujarPantallaOLED() {
  if (!oledOk) return;

  char horaCorta[10] = "--:--:--";
  if (rtcOk) {
    DateTime now = rtc.now();
    snprintf(horaCorta, sizeof(horaCorta), "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
  }

  bool fix = gpsConFix();
  double lat = fix ? gps.location.lat() : 0.0;
  double lng = fix ? gps.location.lng() : 0.0;
  int sats = gps.satellites.isValid() ? gps.satellites.value() : 0;

  display.clearDisplay();
  display.setCursor(0, 0);
  display.print(horaCorta);
  display.print(" "); display.print(modoActual.substring(0, 3));
  display.print(" D:"); display.println(distanciaActual);

  display.print("T:");
  if (!isnan(tempCache)) display.print(tempCache, 1); else display.print("--");
  display.print("C P:");
  if (!isnan(presionCache)) display.print(presionCache, 0); else display.print("--");
  display.println("hPa");

  display.print("Sat:"); display.print(sats);
  display.print(" LR:"); display.print(loraListo ? "OK" : "ER");
  display.print(" SD:"); display.println(sdLista ? "OK" : "ER");

  display.print("Lat:"); if (fix) display.println(lat, 5); else display.println("--");
  display.print("Lng:"); if (fix) display.println(lng, 5); else display.println("--");
  display.print("RX:"); display.println(ultimaOrden);
  display.display();
}

// ================= MODO CIRCUITO DE PRUEBA =================
// Secuencia: 1m adelante -> gira 90 izq -> 1m adelante -> gira 180 -> detente
void ejecutarCircuitoPrueba(long distObs) {
  if (distObs <= DIST_OBSTACULO_CM && pasoCircuito < 7) {
    detenerInmediato();
    ultimaOrden = "CIRC:OBSTACULO!";
    forzarRefrescoOled = true;
    modoActual = "MANUAL";
    iniciarBeep(200, 2);
    return;
  }

  pasoRampa = PASO_RAMPA_GIRO;

  switch (pasoCircuito) {
    case 0: // Iniciar tramo 1 (1m adelante)
      ultimaOrden = "CIRC:1/4 1m Adel";
      forzarRefrescoOled = true;
      setMotoresObjetivo(VEL_AVANCE, VEL_AVANCE);
      timerCircuito = millis();
      pasoCircuito = 1;
      break;

    case 1:
      if (millis() - timerCircuito >= TIEMPO_1M_MS) {
        detenerInmediato();
        timerCircuito = millis();
        pasoCircuito = 2;
      }
      break;

    case 2: // Pausa breve de estabilizacion
      if (millis() - timerCircuito >= 250) {
        ultimaOrden = "CIRC:2/4 Giro 90I";
        forzarRefrescoOled = true;
        setMotoresObjetivo(-VEL_GIRO, VEL_GIRO); // Giro sobre eje a la izquierda
        timerCircuito = millis();
        pasoCircuito = 3;
      }
      break;

    case 3:
      if (millis() - timerCircuito >= TIEMPO_GIRO_90_MS) {
        detenerInmediato();
        timerCircuito = millis();
        pasoCircuito = 4;
      }
      break;

    case 4: // Pausa breve y arranque tramo 2 (1m adelante)
      if (millis() - timerCircuito >= 250) {
        ultimaOrden = "CIRC:3/4 1m Adel";
        forzarRefrescoOled = true;
        setMotoresObjetivo(VEL_AVANCE, VEL_AVANCE);
        timerCircuito = millis();
        pasoCircuito = 5;
      }
      break;

    case 5:
      if (millis() - timerCircuito >= TIEMPO_1M_MS) {
        detenerInmediato();
        timerCircuito = millis();
        pasoCircuito = 6;
      }
      break;

    case 6: // Giro 180 grados
      if (millis() - timerCircuito >= 250) {
        ultimaOrden = "CIRC:4/4 Giro 180";
        forzarRefrescoOled = true;
        setMotoresObjetivo(VEL_GIRO, -VEL_GIRO); // Giro 180 sobre eje
        timerCircuito = millis();
        pasoCircuito = 7;
      }
      break;

    case 7:
      if (millis() - timerCircuito >= TIEMPO_GIRO_180_MS) {
        detenerInmediato();
        ultimaOrden = "CIRC:FIN!";
        forzarRefrescoOled = true;
        modoActual = "MANUAL";
        iniciarBeep(300, 1);
      }
      break;
  }
}

// ================= MODOS DE CONDUCCION =================
void ejecutarSeguidorLinea() {
  bool izq = digitalRead(IR_IZQ) == HIGH;
  bool cen = digitalRead(IR_CEN) == HIGH;
  bool der = digitalRead(IR_DER) == HIGH;

  if (cen && !izq && !der) {
    setMotoresObjetivo(150, 150);
  } else if (izq && der) {
    setMotoresObjetivo(130, 130);
  } else if (izq) {
    setMotoresObjetivo(-140, 190);
    ultimoLadoLinea = -1;
  } else if (der) {
    setMotoresObjetivo(190, -140);
    ultimoLadoLinea = 1;
  } else {
    if (ultimoLadoLinea < 0)      setMotoresObjetivo(-140, 170);
    else if (ultimoLadoLinea > 0) setMotoresObjetivo(170, -140);
    else                          setMotoresObjetivo(120, 120);
  }
}

void ejecutarEvitadorObstaculos(long dist) {
  pasoRampa = PASO_RAMPA_GIRO;
  switch (estadoEv) {
    case EV_AVANZAR:
      if (dist > DIST_OBSTACULO_CM) {
        setMotoresObjetivo(VEL_AVANCE, VEL_AVANCE);
      } else {
        detenerInmediato();
        registrarEvento("Obstaculo a " + String(dist) + " cm");
        if (gpsConFix()) registrarCeldaMapa(gps.location.lat(), gps.location.lng(), true);
        estadoEv = EV_PAUSA;
        timerEv = millis();
      }
      break;
    case EV_PAUSA:
      if (millis() - timerEv >= 150) {
        setMotoresObjetivo(-VEL_AVANCE, -VEL_AVANCE);
        estadoEv = EV_RETROCEDER;
        timerEv = millis();
      }
      break;
    case EV_RETROCEDER:
      if (millis() - timerEv >= 450) {
        setMotoresObjetivo(VEL_GIRO, -VEL_GIRO);
        estadoEv = EV_GIRAR;
        timerEv = millis();
      }
      break;
    case EV_GIRAR:
      if (millis() - timerEv >= TIEMPO_GIRO_90_MS) {
        detenerInmediato();
        estadoEv = EV_AVANZAR;
      }
      break;
  }
}

void ejecutarNavegacionAutoGPS(long distObs) {
  if (!destinoActivo) {
    setMotoresObjetivo(0, 0);
    return;
  }

  if (estadoEv != EV_AVANZAR || distObs <= DIST_OBSTACULO_CM) {
    ejecutarEvitadorObstaculos(distObs);
    estadoAuto = 0;
    return;
  }

  if (!gpsConFix()) {
    setMotoresObjetivo(0, 0);
    if (ultimaOrden != "AUTO:Sin GPS") {
      ultimaOrden = "AUTO:Sin GPS";
      forzarRefrescoOled = true;
    }
    estadoAuto = 0;
    return;
  }

  double currLat = gps.location.lat();
  double currLng = gps.location.lng();
  double distDestino = calcularDistanciaM(currLat, currLng, targetLat, targetLng);

  if (distDestino <= RADIO_META_M) {
    detenerInmediato();
    registrarEvento("META ALCANZADA");
    destinoActivo = false;
    modoActual = "MANUAL";
    ultimaOrden = "META ALCANZADA!";
    forzarRefrescoOled = true;
    iniciarBeep(400, 1);
    return;
  }

  switch (estadoAuto) {
    case 0:
      prevLat = currLat;
      prevLng = currLng;
      registrarCeldaMapa(currLat, currLng, false);
      setMotoresObjetivo(VEL_AVANCE, VEL_AVANCE);
      timerAutoGps = millis();
      estadoAuto = 1;
      break;

    case 1: {
      double recorrido = calcularDistanciaM(prevLat, prevLng, currLat, currLng);
      if (recorrido >= DIST_MIN_RUMBO_M) {
        registrarCeldaMapa(currLat, currLng, false);
        double rumboActual = calcularRumboDeg(prevLat, prevLng, currLat, currLng);
        double rumboMeta   = calcularRumboDeg(currLat, currLng, targetLat, targetLng);

        // Evaluar 3 rumbos candidatos (-60 izq, 0 directo, +60 der) combinando meta + memoria
        double mejorRumbo = rumboMeta;
        double mejorScore = 999999.0;
        double candidatos[3] = {
          fmod(rumboMeta + 360.0, 360.0),
          fmod(rumboMeta - 60.0 + 360.0, 360.0),
          fmod(rumboMeta + 60.0 + 360.0, 360.0)
        };

        for (int i = 0; i < 3; i++) {
          uint8_t costoCelda = consultarCostoSiguienteCelda(currLat, currLng, candidatos[i]);
          double penalizacionAngulo = (i == 0) ? 0.0 : 15.0;
          double score = (costoCelda * 25.0) + penalizacionAngulo;
          if (score < mejorScore) {
            mejorScore = score;
            mejorRumbo = candidatos[i];
          }
        }

        double err = normalizarAngulo(mejorRumbo - rumboActual);

        if (fabs(err) < 20.0) {
          prevLat = currLat;
          prevLng = currLng;
          timerAutoGps = millis();
        } else {
          duracionGiro = (unsigned long)constrain((long)(fabs(err) / 90.0 * TIEMPO_GIRO_90_MS), 180L, TIEMPO_GIRO_180_MS);
          pasoRampa = PASO_RAMPA_GIRO;
          if (err > 0) setMotoresObjetivo(VEL_GIRO, -VEL_GIRO);
          else         setMotoresObjetivo(-VEL_GIRO, VEL_GIRO);
          timerAutoGps = millis();
          estadoAuto = 2;
        }
      } else if (millis() - timerAutoGps >= TIMEOUT_AVANCE_MS) {
        registrarCeldaMapa(currLat, currLng, true); // Marca zona sin avance como bloqueada
        ultimaOrden = "AUTO:Re-enrutando";
        forzarRefrescoOled = true;
        prevLat = currLat;
        prevLng = currLng;
        timerAutoGps = millis();
      }
      break;
    }

    case 2:
      pasoRampa = PASO_RAMPA_GIRO;
      if (millis() - timerAutoGps >= duracionGiro) {
        detenerInmediato();
        estadoAuto = 0;
      }
      break;
  }
}

// ================= COMANDOS JSON =================
void procesarComandoJSON(String mensaje, String origen) {
  mensaje.trim();
  if (mensaje.length() == 0) return;

  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, mensaje);
  if (error) {
    Serial.println("[" + origen + "] JSON invalido (" + String(error.c_str()) + "): " + mensaje);
    return;
  }

  String cmd = doc["cmd"] | "";
  String detalleLog = "";
  bool registrar = true;
  bool sonar = true;

  if (cmd == "MODO") {
    String val = doc["val"] | "";
    val.toUpperCase();
    if (!modoValido(val)) {
      Serial.println("[" + origen + "] Modo invalido: " + val);
      return;
    }
    modoActual = val;
    reiniciarMovimiento();
    ultimaOrden = "MODO:" + modoActual;
    detalleLog = ultimaOrden;

  } else if (cmd == "MOV") {
    String dir = doc["dir"] | "STOP";
    dir.toUpperCase();
    int vel = abs((int)(doc["vel"] | VEL_AVANCE));

    bool veniaDeOtroModo = (modoActual != "MANUAL");
    if (veniaDeOtroModo) {
      modoActual = "MANUAL";
      reiniciarMovimiento();
    }

    String nueva = "MOV:" + dir;
    registrar = veniaDeOtroModo || (nueva != ultimaOrden);
    sonar = false;
    ultimaOrden = nueva;
    detalleLog = nueva;
    ultimaOrdenManualMs = millis();

    if (dir == "ADELANTE")      setMotoresObjetivo(vel, vel);
    else if (dir == "ATRAS")    setMotoresObjetivo(-vel, -vel);
    else if (dir == "IZQ")      setMotoresObjetivo(-VEL_GIRO, VEL_GIRO);
    else if (dir == "DER")      setMotoresObjetivo(VEL_GIRO, -VEL_GIRO);
    else                        setMotoresObjetivo(0, 0);

  } else if (cmd == "GIRO") {
    // Comando rapido para calibrar giros exactos: {"cmd":"GIRO","grados":90} o -90 o 180
    int grados = doc["grados"] | 90;
    modoActual = "MANUAL";
    reiniciarMovimiento();
    pasoRampa = PASO_RAMPA_GIRO;
    unsigned long tGiro = (unsigned long)(abs(grados) / 90.0 * TIEMPO_GIRO_90_MS);
    ultimaOrden = "GIRO:" + String(grados) + "g";
    detalleLog = ultimaOrden;
    // Usamos TIMEOUT_MANUAL_MS ajustado al tiempo exacto del giro
    ultimaOrdenManualMs = millis() - (TIMEOUT_MANUAL_MS - tGiro);
    if (grados > 0) setMotoresObjetivo(VEL_GIRO, -VEL_GIRO);
    else            setMotoresObjetivo(-VEL_GIRO, VEL_GIRO);

  } else if (cmd == "GOTO") {
    double lat = doc["lat"] | 0.0;
    double lng = doc["lng"] | 0.0;
    bool fueraRango = (lat < -90.0 || lat > 90.0 || lng < -180.0 || lng > 180.0);
    if (fueraRango || (lat == 0.0 && lng == 0.0)) {
      Serial.println("[" + origen + "] GOTO invalido");
      return;
    }
    limpiarMapaMemoria();
    targetLat = lat;
    targetLng = lng;
    modoActual = "AUTO";
    destinoActivo = true;
    estadoAuto = 0;
    estadoEv = EV_AVANZAR;
    ultimaOrden = "GOTO:" + String(targetLat, 5);
    detalleLog = "GOTO " + String(targetLat, 6) + " " + String(targetLng, 6);

  } else {
    Serial.println("[" + origen + "] Comando 'cmd' desconocido: " + cmd);
    return;
  }

  forzarRefrescoOled = true;

  if (registrar) {
    Serial.println("[" + origen + "] Ejecutado -> " + ultimaOrden + " | Modo: " + modoActual);
    registrarEvento(origen + " RX: " + detalleLog);
  }
  if (sonar) iniciarBeep(50, 1);
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);

  pinMode(MOT_IZQ_1, OUTPUT); analogWrite(MOT_IZQ_1, 0);
  pinMode(MOT_IZQ_2, OUTPUT); analogWrite(MOT_IZQ_2, 0);
  pinMode(MOT_DER_1, OUTPUT); analogWrite(MOT_DER_1, 0);
  pinMode(MOT_DER_2, OUTPUT); analogWrite(MOT_DER_2, 0);

  pinMode(TRIG_PIN, OUTPUT); digitalWrite(TRIG_PIN, LOW);
  pinMode(ECHO_PIN, INPUT);
  pinMode(IR_IZQ, INPUT);
  pinMode(IR_CEN, INPUT);
  pinMode(IR_DER, INPUT);

  pinMode(BUZZER_PIN, OUTPUT); digitalWrite(BUZZER_PIN, LOW);
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  setLed(true, false); // Rojo puro al iniciar (anodo comun)

  Wire.begin(21, 22);
  Wire.setClock(100000);
  Wire.setTimeOut(50);

  if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    oledOk = true;
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Iniciando Robot...");
    display.display();
  } else {
    Serial.println("OLED no encontrada");
  }

  bmpOk = bmp.begin(0x77) || bmp.begin(0x76);
  ahtOk = aht.begin();
  if (!bmpOk) Serial.println("BMP280 no encontrado");
  if (!ahtOk) Serial.println("AHT20 no encontrado");

  if (rtc.begin()) {
    rtcOk = true;
    if (!rtc.isrunning()) {
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
  } else {
    Serial.println("RTC no encontrado");
  }

  gpsSerial.setRxBufferSize(1024);
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  pinMode(LORA_CS, OUTPUT); digitalWrite(LORA_CS, HIGH);
  pinMode(SD_CS, OUTPUT);   digitalWrite(SD_CS, HIGH);
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI);

  if (SD.begin(SD_CS)) {
    sdLista = true;
    inicializarArchivosSD();
  }

  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
  if (LoRa.begin(433E6)) {
    LoRa.setSyncWord(0xF3);
    LoRa.enableCrc();
    loraListo = true;
  }

  Serial.println(String("SD: ") + (sdLista ? "OK" : "ERROR") +
                 " | LoRa: " + (loraListo ? "OK" : "ERROR"));

  actualizarLedEstado();
  if (sdLista && loraListo) {
    iniciarBeep(100, 1);
  } else {
    iniciarBeep(150, 3);
  }

  actualizarLecturasSensores();
  dibujarPantallaOLED();

  #if ESP_ARDUINO_VERSION_MAJOR >= 3
    esp_task_wdt_config_t wdt_config = {
      .timeout_ms = 4000,
      .idle_core_mask = 0,
      .trigger_panic = true
    };
    esp_task_wdt_reconfigure(&wdt_config);
  #else
    esp_task_wdt_init(4, true);
  #endif
  esp_task_wdt_add(NULL);

  Serial.println("Robot listo. Ultrasonico ACTIVO (LED Amarillo <= 25cm).");
}

// ================= LOOP =================
void loop() {
  esp_task_wdt_reset();

  actualizarBuzzer();

  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  if (rtcOk && !rtcSincronizado && gps.date.isValid() && gps.time.isValid() && gps.date.year() >= 2026) {
    DateTime now = rtc.now();
    if (now.year() < 2026) {
      DateTime gpsUTC(gps.date.year(), gps.date.month(), gps.date.day(),
                      gps.time.hour(), gps.time.minute(), gps.time.second());
      rtc.adjust(gpsUTC - TimeSpan(0, 6, 0, 0));
    }
    rtcSincronizado = true;
  }

  if (loraListo) {
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
      String rxData = "";
      while (LoRa.available()) rxData += (char)LoRa.read();
      procesarComandoJSON(rxData, "LORA");
    }
  }

  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
      serialBuffer.trim();
      if (serialBuffer.length() > 0) {
        procesarComandoJSON(serialBuffer, "PC-MQTT");
      }
      serialBuffer = "";
    } else if (c != '\r') {
      if (serialBuffer.length() < 250) {
        serialBuffer += c;
      } else {
        serialBuffer = "";
      }
    }
  }

  // 3. Control de locomocion y lectura ultrasonica (cada 60 ms)
  static long lecturas[3] = {999, 999, 999};
  static int idxLectura = 0;
  unsigned long ahora = millis();

  if (ahora - ultimoControlMs >= INTERVALO_CONTROL) {
    ultimoControlMs = ahora;

    lecturas[idxLectura] = medirDistanciaCm();
    idxLectura = (idxLectura + 1) % 3;
    distanciaActual = medianaDe3(lecturas[0], lecturas[1], lecturas[2]);

    // Actualizar LED (Amarillo si detecta objeto <= 25 cm) y avisar en consola
    bool detectadoAhora = (USAR_ULTRASONICO && distanciaActual <= DIST_OBSTACULO_CM);
    if (detectadoAhora != objetoDetectadoPrev) {
      objetoDetectadoPrev = detectadoAhora;
      actualizarLedEstado();
      forzarRefrescoOled = true;
      if (detectadoAhora) {
        Serial.println("[ULTRASONICO] Objeto detectado a " + String(distanciaActual) + " cm -> LED AMARILLO");
      } else {
        Serial.println("[ULTRASONICO] Via libre (" + String(distanciaActual) + " cm)");
      }
    }

    if (modoActual == "LINEA")          pasoRampa = PASO_RAMPA_LINEA;
    else if (modoActual == "OBSTACULO" || modoActual == "CIRCUITO") pasoRampa = PASO_RAMPA_GIRO;
    else                                pasoRampa = PASO_RAMPA_NORMAL;

    if (modoActual == "MANUAL") {
      if (millis() - ultimaOrdenManualMs > TIMEOUT_MANUAL_MS) {
        setMotoresObjetivo(0, 0);
        if (ultimaOrden.startsWith("MOV:") || ultimaOrden.startsWith("GIRO:")) {
          if (ultimaOrden != "MOV:STOP") {
            ultimaOrden = "MOV:STOP";
            forzarRefrescoOled = true;
          }
        }
      }
    } else if (modoActual == "CIRCUITO") {
      ejecutarCircuitoPrueba(distanciaActual);
    } else if (modoActual == "LINEA") {
      ejecutarSeguidorLinea();
    } else if (modoActual == "OBSTACULO") {
      ejecutarEvitadorObstaculos(distanciaActual);
    } else if (modoActual == "AUTO") {
      ejecutarNavegacionAutoGPS(distanciaActual);
    }
  }

  if (ahora - ultimoRampaMs >= INTERVALO_RAMPA) {
    ultimoRampaMs = ahora;
    actualizarMotoresSuave();
  }

  if (ahora - ultimoOledMs >= INTERVALO_OLED) {
    ultimoOledMs = ahora;
    actualizarLecturasSensores();
    dibujarPantallaOLED();
    forzarRefrescoOled = false;
  } else if (forzarRefrescoOled) {
    dibujarPantallaOLED();
    forzarRefrescoOled = false;
  }

  if (ahora - ultimaTelemetriaMs >= INTERVALO_TELEMETRIA) {
    ultimaTelemetriaMs = ahora;

    String timestamp = obtenerTimestamp();
    bool fix = gpsConFix();
    double lat = fix ? gps.location.lat() : 0.0;
    double lng = fix ? gps.location.lng() : 0.0;
    int sats = gps.satellites.isValid() ? gps.satellites.value() : 0;

    if (sdLista) {
      if (fix) {
        File fUbi = SD.open("/ubicacion.csv", FILE_APPEND);
        if (fUbi) {
          fUbi.println(timestamp + "," + String(lat, 6) + "," + String(lng, 6) + "," + String(sats));
          fUbi.close();
        }
      }
      File fClima = SD.open("/clima.csv", FILE_APPEND);
      if (fClima) {
        fClima.println(timestamp + "," + String(tempCache, 1) + "," + String(humCache, 1) + "," +
                       String(presionCache, 1) + "," + String(alturaCache, 1));
        fClima.close();
      }
    }

    if (loraListo) {
      StaticJsonDocument<384> txDoc;
      txDoc["ts"] = timestamp;
      txDoc["fix"] = fix;
      txDoc["sats"] = sats;
      if (fix) {
        txDoc["lat"] = lat;
        txDoc["lng"] = lng;
      }
      if (!isnan(tempCache))    txDoc["temp"] = round(tempCache * 10) / 10.0;
      if (!isnan(humCache))     txDoc["hum"]  = round(humCache * 10) / 10.0;
      if (!isnan(presionCache)) txDoc["pres"] = round(presionCache * 10) / 10.0;
      if (!isnan(alturaCache))  txDoc["alt"]  = round(alturaCache * 10) / 10.0;
      txDoc["modo"] = modoActual;
      txDoc["dist_obs"] = distanciaActual;

      String payload;
      serializeJson(txDoc, payload);

      setLed(true, true);
      LoRa.beginPacket();
      LoRa.print(payload);
      LoRa.endPacket();
      actualizarLedEstado();
    }
  }
}