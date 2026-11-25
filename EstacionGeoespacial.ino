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

// Ultrasonico HC-SR04 (Divisor de voltaje en ECHO hacia GPIO 34)
#define TRIG_PIN   33
#define ECHO_PIN   34

// Seguidor de linea TCRT5000 (2 sensores separados ~3.5 cm: Izq en GPIO 35, Der en GPIO 36)
#define IR_IZQ     35
#define IR_CEN     36
#define IR_DER     39

// ACTIVADO: Divisor de voltaje instalado en GPIO 34
#define USAR_ULTRASONICO true

// ================= CONSTANTES DE TIEMPO Y CALIBRACION =================
const unsigned long INTERVALO_OLED        = 1000;
const unsigned long INTERVALO_TELEMETRIA  = 4000; // 4s para reducir colisiones half-duplex en LoRa
const unsigned long INTERVALO_CONTROL     = 15;   // Lectura rapida cada 15 ms para curvas en modo LINEA
const unsigned long INTERVALO_ULTRASONICO = 60;   // Lectura HC-SR04 cada 60 ms para evitar ecos cruzados
const unsigned long INTERVALO_RAMPA       = 15;   // Actualizacion de rampa cada 15 ms
const unsigned long TIMEOUT_MANUAL_MS     = 5000;

// Auto-recuperacion de LoRa
const unsigned long INTERVALO_SALUD_LORA_MS = 5000;  // Verifica registro SyncWord 0x39 cada 5s
const unsigned long TIMEOUT_RX_LORA_MS      = 25000; // Reinicio suave si lleva 25s sin recibir nada

const int PASO_RAMPA_NORMAL  = 20;
const int PASO_RAMPA_LINEA   = 160;  // Respuesta instantanea al detectar curva
const int PASO_RAMPA_EVASION = 14;   // Aceleracion suave en modo OBSTACULO / LABERINTO
const int PASO_RAMPA_GIRO    = 160;  // Torque directo para giros precisos sobre eje

const int VEL_MAX               = 160;  // Tope maximo general
const int VEL_AVANCE            = 120;  // Velocidad base inicial en modo MANUAL / AUTO / CIRCUITO
const int VEL_GIRO              = 160;  // Velocidad de giro sobre su eje

// Velocidades y tiempos calibrados para modo LINEA (sensores separados ~3.5 cm + Kickstart)
const int VEL_LINEA_RECTO                 = 62;
const int VEL_LINEA_GIRO_EXT              = 160;
const int VEL_LINEA_GIRO_INT              = -110;
const int VEL_KICKSTART_LINEA             = 120;
const unsigned long DURACION_KICKSTART_MS = 95;
const unsigned long RETENCION_CURVA_MS    = 310;

// Velocidades y umbrales para modo OBSTACULO ({90°, 135°} + micro-barrido +-20°)
const int VEL_OBS_NORMAL        = 95;   // Crucero cuando dist >= 100 cm
const int VEL_OBS_MINIMA        = 65;   // Velocidad minima de aproximacion cerca de 25 cm
const long DIST_DESPEJADO_CM    = 100;  // Inicia desaceleracion proporcional
const long DIST_OBSTACULO_CM    = 25;   // Detencion total
const long DIST_SALIDA_LIBRE_CM = 38;   // Distancia minima para aceptar una ruta lateral como libre
const int ANGULO_MICRO_BARRIDO  = 20;   // +-20 grados para cubrir el ancho completo del chasis
const unsigned long TIEMPO_CONFIRMAR_LIBRE_MS = 300; // Espera breve tras finalizar micro-barrido

// Umbrales Laberinto
const long DIST_PARED_LAB_CM    = 35;
const long DIST_SALIDA_LAB_CM   = 115;

// CALIBRAR ESTOS TIEMPOS EN EL PISO REAL:
const unsigned long TIEMPO_1M_MS                 = 3300; // ms que tarda en avanzar 1 metro a vel 120
const unsigned long TIEMPO_CELDA_LAB_MS          = 1650; // ms para avanzar 1 celda del laberinto (~50 cm)
const unsigned long TIEMPO_GIRO_90_MS            = 800;  // ms que tarda en girar 90 grados a vel 160
const unsigned long TIEMPO_GIRO_180_MS           = 1600; // ms que tarda en girar 180 grados a vel 160
const unsigned long TIEMPO_RETROCESO_BLOQUEO_MS  = 550;
const unsigned long TIEMPO_DESPEJE_GPS_MS        = 1600;

// Tiempos de rodeo para modo LINEA_OBS
const unsigned long TIEMPO_RODEO_LATERAL_MS      = 950;
const unsigned long TIEMPO_RODEO_PARALELO_MS     = 1400;
const unsigned long TIMEOUT_BUSCAR_LINEA_MS      = 4500;

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
enum EstadoEvasion {
  EV_CONFIRMAR_ANTES_AVANZAR,
  EV_AVANZAR,
  EV_DETENER_ESPERA,
  EV_GIRAR_PRUEBA,
  EV_ESPERA_LECTURA,
  EV_BARRIDO_GIRAR,
  EV_BARRIDO_LECTURA,
  EV_RETROCEDER_BLOQUEO
};

enum EstadoRodeoLinea {
  RL_SIGUIENDO_LINEA,
  RL_GIRO_SALIDA_90,
  RL_AVANCE_LATERAL,
  RL_GIRO_PARALELO_90,
  RL_AVANCE_PARALELO,
  RL_GIRO_RETORNO_45,
  RL_BUSCANDO_CINTA
};

enum EstadoLaberinto {
  LB_EVALUAR_CELDA,
  LB_GIRAR_ESCANEO,
  LB_ESPERA_LECTURA_SCAN,
  LB_AVANZAR_CELDA,
  LB_RESUELTO
};

String modoActual = "MANUAL"; // MANUAL, LINEA, LINEA_OBS, OBSTACULO, LABERINTO, AUTO, CIRCUITO
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
unsigned long ultimoUltrasonicoMs = 0;
unsigned long ultimoRampaMs       = 0;
unsigned long ultimaOrdenManualMs = 0;
unsigned long ultimaSaludLoraMs   = 0;
unsigned long ultimoRxLoraMs      = 0;

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
int velManualWeb = VEL_AVANCE;

// ================= EVASION {90°, 135°} + MICRO-BARRIDO + MEMORIA CIRCULAR (X, Y) =================
EstadoEvasion estadoEv = EV_CONFIRMAR_ANTES_AVANZAR;
unsigned long timerEv = 0;
unsigned long timerConfirmacionLibre = 0;
unsigned long timerDespejeGps = 0;
bool enTramoDespejeGps = false;
unsigned long duracionGiroEv = 0;

int anguloActualRel = 0;
int nivelAperturaIdx = 0;    // 0: 90°, 1: 135°
int primerSignoOrden = -1;   // -1 (Izq primero) o +1 (Der primero)
int intentoEnNivel = 0;      // 0: primer lado elegido, 1: lado opuesto
int signoPreferidoReciente = 0;
unsigned long timerPreferenciaGiro = 0;

// Variables de micro-barrido de confirmacion (+-20°)
int anguloBaseBarrido = 0;
int pasoBarridoIdx = 0;      // 0: -20°, 1: +20°, 2: volver al centro del nuevo rumbo

// Odometria relativa e historial circular de ultimas 12 posiciones (en cm)
const int HIST_OBS_SIZE = 12;
float obsPosX = 0.0f, obsPosY = 0.0f;
float obsRumboDeg = 0.0f;
float histObsX[HIST_OBS_SIZE];
float histObsY[HIST_OBS_SIZE];
int histObsIdx = 0;
int histObsCount = 0;
float distAcumuladaHist = 0.0f;
unsigned long ultimoPasoOdomMs = 0;

// Rodeo de obstaculos sobre linea (modo LINEA_OBS)
EstadoRodeoLinea estadoRodeo = RL_SIGUIENDO_LINEA;
unsigned long timerRodeo = 0;
unsigned long duracionGiroRodeo = 0;

// ================= ALGORITMO DE TREMAUX (MODO LABERINTO) =================
const int LAB_SIZE = 15;
uint8_t visitasLab[LAB_SIZE][LAB_SIZE];
int labX = LAB_SIZE / 2;
int labY = LAB_SIZE / 2;
int labDir = 0; // 0: Norte (+Y), 1: Este (+X), 2: Sur (-Y), 3: Oeste (-X)
EstadoLaberinto estadoLab = LB_EVALUAR_CELDA;
int subPasoLab = 0;
unsigned long timerLab = 0;
unsigned long duracionGiroLab = 0;
long dFrenteLab = 0, dIzqLab = 0, dDerLab = 0;

// Modo CIRCUITO de prueba
int pasoCircuito = 0;
unsigned long timerCircuito = 0;

// Seguidor de linea (2 sensores con retencion de curva y kickstart)
int ultimoLadoLinea = 0;
unsigned long timerRetencionCurva = 0;
unsigned long timerKickstartLinea = 0;
bool kickstartLineaActivo = false;

// Navegacion GPS + Memoria de Celdas Visitadas
bool destinoActivo = false;
double targetLat = 0.0, targetLng = 0.0;
double origenLat = 0.0, origenLng = 0.0;
bool origenFijado = false;
double prevLat = 0.0, prevLng = 0.0;
unsigned long timerAutoGps = 0;
unsigned long duracionGiro = 0;
int estadoAuto = 0;

const int GRID_SIZE = 21;
const double CELDA_M = 1.5;
uint8_t mapaCosto[GRID_SIZE][GRID_SIZE];

// Buzzer no bloqueante
bool buzzerActivo = false;
bool enPausaBeep = false;
unsigned long timerBuzzer = 0;
unsigned long duracionBuzzer = 0;
int beepsRestantes = 0;

// ================= LED ANODO COMUN (LOW ENCIENDE, HIGH APAGA) =================
void setLed(bool r, bool g) {
  digitalWrite(LED_R, !r);
  digitalWrite(LED_G, !g);
}

void actualizarLedEstado() {
  if (modoActual == "LINEA" || (modoActual == "LINEA_OBS" && estadoRodeo == RL_SIGUIENDO_LINEA)) {
    bool izq = (digitalRead(IR_IZQ) == HIGH);
    bool der = (digitalRead(IR_CEN) == HIGH);
    bool veLinea = (izq || der);
    setLed(!veLinea, veLinea);
    return;
  }

  if (USAR_ULTRASONICO && distanciaActual <= DIST_OBSTACULO_CM) {
    setLed(true, true);
    return;
  }

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
  return m == "MANUAL" || m == "LINEA" || m == "LINEA_OBS" ||
         m == "OBSTACULO" || m == "LABERINTO" || m == "AUTO" || m == "CIRCUITO";
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

int ajustarPwmWeb(int valorCrudo) {
  if (valorCrudo > VEL_MAX) {
    return map(constrain(valorCrudo, 0, 255), 0, 255, 55, VEL_MAX);
  }
  return constrain(valorCrudo, 55, VEL_MAX);
}

// ================= INICIALIZACION Y AUTO-RECUPERACION DE LORA =================
uint8_t leerRegistroLoRaSPI(uint8_t direccion) {
  digitalWrite(SD_CS, HIGH);
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(LORA_CS, LOW);
  SPI.transfer(direccion & 0x7F); // Bit 7 en 0 indica lectura en SX1278
  uint8_t valor = SPI.transfer(0x00);
  digitalWrite(LORA_CS, HIGH);
  SPI.endTransaction();
  return valor;
}

bool inicializarModuloLoRa(bool hardwareReset) {
  digitalWrite(SD_CS, HIGH);
  if (hardwareReset) {
    pinMode(LORA_RST, OUTPUT);
    digitalWrite(LORA_RST, LOW);
    delay(10);
    digitalWrite(LORA_RST, HIGH);
    delay(10);
  }
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
  if (LoRa.begin(433E6)) {
    LoRa.setSyncWord(0xF3);
    LoRa.enableCrc();
    LoRa.receive();
    loraListo = true;
    ultimoRxLoraMs = millis();
    return true;
  }
  loraListo = false;
  return false;
}

void verificarSaludLoRa() {
  unsigned long ahora = millis();
  if (ahora - ultimaSaludLoraMs < INTERVALO_SALUD_LORA_MS) return;
  ultimaSaludLoraMs = ahora;

  // 1. Verifica por SPI si el chip SX1278 perdio su SyncWord (0xF3) por micro-caida de voltaje
  uint8_t syncWordLeido = leerRegistroLoRaSPI(0x39);
  bool chipDesconfigurado = (!loraListo || syncWordLeido != 0xF3);

  // 2. Verifica si lleva demasiado tiempo sin recibir paquetes del gateway
  bool silencioProlongado = (ahora - ultimoRxLoraMs >= TIMEOUT_RX_LORA_MS);

  if (chipDesconfigurado || silencioProlongado) {
    Serial.println(chipDesconfigurado
      ? "[LORA-WDT] Registro perdido (0x" + String(syncWordLeido, HEX) + ") -> Reiniciando RF..."
      : "[LORA-WDT] Sin RX por 25s -> Refrescando transceptor...");
    inicializarModuloLoRa(true);
    actualizarLedEstado();
  } else {
    LoRa.receive();
  }
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

void reiniciarMemoriaObstaculos() {
  obsPosX = 0.0f;
  obsPosY = 0.0f;
  obsRumboDeg = 0.0f;
  histObsIdx = 0;
  histObsCount = 0;
  distAcumuladaHist = 0.0f;
  ultimoPasoOdomMs = millis();
  signoPreferidoReciente = 0;
  anguloActualRel = 0;
  nivelAperturaIdx = 0;
  intentoEnNivel = 0;
  pasoBarridoIdx = 0;
}

void reiniciarLaberinto() {
  memset(visitasLab, 0, sizeof(visitasLab));
  labX = LAB_SIZE / 2;
  labY = LAB_SIZE / 2;
  labDir = 0;
  estadoLab = LB_EVALUAR_CELDA;
  subPasoLab = 0;
  timerLab = millis();
}

void reiniciarMovimiento() {
  detenerInmediato();
  destinoActivo = false;
  estadoAuto = 0;
  estadoEv = EV_CONFIRMAR_ANTES_AVANZAR;
  timerConfirmacionLibre = millis();
  enTramoDespejeGps = false;
  pasoCircuito = 0;
  ultimoLadoLinea = 0;
  timerRetencionCurva = 0;
  kickstartLineaActivo = false;
  estadoRodeo = RL_SIGUIENDO_LINEA;
  reiniciarMemoriaObstaculos();
  reiniciarLaberinto();
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
    mapaCosto[gx][gy] = 255;
  } else if (mapaCosto[gx][gy] < 200) {
    mapaCosto[gx][gy] += 2;
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
  digitalWrite(LORA_CS, HIGH);
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
  digitalWrite(SD_CS, HIGH);
}

void registrarEvento(const String &evento) {
  if (!sdLista) return;
  digitalWrite(LORA_CS, HIGH);
  File f = SD.open("/eventos.csv", FILE_APPEND);
  if (f) {
    f.println(obtenerTimestamp() + "," + modoActual + "," + evento);
    f.close();
  }
  digitalWrite(SD_CS, HIGH);
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
  display.print(" "); display.print(modoActual.substring(0, 4));
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
    case 0:
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

    case 2:
      if (millis() - timerCircuito >= 250) {
        ultimaOrden = "CIRC:2/4 Giro 90I";
        forzarRefrescoOled = true;
        setMotoresObjetivo(-VEL_GIRO, VEL_GIRO);
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

    case 4:
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

    case 6:
      if (millis() - timerCircuito >= 250) {
        ultimaOrden = "CIRC:4/4 Giro 180";
        forzarRefrescoOled = true;
        setMotoresObjetivo(VEL_GIRO, -VEL_GIRO);
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

// ================= SEGUIDOR DE LINEA PURO (3.5 CM + KICKSTART + RETENCION) =================
void ejecutarSeguidorLinea() {
  bool izq = (digitalRead(IR_IZQ) == HIGH);
  bool der = (digitalRead(IR_CEN) == HIGH);

  actualizarLedEstado();

  if (izq && !der) {
    ultimoLadoLinea = -1;
    kickstartLineaActivo = false;
    timerRetencionCurva = millis();
    setMotoresObjetivo(VEL_LINEA_GIRO_INT, VEL_LINEA_GIRO_EXT);

  } else if (!izq && der) {
    ultimoLadoLinea = 1;
    kickstartLineaActivo = false;
    timerRetencionCurva = millis();
    setMotoresObjetivo(VEL_LINEA_GIRO_EXT, VEL_LINEA_GIRO_INT);

  } else if (izq && der) {
    kickstartLineaActivo = false;
    timerRetencionCurva = millis();
    if (ultimoLadoLinea <= 0) {
      setMotoresObjetivo(VEL_LINEA_GIRO_INT, VEL_LINEA_GIRO_EXT);
    } else {
      setMotoresObjetivo(VEL_LINEA_GIRO_EXT, VEL_LINEA_GIRO_INT);
    }

  } else {
    if (ultimoLadoLinea != 0 && (millis() - timerRetencionCurva < RETENCION_CURVA_MS)) {
      if (ultimoLadoLinea < 0) {
        setMotoresObjetivo(VEL_LINEA_GIRO_INT, VEL_LINEA_GIRO_EXT);
      } else {
        setMotoresObjetivo(VEL_LINEA_GIRO_EXT, VEL_LINEA_GIRO_INT);
      }
    } else {
      ultimoLadoLinea = 0;

      if (velActualIzq == 0 && velActualDer == 0 && !kickstartLineaActivo) {
        kickstartLineaActivo = true;
        timerKickstartLinea = millis();
      }

      if (kickstartLineaActivo && (millis() - timerKickstartLinea < DURACION_KICKSTART_MS)) {
        setMotoresObjetivo(VEL_KICKSTART_LINEA, VEL_KICKSTART_LINEA);
      } else {
        kickstartLineaActivo = false;
        setMotoresObjetivo(VEL_LINEA_RECTO, VEL_LINEA_RECTO);
      }
    }
  }
}

// ================= MODO LINEA + OBSTACULOS (RODEO Y REENGANCHE DE CINTA) =================
void ejecutarLineaConObstaculos(long dist) {
  bool izq = (digitalRead(IR_IZQ) == HIGH);
  bool der = (digitalRead(IR_CEN) == HIGH);

  switch (estadoRodeo) {
    case RL_SIGUIENDO_LINEA:
      if (dist <= DIST_OBSTACULO_CM) {
        detenerInmediato();
        registrarEvento("LINEA_OBS: Obstaculo a " + String(dist) + " cm -> Rodeando");
        ultimaOrden = "L_OBS:Rodeo 90D";
        forzarRefrescoOled = true;
        pasoRampa = PASO_RAMPA_GIRO;
        duracionGiroRodeo = TIEMPO_GIRO_90_MS;
        setMotoresObjetivo(VEL_GIRO, -VEL_GIRO);
        timerRodeo = millis();
        estadoRodeo = RL_GIRO_SALIDA_90;
      } else {
        ejecutarSeguidorLinea();
      }
      break;

    case RL_GIRO_SALIDA_90:
      if (millis() - timerRodeo >= duracionGiroRodeo) {
        detenerInmediato();
        ultimaOrden = "L_OBS:Lateral";
        forzarRefrescoOled = true;
        pasoRampa = PASO_RAMPA_EVASION;
        setMotoresObjetivo(80, 80);
        timerRodeo = millis();
        estadoRodeo = RL_AVANCE_LATERAL;
      }
      break;

    case RL_AVANCE_LATERAL:
      if (dist <= DIST_OBSTACULO_CM) {
        detenerInmediato();
        ultimaOrden = "L_OBS:Bloq Lat";
        forzarRefrescoOled = true;
        modoActual = "MANUAL";
        iniciarBeep(200, 2);
      } else if (millis() - timerRodeo >= TIEMPO_RODEO_LATERAL_MS) {
        detenerInmediato();
        ultimaOrden = "L_OBS:Paralelo";
        forzarRefrescoOled = true;
        pasoRampa = PASO_RAMPA_GIRO;
        duracionGiroRodeo = TIEMPO_GIRO_90_MS;
        setMotoresObjetivo(-VEL_GIRO, VEL_GIRO);
        timerRodeo = millis();
        estadoRodeo = RL_GIRO_PARALELO_90;
      }
      break;

    case RL_GIRO_PARALELO_90:
      if (millis() - timerRodeo >= duracionGiroRodeo) {
        detenerInmediato();
        pasoRampa = PASO_RAMPA_EVASION;
        setMotoresObjetivo(80, 80);
        timerRodeo = millis();
        estadoRodeo = RL_AVANCE_PARALELO;
      }
      break;

    case RL_AVANCE_PARALELO:
      if (dist <= DIST_OBSTACULO_CM) {
        detenerInmediato();
        ultimaOrden = "L_OBS:Bloq Par";
        forzarRefrescoOled = true;
        modoActual = "MANUAL";
        iniciarBeep(200, 2);
      } else if (millis() - timerRodeo >= TIEMPO_RODEO_PARALELO_MS) {
        detenerInmediato();
        ultimaOrden = "L_OBS:BuscaLin";
        forzarRefrescoOled = true;
        pasoRampa = PASO_RAMPA_GIRO;
        duracionGiroRodeo = TIEMPO_GIRO_90_MS / 2;
        setMotoresObjetivo(-VEL_GIRO, VEL_GIRO);
        timerRodeo = millis();
        estadoRodeo = RL_GIRO_RETORNO_45;
      }
      break;

    case RL_GIRO_RETORNO_45:
      if (millis() - timerRodeo >= duracionGiroRodeo) {
        detenerInmediato();
        pasoRampa = PASO_RAMPA_LINEA;
        kickstartLineaActivo = true;
        timerKickstartLinea = millis();
        timerRodeo = millis();
        estadoRodeo = RL_BUSCANDO_CINTA;
      }
      break;

    case RL_BUSCANDO_CINTA:
      if (izq || der) {
        ultimoLadoLinea = 1;
        timerRetencionCurva = millis();
        ultimaOrden = "L_OBS:Linea OK";
        forzarRefrescoOled = true;
        estadoRodeo = RL_SIGUIENDO_LINEA;
      } else if (millis() - timerRodeo >= TIMEOUT_BUSCAR_LINEA_MS) {
        detenerInmediato();
        ultimaOrden = "L_OBS:SinLinea";
        forzarRefrescoOled = true;
        modoActual = "MANUAL";
        iniciarBeep(200, 3);
      } else {
        if (kickstartLineaActivo && (millis() - timerKickstartLinea < DURACION_KICKSTART_MS)) {
          setMotoresObjetivo(VEL_KICKSTART_LINEA, VEL_KICKSTART_LINEA);
        } else {
          kickstartLineaActivo = false;
          setMotoresObjetivo(VEL_LINEA_RECTO, VEL_LINEA_RECTO);
        }
      }
      break;
  }
}

// ================= ALGORITMO DE TREMAUX PARA LABERINTO EN INTERIORES =================
uint8_t obtenerVisitasVecino(int x, int y, int dirEval) {
  int nx = x;
  int ny = y;
  if (dirEval == 0) ny++;
  else if (dirEval == 1) nx++;
  else if (dirEval == 2) ny--;
  else if (dirEval == 3) nx--;

  if (nx < 0 || nx >= LAB_SIZE || ny < 0 || ny >= LAB_SIZE) return 255;
  return visitasLab[nx][ny];
}

void avanzarCoordenadaLaberinto() {
  if (labDir == 0) labY++;
  else if (labDir == 1) labX++;
  else if (labDir == 2) labY--;
  else if (labDir == 3) labX--;
  labX = constrain(labX, 0, LAB_SIZE - 1);
  labY = constrain(labY, 0, LAB_SIZE - 1);
}

void iniciarGiroLaberinto(int gradosRel) {
  pasoRampa = PASO_RAMPA_GIRO;
  int norm = (int)normalizarAngulo(gradosRel);
  if (norm == 0) {
    duracionGiroLab = 0;
  } else {
    duracionGiroLab = (unsigned long)(abs(norm) / 90.0 * TIEMPO_GIRO_90_MS);
    if (norm > 0) setMotoresObjetivo(VEL_GIRO, -VEL_GIRO);
    else          setMotoresObjetivo(-VEL_GIRO, VEL_GIRO);
  }
  timerLab = millis();
  estadoLab = LB_GIRAR_ESCANEO;
}

void ejecutarLaberintoTremaux(long dist) {
  switch (estadoLab) {
    case LB_RESUELTO:
      detenerInmediato();
      break;

    case LB_EVALUAR_CELDA:
      detenerInmediato();
      if (millis() - timerLab >= 200) {
        dFrenteLab = dist;
        subPasoLab = 1;
        ultimaOrden = "LAB(" + String(labX) + "," + String(labY) + ")Izq";
        forzarRefrescoOled = true;
        iniciarGiroLaberinto(-90);
      }
      break;

    case LB_GIRAR_ESCANEO:
      if (millis() - timerLab >= duracionGiroLab) {
        detenerInmediato();
        timerLab = millis();
        estadoLab = LB_ESPERA_LECTURA_SCAN;
      }
      break;

    case LB_ESPERA_LECTURA_SCAN:
      if (millis() - timerLab >= 200) {
        if (subPasoLab == 1) {
          dIzqLab = distanciaActual;
          subPasoLab = 2;
          ultimaOrden = "LAB(" + String(labX) + "," + String(labY) + ")Der";
          forzarRefrescoOled = true;
          iniciarGiroLaberinto(180);

        } else if (subPasoLab == 2) {
          dDerLab = distanciaActual;

          if (dFrenteLab >= DIST_SALIDA_LAB_CM && dIzqLab >= DIST_SALIDA_LAB_CM && dDerLab >= DIST_SALIDA_LAB_CM) {
            detenerInmediato();
            registrarEvento("LABERINTO RESUELTO en (" + String(labX) + "," + String(labY) + ")");
            ultimaOrden = "LAB:SALIDA OK!";
            forzarRefrescoOled = true;
            iniciarBeep(300, 3);
            estadoLab = LB_RESUELTO;
            return;
          }

          int dirFrente = labDir;
          int dirIzq    = (labDir + 3) % 4;
          int dirDer    = (labDir + 1) % 4;
          int dirAtras  = (labDir + 2) % 4;

          bool libreFrente = (dFrenteLab > DIST_PARED_LAB_CM);
          bool libreIzq    = (dIzqLab    > DIST_PARED_LAB_CM);
          bool libreDer    = (dDerLab    > DIST_PARED_LAB_CM);

          uint8_t vFrente = libreFrente ? obtenerVisitasVecino(labX, labY, dirFrente) : 255;
          uint8_t vIzq    = libreIzq    ? obtenerVisitasVecino(labX, labY, dirIzq)    : 255;
          uint8_t vDer    = libreDer    ? obtenerVisitasVecino(labX, labY, dirDer)    : 255;

          int giroDesdeDerecha = 0;
          bool hayRutaNueva = false;

          if (vFrente == 0) {
            labDir = dirFrente;
            giroDesdeDerecha = -90;
            hayRutaNueva = true;
          } else if (vIzq == 0) {
            labDir = dirIzq;
            giroDesdeDerecha = 180;
            hayRutaNueva = true;
          } else if (vDer == 0) {
            labDir = dirDer;
            giroDesdeDerecha = 0;
            hayRutaNueva = true;
          }

          if (hayRutaNueva) {
            if (visitasLab[labX][labY] == 0) visitasLab[labX][labY] = 1;
            registrarEvento("Tremaux avanza (" + String(labX) + "," + String(labY) + ") dir=" + String(labDir));
          } else {
            visitasLab[labX][labY] = 2;
            registrarEvento("Tremaux cierra callejon (" + String(labX) + "," + String(labY) + ")");

            if (vFrente == 1) {
              labDir = dirFrente;
              giroDesdeDerecha = -90;
            } else if (vIzq == 1) {
              labDir = dirIzq;
              giroDesdeDerecha = 180;
            } else if (vDer == 1) {
              labDir = dirDer;
              giroDesdeDerecha = 0;
            } else {
              labDir = dirAtras;
              giroDesdeDerecha = 90;
            }
          }

          subPasoLab = 3;
          ultimaOrden = "LAB:Dir->" + String(labDir);
          forzarRefrescoOled = true;
          iniciarGiroLaberinto(giroDesdeDerecha);

        } else if (subPasoLab == 3) {
          avanzarCoordenadaLaberinto();
          pasoRampa = PASO_RAMPA_EVASION;
          timerLab = millis();
          ultimaOrden = "LAB:Mov(" + String(labX) + "," + String(labY) + ")";
          forzarRefrescoOled = true;
          estadoLab = LB_AVANZAR_CELDA;
        }
      }
      break;

    case LB_AVANZAR_CELDA:
      if (dist <= DIST_OBSTACULO_CM || (millis() - timerLab >= TIEMPO_CELDA_LAB_MS)) {
        detenerInmediato();
        timerLab = millis();
        estadoLab = LB_EVALUAR_CELDA;
      } else {
        pasoRampa = PASO_RAMPA_EVASION;
        setMotoresObjetivo(VEL_OBS_NORMAL, VEL_OBS_NORMAL);
      }
      break;
  }
}

// ================= EVASION {90°, 135°} + MICRO-BARRIDO + MEMORIA CIRCULAR (X, Y) =================
const int NIVELES_APERTURA_AGIL[2] = {90, 135};

void actualizarOdometriaObstaculos(int velAvanceActual) {
  unsigned long ahora = millis();
  unsigned long dt = ahora - ultimoPasoOdomMs;
  ultimoPasoOdomMs = ahora;

  if (velAvanceActual <= 0 || dt == 0 || dt > 500) return;

  float velCmPorMs = (100.0f / (float)TIEMPO_1M_MS) * ((float)velAvanceActual / (float)VEL_AVANCE);
  float dCm = velCmPorMs * (float)dt;
  float rad = radians(obsRumboDeg);

  obsPosX += dCm * sin(rad);
  obsPosY += dCm * cos(rad);
  distAcumuladaHist += dCm;

  if (distAcumuladaHist >= 45.0f) {
    distAcumuladaHist = 0.0f;
    histObsX[histObsIdx] = obsPosX;
    histObsY[histObsIdx] = obsPosY;
    histObsIdx = (histObsIdx + 1) % HIST_OBS_SIZE;
    if (histObsCount < HIST_OBS_SIZE) histObsCount++;
  }
}

float evaluarFamiliaridadRuta(float rumboPropuestoDeg) {
  float rad = radians(rumboPropuestoDeg);
  float futX = obsPosX + 60.0f * sin(rad);
  float futY = obsPosY + 60.0f * cos(rad);
  float penalizacion = 0.0f;

  for (int i = 0; i < histObsCount; i++) {
    float dx = futX - histObsX[i];
    float dy = futY - histObsY[i];
    float distSq = dx * dx + dy * dy;
    if (distSq < 2500.0f) {
      penalizacion += (2500.0f - distSq) / 500.0f;
    }
  }
  return penalizacion;
}

int elegirPrimerLadoPrometedor() {
  float famIzq = evaluarFamiliaridadRuta(obsRumboDeg - 90.0f);
  float famDer = evaluarFamiliaridadRuta(obsRumboDeg + 90.0f);

  if (fabs(famIzq - famDer) > 1.5f) {
    return (famIzq < famDer) ? -1 : 1;
  }

  if (signoPreferidoReciente != 0 && (millis() - timerPreferenciaGiro < 5000)) {
    return signoPreferidoReciente;
  }

  return (histObsIdx % 2 == 0) ? -1 : 1;
}

void comandarGiroHacia(int anguloObjetivoRelativo, EstadoEvasion siguienteEstado) {
  int delta = (int)normalizarAngulo(anguloObjetivoRelativo - anguloActualRel);
  anguloActualRel = (int)normalizarAngulo(anguloObjetivoRelativo);
  obsRumboDeg = (float)normalizarAngulo(obsRumboDeg + delta);
  pasoRampa = PASO_RAMPA_GIRO;

  if (delta == 0) {
    duracionGiroEv = 0;
  } else {
    duracionGiroEv = (unsigned long)(abs(delta) / 90.0 * TIEMPO_GIRO_90_MS);
    if (delta > 0) setMotoresObjetivo(VEL_GIRO, -VEL_GIRO);
    else           setMotoresObjetivo(-VEL_GIRO, VEL_GIRO);
  }
  timerEv = millis();
  estadoEv = siguienteEstado;
}

void avanzarSiguienteIntentoEvasion() {
  intentoEnNivel++;
  if (intentoEnNivel < 2) {
    timerEv = millis();
    estadoEv = EV_DETENER_ESPERA;
  } else {
    intentoEnNivel = 0;
    nivelAperturaIdx++;
    if (nivelAperturaIdx < 2) {
      timerEv = millis();
      estadoEv = EV_DETENER_ESPERA;
    } else {
      ultimaOrden = "OBS:Callejon->Atras";
      forzarRefrescoOled = true;
      pasoRampa = PASO_RAMPA_EVASION;
      setMotoresObjetivo(-VEL_OBS_MINIMA, -VEL_OBS_MINIMA);
      timerEv = millis();
      estadoEv = EV_RETROCEDER_BLOQUEO;
    }
  }
}

void ejecutarEvitadorObstaculos(long dist) {
  switch (estadoEv) {
    case EV_CONFIRMAR_ANTES_AVANZAR:
      detenerInmediato();
      ultimoPasoOdomMs = millis();
      if (dist <= DIST_OBSTACULO_CM) {
        anguloActualRel = 0;
        nivelAperturaIdx = 0;
        intentoEnNivel = 0;
        primerSignoOrden = elegirPrimerLadoPrometedor();
        timerEv = millis();
        estadoEv = EV_DETENER_ESPERA;
        ultimaOrden = "OBS:Bloq " + String(dist) + "cm";
        forzarRefrescoOled = true;
      } else if (millis() - timerConfirmacionLibre >= TIEMPO_CONFIRMAR_LIBRE_MS) {
        pasoRampa = PASO_RAMPA_EVASION;
        timerDespejeGps = millis();
        ultimaOrden = "OBS:Avanza OK";
        forzarRefrescoOled = true;
        estadoEv = EV_AVANZAR;
      }
      break;

    case EV_AVANZAR:
      if (dist <= DIST_OBSTACULO_CM) {
        detenerInmediato();
        registrarEvento("Obstaculo detectado a " + String(dist) + " cm");
        if (gpsConFix()) registrarCeldaMapa(gps.location.lat(), gps.location.lng(), true);
        anguloActualRel = 0;
        nivelAperturaIdx = 0;
        intentoEnNivel = 0;
        primerSignoOrden = elegirPrimerLadoPrometedor();
        timerEv = millis();
        estadoEv = EV_DETENER_ESPERA;
        ultimaOrden = "OBS:STOP " + String(dist) + "cm";
        forzarRefrescoOled = true;
      } else {
        pasoRampa = PASO_RAMPA_EVASION;
        int velProporcional = VEL_OBS_NORMAL;
        if (dist < DIST_DESPEJADO_CM) {
          velProporcional = (int)map(constrain(dist, DIST_OBSTACULO_CM, DIST_DESPEJADO_CM),
                                     DIST_OBSTACULO_CM, DIST_DESPEJADO_CM,
                                     VEL_OBS_MINIMA, VEL_OBS_NORMAL);
        }
        actualizarOdometriaObstaculos(velActualIzq);
        setMotoresObjetivo(velProporcional, velProporcional);
      }
      break;

    case EV_DETENER_ESPERA:
      detenerInmediato();
      if (millis() - timerEv >= 150) {
        int anguloBase = NIVELES_APERTURA_AGIL[nivelAperturaIdx];
        int signoActual = (intentoEnNivel == 0) ? primerSignoOrden : -primerSignoOrden;
        int objetivoAng = signoActual * anguloBase;

        ultimaOrden = "OBS:Prueba " + String(objetivoAng) + "g";
        forzarRefrescoOled = true;
        comandarGiroHacia(objetivoAng, EV_GIRAR_PRUEBA);
      }
      break;

    case EV_GIRAR_PRUEBA:
      if (millis() - timerEv >= duracionGiroEv) {
        detenerInmediato();
        timerEv = millis();
        estadoEv = EV_ESPERA_LECTURA;
      }
      break;

    case EV_ESPERA_LECTURA:
      if (millis() - timerEv >= 160) {
        if (distanciaActual > DIST_SALIDA_LIBRE_CM) {
          // Centro libre: inicia micro-barrido (-20° y +20°) para verificar las esquinas del chasis
          anguloBaseBarrido = anguloActualRel;
          pasoBarridoIdx = 0;
          ultimaOrden = "OBS:Barrido -20g";
          forzarRefrescoOled = true;
          comandarGiroHacia(anguloBaseBarrido - ANGULO_MICRO_BARRIDO, EV_BARRIDO_GIRAR);
        } else {
          avanzarSiguienteIntentoEvasion();
        }
      }
      break;

    case EV_BARRIDO_GIRAR:
      if (millis() - timerEv >= duracionGiroEv) {
        detenerInmediato();
        timerEv = millis();
        estadoEv = EV_BARRIDO_LECTURA;
      }
      break;

    case EV_BARRIDO_LECTURA:
      if (millis() - timerEv >= 140) {
        if (pasoBarridoIdx == 0) {
          // Evalua borde izquierdo (-20°)
          if (distanciaActual <= DIST_SALIDA_LIBRE_CM) {
            avanzarSiguienteIntentoEvasion();
          } else {
            pasoBarridoIdx = 1;
            ultimaOrden = "OBS:Barrido +20g";
            forzarRefrescoOled = true;
            comandarGiroHacia(anguloBaseBarrido + ANGULO_MICRO_BARRIDO, EV_BARRIDO_GIRAR);
          }
        } else if (pasoBarridoIdx == 1) {
          // Evalua borde derecho (+20°)
          if (distanciaActual <= DIST_SALIDA_LIBRE_CM) {
            avanzarSiguienteIntentoEvasion();
          } else {
            pasoBarridoIdx = 2;
            comandarGiroHacia(anguloBaseBarrido, EV_BARRIDO_GIRAR);
          }
        } else if (pasoBarridoIdx == 2) {
          // Ambas esquinas libres y robot centrado en el nuevo rumbo
          int signoActual = (intentoEnNivel == 0) ? primerSignoOrden : -primerSignoOrden;
          signoPreferidoReciente = signoActual;
          timerPreferenciaGiro = millis();
          timerConfirmacionLibre = millis();
          ultimaOrden = "OBS:Libre " + String(anguloBaseBarrido) + "g";
          forzarRefrescoOled = true;
          estadoEv = EV_CONFIRMAR_ANTES_AVANZAR;
        }
      }
      break;

    case EV_RETROCEDER_BLOQUEO:
      if (millis() - timerEv >= TIEMPO_RETROCESO_BLOQUEO_MS) {
        detenerInmediato();
        signoPreferidoReciente = 0;
        nivelAperturaIdx = 0;
        intentoEnNivel = 0;
        comandarGiroHacia(anguloActualRel + 180, EV_GIRAR_PRUEBA);
      }
      break;
  }
}

// ================= NAVEGACION GPS + EVASION CON TRAMO DE DESPEJE =================
void ejecutarNavegacionAutoGPS(long distObs) {
  if (!destinoActivo) {
    setMotoresObjetivo(0, 0);
    return;
  }

  if (estadoEv != EV_AVANZAR || distObs <= DIST_OBSTACULO_CM) {
    enTramoDespejeGps = true;
    ejecutarEvitadorObstaculos(distObs);
    estadoAuto = 0;
    return;
  }

  if (enTramoDespejeGps) {
    if (millis() - timerDespejeGps < TIEMPO_DESPEJE_GPS_MS) {
      ejecutarEvitadorObstaculos(distObs);
      return;
    } else {
      enTramoDespejeGps = false;
      estadoAuto = 0;
    }
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

  int velCruceroGps = VEL_AVANCE;
  if (distObs < DIST_DESPEJADO_CM) {
    velCruceroGps = (int)map(constrain(distObs, DIST_OBSTACULO_CM, DIST_DESPEJADO_CM),
                             DIST_OBSTACULO_CM, DIST_DESPEJADO_CM,
                             VEL_OBS_MINIMA, VEL_AVANCE);
  }

  switch (estadoAuto) {
    case 0:
      prevLat = currLat;
      prevLng = currLng;
      registrarCeldaMapa(currLat, currLng, false);
      pasoRampa = PASO_RAMPA_NORMAL;
      setMotoresObjetivo(velCruceroGps, velCruceroGps);
      timerAutoGps = millis();
      estadoAuto = 1;
      break;

    case 1: {
      setMotoresObjetivo(velCruceroGps, velCruceroGps);
      double recorrido = calcularDistanciaM(prevLat, prevLng, currLat, currLng);
      if (recorrido >= DIST_MIN_RUMBO_M) {
        registrarCeldaMapa(currLat, currLng, false);
        double rumboActual = calcularRumboDeg(prevLat, prevLng, currLat, currLng);
        double rumboMeta   = calcularRumboDeg(currLat, currLng, targetLat, targetLng);

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
        registrarCeldaMapa(currLat, currLng, true);
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
  cmd.toUpperCase();
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
    actualizarLedEstado();
    ultimaOrden = "MODO:" + modoActual;
    detalleLog = ultimaOrden;

  } else if (cmd == "PWM" || cmd == "VEL") {
    int rawVal = velManualWeb;
    if (doc.containsKey("val"))        rawVal = (int)doc["val"];
    else if (doc.containsKey("pwm"))   rawVal = (int)doc["pwm"];
    else if (doc.containsKey("vel"))   rawVal = (int)doc["vel"];
    else if (doc.containsKey("speed")) rawVal = (int)doc["speed"];

    velManualWeb = ajustarPwmWeb(rawVal);
    sonar = false;
    ultimaOrden = "PWM:" + String(velManualWeb);
    detalleLog = ultimaOrden;

    if (modoActual == "MANUAL") {
      if (velObjetivoIzq > 0 && velObjetivoDer > 0) {
        setMotoresObjetivo(velManualWeb, velManualWeb);
      } else if (velObjetivoIzq < 0 && velObjetivoDer < 0) {
        setMotoresObjetivo(-velManualWeb, -velManualWeb);
      }
    }

  } else if (cmd == "MOV") {
    String dir = doc["dir"] | "STOP";
    dir.toUpperCase();

    if (doc.containsKey("pwm"))        velManualWeb = ajustarPwmWeb((int)doc["pwm"]);
    else if (doc.containsKey("vel"))   velManualWeb = ajustarPwmWeb((int)doc["vel"]);
    else if (doc.containsKey("speed")) velManualWeb = ajustarPwmWeb((int)doc["speed"]);

    int velLineal = velManualWeb;

    // STOP funciona siempre como parado de emergencia y regresa a MANUAL
    bool veniaDeOtroModo = (modoActual != "MANUAL");
    if (veniaDeOtroModo) {
      modoActual = "MANUAL";
      reiniciarMovimiento();
      actualizarLedEstado();
    }

    String nueva = "MOV:" + dir + " " + String(velLineal);
    registrar = veniaDeOtroModo || (nueva != ultimaOrden);
    sonar = false;
    ultimaOrden = nueva;
    detalleLog = nueva;
    ultimaOrdenManualMs = millis();

    if (dir == "ADELANTE")      setMotoresObjetivo(velLineal, velLineal);
    else if (dir == "ATRAS")    setMotoresObjetivo(-velLineal, -velLineal);
    else if (dir == "IZQ")      setMotoresObjetivo(-VEL_GIRO, VEL_GIRO);
    else if (dir == "DER")      setMotoresObjetivo(VEL_GIRO, -VEL_GIRO);
    else                        setMotoresObjetivo(0, 0);

  } else if (cmd == "GIRO") {
    int grados = doc["grados"] | 90;
    modoActual = "MANUAL";
    reiniciarMovimiento();
    actualizarLedEstado();
    pasoRampa = PASO_RAMPA_GIRO;
    unsigned long tGiro = (unsigned long)(abs(grados) / 90.0 * TIEMPO_GIRO_90_MS);
    ultimaOrden = "GIRO:" + String(grados) + "g";
    detalleLog = ultimaOrden;
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
    enTramoDespejeGps = false;
    estadoEv = EV_CONFIRMAR_ANTES_AVANZAR;
    timerConfirmacionLibre = millis();
    anguloActualRel = 0;
    ultimaOrden = "GOTO:" + String(targetLat, 5);
    detalleLog = "GOTO " + String(targetLat, 6) + " " + String(targetLng, 6);

  } else if (cmd == "PING") {
    sonar = false;
    registrar = false;
    ultimaOrden = "PING OK";

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
  setLed(true, false);

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
  digitalWrite(SD_CS, HIGH);

  inicializarModuloLoRa(true);

  Serial.println(String("SD: ") + (sdLista ? "OK" : "ERROR") +
                 " | LoRa: " + (loraListo ? "OK" : "ERROR"));

  actualizarLedEstado();
  if (sdLista && loraListo) {
    iniciarBeep(100, 1);
  } else {
    iniciarBeep(150, 3);
  }

  reiniciarMemoriaObstaculos();
  reiniciarLaberinto();
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

  Serial.println("Robot listo. Todos los modos, micro-barrido +-20g y LoRa SPI WDT activos.");
}

// ================= LOOP =================
void loop() {
  esp_task_wdt_reset();

  actualizarBuzzer();
  verificarSaludLoRa();

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
    digitalWrite(SD_CS, HIGH);
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
      ultimoRxLoraMs = millis();
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

  unsigned long ahora = millis();

  // 3A. Lectura ultrasonica cada 60 ms
  static long lecturas[3] = {999, 999, 999};
  static int idxLectura = 0;

  if (ahora - ultimoUltrasonicoMs >= INTERVALO_ULTRASONICO) {
    ultimoUltrasonicoMs = ahora;

    lecturas[idxLectura] = medirDistanciaCm();
    idxLectura = (idxLectura + 1) % 3;
    distanciaActual = medianaDe3(lecturas[0], lecturas[1], lecturas[2]);

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
  }

  // 3B. Control de locomocion rapido (cada 15 ms)
  if (ahora - ultimoControlMs >= INTERVALO_CONTROL) {
    ultimoControlMs = ahora;

    if (modoActual == "LINEA" || (modoActual == "LINEA_OBS" && estadoRodeo == RL_SIGUIENDO_LINEA)) {
      pasoRampa = PASO_RAMPA_LINEA;
    } else if (modoActual == "CIRCUITO") {
      pasoRampa = PASO_RAMPA_GIRO;
    } else if (modoActual != "OBSTACULO" && modoActual != "AUTO" && modoActual != "LABERINTO" && modoActual != "LINEA_OBS") {
      pasoRampa = PASO_RAMPA_NORMAL;
    }

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
    } else if (modoActual == "LINEA_OBS") {
      ejecutarLineaConObstaculos(distanciaActual);
    } else if (modoActual == "OBSTACULO") {
      ejecutarEvitadorObstaculos(distanciaActual);
    } else if (modoActual == "LABERINTO") {
      ejecutarLaberintoTremaux(distanciaActual);
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
      digitalWrite(LORA_CS, HIGH);
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
      digitalWrite(SD_CS, HIGH);
    }

    bool recibiendoManualActivo = (modoActual == "MANUAL" && (ahora - ultimaOrdenManualMs < 1500));

    if (loraListo && !recibiendoManualActivo) {
      digitalWrite(SD_CS, HIGH);
      StaticJsonDocument<320> txDoc;
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
      LoRa.receive();
      actualizarLedEstado();
    }
  }
}