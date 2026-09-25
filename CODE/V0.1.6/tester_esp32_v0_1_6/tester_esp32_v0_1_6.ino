/*
 * QC LemBot - ESP32-C3 (sin pantalla ni botones)
 * Version: 0.1.6
 * MCU objetivo: ESP32-C3 (SuperMini / C3 Dev). Core arduino-esp32 v3.x,
 * placa "ESP32C3 Dev Module", USB CDC On Boot: Enabled, Flash Mode: DIO.
 *
 * Todo el equipo se maneja desde la web, sin OLED, encoder ni botones:
 *   Red:   LemPDA-XXXX  (XXXX = ultimos 4 digitos hex de la MAC)
 *   Clave: 12345678     (WPA2 exige al menos 8 caracteres; ver AP_PASS)
 *   Web:   http://192.168.4.1 (el telefono la abre solo al conectarse)
 * Pestanas: Corriente (valores, grafico, CSV), Sensores (bateria, T/H, ADS,
 * GPIO, Soil, UART), SDI-12 (escaneo, aI!, aC!, aM!, cambio de ID), Tests
 * (Sense-QC, Weather-QC, descarga) y Equipo (configuracion y sistema).
 * Es la misma GUI de la 0.2.2 (S3). La pagina va en index_html.h, en la misma
 * carpeta que este .ino.
 *
 * Pines (cambia el I2C respecto de la 0.1.5):
 *   GP0=SDA GP1=SCL (INA228, ADS1115, BME280, SHT30) - antes era GP8/GP9;
 *        GP0/GP1 eran el encoder, no son strapping y quedan juntos en la placa
 *   GP8=LED RGB WS2812 onboard (estado del equipo; GP8 es strapping, el LED
 *        no lo afecta)
 *   GP6=Pulse1 / SDI-12 RX   GP7=Pulse2 / SDI-12 TX / SHT10 data
 *   GP10=SHT10 clk   GP20=UART monitor RX
 *   Libres: GP2 (strapping), GP3, GP4, GP5, GP9 (strapping/BOOT), GP21
 *   Bateria: ADS1115 A3 (divisor 2:1)
 * LED de estado: rojo fijo = INA228 no responde; rojo fuerte = sobrecorriente;
 * rojo parpadeando = grabando corriente; azul = test, descarga o SDI-12 en
 * curso; verde tenue = listo. Poner USE_RGB_LED en 0 si la placa no tiene
 * WS2812 en GP8 (por ejemplo, un LED azul simple).
 * En el C3 el bus SDI-12 comparte GP6/GP7 con Sense-QC y Weather-QC: la web
 * no deja usar ambos a la vez.
 *
 * Basado en 0.1.5 (misma logica de medicion y tests, con sus arreglos).
 * Sin INA228 la web sigue funcionando (error visible y gestor SDI-12).
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA228.h>
#include <Adafruit_ADS1X15.h>
#include <EEPROM.h>
#include <Adafruit_BME280.h>
#include <Adafruit_SHT31.h>
#include <SHT1x-ESP.h>  // Libreria: "SHT1x" by Practical Maker
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>

// ============================================
// PIN DEFINITIONS (ESP32-C3)
// ============================================
#define SDA_PIN     0    // I2C movido desde GP8/GP9 para dejar GP8 al LED RGB
#define SCL_PIN     1
#define RGB_LED_PIN 8    // WS2812 onboard de los C3 con LED RGB
#define USE_RGB_LED 1    // 0 si la placa no tiene WS2812 en GP8
#define PULSE1_PIN  6
#define PULSE2_PIN  7
#define UART_RX_PIN 20

#define SHT10_DATA_PIN  7
#define SHT10_CLOCK_PIN 10

// ============================================
// SDI-12 FRONT-END DE HARDWARE (2 pines)
// ============================================
// TX = GPIO7 -> SN74LVC1G3157 -> SN74LVC1G240 (inversor) -> bus SDI-12
// RX = GPIO6 <- SN74AHC1G14 (inversor Schmitt) <- bus SDI-12
// El bus SDI-12 es logica invertida (marca/idle ~0V, espacio ~5V). Con una
// inversion en TX y una en RX, ambas cancelan la del bus => el MCU ve un UART
// estandar NO invertido: reposo/marca = ALTO, start/espacio = BAJO.
#define SDI12_TX_PIN    7   // hacia el bus (compartido con PULSE2 y SHT10 data)
#define SDI12_RX_PIN    6   // desde el bus (compartido con PULSE1)

// Niveles GPIO (cambiar a la inversa solo si el banco lo exige)
#define SDI12_TX_MARK   HIGH   // reposo / bit 1 / stop
#define SDI12_TX_SPACE  LOW    // start / bit 0 / break
#define SDI12_RX_MARK   HIGH   // leer ALTO en RX = marca (1)

// Temporizacion: 1200 baud => 833.33 us por bit
#define SDI12_BIT_US    833
#define SDI12_HALF_US   417

// ============================================
// EEPROM CONFIGURATION
// ============================================
#define EEPROM_SIZE         128
#define EEPROM_MAGIC        0
#define ADDR_START_THRESH   4
#define ADDR_END_THRESH     8
#define ADDR_SAMPLE_PERIOD  12
#define ADDR_MAX_REC_TIME   16
#define ADDR_OC_THRESHOLD   20
#define ADDR_MAX_SAMPLES    24
#define ADDR_BAT_CAPACITY   28
#define ADDR_SA_VOLT_REF    32
#define ADDR_SA_VOLT_TOL    36
#define ADDR_SA_CURR_MIN    40
#define ADDR_SA_CURR_MAX    44
#define ADDR_DISCH_CURR_LIM 48

// ============================================
// CONFIGURACION DE MEDICION (defaults)
// ============================================
float  CURRENT_START_THRESHOLD  = 0.8f;
float  CURRENT_END_THRESHOLD    = 9.0f;
unsigned long SAMPLE_PERIOD     = 1000;
unsigned long MAX_RECORD_TIME   = 120000;
float  OC_THRESHOLD             = 10.0f;
int    MAX_SAMPLES_CONFIG       = 120;
float  BAT_CAPACITY_MAH         = 3500.0f;
float  SA_VOLT_REF              = 1.12f;
float  SA_VOLT_TOL              = 0.05f;   // 5%
float  SA_CURR_MIN              = 0.5f;
float  SA_CURR_MAX              = 50.0f;
float  DISCH_CURR_LIMIT         = 200.0f;

const float  DEF_START_THRESH  = 0.8f;
const float  DEF_END_THRESH    = 9.0f;
const unsigned long DEF_SAMPLE  = 1000;
const unsigned long DEF_MAX_TIME= 120000;
const float  DEF_OC_THRESH     = 10.0f;
const int    DEF_MAX_SAMPLES    = 120;
const float  DEF_BAT_CAPACITY  = 3500.0f;
const float  DEF_SA_VOLT_REF   = 1.12f;
const float  DEF_SA_VOLT_TOL   = 0.05f;
const float  DEF_SA_CURR_MIN   = 0.5f;
const float  DEF_SA_CURR_MAX   = 50.0f;
const float  DEF_DISCH_CURR    = 200.0f;

// ============================================
// HARDWARE OBJECTS
// ============================================
Adafruit_INA228 ina228;
Adafruit_ADS1115 ads;
Adafruit_BME280 bme;
Adafruit_SHT31  sht30;
SHT1x sht10(SHT10_DATA_PIN, SHT10_CLOCK_PIN);

// ============================================
// BATTERY
// ============================================
float batteryVoltage   = 0.0f;
int   batteryPercent   = 0;

// ============================================
// CURRENT MONITORING
// ============================================
float currentVoltage    = 0.0f;
float currentCurrent    = 0.0f;
float currentMAh        = 0.0f;
float max_current_mA    = 0.0f;
float min_current_mA    = 0.0f;   // corriente minima > 0 observada (capta uA)

// Estimacion de uso (independiente del registro del grafico)
float estMAh            = 0.0f;   // carga integrada por software desde que se entro
unsigned long estStartTime = 0;   // inicio de la medicion de la vista
unsigned long estLastTime  = 0;   // ultimo timestamp para integrar

int   MAX_SAMPLES       = 120;
float*         recordedCurrents = nullptr;
unsigned long* recordedTimes    = nullptr;
float*         recordedMAh      = nullptr;

int           sampleCount       = 0;
bool          isRecording       = false;
bool          recordingComplete = false;
unsigned long recordStartTime   = 0;
unsigned long lastSampleTime    = 0;

bool          endingRecording   = false;
unsigned long endRecordingTime  = 0;
const unsigned long END_DELAY   = 3000;

// ============================================
// SENSE-QC TESTER
// ============================================
enum TesterStateSA {
  SA_IDLE,
  SA_TESTING,
  SA_RESULT
};
TesterStateSA testerStateSA = SA_IDLE;
bool  sa_currentOk  = false;
bool  sa_pulse1Ok   = false;
bool  sa_pulse2Ok   = false;
bool  sa_a0Ok       = false;
bool  sa_a1Ok       = false;
bool  sa_passed     = false;
float sa_current    = 0.0f;
float sa_a0v        = 0.0f;
float sa_a1v        = 0.0f;
String sa_failReason = "";
unsigned long sa_testStart = 0;
bool  sa_pulseReset = false;     // reiniciar los pulsos vistos al empezar cada test
const unsigned long SA_TEST_DURATION = 5000; // 5s de observacion pulsos

// ============================================
// WEATHER-QC TESTER
// ============================================
enum TesterStateWX {
  WX_IDLE,
  WX_TESTING,
  WX_RESULT
};
TesterStateWX testerStateWX = WX_IDLE;
bool  wx_currentOk  = false;
bool  wx_sht10Ok    = false;
bool  wx_sht30Ok    = false;
bool  wx_passed     = false;
float wx_current    = 0.0f;
float wx_bme_temp   = 0.0f;
float wx_bme_hum    = 0.0f;
float wx_sht_temp   = 0.0f;
float wx_sht_hum    = 0.0f;
String wx_failReason = "";
const float WX_TEMP_TOL = 3.0f;   // ±3°C diferencia max
const float WX_HUM_TOL  = 10.0f;  // ±10% HR diferencia max

// ============================================
// SOIL-QC (DFM Sensor via Serial)
// ============================================
// Protocolo DFM: paquete de datos con 12 humedad + 13 temp
float soil_hum[12]  = {0};
float soil_temp[13] = {0};
bool  soil_dataReady = false;
unsigned long lastSoilParse = 0;
String soilBuffer = "";

// ============================================
// UART MONITOR
// ============================================
#define UART_BUF_LINES 5
#define UART_LINE_LEN  22
char uartLines[UART_BUF_LINES][UART_LINE_LEN] = {};
int  uartLineIndex = 0;
bool uartInit      = false;

// ============================================
// SDI-12 (bit-bang por GP7/GP6, compartido con Pulse/SHT10)
// ============================================
#define SDI12_NUM_ADDR   62   // direcciones validas: 0-9, a-z, A-Z
bool   sdi12Active      = false;   // pines tomados por el bus SDI-12

// ============================================
// BATTERY DISCHARGE TEST
// ============================================
enum DischState { DISCH_IDLE, DISCH_RUNNING, DISCH_DONE };
DischState dischState       = DISCH_IDLE;
float  disch_mAh            = 0.0f;
float  disch_mWh            = 0.0f;
float  disch_voltage        = 0.0f;
float  disch_current        = 0.0f;
float  disch_peakCurrent    = 0.0f;
unsigned long disch_startTime = 0;
unsigned long disch_duration  = 0;
unsigned long disch_lastSample= 0;
bool   disch_healthy        = false;

// ============================================
// PORTAL WEB (igual que la 0.2.2 del S3)
// ============================================
#define AP_PASS          "12345678"   // clave de la red (WPA2: minimo 8 caracteres)
#define WEB_MAX_SENSORS  16           // sensores SDI-12 que guarda la web
#define WEB_MAX_WAIT_S   30           // tope de espera de una medicion SDI-12 [s]
const IPAddress AP_IP(192, 168, 4, 1);
const IPAddress AP_MASK(255, 255, 255, 0);
WebServer webServer(80);
DNSServer webDns;
String    apName = "";

// Sensores I2C detectados al arrancar (la web y los tests no leen los ausentes:
// el ADS1115 ausente dejaria colgada su lectura)
bool inaFailed    = false;
bool adsPresent   = false;
bool bmePresent   = false;
bool sht30Present = false;

// Lecturas de sensores para la web: solo con la pestana Sensores abierta
const unsigned long WEB_ACTIVE_MS = 4000UL;
unsigned long webSenHit = 0, webTAds = 0, webTEnv = 0, webTS10 = 0, envAt = 0;
float envBmeT = NAN, envBmeH = NAN, envBmeP = NAN;
float envS30T = NAN, envS30H = NAN, envS10T = NAN, envS10H = NAN;
float adsV[4] = {NAN, NAN, NAN, NAN};
bool  gpioP1 = false, gpioP2 = false, gpioUrx = false;

// Tareas SDI-12 de la web (por pasos, la web sigue respondiendo mientras un
// sensor mide). Los tipos van antes de la primera funcion: el IDE inserta ahi
// los prototipos automaticos y necesitan conocerlos.
struct WMeas {
  int n = -1;            // -1 = sin medir, -2 = el sensor no soporta el comando
  int t = 0;             // segundos de espera que pidio el sensor
  uint32_t at = 0;       // segundo de uptime en que se midio
  String raw;            // valores concatenados: "+1.23-4.5+6"
};
struct WSensor {
  char   addr = 0;
  String info;           // respuesta de aI! sin la direccion
  WMeas  c, m;           // resultados de aC! y aM!
};
enum WJobType : uint8_t { WJOB_NONE, WJOB_SCAN, WJOB_INFO, WJOB_MEASURE, WJOB_CHID };
struct WJob {
  WJobType type = WJOB_NONE;
  uint8_t phase = 0;
  int  i = 0;            // escaneo: direccion en curso
  int  k = 0;            // escaneo: sensor en curso / medicion: indice de aD
  char target = 0;       // sensor sobre el que se trabaja
  char mode = 'C';       // 'C' o 'M'
  bool both = false;     // medir aC! y despues aM!
  char newAddr = 0;
  int  n = -1, t = 0;
  unsigned long waitStart = 0, waitMs = 0;
  String raw;
};
WSensor wSensors[WEB_MAX_SENSORS];
int     wNSensors = 0;
WJob    wJob;
char    wScanFound[SDI12_NUM_ADDR];
int     wScanCount = 0;
String  wLastMsg = "";
bool    wLastErr = false;

// ============================================
// LOGICA (copiada de 0.1.5)
// ============================================
void saveConfig() {
  EEPROM.write(EEPROM_MAGIC, 0xBB);
  EEPROM.put(ADDR_START_THRESH,   CURRENT_START_THRESHOLD);
  EEPROM.put(ADDR_END_THRESH,     CURRENT_END_THRESHOLD);
  EEPROM.put(ADDR_SAMPLE_PERIOD,  SAMPLE_PERIOD);
  EEPROM.put(ADDR_MAX_REC_TIME,   MAX_RECORD_TIME);
  EEPROM.put(ADDR_OC_THRESHOLD,   OC_THRESHOLD);
  EEPROM.put(ADDR_MAX_SAMPLES,    MAX_SAMPLES_CONFIG);
  EEPROM.put(ADDR_BAT_CAPACITY,   BAT_CAPACITY_MAH);
  EEPROM.put(ADDR_SA_VOLT_REF,    SA_VOLT_REF);
  EEPROM.put(ADDR_SA_VOLT_TOL,    SA_VOLT_TOL);
  EEPROM.put(ADDR_SA_CURR_MIN,    SA_CURR_MIN);
  EEPROM.put(ADDR_SA_CURR_MAX,    SA_CURR_MAX);
  EEPROM.put(ADDR_DISCH_CURR_LIM, DISCH_CURR_LIMIT);
  EEPROM.commit();
}

void loadConfig() {
  if (EEPROM.read(EEPROM_MAGIC) == 0xBB) {
    EEPROM.get(ADDR_START_THRESH,   CURRENT_START_THRESHOLD);
    EEPROM.get(ADDR_END_THRESH,     CURRENT_END_THRESHOLD);
    EEPROM.get(ADDR_SAMPLE_PERIOD,  SAMPLE_PERIOD);
    EEPROM.get(ADDR_MAX_REC_TIME,   MAX_RECORD_TIME);
    EEPROM.get(ADDR_OC_THRESHOLD,   OC_THRESHOLD);
    EEPROM.get(ADDR_MAX_SAMPLES,    MAX_SAMPLES_CONFIG);
    EEPROM.get(ADDR_BAT_CAPACITY,   BAT_CAPACITY_MAH);
    EEPROM.get(ADDR_SA_VOLT_REF,    SA_VOLT_REF);
    EEPROM.get(ADDR_SA_VOLT_TOL,    SA_VOLT_TOL);
    EEPROM.get(ADDR_SA_CURR_MIN,    SA_CURR_MIN);
    EEPROM.get(ADDR_SA_CURR_MAX,    SA_CURR_MAX);
    EEPROM.get(ADDR_DISCH_CURR_LIM, DISCH_CURR_LIMIT);
  } else {
    restoreDefaults();
  }
}

void restoreDefaults() {
  CURRENT_START_THRESHOLD = DEF_START_THRESH;
  CURRENT_END_THRESHOLD   = DEF_END_THRESH;
  SAMPLE_PERIOD           = DEF_SAMPLE;
  MAX_RECORD_TIME         = DEF_MAX_TIME;
  OC_THRESHOLD            = DEF_OC_THRESH;
  MAX_SAMPLES_CONFIG      = DEF_MAX_SAMPLES;
  BAT_CAPACITY_MAH        = DEF_BAT_CAPACITY;
  SA_VOLT_REF             = DEF_SA_VOLT_REF;
  SA_VOLT_TOL             = DEF_SA_VOLT_TOL;
  SA_CURR_MIN             = DEF_SA_CURR_MIN;
  SA_CURR_MAX             = DEF_SA_CURR_MAX;
  DISCH_CURR_LIMIT        = DEF_DISCH_CURR;
  reallocArrays();
  saveConfig();
}

void reallocArrays() {
  if (recordedCurrents) { delete[] recordedCurrents; delete[] recordedTimes; delete[] recordedMAh; }
  MAX_SAMPLES_CONFIG = constrain(MAX_SAMPLES_CONFIG, 10, 2000);   // evita agotar la RAM
  MAX_SAMPLES = MAX_SAMPLES_CONFIG;
  recordedCurrents = new float[MAX_SAMPLES];
  recordedTimes    = new unsigned long[MAX_SAMPLES];
  recordedMAh      = new float[MAX_SAMPLES];
  for (int i = 0; i < MAX_SAMPLES; i++) { recordedCurrents[i] = 0; recordedTimes[i] = 0; recordedMAh[i] = 0; }
  // Los arreglos nuevos estan vacios: reiniciar el conteo evita leer fuera de ellos
  sampleCount       = 0;
  isRecording       = false;
  recordingComplete = false;
  endingRecording   = false;
}

void updateCurrentReadings() {
  currentVoltage = ina228.getBusVoltage_V();
  currentCurrent = ina228.getCurrent_mA();
  if (currentCurrent > max_current_mA) max_current_mA = currentCurrent;

  // Minima corriente > 0 (captura valores muy bajos, ej. 0.012 mA)
  if (currentCurrent > 0.0f) {
    if (min_current_mA <= 0.0f || currentCurrent < min_current_mA) min_current_mA = currentCurrent;
  }

  float charge_C = ina228.readCharge();
  currentMAh = charge_C / 3.6f;

  // Integracion independiente para el estimado de uso (mAh = mA * h)
  unsigned long now = millis();
  if (estLastTime != 0 && currentCurrent > 0.0f) {
    float dt_h = (now - estLastTime) / 3600000.0f;
    estMAh += currentCurrent * dt_h;
  }
  estLastTime = now;
}

// Estadisticas de las muestras grabadas: mediana y promedio
void computeCurrentStats(float& median, float& avg, bool& valid) {
  valid = sampleCount > 0;
  if (!valid) { median = 0; avg = 0; return; }

  float sum  = 0;
  float bmin = recordedCurrents[0], bmax = recordedCurrents[0];
  for (int i = 0; i < sampleCount; i++) {
    float v = recordedCurrents[i];
    sum += v;
    if (v < bmin) bmin = v;
    if (v > bmax) bmax = v;
  }
  avg = sum / sampleCount;

  // Mediana via histograma (sin ordenar ni buffers extra)
  const int NB = 40;
  int hist[NB];
  for (int b = 0; b < NB; b++) hist[b] = 0;
  float range = bmax - bmin;
  if (range < 0.0001f) range = 0.0001f;
  float bs = range / NB;
  for (int i = 0; i < sampleCount; i++) {
    int b = constrain((int)((recordedCurrents[i] - bmin) / bs), 0, NB - 1);
    hist[b]++;
  }
  int half = (sampleCount + 1) / 2, cum = 0;
  median = bmax;
  for (int b = 0; b < NB; b++) {
    cum += hist[b];
    if (cum >= half) { median = bmin + (b + 0.5f) * bs; break; }
  }
}

void handleRecording() {
  unsigned long now = millis();
  if (!isRecording && !recordingComplete && currentCurrent > CURRENT_START_THRESHOLD) startRecording();

  if (isRecording) {
    if (!endingRecording) {
      if (currentCurrent < CURRENT_END_THRESHOLD ||
          (now - recordStartTime) >= MAX_RECORD_TIME ||
          sampleCount >= MAX_SAMPLES) {
        endingRecording   = true;
        endRecordingTime  = now;
      }
    }
    if (endingRecording && now - endRecordingTime >= END_DELAY) {
      stopRecording(); endingRecording = false; return;
    }
    if (now - lastSampleTime >= SAMPLE_PERIOD) {
      recordSample(now); lastSampleTime = now;
    }
  }
}

void startRecording() {
  isRecording       = true;
  recordStartTime   = millis();
  lastSampleTime    = recordStartTime;
  sampleCount       = 0;
  endingRecording   = false;
  ina228.resetAccumulators();
}

void stopRecording() {
  isRecording       = false;
  recordingComplete = true;
}

void recordSample(unsigned long ts) {
  if (sampleCount < MAX_SAMPLES) {
    recordedCurrents[sampleCount] = currentCurrent;
    recordedTimes[sampleCount]    = ts - recordStartTime;
    float charge_C = ina228.readCharge();
    recordedMAh[sampleCount]      = charge_C / 3.6f;
    sampleCount++;
  }
}

void resetRecording() {
  sampleCount       = 0;
  isRecording       = false;
  recordingComplete = false;
  endingRecording   = false;
  max_current_mA    = 0;
  min_current_mA    = 0;
  currentMAh        = 0;
  estMAh            = 0;
  estStartTime      = millis();
  estLastTime       = 0;
  // La descarga de bateria usa el acumulador de carga: no borrarlo
  if (dischState != DISCH_RUNNING) ina228.resetAccumulators();
}

// Porcentaje continuo 0-100% por interpolacion de la curva de una celda LiPo
int batPercentFromV(float v) {
  static const float vv[] = {3.00f, 3.30f, 3.50f, 3.65f, 3.80f, 3.95f, 4.10f, 4.20f};
  static const float pp[] = {0.0f,  5.0f,  15.0f, 35.0f, 55.0f, 80.0f, 95.0f, 100.0f};
  const int N = 8;
  if (v <= vv[0])     return 0;
  if (v >= vv[N - 1]) return 100;
  for (int i = 1; i < N; i++) {
    if (v < vv[i]) {
      float t = (v - vv[i - 1]) / (vv[i] - vv[i - 1]);
      float p = pp[i - 1] + t * (pp[i] - pp[i - 1]);
      return (int)(p + 0.5f);
    }
  }
  return 100;
}

void updateSA() {
  if (testerStateSA != SA_TESTING) return;

  unsigned long elapsed = millis() - sa_testStart;

  // Leer valores actuales
  sa_current = ina228.getCurrent_mA();
  sa_a0v     = adsPresent ? ads.readADC_SingleEnded(0) * 0.000125f : 0.0f;   // ADS ausente: no leer
  sa_a1v     = adsPresent ? ads.readADC_SingleEnded(1) * 0.000125f : 0.0f;

  bool pulse1 = digitalRead(PULSE1_PIN);
  bool pulse2 = digitalRead(PULSE2_PIN);

  // Acumular deteccion de pulsos durante SA_TEST_DURATION
  static bool p1_high_seen = false, p1_low_seen = false;
  static bool p2_high_seen = false, p2_low_seen = false;

  // Antes se reiniciaba solo si elapsed == 0 exacto, y a veces se arrastraban
  // los pulsos de la placa anterior (PASS falso en P1/P2)
  if (sa_pulseReset) { p1_high_seen = p1_low_seen = p2_high_seen = p2_low_seen = false; sa_pulseReset = false; }

  if (pulse1) p1_high_seen = true; else p1_low_seen = true;
  if (pulse2) p2_high_seen = true; else p2_low_seen = true;

  if (elapsed >= SA_TEST_DURATION) {
    // Evaluar
    float vRef  = SA_VOLT_REF;
    float vTol  = SA_VOLT_TOL;
    sa_a0Ok     = (sa_a0v >= vRef * (1.0f - vTol)) && (sa_a0v <= vRef * (1.0f + vTol));
    sa_a1Ok     = (sa_a1v >= vRef * (1.0f - vTol)) && (sa_a1v <= vRef * (1.0f + vTol));
    sa_currentOk= (sa_current >= SA_CURR_MIN) && (sa_current <= SA_CURR_MAX);
    sa_pulse1Ok = p1_high_seen && p1_low_seen;
    sa_pulse2Ok = p2_high_seen && p2_low_seen;

    sa_passed   = sa_a0Ok && sa_a1Ok && sa_currentOk && sa_pulse1Ok && sa_pulse2Ok;

    if (!sa_passed) {
      sa_failReason = "";
      if (!sa_currentOk) sa_failReason += "I ";
      if (!sa_a0Ok)      sa_failReason += "A0 ";
      if (!sa_a1Ok)      sa_failReason += "A1 ";
      if (!sa_pulse1Ok)  sa_failReason += "P1 ";
      if (!sa_pulse2Ok)  sa_failReason += "P2 ";
    }

    testerStateSA = SA_RESULT;
  }
}

// El SHT1x-ESP no devuelve NaN si el SHT10 no responde: convierte el NAN del
// dato crudo en 0, o sea T = -40.1 C y H = -2.7 %. Se descarta lo fuera de rango.
bool sht10Valid(float t, float h) {
  return !isnan(t) && !isnan(h) && t > -39.0f && t < 124.0f && h > -1.0f && h < 105.0f;
}

void updateWX() {
  if (testerStateWX != WX_TESTING) return;

  unsigned long elapsed = millis() - sa_testStart;

  wx_current  = ina228.getCurrent_mA();
  wx_bme_temp = bme.readTemperature();
  wx_bme_hum  = bme.readHumidity();

  // Intentar SHT10 primero, luego SHT30
  wx_sht_temp = sht10.readTemperatureC();
  wx_sht_hum  = sht10.readHumidity();

  bool sht10valid = sht10Valid(wx_sht_temp, wx_sht_hum);   // si no, se usa el SHT30

  if (!sht10valid) {
    wx_sht_temp = sht30.readTemperature();
    wx_sht_hum  = sht30.readHumidity();
  }

  bool shtValid = !isnan(wx_sht_temp) && !isnan(wx_sht_hum);

  if (elapsed >= 4000) { // 4s para estabilizar
    wx_currentOk  = (wx_current >= SA_CURR_MIN) && (wx_current <= SA_CURR_MAX);
    wx_sht10Ok    = shtValid; // usamos el sensor que respondio

    bool tempOk = false, humOk = false;
    if (shtValid && !isnan(wx_bme_temp)) {
      tempOk = fabs(wx_sht_temp - wx_bme_temp) <= WX_TEMP_TOL;
      humOk  = fabs(wx_sht_hum  - wx_bme_hum)  <= WX_HUM_TOL;
    }
    wx_sht30Ok = tempOk && humOk;
    wx_passed  = wx_currentOk && wx_sht10Ok && wx_sht30Ok;

    if (!wx_passed) {
      wx_failReason = "";
      if (!wx_currentOk) wx_failReason += "I ";
      if (!wx_sht10Ok)   wx_failReason += "SHT-NA ";
      if (!tempOk)       wx_failReason += "T ";
      if (!humOk)        wx_failReason += "H ";
    }
    testerStateWX = WX_RESULT;
  }
}

void parseSoilData(String& line) {
  line.trim();
  if (line.length() < 5) return;

  // Intentar parsear CSV simple: primero 12 son humedad, siguientes 13 son temp
  // Ejemplo: "45.2,46.1,44.8,...,25.1,25.2,..."
  int start = 0;
  bool onTemp = false;
  int hCount = 0, tCount = 0;

  for (int i = 0; i <= (int)line.length(); i++) {
    if (i == (int)line.length() || line[i] == ',' || line[i] == ';') {
      String token = line.substring(start, i);
      token.trim();
      float val = token.toFloat();

      if (!onTemp && hCount < 12) {
        soil_hum[hCount++] = val;
        if (hCount == 12) onTemp = true;
      } else if (onTemp && tCount < 13) {
        soil_temp[tCount++] = val;
      }
      start = i + 1;
    }
    // ; tambien separa bloque humedad/temperatura
    if (i < (int)line.length() && line[i] == ';') onTemp = true;
  }

  if (hCount >= 6 || tCount >= 6) {
    soil_dataReady = true;
    lastSoilParse  = millis();
  }
}

void updateUART() {
  if (!uartInit) {
    Serial1.begin(115200, SERIAL_8N1, UART_RX_PIN, -1);
    uartInit = true;
  }

  while (Serial1.available()) {
    char c = (char)Serial1.read();
    if (c == '\n' || c == '\r') {
      // Nueva linea: mover buffer
      if (uartLineIndex < UART_BUF_LINES - 1) uartLineIndex++;
      else {
        // Scroll: mover lineas hacia arriba
        for (int i = 0; i < UART_BUF_LINES - 1; i++) {
          memcpy(uartLines[i], uartLines[i+1], UART_LINE_LEN);
        }
      }
      memset(uartLines[uartLineIndex], 0, UART_LINE_LEN);
    } else {
      int len = strlen(uartLines[uartLineIndex]);
      if (len < UART_LINE_LEN - 1) {
        uartLines[uartLineIndex][len] = c;
        uartLines[uartLineIndex][len+1] = 0;
      }
    }
  }
}

// Devuelve la direccion SDI-12 correspondiente al indice 0..61
char sdi12AddrAt(int i) {
  if (i < 10)  return '0' + i;          // 0-9
  if (i < 36)  return 'a' + (i - 10);   // a-z
  return 'A' + (i - 36);                // A-Z
}

// Indice 0..61 de una direccion (o -1 si invalida)
int sdi12IndexOf(char addr) {
  if (addr >= '0' && addr <= '9') return addr - '0';
  if (addr >= 'a' && addr <= 'z') return 10 + (addr - 'a');
  if (addr >= 'A' && addr <= 'Z') return 36 + (addr - 'A');
  return -1;
}

// Inicia la interfaz SDI-12: TX como salida en reposo (marca), RX como entrada.
// Mantener TX en marca (alto) deja el bus libre para que el sensor responda.
void sdi12Begin() {
  pinMode(SDI12_TX_PIN, OUTPUT);
  digitalWrite(SDI12_TX_PIN, SDI12_TX_MARK);   // reposo
  pinMode(SDI12_RX_PIN, INPUT);
  sdi12Active = true;
}

// Libera los pines (vuelven a entrada) para Pulse1/Pulse2/SHT10.
void exitSDI12() {
  sdi12Active = false;
  pinMode(SDI12_TX_PIN, INPUT);
  pinMode(SDI12_RX_PIN, INPUT);
}

// Envia un caracter por TX en formato SDI-12: 1 start, 7 datos LSB primero,
// 1 paridad par, 1 stop. 1200 baud. Interrupciones off para timing preciso.
void sdi12SendChar(uint8_t ch) {
  noInterrupts();
  digitalWrite(SDI12_TX_PIN, SDI12_TX_SPACE);          // start bit
  delayMicroseconds(SDI12_BIT_US);
  uint8_t parity = 0;
  for (int i = 0; i < 7; i++) {
    uint8_t bit = (ch >> i) & 0x01;
    parity ^= bit;
    digitalWrite(SDI12_TX_PIN, bit ? SDI12_TX_MARK : SDI12_TX_SPACE);
    delayMicroseconds(SDI12_BIT_US);
  }
  digitalWrite(SDI12_TX_PIN, parity ? SDI12_TX_MARK : SDI12_TX_SPACE);  // paridad par
  delayMicroseconds(SDI12_BIT_US);
  digitalWrite(SDI12_TX_PIN, SDI12_TX_MARK);           // stop bit
  delayMicroseconds(SDI12_BIT_US);
  interrupts();
}

// Secuencia SDI-12: break (espacio >=12ms) + marca (>=8.33ms) + comando.
void sdi12SendBreakAndCommand(const String& cmd) {
  pinMode(SDI12_TX_PIN, OUTPUT);
  digitalWrite(SDI12_TX_PIN, SDI12_TX_SPACE);   // break
  delay(13);
  digitalWrite(SDI12_TX_PIN, SDI12_TX_MARK);    // marca de arranque
  delay(9);
  for (int i = 0; i < (int)cmd.length(); i++) sdi12SendChar((uint8_t)cmd.charAt(i));
  digitalWrite(SDI12_TX_PIN, SDI12_TX_MARK);    // reposo -> libera el bus
}

// Lee un caracter de RX (7 bits). Devuelve -1 si no llega start antes del timeout.
int sdi12ReadChar(unsigned long timeoutMs) {
  unsigned long t0 = millis();
  while (digitalRead(SDI12_RX_PIN) == SDI12_RX_MARK) {   // esperar start bit
    if (millis() - t0 >= timeoutMs) return -1;
  }
  noInterrupts();
  delayMicroseconds(SDI12_BIT_US + SDI12_HALF_US);       // centro del bit 0
  uint8_t c = 0;
  for (int i = 0; i < 7; i++) {
    if (digitalRead(SDI12_RX_PIN) == SDI12_RX_MARK) c |= (1 << i);  // marca = 1
    delayMicroseconds(SDI12_BIT_US);
  }
  delayMicroseconds(SDI12_BIT_US);   // saltar paridad y terminar en el stop
  interrupts();
  return c & 0x7F;
}

// Envia un comando SDI-12 y devuelve la respuesta (sin CR/LF), o "" si timeout.
String sdi12Command(const String& cmd, uint16_t waitMs) {
  sdi12SendBreakAndCommand(cmd);
  String resp = "";
  unsigned long t0 = millis();
  while (millis() - t0 < waitMs) {
    unsigned long remaining = waitMs - (millis() - t0);
    int ch = sdi12ReadChar(remaining);
    if (ch < 0) break;                 // timeout
    if (ch == '\n') break;             // fin de respuesta
    if (ch == '\r') continue;          // ignorar CR
    if (ch >= ' ' && ch < 127) resp += (char)ch;
    if (resp.length() >= 80) break;    // proteccion (respuesta SDI-12 max ~75)
  }
  resp.trim();
  return resp;
}

// Comprueba si hay un sensor en una direccion (comando "a!")
bool sdi12Probe(char addr) {
  String cmd = String(addr) + "!";
  String r = sdi12Command(cmd, 120);
  return (r.length() > 0 && r.charAt(0) == addr);
}

void updateDischarge() {
  if (dischState != DISCH_RUNNING) return;

  unsigned long now     = millis();
  disch_voltage         = ina228.getBusVoltage_V();
  disch_current         = ina228.getCurrent_mA();
  disch_duration        = now - disch_startTime;

  if (disch_current > disch_peakCurrent) disch_peakCurrent = disch_current;

  if (now - disch_lastSample >= 1000) {
    float charge_C = ina228.readCharge();
    disch_mAh      = charge_C / 3.6f;
    disch_mWh      = disch_mAh * disch_voltage;
    disch_lastSample = now;
  }

  // Parar si voltaje muy bajo (3.0V) o corriente supera limite
  if (disch_voltage < 3.0f || disch_current > DISCH_CURR_LIMIT) {
    disch_healthy = (disch_voltage >= 3.0f);
    dischState    = DISCH_DONE;
  }
}

// ============================================
// LED DE ESTADO Y BATERIA
// ============================================
// LED RGB de estado (el equipo no tiene pantalla). Colores tenues y solo se
// reescribe cuando cambia.
void rgbSet(uint8_t r, uint8_t g, uint8_t b) {
#if USE_RGB_LED
  static int lr = -1, lg = -1, lb = -1;
  if (r == lr && g == lg && b == lb) return;
  rgbLedWrite(RGB_LED_PIN, r, g, b);
  lr = r; lg = g; lb = b;
#else
  (void)r; (void)g; (void)b;
#endif
}

void updateStatusLED() {
  static unsigned long lastBlink = 0;
  static bool blinkOn = false;
  if (millis() - lastBlink > 400) { blinkOn = !blinkOn; lastBlink = millis(); }
  if (inaFailed)                                   rgbSet(40, 0, 0);
  else if (dischState == DISCH_RUNNING || testerStateSA == SA_TESTING ||
           testerStateWX == WX_TESTING || wJob.type != WJOB_NONE) rgbSet(0, 0, 30);
  else if (currentCurrent > OC_THRESHOLD)          rgbSet(60, 0, 0);
  else if (isRecording)                            rgbSet(blinkOn ? 25 : 0, 0, 0);
  else                                             rgbSet(0, 6, 0);
}

void updateBattery() {
  if (!adsPresent) { batteryVoltage = NAN; batteryPercent = 0; return; }   // sin ADS no hay dato
  int16_t adc3  = ads.readADC_SingleEnded(3);
  batteryVoltage = (adc3 * 0.000125f) * 2.0f;
  batteryPercent = batPercentFromV(batteryVoltage);
}

// UART monitor (GP20) y sonda DFM (Serial): se leen siempre para la web
void updateSerialInputs() {
  updateUART();
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      parseSoilData(soilBuffer);
      soilBuffer = "";
    } else if (soilBuffer.length() < 512) {    // tope por si llega basura sin fin de linea
      soilBuffer += c;
    }
  }
}


// =====================================================================
//  PORTAL WEB (igual que la 0.2.2 del S3): red propia LemPDA-XXXX, portal cautivo y API JSON
//  En el C3 (0.1.6) la web es la unica interfaz: no hay pantalla, encoder ni
//  botones.
// =====================================================================
#include "index_html.h"      // pagina web (PAGE)

// ---------- SDI-12: lectura de una linea (aviso de fin de aM!) ----------
String sdi12ReadLine(uint16_t waitMs) {
  String resp = "";
  unsigned long t0 = millis();
  while (millis() - t0 < waitMs) {
    int ch = sdi12ReadChar(waitMs - (millis() - t0));
    if (ch < 0 || ch == '\n') break;
    if (ch == '\r') continue;
    if (ch >= ' ' && ch < 127) resp += (char)ch;
    if (resp.length() >= 80) break;
  }
  resp.trim();
  return resp;
}

// En el C3 el bus SDI-12 comparte GP6/GP7 con Pulse1/Pulse2/SHT10: mientras
// corre Sense-QC o Weather-QC no se usa el bus
bool sdi12PinsBusy() {
  return testerStateSA == SA_TESTING || testerStateWX == WX_TESTING;
}

// ---------- tareas SDI-12 de la web (por pasos, igual que el gestor S2) ----------
int wFindSensor(char a) {
  for (int i = 0; i < wNSensors; i++) if (wSensors[i].addr == a) return i;
  return -1;
}

void wFinishJob(const String& msg, bool err) {
  exitSDI12();                     // devuelve GP6/GP7 a Pulse/SHT10
  wLastMsg = msg;
  wLastErr = err;
  wJob.type = WJOB_NONE;
  wJob.target = 0;
}

void wRebuildList() {
  WSensor old[WEB_MAX_SENSORS];
  int nOld = wNSensors;
  for (int i = 0; i < nOld; i++) old[i] = wSensors[i];
  wNSensors = 0;
  for (int f = 0; f < wScanCount && wNSensors < WEB_MAX_SENSORS; f++) {
    WSensor s;
    s.addr = wScanFound[f];
    for (int i = 0; i < nOld; i++) if (old[i].addr == s.addr) { s = old[i]; break; }
    wSensors[wNSensors++] = s;
  }
}

void wReadInfo(int idx) {
  String r = sdi12Command(String(wSensors[idx].addr) + "I!", 400);
  if (r.length() > 1 && r.charAt(0) == wSensors[idx].addr) wSensors[idx].info = r.substring(1);
}

void wStoreMeas() {
  int idx = wFindSensor(wJob.target);
  if (idx < 0) return;
  WMeas& m = (wJob.mode == 'C') ? wSensors[idx].c : wSensors[idx].m;
  m.n   = wJob.n;
  m.t   = wJob.t;
  m.at  = millis() / 1000;
  m.raw = wJob.raw;
}

void wJobStep() {
  if (wJob.type == WJOB_NONE) return;
  if (!sdi12Active) sdi12Begin();  // toma GP6/GP7 para el bus

  if (wJob.type == WJOB_SCAN) {
    if (wJob.phase == 0) {
      char a = sdi12AddrAt(wJob.i);
      if (sdi12Probe(a)) wScanFound[wScanCount++] = a;
      if (++wJob.i >= SDI12_NUM_ADDR) { wRebuildList(); wJob.phase = 1; wJob.k = 0; }
      return;
    }
    if (wJob.k < wNSensors) {
      if (wSensors[wJob.k].info.length() == 0) wReadInfo(wJob.k);
      wJob.k++;
      return;
    }
    String msg = wNSensors == 1 ? "1 sensor encontrado" : String(wNSensors) + " sensores encontrados";
    if (wScanCount > WEB_MAX_SENSORS) msg += " (se muestran " + String(WEB_MAX_SENSORS) + ")";
    wFinishJob(msg, false);
    return;
  }

  if (wJob.type == WJOB_INFO) {
    int idx = wFindSensor(wJob.target);
    if (idx >= 0) wReadInfo(idx);
    wFinishJob(idx >= 0 && wSensors[idx].info.length() ? String("Identificación de ") + wJob.target + " leída"
                                                        : String("El sensor ") + wJob.target + " no respondió a aI!", idx < 0);
    return;
  }

  if (wJob.type == WJOB_MEASURE) {
    if (wJob.phase == 0) {                       // enviar aC! o aM!
      String r = sdi12Command(String(wJob.target) + wJob.mode + "!", 400);
      bool ok = r.length() >= 5 && r.charAt(0) == wJob.target;
      if (!ok && wJob.mode == 'C') {
        int idx = wFindSensor(wJob.target);
        if (idx >= 0) { wSensors[idx].c.n = -2; wSensors[idx].c.raw = ""; }
        wJob.mode = 'M';                         // sin aC!: se mide con aM!
        return;
      }
      if (!ok) { wFinishJob(String("El sensor ") + wJob.target + " no respondió a a" + wJob.mode + "!", true); return; }
      wJob.t = r.substring(1, 4).toInt();
      wJob.n = r.substring(4).toInt();
      wJob.raw = "";
      wJob.waitStart = millis();
      wJob.waitMs = (unsigned long)min(wJob.t, WEB_MAX_WAIT_S) * 1000UL + 150UL;
      wJob.phase = 1;
      return;
    }
    if (wJob.phase == 1) {                       // esperar sin bloquear
      bool ready = millis() - wJob.waitStart >= wJob.waitMs;
      if (!ready && wJob.mode == 'M' && digitalRead(SDI12_RX_PIN) != SDI12_RX_MARK) {
        String sr = sdi12ReadLine(60);           // aviso "a<CR><LF>" de fin de aM!
        ready = sr.length() > 0 && sr.charAt(0) == wJob.target;
      }
      if (ready) { wJob.phase = 2; wJob.k = 0; }
      return;
    }
    String dr = sdi12Command(String(wJob.target) + "D" + String(wJob.k) + "!", 900);
    String body = (dr.length() > 1 && dr.charAt(0) == wJob.target) ? dr.substring(1) : "";
    bool hasValues = body.indexOf('+') >= 0 || body.indexOf('-') >= 0;
    if (hasValues) wJob.raw += body;
    if (hasValues && ++wJob.k <= 9) return;
    wStoreMeas();
    if (wJob.both && wJob.mode == 'C') { wJob.mode = 'M'; wJob.phase = 0; return; }
    wFinishJob(String("Medición de ") + wJob.target + " lista", false);
    return;
  }

  if (wJob.type == WJOB_CHID) {
    char oldA = wJob.target, newA = wJob.newAddr;
    if (sdi12Probe(newA))  { wFinishJob(String("El ID ") + newA + " ya está en uso", true); return; }
    if (!sdi12Probe(oldA)) { wFinishJob(String("El sensor ") + oldA + " no responde", true); return; }
    sdi12Command(String(oldA) + "A" + newA + "!", 400);
    bool newOk = sdi12Probe(newA);
    bool oldGone = !sdi12Probe(oldA);
    if (!newOk) { wFinishJob(String("No se pudo cambiar el ID de ") + oldA, true); return; }
    int idx = wFindSensor(oldA);
    if (idx >= 0) wSensors[idx].addr = newA;
    for (int a = 0; a < wNSensors; a++)
      for (int b = a + 1; b < wNSensors; b++)
        if (sdi12IndexOf(wSensors[b].addr) < sdi12IndexOf(wSensors[a].addr)) { WSensor t = wSensors[a]; wSensors[a] = wSensors[b]; wSensors[b] = t; }
    wFinishJob(String("ID cambiado: ") + oldA + " -> " + newA + (oldGone ? "" : " (la dirección anterior aún responde)"), false);
    return;
  }
}

String wJobText(int& p, int& t) {
  p = 0; t = 0;
  switch (wJob.type) {
    case WJOB_SCAN:
      if (wJob.phase == 0) { p = wJob.i; t = SDI12_NUM_ADDR; return "Escaneando " + String(wJob.i) + "/62"; }
      p = wJob.k; t = wNSensors;
      return "Leyendo identificación " + String(min(wJob.k + 1, wNSensors)) + "/" + String(wNSensors);
    case WJOB_INFO:    return String("Leyendo aI! de ") + wJob.target;
    case WJOB_CHID:    return String("Cambiando ID ") + wJob.target + " -> " + wJob.newAddr;
    case WJOB_MEASURE: {
      String m = String("a") + wJob.mode + "!";
      if (wJob.phase == 0) return "Enviando " + m + " a " + wJob.target;
      if (wJob.phase == 1) {
        p = millis() - wJob.waitStart; t = wJob.waitMs;
        long rest = ((long)wJob.waitMs - p) / 1000;
        return "Midiendo " + m + " en " + wJob.target + ": faltan " + String(max(rest, 0L)) + " s";
      }
      return "Leyendo datos " + m + " (aD" + String(wJob.k) + "!) de " + wJob.target;
    }
    default: return "";
  }
}

// ---------- JSON ----------
String jsonEsc(const String& s) {
  String o;
  o.reserve(s.length() + 4);
  for (size_t i = 0; i < s.length(); i++) {
    char c = s.charAt(i);
    if (c == '"' || c == '\\') { o += '\\'; o += c; }
    else if ((uint8_t)c < 0x20) o += ' ';
    else o += c;
  }
  return o;
}

// Numero JSON; NaN/inf -> null (JSON no admite NaN)
void jNum(String& j, float v, int dec) {
  if (isnan(v) || isinf(v)) j += "null";
  else j += String(v, dec);
}

float useHours() {
  unsigned long el = millis() - estStartTime;
  if (estMAh <= 0.002f || el <= 5000UL) return NAN;
  float rate = estMAh / (el / 3600000.0f);          // mAh por hora
  if (rate <= 0.0001f) return NAN;
  return BAT_CAPACITY_MAH / rate;
}

String buildDataJson() {
  float median = 0, avg = 0;
  bool statsOk = false;
  computeCurrentStats(median, avg, statsOk);
  unsigned long now = millis();
  String j;
  j.reserve(3000);
  j += "{\"fw\":\"0.1.6\",\"up\":"; j += now / 1000;
  j += ",\"ap\":\""; j += apName; j += "\",\"cl\":"; j += (unsigned int)WiFi.softAPgetStationNum();
  j += ",\"heap\":"; j += (unsigned long)ESP.getFreeHeap();
  j += ",\"sdib\":"; j += wJob.type != WJOB_NONE ? "true" : "false";   // bus SDI-12 en uso
  j += ",\"err\":\""; if (inaFailed) j += "El INA228 no responde: revise el I2C (SDA GP0 / SCL GP1). La medición de corriente y los tests están detenidos; el gestor SDI-12 funciona."; j += "\"";

  // Bateria: v = null si no hay ADS1115 (la bateria se mide en su A3)
  j += ",\"bat\":{\"v\":"; jNum(j, batteryVoltage, 3); j += ",\"p\":"; j += batteryPercent; j += "}";

  j += ",\"cur\":{\"i\":"; jNum(j, inaFailed ? NAN : currentCurrent, 3);
  j += ",\"med\":"; jNum(j, statsOk ? median : NAN, 2);
  j += ",\"max\":"; jNum(j, max_current_mA > 0.0f ? max_current_mA : NAN, 2);
  j += ",\"min\":"; jNum(j, min_current_mA > 0.0f ? min_current_mA : NAN, 3);
  j += ",\"v\":";   jNum(j, inaFailed ? NAN : currentVoltage, 3);
  j += ",\"mah\":"; jNum(j, inaFailed ? NAN : estMAh, 2);
  j += ",\"avg\":"; jNum(j, statsOk ? avg : NAN, 2);
  j += ",\"use\":"; jNum(j, useHours(), 1);
  j += ",\"st\":\""; j += isRecording ? (endingRecording ? "fin" : "rec") : (recordingComplete ? "done" : "wait"); j += "\"";
  j += ",\"n\":"; j += sampleCount;
  j += ",\"oc\":"; j += currentCurrent > OC_THRESHOLD ? "true" : "false";
  j += ",\"paused\":"; j += dischState == DISCH_RUNNING ? "true" : "false";
  j += ",\"thS\":"; jNum(j, CURRENT_START_THRESHOLD, 2);
  j += ",\"thO\":"; jNum(j, OC_THRESHOLD, 1);
  j += "}";

  j += ",\"env\":{\"bt\":"; jNum(j, bmePresent ? envBmeT : NAN, 1);
  j += ",\"bh\":"; jNum(j, bmePresent ? envBmeH : NAN, 0);
  j += ",\"bp\":"; jNum(j, bmePresent ? envBmeP : NAN, 0);
  j += ",\"s30t\":"; jNum(j, sht30Present ? envS30T : NAN, 1);
  j += ",\"s30h\":"; jNum(j, sht30Present ? envS30H : NAN, 0);
  j += ",\"s10t\":"; jNum(j, envS10T, 1);
  j += ",\"s10h\":"; jNum(j, envS10H, 0);
  j += ",\"age\":"; j += envAt ? (long)((now - envAt) / 1000) : -1L;
  j += "}";

  j += ",\"ads\":";
  if (adsPresent) {
    j += "[";
    for (int ch = 0; ch < 4; ch++) { if (ch) j += ','; jNum(j, adsV[ch], 4); }
    j += "]";
  } else j += "null";
  j += ",\"p1\":"; j += gpioP1 ? 1 : 0;
  j += ",\"p2\":"; j += gpioP2 ? 1 : 0;
  j += ",\"urx\":"; j += gpioUrx ? 1 : 0;

  j += ",\"uart\":[";
  for (int i = 0; i < UART_BUF_LINES; i++) {
    if (i) j += ',';
    j += '"'; j += jsonEsc(String(uartLines[i])); j += '"';
  }
  j += "]";

  j += ",\"soil\":{\"ok\":"; j += soil_dataReady ? "true" : "false";
  j += ",\"age\":"; j += soil_dataReady ? (long)((now - lastSoilParse) / 1000) : -1L;
  j += ",\"h\":["; for (int i = 0; i < 12; i++) { if (i) j += ','; jNum(j, soil_hum[i], 1); }
  j += "],\"t\":["; for (int i = 0; i < 13; i++) { if (i) j += ','; jNum(j, soil_temp[i], 1); }
  j += "]}";

  unsigned long el = now - sa_testStart;
  j += ",\"sa\":{\"st\":\""; j += testerStateSA == SA_TESTING ? "test" : (testerStateSA == SA_RESULT ? "res" : "idle"); j += "\"";
  j += ",\"prog\":"; j += testerStateSA == SA_TESTING ? (int)min(100UL, el * 100UL / SA_TEST_DURATION) : 0;
  j += ",\"pass\":"; j += sa_passed ? "true" : "false";
  j += ",\"why\":\""; j += jsonEsc(sa_failReason); j += "\"";
  j += ",\"i\":"; jNum(j, sa_current, 1); j += ",\"a0\":"; jNum(j, sa_a0v, 3); j += ",\"a1\":"; jNum(j, sa_a1v, 3);
  j += ",\"ok\":["; j += sa_currentOk ? 1 : 0; j += ','; j += sa_a0Ok ? 1 : 0; j += ','; j += sa_a1Ok ? 1 : 0;
  j += ','; j += sa_pulse1Ok ? 1 : 0; j += ','; j += sa_pulse2Ok ? 1 : 0; j += "]}";

  j += ",\"wx\":{\"st\":\""; j += testerStateWX == WX_TESTING ? "test" : (testerStateWX == WX_RESULT ? "res" : "idle"); j += "\"";
  j += ",\"prog\":"; j += testerStateWX == WX_TESTING ? (int)min(100UL, el * 100UL / 4000UL) : 0;
  j += ",\"pass\":"; j += wx_passed ? "true" : "false";
  j += ",\"why\":\""; j += jsonEsc(wx_failReason); j += "\"";
  j += ",\"i\":"; jNum(j, wx_current, 1);
  j += ",\"bt\":"; jNum(j, wx_bme_temp, 1); j += ",\"bh\":"; jNum(j, wx_bme_hum, 0);
  j += ",\"tt\":"; jNum(j, wx_sht_temp, 1); j += ",\"th\":"; jNum(j, wx_sht_hum, 0);
  j += ",\"ok\":["; j += wx_currentOk ? 1 : 0; j += ','; j += wx_sht10Ok ? 1 : 0; j += ','; j += wx_sht30Ok ? 1 : 0; j += "]}";

  j += ",\"dis\":{\"st\":\""; j += dischState == DISCH_RUNNING ? "run" : (dischState == DISCH_DONE ? "done" : "idle"); j += "\"";
  j += ",\"v\":"; jNum(j, disch_voltage, 3); j += ",\"i\":"; jNum(j, disch_current, 1);
  j += ",\"mah\":"; jNum(j, disch_mAh, 2); j += ",\"mwh\":"; jNum(j, disch_mWh, 1);
  j += ",\"s\":"; j += disch_duration / 1000; j += ",\"pk\":"; jNum(j, disch_peakCurrent, 1);
  j += ",\"pct\":"; jNum(j, BAT_CAPACITY_MAH > 0 ? disch_mAh / BAT_CAPACITY_MAH * 100.0f : NAN, 0);
  j += "}}";
  return j;
}

String buildSamplesJson() {
  String j;
  j.reserve(64 + sampleCount * 30);
  j += "{\"n\":"; j += sampleCount;
  j += ",\"t\":["; for (int i = 0; i < sampleCount; i++) { if (i) j += ','; jNum(j, recordedTimes[i] / 1000.0f, 1); }
  j += "],\"i\":["; for (int i = 0; i < sampleCount; i++) { if (i) j += ','; jNum(j, recordedCurrents[i], 3); }
  j += "],\"q\":["; for (int i = 0; i < sampleCount; i++) { if (i) j += ','; jNum(j, recordedMAh[i], 3); }
  j += "]}";
  return j;
}

void wMeasJson(String& j, const WMeas& m) {
  j += "{\"n\":"; j += m.n;
  j += ",\"t\":"; j += m.t;
  j += ",\"at\":"; j += m.at;
  j += ",\"raw\":\""; j += jsonEsc(m.raw); j += "\"}";
}

String buildSdiJson() {
  int p, t;
  String txt = wJobText(p, t);
  String j;
  j.reserve(1024 + wNSensors * 400);
  j += "{\"up\":"; j += millis() / 1000;
  j += ",\"ap\":\""; j += apName; j += "\"";
  j += ",\"busy\":"; j += wJob.type != WJOB_NONE ? "true" : "false";
  j += ",\"dev\":"; j += sdi12PinsBusy() ? "true" : "false";
  j += ",\"a\":\""; if (wJob.target) j += wJob.target; j += "\"";
  j += ",\"p\":"; j += p; j += ",\"t\":"; j += t;
  j += ",\"txt\":\""; j += jsonEsc(txt); j += "\"";
  j += ",\"msg\":\""; j += jsonEsc(wLastMsg); j += "\"";
  j += ",\"err\":"; j += wLastErr ? "true" : "false";
  j += ",\"sensors\":[";
  for (int i = 0; i < wNSensors; i++) {
    if (i) j += ',';
    j += "{\"a\":\""; j += wSensors[i].addr; j += "\"";
    j += ",\"i\":\""; j += jsonEsc(wSensors[i].info); j += "\"";
    j += ",\"c\":"; wMeasJson(j, wSensors[i].c);
    j += ",\"m\":"; wMeasJson(j, wSensors[i].m);
    j += "}";
  }
  j += "]}";
  return j;
}

String buildCfgJson() {
  String j = "{";
  j += "\"start\":";   jNum(j, CURRENT_START_THRESHOLD, 3);
  j += ",\"end\":";    jNum(j, CURRENT_END_THRESHOLD, 3);
  j += ",\"period\":"; jNum(j, SAMPLE_PERIOD / 1000.0f, 2);
  j += ",\"maxt\":";   jNum(j, MAX_RECORD_TIME / 1000.0f, 1);
  j += ",\"oc\":";     jNum(j, OC_THRESHOLD, 2);
  j += ",\"samples\":"; j += MAX_SAMPLES_CONFIG;
  j += ",\"bat\":";    jNum(j, BAT_CAPACITY_MAH, 0);
  j += ",\"vref\":";   jNum(j, SA_VOLT_REF, 3);
  j += ",\"vtol\":";   jNum(j, SA_VOLT_TOL * 100.0f, 2);
  j += ",\"imin\":";   jNum(j, SA_CURR_MIN, 3);
  j += ",\"imax\":";   jNum(j, SA_CURR_MAX, 2);
  j += ",\"dlim\":";   jNum(j, DISCH_CURR_LIMIT, 1);
  j += "}";
  return j;
}

// ---------- handlers HTTP ----------
void webJson(int code, const String& body) {
  webServer.sendHeader("Cache-Control", "no-store");
  webServer.send(code, "application/json; charset=utf-8", body);
}
void webOk() { webJson(200, "{\"ok\":true}"); }
void webErr(int code, const String& msg) { webJson(code, "{\"ok\":false,\"err\":\"" + jsonEsc(msg) + "\"}"); }

bool inaGuard() {
  if (!inaFailed) return false;
  webErr(503, "El INA228 no responde: la medición y los tests están detenidos");
  return true;
}

void hRoot() {
  webServer.sendHeader("Cache-Control", "no-store");
  webServer.send_P(200, "text/html; charset=utf-8", PAGE);
}

void hData() {
  if (webServer.hasArg("sen")) {         // pestana Sensores abierta: leerlos
    if (millis() - webSenHit > WEB_ACTIVE_MS) { webTAds = 0; webTEnv = 0; webTS10 = 0; }   // refresco inmediato
    webSenHit = millis();
  }
  webJson(200, buildDataJson());
}

void hSamples() { webJson(200, buildSamplesJson()); }

void hCurReset() {
  if (inaGuard()) return;
  if (dischState == DISCH_RUNNING) { webErr(409, "Hay una descarga de batería en curso"); return; }
  resetRecording();
  webOk();
}

bool anyTestRunning() {
  return testerStateSA == SA_TESTING || testerStateWX == WX_TESTING || dischState == DISCH_RUNNING;
}

void hTest() {
  if (inaGuard()) return;
  if (anyTestRunning()) { webErr(409, "Ya hay una prueba en curso"); return; }
  if (wJob.type != WJOB_NONE) { webErr(409, "El bus SDI-12 está en uso y comparte pines con esta prueba: espere"); return; }
  String t = webServer.arg("t");
  if (t == "sa") {
    pinMode(PULSE1_PIN, INPUT);          // GP6/GP7 como entradas aunque antes los
    pinMode(PULSE2_PIN, INPUT);          // usara el SDI-12 o el SHT10
    testerStateSA = SA_TESTING;
    sa_testStart  = millis();
    sa_pulseReset = true;
    sa_passed     = false;
    sa_failReason = "";
  } else if (t == "wx") {
    testerStateWX = WX_TESTING;
    sa_testStart  = millis();
    wx_passed     = false;
    wx_failReason = "";
  } else { webErr(400, "Prueba desconocida"); return; }
  webOk();
}

void hDisch() {
  if (inaGuard()) return;
  String op = webServer.arg("op");
  if (op == "start") {
    if (anyTestRunning()) { webErr(409, "Ya hay una prueba en curso"); return; }
    if (isRecording) { stopRecording(); endingRecording = false; }
    dischState        = DISCH_RUNNING;
    disch_startTime   = millis();
    disch_lastSample  = millis();
    disch_mAh         = 0;
    disch_mWh         = 0;
    disch_peakCurrent = 0;
    ina228.resetAccumulators();
  } else if (op == "stop") {
    if (dischState == DISCH_RUNNING) dischState = DISCH_DONE;
  } else { webErr(400, "Operación desconocida"); return; }
  webOk();
}

void hCfgGet() { webJson(200, buildCfgJson()); }

// Lee un parametro numerico de la peticion; false si no vino o no es valido
bool cfgArg(const char* key, float& out, float lo, float hi) {
  if (!webServer.hasArg(key)) return false;
  String s = webServer.arg(key);
  s.trim();
  if (s.length() == 0) return false;
  float v = s.toFloat();
  if (isnan(v) || v < lo || v > hi) return false;
  out = v;
  return true;
}

void hCfgSet() {
  float v;
  int oldSamples = MAX_SAMPLES_CONFIG;
  if (cfgArg("start", v, 0, 100000))  CURRENT_START_THRESHOLD = v;
  if (cfgArg("end", v, 0, 100000))    CURRENT_END_THRESHOLD   = v;
  if (cfgArg("period", v, 0.1f, 3600)) SAMPLE_PERIOD          = (unsigned long)(v * 1000.0f);
  if (cfgArg("maxt", v, 1, 86400))    MAX_RECORD_TIME         = (unsigned long)(v * 1000.0f);
  if (cfgArg("oc", v, 0, 100000))     OC_THRESHOLD            = v;
  if (cfgArg("samples", v, 10, 2000)) MAX_SAMPLES_CONFIG      = (int)v;
  if (cfgArg("bat", v, 1, 1000000))   BAT_CAPACITY_MAH        = v;
  if (cfgArg("vref", v, 0, 10))       SA_VOLT_REF             = v;
  if (cfgArg("vtol", v, 0, 100))      SA_VOLT_TOL             = v / 100.0f;
  if (cfgArg("imin", v, 0, 100000))   SA_CURR_MIN             = v;
  if (cfgArg("imax", v, 0, 100000))   SA_CURR_MAX             = v;
  if (cfgArg("dlim", v, 0, 100000))   DISCH_CURR_LIMIT        = v;
  if (MAX_SAMPLES_CONFIG != oldSamples) {
    reallocArrays();
    if (!inaFailed) resetRecording();
  }
  saveConfig();
  webOk();
}

void hCfgReset() {
  restoreDefaults();                // incluye reallocArrays() y saveConfig()
  if (!inaFailed) resetRecording();
  webOk();
}

// SHT10 a pedido (boton "Leer SHT10"). En el C3 su linea de datos es GP7, la
// misma de Pulse2 y SDI-12 TX: no se lee si el bus o un test usan esos pines.
void hSht10() {
  if (wJob.type != WJOB_NONE || sdi12PinsBusy()) {
    webErr(409, "Espere: el bus SDI-12 o un test están usando GP7");
    return;
  }
  envS10T = sht10.readTemperatureC();
  envS10H = sht10.readHumidity();
  pinMode(SHT10_DATA_PIN, INPUT);        // GP7 vuelve a ser entrada (Pulse2)
  if (!sht10Valid(envS10T, envS10H)) {   // sin sensor la libreria da -40.1 C
    envS10T = NAN; envS10H = NAN;
    webErr(504, "El SHT10 no respondió");
    return;
  }
  webOk();
}

// SDI-12 desde la web
bool wBusy() {
  if (wJob.type != WJOB_NONE) { webErr(409, "Ocupado: espere a que termine la operación en curso"); return true; }
  if (sdi12PinsBusy())        { webErr(409, "Hay una prueba usando los pines compartidos con SDI-12 (Pulse/SHT10)"); return true; }
  return false;
}

bool wArgSensor(char& a) {
  String s = webServer.arg("a");
  a = s.length() == 1 ? s.charAt(0) : 0;
  if (sdi12IndexOf(a) < 0 || wFindSensor(a) < 0) { webErr(400, "Sensor no válido: escanee de nuevo"); return false; }
  return true;
}

void hSdi() { webJson(200, buildSdiJson()); }

void hSdiScan() {
  if (wBusy()) return;
  wJob = WJob();
  wJob.type = WJOB_SCAN;
  wScanCount = 0;
  wLastMsg = "";
  webOk();
}

void hSdiInfo() {
  char a;
  if (wBusy() || !wArgSensor(a)) return;
  wJob = WJob();
  wJob.type = WJOB_INFO;
  wJob.target = a;
  webOk();
}

void hSdiMeasure() {
  char a;
  if (wBusy() || !wArgSensor(a)) return;
  String m = webServer.arg("m");
  wJob = WJob();
  wJob.type = WJOB_MEASURE;
  wJob.target = a;
  wJob.both = (m == "B");
  wJob.mode = (m == "M") ? 'M' : 'C';
  webOk();
}

void hSdiChid() {
  char a;
  if (wBusy() || !wArgSensor(a)) return;
  String n = webServer.arg("n");
  char na = n.length() == 1 ? n.charAt(0) : 0;
  if (sdi12IndexOf(na) < 0 || wFindSensor(na) >= 0) { webErr(400, "El ID nuevo no es válido o ya está en uso"); return; }
  wJob = WJob();
  wJob.type = WJOB_CHID;
  wJob.target = a;
  wJob.newAddr = na;
  webOk();
}

// Portal cautivo: cualquier otra URL (pruebas de conexion de Android, iOS y
// Windows incluidas) redirige a la pagina del equipo
void hPortal() {
  webServer.sendHeader("Location", String("http://") + AP_IP.toString() + "/", true);
  webServer.send(302, "text/plain", "");
}

void webSetup() {
  WiFi.mode(WIFI_AP);
  uint8_t mac[6] = {0};
  WiFi.softAPmacAddress(mac);
  char ssid[16];
  snprintf(ssid, sizeof(ssid), "LemPDA-%02X%02X", mac[4], mac[5]);
  apName = ssid;
  WiFi.softAPConfig(AP_IP, AP_IP, AP_MASK, IPAddress((uint32_t)0), AP_IP);   // DNS = el equipo
  if (!WiFi.softAP(ssid, AP_PASS)) Serial.println("ERROR: no se pudo crear la red WiFi");
  // Muchos C3 SuperMini tienen una antena que falla a potencia maxima: con
  // 8,5 dBm la red funciona bien a unos metros
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  webDns.start(53, "*", AP_IP);

  webServer.on("/", HTTP_GET, hRoot);
  webServer.on("/api/data", HTTP_GET, hData);
  webServer.on("/api/samples", HTTP_GET, hSamples);
  webServer.on("/api/cur/reset", HTTP_POST, hCurReset);
  webServer.on("/api/test", HTTP_POST, hTest);
  webServer.on("/api/disch", HTTP_POST, hDisch);
  webServer.on("/api/cfg", HTTP_GET, hCfgGet);
  webServer.on("/api/cfg", HTTP_POST, hCfgSet);
  webServer.on("/api/cfg/reset", HTTP_POST, hCfgReset);
  webServer.on("/api/sdi", HTTP_GET, hSdi);
  webServer.on("/api/sht10", HTTP_POST, hSht10);
  webServer.on("/api/sdi/scan", HTTP_POST, hSdiScan);
  webServer.on("/api/sdi/info", HTTP_POST, hSdiInfo);
  webServer.on("/api/sdi/measure", HTTP_POST, hSdiMeasure);
  webServer.on("/api/sdi/chid", HTTP_POST, hSdiChid);
  webServer.onNotFound(hPortal);
  webServer.begin();
  Serial.printf("Red %s (clave %s) -> http://%s/\n", ssid, AP_PASS, AP_IP.toString().c_str());
}

// Llamada en cada vuelta del loop
void webLoop() {
  webDns.processNextRequest();      // en el core 3.x el DNS es asincrono; no hace dano
  webServer.handleClient();
  wJobStep();

  // Sensores para la web: solo se leen con la pestana Sensores abierta
  unsigned long now = millis();
  if (now - webSenHit > WEB_ACTIVE_MS) return;
  if (now - webTAds >= 1000UL) {
    webTAds = now;
    if (adsPresent) for (int ch = 0; ch < 4; ch++) adsV[ch] = ads.readADC_SingleEnded(ch) * 0.000125f;
    gpioP1  = digitalRead(PULSE1_PIN);
    gpioP2  = digitalRead(PULSE2_PIN);
    gpioUrx = digitalRead(UART_RX_PIN);
  }
  if (now - webTEnv >= 5000UL) {
    webTEnv = now;
    if (bmePresent)   { envBmeT = bme.readTemperature(); envBmeH = bme.readHumidity(); envBmeP = bme.readPressure() / 100.0f; }
    if (sht30Present) { envS30T = sht30.readTemperature(); envS30H = sht30.readHumidity(); }
    envAt = now;
  }
  // El SHT10 no se lee en segundo plano: en el C3 su linea de datos es GP7
  // (Pulse2 / SDI-12 TX). Se lee solo con el boton de la web (hSht10).
}

// ============================================
// SETUP
// ============================================
void setup() {
  Serial.begin(115200);
  rgbSet(0, 0, 25);              // azul = arrancando

  EEPROM.begin(EEPROM_SIZE);
  loadConfig();

  Wire.begin(SDA_PIN, SCL_PIN);

  // Sensores I2C: se anota cuales respondieron (los ausentes no se leen)
  adsPresent = ads.begin();
  if (!adsPresent) Serial.println("ADS1115 no encontrado (sin A0/A1 ni bateria)");
  ads.setGain(GAIN_ONE);
  bmePresent = bme.begin(0x76);
  if (!bmePresent) Serial.println("BME280 no encontrado");
  sht30Present = sht30.begin(0x44);
  if (!sht30Present) Serial.println("SHT30 no encontrado");

  // INA228: sin el no hay medicion ni tests, pero la web y el SDI-12 siguen
  inaFailed = !ina228.begin();
  if (inaFailed) {
    Serial.println("ERROR: el INA228 no responde (I2C SDA=GP0 SCL=GP1). Solo web y SDI-12.");
  } else {
    ina228.setShunt(0.015, 20.0);
    ina228.setAveragingCount(INA228_COUNT_128);
  }

  pinMode(PULSE1_PIN,  INPUT);
  pinMode(PULSE2_PIN,  INPUT);
  pinMode(UART_RX_PIN, INPUT);

  reallocArrays();
  updateBattery();

  // Red WiFi LemPDA-XXXX + portal web
  webSetup();
}

// ============================================
// MAIN LOOP
// ============================================
void loop() {
  // Bateria cada 2s
  static unsigned long lastBatUpdate = 0;
  if (millis() - lastBatUpdate > 2000) {
    updateBattery();
    lastBatUpdate = millis();
  }

  if (!inaFailed) {
    // Corriente: siempre, salvo durante la descarga de bateria (que usa el
    // acumulador de carga del INA228)
    // El INA228 promedia 128 conversiones (~0,4 s por dato): leerlo cada 50 ms
    // basta y no satura el I2C ni el unico nucleo del C3, que tambien usa el WiFi
    if (dischState != DISCH_RUNNING) {
      static unsigned long lastCur = 0;
      if (millis() - lastCur >= 50) { lastCur = millis(); updateCurrentReadings(); }
      handleRecording();
    }
    // Tests y descarga (cada uno no hace nada si no esta activo)
    updateSA();
    updateWX();
    updateDischarge();
  }

  updateSerialInputs();

  // Red WiFi, portal web y tareas SDI-12 pedidas desde la web
  webLoop();
  updateStatusLED();
  delay(2);
}
