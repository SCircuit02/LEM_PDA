/*
 * QC LemBot - ESP32-S3 LOLIN Mini (Wemos S3 Mini)
 * Version: 0.2.1
 * Port a ESP32-S3 (Xtensa LX7 dual, WiFi + BT 5 LE). Requiere el core
 * arduino-esp32 v3.x ("LOLIN S3 Mini" o "ESP32S3 Dev Module" en el IDE).
 *
 * MCU objetivo: ESP32-S3 (LOLIN S3 Mini). Gemelo del port C6
 * (tester_esp32c6_v0_2_0.ino), con el mapa de pines del S3.
 *
 * Mapa de pines S3 (LOLIN S3 Mini):
 *   Siempre activos:
 *     GP5=EncA GP6=EncB GP7=Back GP8=Confirm GP9=BtnEnc GP11=SDA GP12=SCL
 *     GP13=SDI-12 TX  GP10=SDI-12 RX
 *     GP1=ADC bateria (ADC1_CH0: no se deshabilita con WiFi/BT)
 *   Modulos (solo activos dentro de su modulo):
 *     GP2=Pulse1 GP4=Pulse2  GP16=SHT10 data GP17=SHT10 clk  GP18=UART RX
 *   LED: GP47=RGB WS2812 onboard (debug visual)  [confirmar 47 vs 48]
 *   Evitados: GP0/3/45/46 (strapping), GP19/20 (USB), GP26-32 (flash),
 *             GP43/44 (UART0 debug), GP33-42/48.
 *
 * Bateria: ADC INTERNO en GP1 (ADC1_CH0), divisor 2:1, analogReadMilliVolts()
 * (calibrado). ADC1 sigue disponible con WiFi/BT activos (ADC2 = GP11..GP20 no).
 * El ADS1115 se conserva para A0/A1 (Sense-QC).
 *
 * Nuevo en 0.2.1: pagina 1 de corriente en orden actual, mediana, maxima,
 * minima; la vista del sensor SDI-12 muestra fabricante, modelo, version,
 * serie y version SDI-12 (aI!) junto a los valores medidos.
 * De 0.2.0: LED RGB de debug visual + menu "Zigbee" (work in progress).
 * Hereda de 0.1.x: front-end SDI-12 de hardware (bit-bang 2 pines), scroll de
 * todos los valores (aD0!..aD9!), medicion aC!/aM! seleccionable, corriente en
 * 2 paginas, cambio de ID multi-sensor, bateria 0-100%.
 */

#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <Adafruit_INA228.h>
#include <Adafruit_ADS1X15.h>
#include <EEPROM.h>
#include <Adafruit_BME280.h>
#include <Adafruit_SHT31.h>
#include <SHT1x-ESP.h>  // Libreria: "SHT1x" by Practical Maker

// ============================================
// PIN DEFINITIONS (ESP32-S3 LOLIN Mini)
// ============================================
// UI e I2C en GPIOs libres del S3 (evitando strapping/USB/flash)
#define ENCODER_TRA 5
#define ENCODER_TRB 6
#define BTN_BACK    7
#define BTN_CONFIRM 8
#define BTN_ENCODER 9
#define SDA_PIN     11   // I2C (uso digital; que sea ADC2 no importa)
#define SCL_PIN     12
// Entradas de modulos (solo activas dentro de su modulo)
#define PULSE1_PIN  2
#define PULSE2_PIN  4
#define UART_RX_PIN 18

#define SHT10_DATA_PIN  16   // solo activo dentro de Weather-QC
#define SHT10_CLOCK_PIN 17

// ADC de bateria: ADC1_CH0 (GP1). ADC1 (GP1..GP10) NO se deshabilita al activar
// WiFi/BT (ADC2 = GP11..GP20 SI). Divisor 2:1.
#define BAT_ADC_PIN     1
#define BAT_DIVIDER     2.0f

// LED RGB WS2812 onboard (debug visual). Solo para el LED.
// En la LOLIN S3 Mini suele ser GPIO47 (algunas variantes usan 48): confirmar.
#define RGB_LED_PIN     47

// ============================================
// SDI-12 FRONT-END DE HARDWARE (2 pines dedicados)
// ============================================
// TX = GP13 -> SN74LVC1G3157 -> SN74LVC1G240 (inversor) -> bus SDI-12
// RX = GP10 <- SN74AHC1G14 (inversor Schmitt) <- bus SDI-12
// El bus SDI-12 es logica invertida (marca/idle ~0V, espacio ~5V). Con una
// inversion en TX y una en RX, ambas cancelan la del bus => el MCU ve un UART
// estandar NO invertido: reposo/marca = ALTO, start/espacio = BAJO.
#define SDI12_TX_PIN    13
#define SDI12_RX_PIN    10

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
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, SCL_PIN, SDA_PIN);
//U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE); // pantalla grande

Adafruit_INA228 ina228;
Adafruit_ADS1115 ads;
Adafruit_BME280 bme;
Adafruit_SHT31  sht30;
SHT1x sht10(SHT10_DATA_PIN, SHT10_CLOCK_PIN);

// ============================================
// MENU STATES
// ============================================
enum MenuState {
  MENU_MAIN,
  MENU_CURRENT_VIEW,
  MENU_GRAPH_VIEW,
  MENU_TABLE_VIEW,
  MENU_TESTER_SA,
  MENU_TESTER_WEATHER,
  MENU_TESTER_SOIL,
  MENU_RAW_DATA,
  MENU_TEMP_HUM,
  MENU_UART,
  MENU_SDI12,
  MENU_ZIGBEE,
  MENU_BATTERY_DISCHARGE,
  MENU_CONFIG,
  MENU_PINOUT
};

MenuState currentState   = MENU_MAIN;
int menuSelection        = 0;
const int menuItems      = 12;
const int MENU_VISIBLE   = 4;
int menuTopIndex         = 0;

// ============================================
// ENCODER / BUTTON
// ============================================
volatile int encoderCounter = 0;
volatile byte lastEncoded   = 0;
int lastEncoderCounter      = 0;

bool lastBtnEncoderState = HIGH;
bool lastBtnBackState    = HIGH;
bool lastBtnConfirmState = HIGH;
unsigned long lastDebounceTime     = 0;
const unsigned long DEBOUNCE_DELAY = 50;

// ============================================
// BATTERY
// ============================================
float batteryVoltage   = 0.0f;
int   batteryPercent   = 0;
bool  batteryBlink     = false;
unsigned long lastBatteryBlink = 0;

// ============================================
// CURRENT MONITORING
// ============================================
float currentVoltage    = 0.0f;
float currentCurrent    = 0.0f;
float currentMAh        = 0.0f;
float max_current_mA    = 0.0f;
float min_current_mA    = 0.0f;   // corriente minima > 0 observada (capta uA)

// Vista de corriente: 2 paginas
int   currentViewPage   = 0;      // 0 = pag1 (mA/med/min/max), 1 = pag2 (V/mAh/prom/uso)

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
int           tableScrollOffset = 0;

bool          endingRecording   = false;
unsigned long endRecordingTime  = 0;
const unsigned long END_DELAY   = 3000;

float ina_avgCurrent = 0.0f;

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
// SDI-12 VIEWER / EDITOR
// ============================================
// Direcciones SDI-12 validas: '0'-'9', 'a'-'z', 'A'-'Z' (62 en total)
#define SDI12_NUM_ADDR   62
#define SDI12_MAX_VALUES 30   // AquaCheck y similares devuelven >12 valores
#define SDI12_DATA_VISIBLE 5  // filas de valores visibles a la vez

enum Sdi12State {
  SDI12_IDLE,            // pantalla inicial "SDI-12"
  SDI12_SCANNING,        // escaneando direcciones (incremental)
  SDI12_LIST,            // lista de sensores encontrados + "Cambiar ID"
  SDI12_SENSOR,          // mostrando datos del sensor seleccionado
  SDI12_CHG_SELSRC,      // eligiendo CUAL sensor renombrar (origen)
  SDI12_NEWID,           // seleccionando ID nuevo (libre) con encoder
  SDI12_CHANGING,        // ejecutando cambio de ID + verificacion
  SDI12_CHGRESULT        // resultado del cambio de ID
};

Sdi12State sdi12State   = SDI12_IDLE;
bool   sdi12Active      = false;   // libreria iniciada?
char   sdi12Found[SDI12_NUM_ADDR]; // direcciones que respondieron
int    sdi12Count       = 0;       // cuantas respondieron
int    sdi12ScanIndex   = 0;       // progreso del escaneo (0..61)
int    sdi12Sel         = 0;       // seleccion en la lista (0..count = "Cambiar ID")
int    sdi12TopIndex    = 0;       // scroll de la lista
const int SDI12_VISIBLE = 4;

bool   sdi12NeedMeasure = false;   // pedir medicion al entrar a SENSOR
char   sdi12CurAddr     = '0';     // direccion del sensor en vista
String sdi12IdStr       = "";      // identificacion (aI!) sin la direccion
// Filas de identificacion para la vista del sensor (fabricante, modelo, serie, version SDI-12)
#define SDI12_INFO_MAX  4
String sdi12InfoRow[SDI12_INFO_MAX];
int    sdi12InfoCount   = 0;
int    sdi12WaitSec     = 0;       // segundos que pidio el sensor en la ultima medicion
float  sdi12Values[SDI12_MAX_VALUES];
int    sdi12NumValues   = 0;
int    sdi12DataScroll  = 0;        // scroll de la lista de valores del sensor
bool   sdi12UseConcurrent = true;   // proxima medicion: true=aC!, false=aM!
char   sdi12MeasMode      = 'C';    // modo usado en la ultima medicion ('C'/'M')

int    sdi12NewIdIndex  = 0;       // indice del ID nuevo a seleccionar (0..61)
char   sdi12ChgSrc      = '0';     // sensor origen elegido para renombrar
int    sdi12ChgSrcSel   = 0;       // seleccion en la lista de origen
bool   sdi12ChgOk       = false;
String sdi12ChgMsg      = "";

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
// RAW DATA PAGE
// ============================================
int rawDataPage        = 0;
const int RAW_PAGES    = 4;

// ============================================
// TEMP & HUM PAGE
// ============================================
// (solo usa los sensores, sin estado extra)

// ============================================
// CONFIG MENU
// ============================================
int  configSelection = 0;
const int configItems = 13;
bool isEditingConfig  = false;
float tempConfigValue = 0.0f;
int  configTopIndex   = 0;
const int CONFIG_VISIBLE = 5;

// ============================================
// PINOUT
// ============================================
int pinoutPage = 0;

// ============================================
// FORWARD DECLARATIONS
// ============================================
void encoderISR();   // IRAM_ATTR va solo en la definicion
void handleEncoder();
void handleButtons();
void handleEncoderButton();
void handleBackButton();
void handleConfirmButton();
void selectMenuItem();

void saveConfig();
void loadConfig();
void restoreDefaults();
void restoreOneDefault(int idx);
void loadTempConfigValue();
void applyConfigValue();

void reallocArrays();
void updateCurrentReadings();
void computeCurrentStats(float& median, float& avg, bool& valid);
void handleRecording();
void startRecording();
void stopRecording();
void recordSample(unsigned long ts);
void resetRecording();

void updateBattery();
int  batPercentFromV(float v);
void updateUART();
void parseSoilData(String& line);
void updateSA();
void updateWX();
void updateDischarge();

void updateSDI12();
void sdi12Begin();
void exitSDI12();
void sdi12SendChar(uint8_t c);
void sdi12SendBreakAndCommand(const String& cmd);
int  sdi12ReadChar(unsigned long timeoutMs);
char sdi12AddrAt(int i);
int  sdi12IndexOf(char addr);
String sdi12Command(const String& cmd, uint16_t waitMs);
bool sdi12Probe(char addr);
bool sdi12AddrUsed(char addr);
int  sdi12NextFreeIndex(int from, int dir);
void sdi12DoScanStep();
int  sdi12ParseValues(const String& dr);
void sdi12DoMeasure(char addr, bool concurrent);
void sdi12BuildInfoRows();
int  sdi12ViewRows();
void sdi12ViewRow(int idx, char* out, size_t n);
void sdi12DoChangeId(char oldAddr, char newAddr);

void updateDisplay();
void drawMainMenu();
void drawCurrentView();
void drawGraphView();
void drawTableView();
void drawSenseQC();
void drawWeatherQC();
void drawSoilQC();
void drawRawDataView();
void drawTempHumView();
void drawUARTView();
void drawBatteryDischarge();
void drawConfigView();
void drawPinoutView();
void drawSDI12View();
void drawZigbeeView();
void sdi12ShowBusy(const char* msg);
void rgbSet(uint8_t r, uint8_t g, uint8_t b);
void updateStatusLED();
void drawBatteryIcon(int x, int y, int percent);
void drawScrollBar(int x, int y, int h, int cur, int total);

// ============================================
// ISR ENCODER
// ============================================
void IRAM_ATTR encoderISR() {
  byte MSB = digitalRead(ENCODER_TRA);
  byte LSB = digitalRead(ENCODER_TRB);
  byte encoded = (MSB << 1) | LSB;
  byte sum     = (lastEncoded << 2) | encoded;
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderCounter = encoderCounter + 1;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderCounter = encoderCounter - 1;
  lastEncoded = encoded;
}

// ============================================
// SETUP
// ============================================
void setup() {
  Serial.begin(115200);

  EEPROM.begin(EEPROM_SIZE);
  loadConfig();

  Wire.begin(SDA_PIN, SCL_PIN);

  u8g2.begin();
  u8g2.setFont(u8g2_font_6x10_tf);

  // INA228 - obligatorio
  if (!ina228.begin()) {
    u8g2.clearBuffer();
    u8g2.drawStr(10, 30, "INA228 ERROR!");
    u8g2.sendBuffer();
    rgbSet(60, 0, 0);            // rojo fijo = error critico de hardware
    while (1) delay(100);
  }
  ina228.setShunt(0.015, 20.0);
  ina228.setAveragingCount(INA228_COUNT_128);

  // ADS1115
  if (!ads.begin()) Serial.println("ADS1115 no encontrado");
  ads.setGain(GAIN_ONE);

  // BME280
  if (!bme.begin(0x76)) Serial.println("BME280 no encontrado");

  // SHT30
  if (!sht30.begin(0x44)) Serial.println("SHT30 no encontrado");

  // Pines
  pinMode(ENCODER_TRA,  INPUT_PULLUP);
  pinMode(ENCODER_TRB,  INPUT_PULLUP);
  pinMode(BTN_ENCODER,  INPUT_PULLUP);
  pinMode(BTN_BACK,     INPUT_PULLUP);
  pinMode(BTN_CONFIRM,  INPUT_PULLUP);
  pinMode(PULSE1_PIN,   INPUT);
  pinMode(PULSE2_PIN,   INPUT);
  pinMode(UART_RX_PIN,  INPUT);

  // ADC de bateria (GP1, ADC1): rango completo para leer el divisor
  analogSetPinAttenuation(BAT_ADC_PIN, ADC_11db);

  lastEncoded = (digitalRead(ENCODER_TRA) << 1) | digitalRead(ENCODER_TRB);
  attachInterrupt(digitalPinToInterrupt(ENCODER_TRA), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_TRB), encoderISR, CHANGE);

  // Pantalla bienvenida
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB10_tr);
  u8g2.drawStr(20, 28, "QC LemBot");
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(14, 46, "v0.2.1 ESP32-S3");
  u8g2.sendBuffer();
  rgbSet(0, 0, 25);              // azul = arrancando
  delay(2000);
  rgbSet(0, 0, 0);

  reallocArrays();
}

// ============================================
// MAIN LOOP
// ============================================
void loop() {
  handleEncoder();
  handleButtons();

  // Bateria cada 2s
  static unsigned long lastBatUpdate = 0;
  if (millis() - lastBatUpdate > 2000) {
    updateBattery();
    lastBatUpdate = millis();
  }

  // Corriente (estados que la necesitan)
  if (currentState == MENU_CURRENT_VIEW ||
      currentState == MENU_GRAPH_VIEW   ||
      currentState == MENU_TABLE_VIEW   ||
      currentState == MENU_RAW_DATA) {
    updateCurrentReadings();
    handleRecording();
  }

  // Modulos activos
  if (currentState == MENU_TESTER_SA)          updateSA();
  if (currentState == MENU_TESTER_WEATHER)     updateWX();
  if (currentState == MENU_TESTER_SOIL)        { /* se procesa en loop serial */ }
  if (currentState == MENU_BATTERY_DISCHARGE)  updateDischarge();
  if (currentState == MENU_UART)               updateUART();
  if (currentState == MENU_SDI12)              updateSDI12();

  // UART monitor - leer siempre si estamos en ese menu
  if (currentState == MENU_UART) updateUART();

  // Soil - leer serial siempre si estamos en ese menu
  if (currentState == MENU_TESTER_SOIL) {
    while (Serial.available()) {
      char c = (char)Serial.read();
      if (c == '\n') {
        parseSoilData(soilBuffer);
        soilBuffer = "";
      } else {
        soilBuffer += c;
      }
    }
  }

  updateStatusLED();
  updateDisplay();
  delay(10);
}

// ============================================
// ENCODER HANDLER
// ============================================
void handleEncoder() {
  if (encoderCounter == lastEncoderCounter) return;
  int diff = (encoderCounter - lastEncoderCounter) / 2;
  if (diff == 0) { lastEncoderCounter = encoderCounter; return; }

  if (currentState == MENU_MAIN) {
    menuSelection += diff;
    while (menuSelection < 0) menuSelection += menuItems;
    while (menuSelection >= menuItems) menuSelection -= menuItems;
    if (menuSelection < menuTopIndex) menuTopIndex = menuSelection;
    else if (menuSelection >= menuTopIndex + MENU_VISIBLE) menuTopIndex = menuSelection - MENU_VISIBLE + 1;
  }
  else if (currentState == MENU_TABLE_VIEW) {
    tableScrollOffset -= diff;
    if (tableScrollOffset < 0) tableScrollOffset = 0;
    if (tableScrollOffset >= sampleCount) tableScrollOffset = max(0, sampleCount - 1);
  }
  else if (currentState == MENU_RAW_DATA) {
    rawDataPage += diff;   // horario avanza a la siguiente pestana
    while (rawDataPage < 0)          rawDataPage += RAW_PAGES;
    while (rawDataPage >= RAW_PAGES) rawDataPage -= RAW_PAGES;
  }
  else if (currentState == MENU_CONFIG && !isEditingConfig) {
    configSelection += diff;
    while (configSelection < 0) configSelection += configItems;
    while (configSelection >= configItems) configSelection -= configItems;
    if (configSelection < configTopIndex) configTopIndex = configSelection;
    else if (configSelection >= configTopIndex + CONFIG_VISIBLE) configTopIndex = configSelection - CONFIG_VISIBLE + 1;
  }
  else if (currentState == MENU_CONFIG && isEditingConfig) {
    // Paso diferente segun tipo de variable
    float step = 0.1f;
    if (configSelection == 2) step = 0.5f;   // sample period
    if (configSelection == 3) step = 5.0f;   // max time
    if (configSelection == 5) step = 10.0f;  // max samples
    if (configSelection == 6) step = 100.0f; // bat capacity
    tempConfigValue += diff * step;
    if (tempConfigValue < 0.0f) tempConfigValue = 0.0f;
  }
  else if (currentState == MENU_PINOUT) {
    pinoutPage += diff;
    while (pinoutPage < 0) pinoutPage += 3;
    while (pinoutPage > 2) pinoutPage -= 3;
  }
  else if (currentState == MENU_SDI12 && sdi12State == SDI12_LIST) {
    int total = sdi12Count + 1; // +1 = opcion "Cambiar ID"
    sdi12Sel += diff;
    while (sdi12Sel < 0)       sdi12Sel += total;
    while (sdi12Sel >= total)  sdi12Sel -= total;
    if (sdi12Sel < sdi12TopIndex) sdi12TopIndex = sdi12Sel;
    else if (sdi12Sel >= sdi12TopIndex + SDI12_VISIBLE) sdi12TopIndex = sdi12Sel - SDI12_VISIBLE + 1;
  }
  else if (currentState == MENU_SDI12 && sdi12State == SDI12_CHG_SELSRC) {
    if (sdi12Count > 0) {
      sdi12ChgSrcSel += diff;
      while (sdi12ChgSrcSel < 0)          sdi12ChgSrcSel += sdi12Count;
      while (sdi12ChgSrcSel >= sdi12Count) sdi12ChgSrcSel -= sdi12Count;
      if (sdi12ChgSrcSel < sdi12TopIndex) sdi12TopIndex = sdi12ChgSrcSel;
      else if (sdi12ChgSrcSel >= sdi12TopIndex + SDI12_VISIBLE) sdi12TopIndex = sdi12ChgSrcSel - SDI12_VISIBLE + 1;
    }
  }
  else if (currentState == MENU_SDI12 && sdi12State == SDI12_NEWID) {
    // Solo se permite elegir IDs que NO esten en uso por sensores conectados
    int dir   = (diff > 0) ? 1 : -1;
    int steps = abs(diff);
    for (int s = 0; s < steps; s++) sdi12NewIdIndex = sdi12NextFreeIndex(sdi12NewIdIndex, dir);
  }
  else if (currentState == MENU_SDI12 && sdi12State == SDI12_SENSOR) {
    // Scroll por todas las filas: identificacion del sensor + valores
    int rows = sdi12ViewRows();
    if (rows > SDI12_DATA_VISIBLE) {
      sdi12DataScroll += diff;
      if (sdi12DataScroll < 0) sdi12DataScroll = 0;
      int maxOff = rows - SDI12_DATA_VISIBLE;
      if (sdi12DataScroll > maxOff) sdi12DataScroll = maxOff;
    }
  }

  lastEncoderCounter = encoderCounter;
}

// ============================================
// BUTTON HANDLERS
// ============================================
void handleButtons() {
  unsigned long now = millis();
  bool btnEnc  = digitalRead(BTN_ENCODER);
  bool btnBack = digitalRead(BTN_BACK);
  bool btnConf = digitalRead(BTN_CONFIRM);

  if (btnEnc  == LOW && lastBtnEncoderState == HIGH && now - lastDebounceTime > DEBOUNCE_DELAY) {
    handleEncoderButton(); lastDebounceTime = now;
  }
  if (btnBack == LOW && lastBtnBackState    == HIGH && now - lastDebounceTime > DEBOUNCE_DELAY) {
    handleBackButton();    lastDebounceTime = now;
  }
  if (btnConf == LOW && lastBtnConfirmState == HIGH && now - lastDebounceTime > DEBOUNCE_DELAY) {
    handleConfirmButton(); lastDebounceTime = now;
  }

  lastBtnEncoderState = btnEnc;
  lastBtnBackState    = btnBack;
  lastBtnConfirmState = btnConf;
}

void handleEncoderButton() {
  if (currentState == MENU_MAIN) {
    selectMenuItem();
  } else if (currentState == MENU_CONFIG && isEditingConfig) {
    // Guardar valor editado
    applyConfigValue();
    isEditingConfig = false;
  } else if (currentState == MENU_SDI12 && sdi12State == SDI12_SENSOR) {
    // En la pantalla de datos: el click re-pide los valores con aM!
    // (algunos sensores devuelven mas valores con aM! que con aC!)
    sdi12UseConcurrent = false;
    sdi12NeedMeasure   = true;
  } else {
    // Desde cualquier pantalla: volver al menu principal
    if (currentState == MENU_SDI12) exitSDI12();
    currentState     = MENU_MAIN;
    menuSelection    = 0;
    menuTopIndex     = 0;
    isEditingConfig  = false;
    configSelection  = 0;
    configTopIndex   = 0;
    // Resetear estados de testers
    testerStateSA = SA_IDLE;
    testerStateWX = WX_IDLE;
    dischState    = DISCH_IDLE;
  }
}

void handleBackButton() {
  if (currentState == MENU_TABLE_VIEW) {
    resetRecording();
    currentState     = MENU_CURRENT_VIEW;
    currentViewPage  = 0;
  }
  else if (currentState == MENU_GRAPH_VIEW) {
    currentState     = MENU_CURRENT_VIEW;
    currentViewPage  = 0;
  }
  else if (currentState == MENU_CURRENT_VIEW) {
    if (currentViewPage == 1) {
      currentViewPage = 0;                 // pag2 -> pag1
    } else {
      currentState  = MENU_MAIN;           // pag1 -> menu principal
      menuSelection = 0;
      menuTopIndex  = 0;
    }
  }
  else if (currentState == MENU_CONFIG) {
    if (isEditingConfig) {
      // Cancelar edicion: restaurar default de esa variable
      restoreOneDefault(configSelection);
      isEditingConfig = false;
    } else if (configSelection == configItems - 1) {
      // Ultimo item = restaurar TODOS los defaults
      restoreDefaults();
    } else {
      currentState    = MENU_MAIN;
      configSelection = 0;
      configTopIndex  = 0;
    }
  }
  else if (currentState == MENU_TESTER_SA) {
    testerStateSA = SA_IDLE;
    currentState  = MENU_MAIN;
  }
  else if (currentState == MENU_TESTER_WEATHER) {
    testerStateWX = WX_IDLE;
    currentState  = MENU_MAIN;
  }
  else if (currentState == MENU_BATTERY_DISCHARGE) {
    if (dischState == DISCH_RUNNING) {
      dischState = DISCH_DONE; // parar
    } else {
      dischState   = DISCH_IDLE;
      currentState = MENU_MAIN;
    }
  }
  else if (currentState == MENU_SDI12) {
    switch (sdi12State) {
      case SDI12_SENSOR:                       // datos -> lista
        sdi12State    = SDI12_LIST;
        break;
      case SDI12_NEWID:                        // nuevo id -> elegir origen
        sdi12State    = SDI12_CHG_SELSRC;
        sdi12TopIndex = 0;
        break;
      case SDI12_CHG_SELSRC:                   // elegir origen -> lista
        sdi12State    = SDI12_LIST;
        sdi12TopIndex = 0;
        break;
      case SDI12_LIST:                         // lista -> inicial
      case SDI12_SCANNING:
      case SDI12_CHGRESULT:
        sdi12State    = SDI12_IDLE;
        break;
      case SDI12_IDLE:                         // inicial -> menu principal
      default:
        exitSDI12();
        currentState  = MENU_MAIN;
        menuSelection = 0;
        menuTopIndex  = 0;
        break;
    }
  }
  else if (currentState != MENU_MAIN) {
    currentState    = MENU_MAIN;
    menuSelection   = 0;
    menuTopIndex    = 0;
    isEditingConfig = false;
  }
}

void handleConfirmButton() {
  if (currentState == MENU_MAIN) {
    selectMenuItem();
  }
  else if (currentState == MENU_CURRENT_VIEW) {
    if (currentViewPage == 0) {
      currentViewPage = 1;                   // pag1 -> pag2
    } else {
      currentViewPage = 0;
      currentState    = MENU_GRAPH_VIEW;     // pag2 -> grafico
    }
  }
  else if (currentState == MENU_GRAPH_VIEW) {
    currentState      = MENU_TABLE_VIEW;     // grafico -> tabla
    tableScrollOffset = 0;
  }
  else if (currentState == MENU_TABLE_VIEW) {
    currentState    = MENU_CURRENT_VIEW;     // tabla -> pag1 (cicla)
    currentViewPage = 0;
  }
  else if (currentState == MENU_RAW_DATA) {
    rawDataPage = (rawDataPage + 1) % RAW_PAGES;
  }
  else if (currentState == MENU_PINOUT) {
    pinoutPage = (pinoutPage + 1) % 3;
  }
  else if (currentState == MENU_CONFIG && !isEditingConfig) {
    if (configSelection == configItems - 1) {
      // Ultimo item = restaurar todos
      restoreDefaults();
    } else {
      isEditingConfig   = true;
      loadTempConfigValue();
    }
  }
  else if (currentState == MENU_CONFIG && isEditingConfig) {
    applyConfigValue();
    isEditingConfig = false;
  }
  else if (currentState == MENU_TESTER_SA) {
    if (testerStateSA == SA_IDLE || testerStateSA == SA_RESULT) {
      // Iniciar nuevo test
      testerStateSA = SA_TESTING;
      sa_testStart  = millis();
      sa_passed     = false;
      sa_failReason = "";
    }
  }
  else if (currentState == MENU_TESTER_WEATHER) {
    if (testerStateWX == WX_IDLE || testerStateWX == WX_RESULT) {
      testerStateWX = WX_TESTING;
      sa_testStart  = millis();
      wx_passed     = false;
      wx_failReason = "";
    }
  }
  else if (currentState == MENU_BATTERY_DISCHARGE) {
    if (dischState == DISCH_IDLE) {
      dischState       = DISCH_RUNNING;
      disch_startTime  = millis();
      disch_lastSample = millis();
      disch_mAh        = 0;
      disch_mWh        = 0;
      disch_peakCurrent= 0;
      ina228.resetAccumulators();
    } else if (dischState == DISCH_DONE) {
      dischState = DISCH_IDLE;
    }
  }
  else if (currentState == MENU_SDI12) {
    switch (sdi12State) {
      case SDI12_IDLE:
        // Iniciar escaneo de todas las direcciones
        sdi12Count     = 0;
        sdi12ScanIndex = 0;
        sdi12Sel       = 0;
        sdi12TopIndex  = 0;
        sdi12State     = SDI12_SCANNING;
        break;
      case SDI12_LIST:
        if (sdi12Sel < sdi12Count) {
          // Sensor seleccionado -> ver sus datos (lectura normal: aC!)
          sdi12CurAddr       = sdi12Found[sdi12Sel];
          sdi12UseConcurrent = true;
          sdi12NeedMeasure   = true;
          sdi12State         = SDI12_SENSOR;
        } else {
          // Ultima opcion = "Cambiar ID" -> elegir sensor origen
          sdi12ChgSrcSel = 0;
          sdi12TopIndex  = 0;
          sdi12State     = SDI12_CHG_SELSRC;
        }
        break;
      case SDI12_CHG_SELSRC:
        if (sdi12Count > 0) {
          sdi12ChgSrc     = sdi12Found[sdi12ChgSrcSel];
          // arrancar en el primer ID libre (no usado por sensores conectados)
          sdi12NewIdIndex = sdi12AddrUsed(sdi12AddrAt(0)) ? sdi12NextFreeIndex(0, 1) : 0;
          sdi12State      = SDI12_NEWID;
        }
        break;
      case SDI12_SENSOR:
        sdi12UseConcurrent = true;  // OK re-pide con aC!
        sdi12NeedMeasure   = true;
        break;
      case SDI12_NEWID:
        // Confirmar nuevo ID -> ejecutar cambio
        sdi12State = SDI12_CHANGING;
        break;
      case SDI12_CHGRESULT:
        sdi12State = SDI12_IDLE;   // volver a la vista inicial
        break;
      default:
        break;
    }
  }
}

void selectMenuItem() {
  switch (menuSelection) {
    case 0: currentState = MENU_CURRENT_VIEW;         resetRecording(); break;
    case 1: currentState = MENU_TESTER_SA;            testerStateSA = SA_IDLE; break;
    case 2: currentState = MENU_TESTER_WEATHER;       testerStateWX = WX_IDLE; break;
    case 3: currentState = MENU_TESTER_SOIL;          soilBuffer = ""; break;
    case 4: currentState = MENU_RAW_DATA;             rawDataPage = 0; break;
    case 5: currentState = MENU_TEMP_HUM;             break;
    case 6: currentState = MENU_UART;                 break;
    case 7: currentState = MENU_SDI12;                sdi12State = SDI12_IDLE;
            sdi12Begin(); break;
    case 8: currentState = MENU_ZIGBEE;               break;
    case 9: currentState = MENU_BATTERY_DISCHARGE;    dischState = DISCH_IDLE; break;
    case 10: currentState = MENU_CONFIG;              configSelection = 0; configTopIndex = 0; break;
    case 11: currentState = MENU_PINOUT;              pinoutPage = 0; break;
  }
}

// ============================================
// CONFIG HELPERS
// ============================================
void loadTempConfigValue() {
  switch (configSelection) {
    case  0: tempConfigValue = CURRENT_START_THRESHOLD; break;
    case  1: tempConfigValue = CURRENT_END_THRESHOLD;   break;
    case  2: tempConfigValue = SAMPLE_PERIOD / 1000.0f; break;
    case  3: tempConfigValue = MAX_RECORD_TIME / 1000.0f; break;
    case  4: tempConfigValue = OC_THRESHOLD;            break;
    case  5: tempConfigValue = (float)MAX_SAMPLES_CONFIG; break;
    case  6: tempConfigValue = BAT_CAPACITY_MAH;        break;
    case  7: tempConfigValue = SA_VOLT_REF;             break;
    case  8: tempConfigValue = SA_VOLT_TOL * 100.0f;    break;
    case  9: tempConfigValue = SA_CURR_MIN;             break;
    case 10: tempConfigValue = SA_CURR_MAX;             break;
    case 11: tempConfigValue = DISCH_CURR_LIMIT;        break;
    case 12: tempConfigValue = 0; break; // Reset all (marcador)
  }
}

void applyConfigValue() {
  switch (configSelection) {
    case  0: CURRENT_START_THRESHOLD = tempConfigValue; break;
    case  1: CURRENT_END_THRESHOLD   = tempConfigValue; break;
    case  2: SAMPLE_PERIOD     = (unsigned long)(tempConfigValue * 1000); break;
    case  3: MAX_RECORD_TIME   = (unsigned long)(tempConfigValue * 1000); break;
    case  4: OC_THRESHOLD      = tempConfigValue; break;
    case  5: MAX_SAMPLES_CONFIG = (int)tempConfigValue; reallocArrays(); break;
    case  6: BAT_CAPACITY_MAH  = tempConfigValue; break;
    case  7: SA_VOLT_REF       = tempConfigValue; break;
    case  8: SA_VOLT_TOL       = tempConfigValue / 100.0f; break;
    case  9: SA_CURR_MIN       = tempConfigValue; break;
    case 10: SA_CURR_MAX       = tempConfigValue; break;
    case 11: DISCH_CURR_LIMIT  = tempConfigValue; break;
    case 12: restoreDefaults(); return;
  }
  saveConfig();
}

void restoreOneDefault(int idx) {
  switch (idx) {
    case  0: CURRENT_START_THRESHOLD = DEF_START_THRESH;  break;
    case  1: CURRENT_END_THRESHOLD   = DEF_END_THRESH;    break;
    case  2: SAMPLE_PERIOD           = DEF_SAMPLE;        break;
    case  3: MAX_RECORD_TIME         = DEF_MAX_TIME;      break;
    case  4: OC_THRESHOLD            = DEF_OC_THRESH;     break;
    case  5: MAX_SAMPLES_CONFIG      = DEF_MAX_SAMPLES; reallocArrays(); break;
    case  6: BAT_CAPACITY_MAH        = DEF_BAT_CAPACITY;  break;
    case  7: SA_VOLT_REF             = DEF_SA_VOLT_REF;   break;
    case  8: SA_VOLT_TOL             = DEF_SA_VOLT_TOL;   break;
    case  9: SA_CURR_MIN             = DEF_SA_CURR_MIN;   break;
    case 10: SA_CURR_MAX             = DEF_SA_CURR_MAX;   break;
    case 11: DISCH_CURR_LIMIT        = DEF_DISCH_CURR;    break;
    case 12: restoreDefaults(); break;
  }
  saveConfig();
}

// ============================================
// EEPROM
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

// ============================================
// MEMORY MANAGEMENT
// ============================================
void reallocArrays() {
  if (recordedCurrents) { delete[] recordedCurrents; delete[] recordedTimes; delete[] recordedMAh; }
  MAX_SAMPLES = MAX_SAMPLES_CONFIG;
  recordedCurrents = new float[MAX_SAMPLES];
  recordedTimes    = new unsigned long[MAX_SAMPLES];
  recordedMAh      = new float[MAX_SAMPLES];
  for (int i = 0; i < MAX_SAMPLES; i++) { recordedCurrents[i] = 0; recordedTimes[i] = 0; recordedMAh[i] = 0; }
}

// ============================================
// CURRENT MONITORING
// ============================================
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
  currentViewPage   = 0;
  estMAh            = 0;
  estStartTime      = millis();
  estLastTime       = 0;
  ina228.resetAccumulators();
}

// ============================================
// BATTERY UPDATE
// ============================================
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

void updateBattery() {
  // ADC interno del S3 en GP1 (ADC1_CH0), calibrado de fabrica. ADC1 sigue
  // disponible con WiFi/BT activos. Divisor 2:1.
  uint32_t mv = analogReadMilliVolts(BAT_ADC_PIN);
  batteryVoltage = (mv / 1000.0f) * BAT_DIVIDER;

  batteryPercent = batPercentFromV(batteryVoltage);

  // Parpadea el icono cuando la bateria esta baja
  if (batteryPercent < 20 && millis() - lastBatteryBlink > 500) {
    batteryBlink = !batteryBlink;
    lastBatteryBlink = millis();
  } else if (batteryPercent >= 20) {
    batteryBlink = true;
  }
}

// ============================================
// SENSE-QC UPDATE
// ============================================
void updateSA() {
  if (testerStateSA != SA_TESTING) return;

  unsigned long elapsed = millis() - sa_testStart;

  // Leer valores actuales
  sa_current = ina228.getCurrent_mA();
  sa_a0v     = ads.readADC_SingleEnded(0) * 0.000125f;
  sa_a1v     = ads.readADC_SingleEnded(1) * 0.000125f;

  bool pulse1 = digitalRead(PULSE1_PIN);
  bool pulse2 = digitalRead(PULSE2_PIN);

  // Acumular deteccion de pulsos durante SA_TEST_DURATION
  static bool p1_high_seen = false, p1_low_seen = false;
  static bool p2_high_seen = false, p2_low_seen = false;

  if (elapsed == 0) { p1_high_seen = p1_low_seen = p2_high_seen = p2_low_seen = false; }

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

// ============================================
// WEATHER-QC UPDATE
// ============================================
void updateWX() {
  if (testerStateWX != WX_TESTING) return;

  unsigned long elapsed = millis() - sa_testStart;

  wx_current  = ina228.getCurrent_mA();
  wx_bme_temp = bme.readTemperature();
  wx_bme_hum  = bme.readHumidity();

  // Intentar SHT10 primero, luego SHT30
  wx_sht_temp = sht10.readTemperatureC();
  wx_sht_hum  = sht10.readHumidity();

  bool sht10valid = !isnan(wx_sht_temp) && !isnan(wx_sht_hum);

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

// ============================================
// SOIL PARSER (DFM por Serial)
// Formato esperado: "H:xx.x,xx.x,...(12 valores);T:xx.x,...(13 valores)"
// Ajustar segun protocolo real del sensor DFM
// ============================================
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

// ============================================
// UART MONITOR UPDATE
// ============================================
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

// ============================================
// SDI-12 VIEWER / EDITOR
// ============================================

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

// Libera los pines (vuelven a entrada) para su uso normal.
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

// Esta la direccion en la lista de sensores conectados (segun el ultimo escaneo)?
bool sdi12AddrUsed(char addr) {
  for (int i = 0; i < sdi12Count; i++) if (sdi12Found[i] == addr) return true;
  return false;
}

// Siguiente indice de direccion LIBRE (no usada) en el sentido dir (+1/-1)
int sdi12NextFreeIndex(int from, int dir) {
  int idx = from;
  for (int k = 0; k < SDI12_NUM_ADDR; k++) {
    idx += dir;
    while (idx < 0)               idx += SDI12_NUM_ADDR;
    while (idx >= SDI12_NUM_ADDR) idx -= SDI12_NUM_ADDR;
    if (!sdi12AddrUsed(sdi12AddrAt(idx))) return idx;
  }
  return from;  // todas usadas (no deberia ocurrir)
}

// Un paso del escaneo incremental (una direccion por llamada)
void sdi12DoScanStep() {
  if (sdi12ScanIndex >= SDI12_NUM_ADDR) {
    sdi12State    = SDI12_LIST;
    sdi12Sel      = 0;
    sdi12TopIndex = 0;
    return;
  }
  char addr = sdi12AddrAt(sdi12ScanIndex);
  if (sdi12Probe(addr) && sdi12Count < SDI12_NUM_ADDR) {
    sdi12Found[sdi12Count++] = addr;
  }
  sdi12ScanIndex++;
  if (sdi12ScanIndex >= SDI12_NUM_ADDR) {
    sdi12State    = SDI12_LIST;
    sdi12Sel      = 0;
    sdi12TopIndex = 0;
  }
}

// Parsea los valores de una respuesta D ("a+1.2-3.4+5...") separados por + o -.
// Los agrega a sdi12Values y devuelve cuantos agrego.
int sdi12ParseValues(const String& dr) {
  if (dr.length() <= 1) return 0;
  int added = 0;
  String body = dr.substring(1);   // quitar la direccion inicial
  String num = "";
  for (int i = 0; i < (int)body.length(); i++) {
    char c = body.charAt(i);
    if ((c == '+' || c == '-') && num.length() > 0) {
      if (sdi12NumValues < SDI12_MAX_VALUES) { sdi12Values[sdi12NumValues++] = num.toFloat(); added++; }
      num = ""; num += c;
    } else {
      num += c;
    }
  }
  if (num.length() > 0 && sdi12NumValues < SDI12_MAX_VALUES) {
    sdi12Values[sdi12NumValues++] = num.toFloat(); added++;
  }
  return added;
}

// Mide un sensor y lee TODOS sus valores (aD0!..aD9!).
// concurrent=true  -> usa aC! (cuenta hasta 2 digitos; fallback a aM!)
// concurrent=false -> fuerza aM! (algunos sensores dan mas valores asi)
void sdi12DoMeasure(char addr, bool concurrent) {
  sdi12IdStr     = "";
  sdi12InfoCount = 0;
  sdi12WaitSec   = 0;
  sdi12NumValues = 0;
  for (int i = 0; i < SDI12_MAX_VALUES; i++) sdi12Values[i] = 0;

  // Identificacion
  String idr = sdi12Command(String(addr) + "I!", 400);
  if (idr.length() > 1) sdi12IdStr = idr.substring(1);

  // Comando de medicion segun el modo pedido
  int   waitSec = 0;
  bool  usedC   = false;
  String mr;
  if (concurrent) {
    mr = sdi12Command(String(addr) + "C!", 400);          // "atttnn"
    if (mr.length() >= 6 && mr.charAt(0) == addr) usedC = true;
    else mr = sdi12Command(String(addr) + "M!", 400);     // fallback si no hay aC!
  } else {
    mr = sdi12Command(String(addr) + "M!", 400);          // "atttn"
  }
  sdi12MeasMode = usedC ? 'C' : 'M';
  if (mr.length() >= 5 && mr.charAt(0) == addr) waitSec = mr.substring(1, 4).toInt();
  sdi12WaitSec = waitSec;

  // Esperar a que el sensor complete la medicion (tiempo indicado, acotado)
  unsigned long limit = (unsigned long)waitSec * 1000UL;
  if (limit > 9000UL) limit = 9000UL;
  delay(limit + 150);

  // Leer aD0!..aD9! acumulando hasta que no haya mas datos (sin cortar por cantidad)
  for (int d = 0; d <= 9; d++) {
    String dr = sdi12Command(String(addr) + "D" + String(d) + "!", 900);
    if (dr.length() <= 1 || dr.charAt(0) != addr) break;   // sin mas datos
    int added = sdi12ParseValues(dr);
    if (added == 0) break;
    if (sdi12NumValues >= SDI12_MAX_VALUES) break;
  }
  sdi12BuildInfoRows();
}

// Arma las filas de identificacion a partir de la respuesta de aI!.
// Formato SDI-12: ll cccccccc mmmmmm vvv xxx..  (sin la direccion)
//   ll = version SDI-12, c = fabricante, m = modelo, v = version, x = serie/extra
void sdi12BuildInfoRows() {
  sdi12InfoCount = 0;
  const String& s = sdi12IdStr;
  if (s.length() == 0) return;
  String ver = s.substring(0, 2);   ver.trim();
  String fab = s.substring(2, 10);  fab.trim();
  String mod = s.substring(10, 16); mod.trim();
  String sv  = s.substring(16, 19); sv.trim();
  String sn  = s.substring(19);     sn.trim();
  if (fab.length() > 0) sdi12InfoRow[sdi12InfoCount++] = "Fab: " + fab;
  if (mod.length() > 0) {
    String m = "Mod: " + mod;
    if (sv.length() > 0) m += " v" + sv;
    sdi12InfoRow[sdi12InfoCount++] = m;
  }
  if (sn.length() > 0) sdi12InfoRow[sdi12InfoCount++] = "SN: " + sn;
  if (ver.length() == 2) {
    String v = "SDI-12 v";
    v += ver.charAt(0); v += '.'; v += ver.charAt(1);
    sdi12InfoRow[sdi12InfoCount++] = v;
  }
}

// Filas de la vista: identificacion + 1 fila de resumen + valores
int sdi12ViewRows() {
  return sdi12InfoCount + 1 + sdi12NumValues;
}

void sdi12ViewRow(int idx, char* out, size_t n) {
  if (idx < sdi12InfoCount) {
    snprintf(out, n, "%s", sdi12InfoRow[idx].c_str());
  } else if (idx == sdi12InfoCount) {
    snprintf(out, n, "Datos a%c!: %d val %ds", sdi12MeasMode, sdi12NumValues, sdi12WaitSec);
  } else {
    snprintf(out, n, "- %.2f", sdi12Values[idx - sdi12InfoCount - 1]);
  }
}

// Cambia el ID del sensor 'oldAddr' a 'newAddr' (multi-sensor) y verifica.
// Funciona con varios sensores en el bus: el comando aAb! va dirigido a oldAddr.
void sdi12DoChangeId(char oldAddr, char newAddr) {
  sdi12ChgOk  = false;
  sdi12ChgMsg = "";

  if (oldAddr == newAddr) {
    sdi12ChgOk  = true;
    sdi12ChgMsg = String("Ya era ") + newAddr;
    return;
  }

  // 1) El sensor origen debe seguir respondiendo
  if (!sdi12Probe(oldAddr)) {
    sdi12ChgMsg = String("Sin sensor en ") + oldAddr;
    return;
  }

  // 2) El destino debe estar libre (ningun otro sensor en esa direccion)
  if (sdi12Probe(newAddr)) {
    sdi12ChgMsg = String(newAddr) + " ya en uso";
    return;
  }

  // 3) Cambiar direccion: comando "aAb!" -> responde "b"
  String cr = sdi12Command(String(oldAddr) + "A" + String(newAddr) + "!", 400);

  // 4) Verificar: la nueva responde y la vieja ya no
  bool newOk   = sdi12Probe(newAddr);
  bool oldGone = !sdi12Probe(oldAddr);

  if (newOk && oldGone) {
    sdi12ChgOk  = true;
    sdi12ChgMsg = String(oldAddr) + " -> " + String(newAddr);
  } else if (newOk) {
    sdi12ChgOk  = true;   // la nueva responde (la vieja puede tardar en liberarse)
    sdi12ChgMsg = String("OK en ") + newAddr;
  } else {
    sdi12ChgOk  = false;
    sdi12ChgMsg = "No se pudo escribir";
  }
  (void)cr;
}

// Maquina de estados del modulo SDI-12 (llamada desde loop)
void updateSDI12() {
  switch (sdi12State) {
    case SDI12_SCANNING:
      sdi12DoScanStep();
      break;
    case SDI12_SENSOR:
      if (sdi12NeedMeasure) {
        sdi12ShowBusy(sdi12UseConcurrent ? "Midiendo aC!..." : "Midiendo aM!...");
        sdi12DoMeasure(sdi12CurAddr, sdi12UseConcurrent);
        sdi12DataScroll  = 0;
        sdi12NeedMeasure = false;
      }
      break;
    case SDI12_CHANGING:
      sdi12ShowBusy("Cambiando ID...");
      sdi12DoChangeId(sdi12ChgSrc, sdi12AddrAt(sdi12NewIdIndex));
      sdi12State = SDI12_CHGRESULT;
      break;
    default:
      break;
  }
}

// ============================================
// BATTERY DISCHARGE UPDATE
// ============================================
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
// LED RGB (WS2812 onboard) - DEBUG VISUAL
// ============================================
// Colores tenues para no encandilar. Solo se reescribe si cambia.
void rgbSet(uint8_t r, uint8_t g, uint8_t b) {
  rgbLedWrite(RGB_LED_PIN, r, g, b);   // API del core arduino-esp32 v3.x
}

void updateStatusLED() {
  static unsigned long lastBlink = 0;
  static bool blinkOn = false;
  if (millis() - lastBlink > 400) { blinkOn = !blinkOn; lastBlink = millis(); }

  uint8_t r = 0, g = 0, b = 0;   // por defecto apagado (menu principal, etc.)

  switch (currentState) {
    case MENU_CURRENT_VIEW:
    case MENU_GRAPH_VIEW:
    case MENU_TABLE_VIEW:
      if (currentCurrent > OC_THRESHOLD)      { r = 60; }                 // OC: rojo fuerte
      else if (isRecording)                   { r = blinkOn ? 25 : 0; }   // REC: rojo parpadeo
      else if (recordingComplete)             { g = 10; }                 // done: verde tenue
      break;
    case MENU_TESTER_SA:
      if      (testerStateSA == SA_TESTING)   { b = 30; }
      else if (testerStateSA == SA_RESULT)    { if (sa_passed) g = 30; else r = 30; }
      break;
    case MENU_TESTER_WEATHER:
      if      (testerStateWX == WX_TESTING)   { b = 30; }
      else if (testerStateWX == WX_RESULT)    { if (wx_passed) g = 30; else r = 30; }
      break;
    case MENU_SDI12:
      switch (sdi12State) {
        case SDI12_SCANNING:  b = blinkOn ? 30 : 5; break;                // escaneando
        case SDI12_SENSOR:    if (sdi12NumValues > 0) g = 12; else r = 12; break;
        case SDI12_CHGRESULT: if (sdi12ChgOk) g = 30; else r = 30; break;
        default:              b = 6; break;                               // modulo activo
      }
      break;
    case MENU_BATTERY_DISCHARGE:
      if      (dischState == DISCH_RUNNING)   { r = 20; g = 12; }         // ambar
      else if (dischState == DISCH_DONE)      { g = 20; }
      break;
    case MENU_ZIGBEE:
      r = 12; b = 12;                                                     // morado = WIP
      break;
    default:
      break;
  }

  // Evitar reescribir el WS2812 en cada pasada del loop
  static uint8_t lr = 255, lg = 255, lb = 255;
  if (r != lr || g != lg || b != lb) { rgbSet(r, g, b); lr = r; lg = g; lb = b; }
}

// ============================================
// DISPLAY
// ============================================
void updateDisplay() {
  u8g2.clearBuffer();
  switch (currentState) {
    case MENU_MAIN:              drawMainMenu();         break;
    case MENU_CURRENT_VIEW:      drawCurrentView();      break;
    case MENU_GRAPH_VIEW:        drawGraphView();        break;
    case MENU_TABLE_VIEW:        drawTableView();        break;
    case MENU_TESTER_SA:         drawSenseQC();          break;
    case MENU_TESTER_WEATHER:    drawWeatherQC();        break;
    case MENU_TESTER_SOIL:       drawSoilQC();           break;
    case MENU_RAW_DATA:          drawRawDataView();      break;
    case MENU_TEMP_HUM:          drawTempHumView();      break;
    case MENU_UART:              drawUARTView();         break;
    case MENU_SDI12:             drawSDI12View();        break;
    case MENU_ZIGBEE:            drawZigbeeView();       break;
    case MENU_BATTERY_DISCHARGE: drawBatteryDischarge(); break;
    case MENU_CONFIG:            drawConfigView();       break;
    case MENU_PINOUT:            drawPinoutView();       break;
  }
  u8g2.sendBuffer();
}

// -------- HELPERS UI --------
void drawBatteryIcon(int x, int y, int percent) {
  u8g2.drawFrame(x, y, 12, 6);
  u8g2.drawBox(x + 12, y + 1, 2, 4);
  int fw = (percent * 10) / 100;
  if (fw > 0) u8g2.drawBox(x + 1, y + 1, fw, 4);
  char buf[5];
  sprintf(buf, "%d%%", percent);
  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.drawStr(x + 16, y + 5, buf);
}

void drawScrollBar(int x, int y, int h, int cur, int total) {
  u8g2.drawFrame(x, y, 3, h);
  if (total <= 1) return;
  int barH = max(4, (h * MENU_VISIBLE) / total);
  if (barH > h - 2) barH = h - 2;
  int barY = y + 1 + ((h - 2 - barH) * cur) / (total - 1);
  u8g2.drawBox(x + 1, barY, 1, barH);
}

// -------- MENU PRINCIPAL --------
void drawMainMenu() {
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(5, 10, "QC LemBot");
  if (batteryBlink) drawBatteryIcon(92, 2, batteryPercent);
  u8g2.drawLine(0, 12, 128, 12);

  const char* menuTexts[] = {
    "1: Corriente",
    "2: Sense-QC",
    "3: Weather-QC",
    "4: Soil-QC",
    "5: Datos RAW",
    "6: Temp & Hum",
    "7: UART Monitor",
    "8: SDI-12",
    "9: Zigbee",
    "10: Descarga Bat",
    "11: Config",
    "12: Pinout"
  };

  u8g2.setFont(u8g2_font_7x13_tf);
  for (int i = 0; i < MENU_VISIBLE && (i + menuTopIndex) < menuItems; i++) {
    int idx  = i + menuTopIndex;
    int yPos = 24 + (i * 11);
    if (idx == menuSelection) u8g2.drawStr(3, yPos, ">");
    u8g2.drawStr(12, yPos, menuTexts[idx]);
  }

  drawScrollBar(122, 14, 48, menuSelection, menuItems);

  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.drawStr(5, 63, "Gira:Nav  OK:Sel");
}

// Dibuja una fila grande de la vista de corriente: numero grande + etiqueta.
// El numero se alinea a la izquierda y la etiqueta se coloca justo despues
// (medida con getStrWidth) para que nunca se solapen.
void drawCurRow(int y, bool valid, const char* fmt, float val, const char* label) {
  char b[16];
  if (valid) {
    u8g2.setFont(u8g2_font_oldwizard_tn);
    sprintf(b, fmt, val);
  } else {
    strcpy(b, "--");
    u8g2.setFont(u8g2_font_ncenB12_tr);
  }
  u8g2.drawStr(2, y, b);
  int nw = u8g2.getStrWidth(b);
  u8g2.setFont(u8g2_font_ncenB12_tr);
  u8g2.drawStr(2 + nw + 3, y, label);
}

// -------- VISTA CORRIENTE (2 paginas) --------
void drawCurrentView() {
  float median = 0, avg = 0;
  bool  statsOk = false;
  computeCurrentStats(median, avg, statsOk);

  // Baselines de las 4 filas grandes (dejan libre la fila inferior y=63)
  const int Y1 = 12, Y2 = 27, Y3 = 42, Y4 = 57;

  if (currentViewPage == 0) {
    // Pagina 1: corriente actual / mediana / maxima / minima>0
    drawCurRow(Y1, true,                       "%.3f", currentCurrent, "mA");
    drawCurRow(Y2, statsOk,                     "%.2f", median,         "med");
    drawCurRow(Y3, (max_current_mA > 0.0f),     "%.2f", max_current_mA, "max");
    drawCurRow(Y4, (min_current_mA > 0.0f),     "%.3f", min_current_mA, "min");
  } else {
    // Pagina 2: voltaje / mAh / promedio / estimado de uso
    drawCurRow(Y1, true,    "%.3f", currentVoltage, "V");
    drawCurRow(Y2, true,    "%.2f", estMAh,         "mAh");
    drawCurRow(Y3, statsOk, "%.2f", avg,            "prom");

    // Estimado de tiempo de uso (capacidad EEPROM / consumo medido)
    unsigned long el = millis() - estStartTime;
    bool  estValid   = false;
    float estTotalH  = 0;
    if (estMAh > 0.002f && el > 5000) {
      float hoursElapsed = el / 3600000.0f;
      float rate = estMAh / hoursElapsed;         // mAh por hora
      if (rate > 0.0001f) { estTotalH = BAT_CAPACITY_MAH / rate; estValid = true; }
    }
    if (!estValid) {
      drawCurRow(Y4, false, "%.0f", 0.0f, "uso");
    } else if (estTotalH < 100.0f) {
      drawCurRow(Y4, true, "%.1f", estTotalH,        "h uso");
    } else {
      drawCurRow(Y4, true, "%.0f", estTotalH / 24.0f, "d uso");
    }
  }

  // -------- Fila inferior: navegacion (izq) + estado (der) --------
  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.drawStr(2, 63, currentViewPage == 0 ? "OK>pag2 BK>menu" : "OK>graf BK>pag1");

  char sb[12];
  if (currentCurrent > OC_THRESHOLD) {
    u8g2.drawStr(102, 63, "!!OC");
  } else if (isRecording) {
    if (endingRecording) {
      unsigned long rem = (END_DELAY > (millis() - endRecordingTime)) ? END_DELAY - (millis() - endRecordingTime) : 0;
      sprintf(sb, "fin%lus", rem / 1000);
      u8g2.drawStr(98, 63, sb);
    } else {
      u8g2.drawStr(104, 63, "REC");
      u8g2.drawDisc(101, 61, 1);
    }
  } else if (recordingComplete) {
    sprintf(sb, "%dsmp", sampleCount);
    u8g2.drawStr(100, 63, sb);
  }
}

// -------- GRAFICO --------
void drawGraphView() {
  if (sampleCount < 2) {
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(20, 35, "Sin datos aun");
    u8g2.setFont(u8g2_font_4x6_tf);
    u8g2.drawStr(0, 63, "OK:Tabla  Enc:Menu");
    return;
  }

  float maxVal = recordedCurrents[0], minVal = recordedCurrents[0], sum = 0;
  for (int i = 0; i < sampleCount; i++) {
    if (recordedCurrents[i] > maxVal) maxVal = recordedCurrents[i];
    if (recordedCurrents[i] < minVal) minVal = recordedCurrents[i];
    sum += recordedCurrents[i];
  }
  float mean  = sum / sampleCount;
  float range = maxVal - minVal;
  if (range < 0.1f) range = 0.1f;

  // Nivel sostenido (tiempo acumulado por bin)
  const int BINS = 40;
  int binTime[BINS] = {0};
  float binSize = range / BINS;
  if (binSize < 0.01f) binSize = 0.01f;
  int curBin = -1, curCons = 0;
  for (int i = 0; i < sampleCount; i++) {
    int b = constrain((int)((recordedCurrents[i] - minVal) / binSize), 0, BINS-1);
    if (b == curBin) curCons++; else { if (curBin >= 0) binTime[curBin] += curCons; curBin = b; curCons = 1; }
  }
  if (curBin >= 0) binTime[curBin] += curCons;
  int maxBT = 0, sustBin = 0;
  for (int i = 0; i < BINS; i++) if (binTime[i] > maxBT) { maxBT = binTime[i]; sustBin = i; }
  float sustained = minVal + (sustBin * binSize) + (binSize / 2);

  int gH = 50, gY = 58, gX = 25, gW = 100;
  u8g2.drawFrame(gX, gY - gH, gW, gH);

  for (int i = 1; i < sampleCount; i++) {
    int x1 = gX + ((i-1) * gW / (sampleCount-1));
    int y1 = gY - (int)((recordedCurrents[i-1] - minVal) * gH / range);
    int x2 = gX + (i   * gW / (sampleCount-1));
    int y2 = gY - (int)((recordedCurrents[i]   - minVal) * gH / range);
    u8g2.drawLine(x1, y1, x2, y2);
  }

  int meanY = gY - (int)((mean      - minVal) * gH / range);
  int sustY = gY - (int)((sustained - minVal) * gH / range);
  for (int x = gX; x < gX + gW; x += 3) u8g2.drawPixel(x, meanY);
  for (int x = gX; x < gX + gW; x += 5) { u8g2.drawPixel(x, sustY); u8g2.drawPixel(x+1, sustY); }

  char buf[12];
  u8g2.setFont(u8g2_font_4x6_tf);
  sprintf(buf, "%.1f", maxVal); u8g2.drawStr(1, 10, buf);
  sprintf(buf, "%.1f", minVal); u8g2.drawStr(1, 58, buf);
  sprintf(buf, "M%.1f", mean);  u8g2.drawStr(96, 63, buf);
  sprintf(buf, "S%.0f", sustained); u8g2.drawStr(62, 63, buf);
  sprintf(buf, "%d", sampleCount); u8g2.drawStr(2, 63, buf);
}

// -------- TABLA --------
void drawTableView() {
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(35, 8, "TABLA");

  if (sampleCount == 0) {
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(20, 35, "Sin datos");
    u8g2.setFont(u8g2_font_4x6_tf);
    u8g2.drawStr(0, 63, "BACK:Reset  Enc:Menu");
    return;
  }

  u8g2.setFont(u8g2_font_5x7_tf);
  u8g2.drawStr(2, 18, "Ts     mA    mAh");
  u8g2.drawLine(0, 20, 128, 20);

  for (int i = 0; i < 4 && (i + tableScrollOffset) < sampleCount; i++) {   // 4 filas: la 5a chocaba con el pie
    int idx = i + tableScrollOffset;
    char buf[30];
    sprintf(buf, "%4.1f %6.1f %5.2f",
      recordedTimes[idx] / 1000.0f,
      recordedCurrents[idx],
      recordedMAh[idx]);
    u8g2.drawStr(2, 30 + (i * 8), buf);
  }

  char buf[16];
  sprintf(buf, "%d/%d", tableScrollOffset + 1, sampleCount);
  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.drawStr(90, 63, buf);
  u8g2.drawStr(2, 63, "BACK:Reset");
}

// -------- SENSE-QC --------
void drawSenseQC() {
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(25, 8, "SENSE-QC");
  u8g2.drawLine(0, 10, 128, 10);

  if (testerStateSA == SA_IDLE) {
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(5, 28, "Conectar placa hija");
    u8g2.drawStr(5, 42, "y presionar OK");
    u8g2.setFont(u8g2_font_4x6_tf);
    u8g2.drawStr(0, 63, "OK:Iniciar  Enc:Menu");
    return;
  }

  if (testerStateSA == SA_TESTING) {
    unsigned long elapsed = millis() - sa_testStart;
    float prog = (float)elapsed / SA_TEST_DURATION * 100.0f;
    u8g2.setFont(u8g2_font_6x10_tf);

    char buf[24];
    sprintf(buf, "I: %.1f mA", ina228.getCurrent_mA());
    u8g2.drawStr(5, 24, buf);
    sprintf(buf, "A0:%.3fV A1:%.3fV", sa_a0v, sa_a1v);
    u8g2.drawStr(5, 36, buf);
    sprintf(buf, "P1:%s P2:%s", digitalRead(PULSE1_PIN)?"H":"L", digitalRead(PULSE2_PIN)?"H":"L");
    u8g2.drawStr(5, 48, buf);

    // Barra de progreso
    int bw = (int)(prog * 100 / 100);
    u8g2.drawFrame(5, 54, 100, 6);
    u8g2.drawBox(5, 54, bw, 6);
    return;
  }

  // SA_RESULT
  u8g2.setFont(u8g2_font_7x13_tf);
  if (sa_passed) {
    u8g2.drawStr(30, 30, "** PASS **");
  } else {
    u8g2.drawStr(20, 24, "*** FAIL ***");
    u8g2.setFont(u8g2_font_5x7_tf);
    u8g2.drawStr(5, 38, "Fallo:");
    u8g2.drawStr(5, 48, sa_failReason.c_str());
  }

  u8g2.setFont(u8g2_font_4x6_tf);
  char buf[40];
  sprintf(buf, "I:%s A0:%s A1:%s P1:%s P2:%s",
    sa_currentOk?"OK":"X",
    sa_a0Ok?"OK":"X",
    sa_a1Ok?"OK":"X",
    sa_pulse1Ok?"OK":"X",
    sa_pulse2Ok?"OK":"X");
  u8g2.drawStr(2, 56, buf);

  u8g2.drawStr(0, 63, "OK:Reiniciar  Enc:Menu");
}

// -------- WEATHER-QC --------
void drawWeatherQC() {
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(20, 8, "WEATHER-QC");
  u8g2.drawLine(0, 10, 128, 10);

  if (testerStateWX == WX_IDLE) {
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(5, 28, "Conectar placa hija");
    u8g2.drawStr(5, 42, "y presionar OK");
    u8g2.setFont(u8g2_font_4x6_tf);
    u8g2.drawStr(0, 63, "OK:Iniciar  Enc:Menu");
    return;
  }

  if (testerStateWX == WX_TESTING) {
    unsigned long elapsed = millis() - sa_testStart;
    u8g2.setFont(u8g2_font_6x10_tf);
    char buf[28];
    sprintf(buf, "I: %.1f mA", wx_current);
    u8g2.drawStr(5, 24, buf);
    sprintf(buf, "BME T:%.1f H:%.0f%%", wx_bme_temp, wx_bme_hum);
    u8g2.drawStr(5, 36, buf);
    sprintf(buf, "SHT T:%.1f H:%.0f%%", wx_sht_temp, wx_sht_hum);
    u8g2.drawStr(5, 48, buf);

    int bw = constrain((int)(elapsed / 40), 0, 100);
    u8g2.drawFrame(5, 54, 100, 6);
    u8g2.drawBox(5, 54, bw, 6);
    return;
  }

  // WX_RESULT
  u8g2.setFont(u8g2_font_7x13_tf);
  if (wx_passed) {
    u8g2.drawStr(30, 28, "** PASS **");
  } else {
    u8g2.drawStr(20, 22, "*** FAIL ***");
    u8g2.setFont(u8g2_font_5x7_tf);
    u8g2.drawStr(5, 36, wx_failReason.c_str());
  }

  u8g2.setFont(u8g2_font_5x7_tf);
  char buf[28];
  sprintf(buf, "T:%.1f>%.1f H:%.0f>%.0f",
    wx_bme_temp, wx_sht_temp, wx_bme_hum, wx_sht_hum);
  u8g2.drawStr(2, 52, buf);

  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.drawStr(0, 63, "OK:Reiniciar  Enc:Menu");
}

// -------- SOIL-QC --------
void drawSoilQC() {
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(25, 8, "SOIL-QC");
  u8g2.drawLine(0, 10, 128, 10);

  if (!soil_dataReady) {
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(5, 30, "Esperando DFM...");
    u8g2.setFont(u8g2_font_4x6_tf);
    u8g2.drawStr(5, 42, "Conectar sonda DFM");
    u8g2.drawStr(5, 50, "Serial 115200 baud");
    u8g2.drawStr(0, 63, "Enc:Menu");
    return;
  }

  u8g2.setFont(u8g2_font_5x7_tf);

  // Mostrar medias
  float hum_sum = 0, temp_sum = 0;
  for (int i = 0; i < 12; i++) hum_sum  += soil_hum[i];
  for (int i = 0; i < 13; i++) temp_sum += soil_temp[i];
  float hum_avg  = hum_sum  / 12.0f;
  float temp_avg = temp_sum / 13.0f;

  char buf[28];
  sprintf(buf, "Hum prom: %.1f%%", hum_avg);
  u8g2.drawStr(5, 22, buf);
  sprintf(buf, "Temp prom: %.1fC", temp_avg);
  u8g2.drawStr(5, 31, buf);

  // Mostrar rango
  float hmin = soil_hum[0], hmax = soil_hum[0];
  float tmin = soil_temp[0], tmax = soil_temp[0];
  for (int i = 1; i < 12; i++) { if (soil_hum[i]<hmin) hmin=soil_hum[i]; if (soil_hum[i]>hmax) hmax=soil_hum[i]; }
  for (int i = 1; i < 13; i++) { if (soil_temp[i]<tmin) tmin=soil_temp[i]; if (soil_temp[i]>tmax) tmax=soil_temp[i]; }

  sprintf(buf, "H:[%.1f-%.1f]%%", hmin, hmax);
  u8g2.drawStr(5, 40, buf);
  sprintf(buf, "T:[%.1f-%.1f]C", tmin, tmax);
  u8g2.drawStr(5, 49, buf);

  // Tiempo desde ultimo dato
  unsigned long age = (millis() - lastSoilParse) / 1000;
  sprintf(buf, "Hace %lus", age);
  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.drawStr(80, 63, buf);
  u8g2.drawStr(0, 63, "Enc:Menu");
}

// -------- RAW DATA --------
void drawRawDataView() {
  char buf[32];
  u8g2.setFont(u8g2_font_ncenB08_tr);
  sprintf(buf, "RAW [%d/%d]", rawDataPage + 1, RAW_PAGES);
  u8g2.drawStr(20, 8, buf);
  u8g2.drawLine(0, 10, 128, 10);
  u8g2.setFont(u8g2_font_5x7_tf);

  switch (rawDataPage) {
    case 0: {
      u8g2.drawStr(35, 18, "INA228");
      sprintf(buf, "I:  %.3f mA",  currentCurrent);  u8g2.drawStr(2, 27, buf);
      sprintf(buf, "Imax:%.3f mA", max_current_mA);  u8g2.drawStr(2, 35, buf);
      float s = 0; for(int i=0;i<sampleCount;i++) s+=recordedCurrents[i];
      ina_avgCurrent = sampleCount > 0 ? s/sampleCount : currentCurrent;
      sprintf(buf, "Iavg:%.3f mA", ina_avgCurrent);  u8g2.drawStr(2, 43, buf);
      sprintf(buf, "V:  %.4f V",   currentVoltage);   u8g2.drawStr(2, 51, buf);
      float pw = currentCurrent * currentVoltage / 1000.0f;
      sprintf(buf, "P:%.3fmW mAh:%.2f", pw, currentMAh); u8g2.drawStr(2, 59, buf);
      break;
    }
    case 1: {
      u8g2.drawStr(28, 18, "ADS1115");
      for (int ch = 0; ch < 4; ch++) {
        float v = ads.readADC_SingleEnded(ch) * 0.000125f;
        sprintf(buf, "A%d: %.4f V", ch, v);
        u8g2.drawStr(2, 27 + ch * 9, buf);
      }
      sprintf(buf, "Bat: %.2fV %d%%", batteryVoltage, batteryPercent);
      u8g2.drawStr(2, 63, buf);
      break;
    }
    case 2: {
      u8g2.drawStr(38, 18, "GPIO");
      bool p1 = digitalRead(PULSE1_PIN);
      bool p2 = digitalRead(PULSE2_PIN);
      sprintf(buf, "GP2 (Pulse1): %s", p1?"HIGH":"LOW"); u8g2.drawStr(2, 27, buf);
      sprintf(buf, "GP4 (Pulse2): %s", p2?"HIGH":"LOW"); u8g2.drawStr(2, 36, buf);
      sprintf(buf, "UART_RX(18):  %s", digitalRead(UART_RX_PIN)?"HIGH":"LOW"); u8g2.drawStr(2, 45, buf);
      sprintf(buf, "Bat:%.2fV  I:%.1fmA", batteryVoltage, currentCurrent); u8g2.drawStr(2, 54, buf);
      break;
    }
    case 3: {
      u8g2.drawStr(22, 18, "SENSORES");
      float bt = bme.readTemperature(), bh = bme.readHumidity(), bp = bme.readPressure()/100.0f;
      if (!isnan(bt)) { sprintf(buf, "BME T:%.1fC H:%.0f%%", bt, bh); u8g2.drawStr(2, 27, buf); }
      else u8g2.drawStr(2, 27, "BME: --");
      sprintf(buf, "    P:%.0fhPa", bp); u8g2.drawStr(2, 35, buf);

      float s10t = sht10.readTemperatureC(), s10h = sht10.readHumidity();
      if (!isnan(s10t)) { sprintf(buf, "SHT10 T:%.1fC H:%.0f%%", s10t, s10h); u8g2.drawStr(2, 43, buf); }
      else u8g2.drawStr(2, 43, "SHT10: --");

      float s30t = sht30.readTemperature(), s30h = sht30.readHumidity();
      if (!isnan(s30t)) { sprintf(buf, "SHT30 T:%.1fC H:%.0f%%", s30t, s30h); u8g2.drawStr(2, 51, buf); }
      else u8g2.drawStr(2, 51, "SHT30: --");
      break;
    }
  }

  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.drawStr(0, 63, "Gira:Pag  OK:Sig  Enc:Menu");
}

// -------- TEMP & HUMEDAD (tabla comparativa) --------
void drawTempHumView() {
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(15, 8, "Temp & Hum%");
  u8g2.drawLine(0, 10, 128, 10);
  u8g2.setFont(u8g2_font_5x7_tf);

  float bt = bme.readTemperature(), bh = bme.readHumidity();
  float s10t = sht10.readTemperatureC(), s10h = sht10.readHumidity();
  float s30t = sht30.readTemperature(),  s30h = sht30.readHumidity();

  char buf[30];

  // Header
  u8g2.drawStr(2, 18, "Sensor  Temp   Hum  DifT");
  u8g2.drawLine(0, 20, 128, 20);

  // BME (referencia)
  if (!isnan(bt)) { sprintf(buf, "BME  %5.1fC %4.0f%%  REF", bt, bh); }
  else              sprintf(buf, "BME  ----   ----  REF");
  u8g2.drawStr(2, 29, buf);

  // SHT10
  if (!isnan(s10t) && !isnan(bt)) {
    float dT = s10t - bt;
    sprintf(buf, "S10  %5.1fC %4.0f%% %+.1f", s10t, s10h, dT);
  } else { sprintf(buf, "SHT10: no detect."); }
  u8g2.drawStr(2, 39, buf);

  // SHT30
  if (!isnan(s30t) && !isnan(bt)) {
    float dT = s30t - bt;
    sprintf(buf, "S30  %5.1fC %4.0f%% %+.1f", s30t, s30h, dT);
  } else { sprintf(buf, "SHT30: no detect."); }
  u8g2.drawStr(2, 49, buf);

  // Estado
  bool s10ok = !isnan(s10t) && fabs(s10t - bt) <= WX_TEMP_TOL && fabs(s10h - bh) <= WX_HUM_TOL;
  bool s30ok = !isnan(s30t) && fabs(s30t - bt) <= WX_TEMP_TOL && fabs(s30h - bh) <= WX_HUM_TOL;

  u8g2.setFont(u8g2_font_4x6_tf);
  sprintf(buf, "S10:%s S30:%s", s10ok?"OK":"X", s30ok?"OK":"X");
  u8g2.drawStr(2, 59, buf);
  u8g2.drawStr(85, 63, "Enc:Menu");
}

// -------- UART MONITOR --------
void drawUARTView() {
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(15, 8, "UART 115200");
  u8g2.drawLine(0, 10, 128, 10);

  u8g2.setFont(u8g2_font_5x7_tf);
  for (int i = 0; i < UART_BUF_LINES; i++) {
    if (strlen(uartLines[i]) > 0) {
      u8g2.drawStr(2, 19 + i * 9, uartLines[i]);
    }
  }

  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.drawStr(80, 63, "Enc:Menu");
}

// -------- ZIGBEE (work in progress) --------
void drawZigbeeView() {
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(40, 8, "ZIGBEE");
  u8g2.drawLine(0, 10, 128, 10);

  u8g2.setFont(u8g2_font_7x13_tf);
  u8g2.drawStr(10, 32, "Work in");
  u8g2.drawStr(10, 46, "progress...");

  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.drawStr(70, 30, "S3 sin radio");
  u8g2.drawStr(70, 38, "802.15.4:");
  u8g2.drawStr(70, 46, "modulo externo");
  u8g2.drawStr(0, 63, "BACK/Enc:Menu");
}

// -------- SDI-12 (pantalla "ocupado" durante operaciones bloqueantes) --------
void sdi12ShowBusy(const char* msg) {
  rgbSet(0, 0, 30);   // azul = bus SDI-12 ocupado (operacion bloqueante)
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(40, 10, "SDI-12");
  u8g2.drawLine(0, 12, 128, 12);
  u8g2.setFont(u8g2_font_6x10_tf);
  int w = u8g2.getStrWidth(msg);
  u8g2.drawStr((128 - w) / 2, 38, msg);
  u8g2.sendBuffer();
}

// -------- SDI-12 VIEW --------
void drawSDI12View() {
  char buf[56];
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(40, 8, "SDI-12");
  u8g2.drawLine(0, 10, 128, 10);

  // -------- Pantalla inicial --------
  if (sdi12State == SDI12_IDLE) {
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(5, 26, "Visor / Editor SDI-12");
    u8g2.drawStr(5, 40, "Bus: TX=GP13 RX=GP10");
    u8g2.setFont(u8g2_font_4x6_tf);
    u8g2.drawStr(0, 63, "OK:Escanear  Enc:Menu");
    return;
  }

  // -------- Escaneando --------
  if (sdi12State == SDI12_SCANNING) {
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(5, 26, "Escaneando...");
    sprintf(buf, "Addr %c  (%d/%d)", sdi12AddrAt(sdi12ScanIndex < SDI12_NUM_ADDR ? sdi12ScanIndex : SDI12_NUM_ADDR - 1),
            sdi12ScanIndex, SDI12_NUM_ADDR);
    u8g2.setFont(u8g2_font_5x7_tf);
    u8g2.drawStr(5, 38, buf);
    int bw = (sdi12ScanIndex * 100) / SDI12_NUM_ADDR;
    u8g2.drawFrame(5, 44, 100, 6);
    u8g2.drawBox(5, 44, bw, 6);
    sprintf(buf, "Encontrados: %d", sdi12Count);
    u8g2.drawStr(5, 60, buf);
    return;
  }

  // -------- Lista de sensores + Cambiar ID --------
  if (sdi12State == SDI12_LIST) {
    int total = sdi12Count + 1;  // +1 = "Cambiar ID"
    u8g2.setFont(u8g2_font_5x7_tf);
    for (int i = 0; i < SDI12_VISIBLE && (i + sdi12TopIndex) < total; i++) {
      int idx  = i + sdi12TopIndex;
      int yPos = 20 + (i * 9);
      if (idx < sdi12Count) {
        sprintf(buf, "%sSensor %c", idx == sdi12Sel ? ">" : " ", sdi12Found[idx]);
      } else {
        sprintf(buf, "%s* Cambiar ID *", idx == sdi12Sel ? ">" : " ");
      }
      u8g2.drawStr(2, yPos, buf);
    }
    if (sdi12Count == 0) {
      u8g2.setFont(u8g2_font_4x6_tf);
      u8g2.drawStr(8, 40, "Sin sensores detectados");
    }
    drawScrollBar(122, 12, 44, sdi12Sel, total);
    u8g2.setFont(u8g2_font_4x6_tf);
    u8g2.drawStr(0, 63, "Gira:Sel OK:Entrar BACK:Atras");
    return;
  }

  // -------- Cambiar ID: elegir el sensor a renombrar (origen) --------
  if (sdi12State == SDI12_CHG_SELSRC) {
    u8g2.setFont(u8g2_font_5x7_tf);
    u8g2.drawStr(2, 19, "Renombrar cual?");
    if (sdi12Count == 0) {
      u8g2.setFont(u8g2_font_4x6_tf);
      u8g2.drawStr(8, 36, "No hay sensores. BACK.");
      u8g2.drawStr(0, 63, "BACK:Atras");
      return;
    }
    for (int i = 0; i < SDI12_VISIBLE && (i + sdi12TopIndex) < sdi12Count; i++) {
      int idx  = i + sdi12TopIndex;
      int yPos = 28 + (i * 9);   // 28, 37, 46, 55
      sprintf(buf, "%sSensor %c", idx == sdi12ChgSrcSel ? ">" : " ", sdi12Found[idx]);
      u8g2.drawStr(4, yPos, buf);
    }
    drawScrollBar(122, 22, 38, sdi12ChgSrcSel, sdi12Count);
    u8g2.setFont(u8g2_font_4x6_tf);
    u8g2.drawStr(0, 63, "Gira:Sel OK:Elegir BACK:Atras");
    return;
  }

  // -------- Datos del sensor: lista scrollable con TODOS los valores --------
  if (sdi12State == SDI12_SENSOR) {
    // Indicador del modo usado (aC! o aM!) arriba a la derecha
    u8g2.setFont(u8g2_font_4x6_tf);
    sprintf(buf, "S%c %s", sdi12CurAddr, sdi12MeasMode == 'C' ? "aC" : "aM");
    u8g2.drawStr(98, 8, buf);

    if (sdi12NumValues == 0 && sdi12InfoCount == 0) {
      u8g2.setFont(u8g2_font_6x10_tf);
      u8g2.drawStr(5, 32, "Sin datos");
      u8g2.setFont(u8g2_font_4x6_tf);
      u8g2.drawStr(0, 63, "OK:aC  Enc:aM  BACK:lista");
      return;
    }
    // Lista con scroll: primero la identificacion (aI!), luego "- <valor>"
    int rows = sdi12ViewRows();
    u8g2.setFont(u8g2_font_5x7_tf);
    for (int i = 0; i < SDI12_DATA_VISIBLE && (i + sdi12DataScroll) < rows; i++) {
      sdi12ViewRow(i + sdi12DataScroll, buf, sizeof(buf));
      u8g2.drawStr(4, 20 + i * 9, buf);   // 20,29,38,47,56
    }
    drawScrollBar(122, 12, 46, sdi12DataScroll, rows);

    // Pie: filas visibles/total + atajos (OK re-pide aC!, click encoder aM!)
    u8g2.setFont(u8g2_font_4x6_tf);
    int first = sdi12DataScroll + 1;
    int last  = sdi12DataScroll + SDI12_DATA_VISIBLE;
    if (last > rows) last = rows;
    snprintf(buf, sizeof(buf), "%d-%d/%d OK:aC Enc:aM BK", first, last, rows);
    u8g2.drawStr(0, 63, buf);
    return;
  }

  // -------- Seleccionar nuevo ID (solo IDs libres) --------
  if (sdi12State == SDI12_NEWID) {
    u8g2.setFont(u8g2_font_6x10_tf);
    sprintf(buf, "Sensor %c ->", sdi12ChgSrc);
    u8g2.drawStr(5, 24, buf);
    char na = sdi12AddrAt(sdi12NewIdIndex);
    char nb[2] = { na, 0 };
    u8g2.setFont(u8g2_font_ncenB12_tr);
    u8g2.drawStr(98, 24, nb);
    u8g2.setFont(u8g2_font_4x6_tf);
    u8g2.drawStr(5, 38, "Nuevo ID (solo libres)");
    u8g2.drawStr(5, 46, "Gira: 0-9, a-z, A-Z");
    u8g2.drawStr(0, 63, "OK:Aplicar  BACK:Atras");
    return;
  }

  // -------- Resultado del cambio de ID --------
  if (sdi12State == SDI12_CHGRESULT) {
    u8g2.setFont(u8g2_font_7x13_tf);
    if (sdi12ChgOk) u8g2.drawStr(35, 30, "** OK **");
    else            u8g2.drawStr(25, 30, "** FALLO **");
    u8g2.setFont(u8g2_font_5x7_tf);
    int w = u8g2.getStrWidth(sdi12ChgMsg.c_str());
    u8g2.drawStr((128 - w) / 2, 44, sdi12ChgMsg.c_str());
    u8g2.setFont(u8g2_font_4x6_tf);
    u8g2.drawStr(0, 63, "OK:Volver  Enc:Menu");
    return;
  }
}

// -------- BATTERY DISCHARGE --------
void drawBatteryDischarge() {
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(10, 8, "Descarga Bat");
  u8g2.drawLine(0, 10, 128, 10);

  char buf[30];

  if (dischState == DISCH_IDLE) {
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(5, 28, "Conectar bateria");
    u8g2.drawStr(5, 40, "y resistencia");
    u8g2.drawStr(5, 52, "Presionar OK");
    u8g2.setFont(u8g2_font_4x6_tf);
    u8g2.drawStr(0, 63, "OK:Iniciar  Enc:Menu");
    return;
  }

  if (dischState == DISCH_RUNNING) {
    u8g2.setFont(u8g2_font_6x10_tf);
    sprintf(buf, "V: %.3f V", disch_voltage);  u8g2.drawStr(2, 22, buf);
    sprintf(buf, "I: %.1f mA", disch_current);  u8g2.drawStr(2, 32, buf);
    sprintf(buf, "mAh: %.2f", disch_mAh);       u8g2.drawStr(2, 42, buf);
    sprintf(buf, "mWh: %.2f", disch_mWh);       u8g2.drawStr(2, 52, buf);
    unsigned long sec = disch_duration / 1000;
    sprintf(buf, "%02lu:%02lu", sec/60, sec%60);
    u8g2.drawStr(90, 52, buf);
    u8g2.setFont(u8g2_font_4x6_tf);
    u8g2.drawStr(0, 63, "BACK:Parar");
    return;
  }

  // DISCH_DONE
  u8g2.setFont(u8g2_font_6x10_tf);
  float capPct = (disch_mAh / BAT_CAPACITY_MAH) * 100.0f;
  sprintf(buf, "mAh: %.1f (%.0f%%)", disch_mAh, capPct);
  u8g2.drawStr(2, 22, buf);
  sprintf(buf, "mWh: %.1f", disch_mWh);
  u8g2.drawStr(2, 32, buf);
  sprintf(buf, "Ipico: %.1f mA", disch_peakCurrent);
  u8g2.drawStr(2, 42, buf);

  u8g2.setFont(u8g2_font_7x13_tf);
  if (capPct >= 70.0f) u8g2.drawStr(50, 55, "BUENA");
  else if (capPct >= 40.0f) u8g2.drawStr(40, 55, "REGULAR");
  else u8g2.drawStr(45, 55, "MALA");

  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.drawStr(0, 63, "OK:Reset  Enc:Menu");
}

// -------- CONFIG --------
void drawConfigView() {
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(30, 8, "CONFIG");
  u8g2.drawLine(0, 10, 128, 10);

  const char* labels[] = {
    "StartI mA:",
    "EndI mA:",
    "Period s:",
    "MaxTime s:",
    "OC mA:",
    "Samples:",
    "Bat mAh:",
    "Vref V:",
    "Vtol %:",
    "Imin mA:",
    "Imax mA:",
    "DischLim:",
    "RESET ALL"
  };

  float vals[] = {
    CURRENT_START_THRESHOLD,
    CURRENT_END_THRESHOLD,
    SAMPLE_PERIOD / 1000.0f,
    MAX_RECORD_TIME / 1000.0f,
    OC_THRESHOLD,
    (float)MAX_SAMPLES_CONFIG,
    BAT_CAPACITY_MAH,
    SA_VOLT_REF,
    SA_VOLT_TOL * 100.0f,
    SA_CURR_MIN,
    SA_CURR_MAX,
    DISCH_CURR_LIMIT,
    0.0f
  };

  u8g2.setFont(u8g2_font_5x7_tf);
  for (int i = 0; i < CONFIG_VISIBLE && (i + configTopIndex) < configItems; i++) {
    int idx  = i + configTopIndex;
    int yPos = 20 + (i * 9);
    char buf[32];

    if (idx == configItems - 1) {
      sprintf(buf, "%s%s", idx == configSelection ? ">" : " ", labels[idx]);
    } else if (idx == configSelection && isEditingConfig) {
      sprintf(buf, ">%s %.1f*", labels[idx], tempConfigValue);
    } else if (idx == configSelection) {
      sprintf(buf, ">%s %.1f", labels[idx], vals[idx]);
    } else {
      sprintf(buf, " %s %.1f", labels[idx], vals[idx]);
    }
    u8g2.drawStr(2, yPos, buf);
  }

  // Scrollbar lateral
  drawScrollBar(122, 12, 48, configSelection, configItems);

  u8g2.setFont(u8g2_font_4x6_tf);
  if (isEditingConfig) {
    u8g2.drawStr(0, 63, "Gira:Edit  OK:Guardar  BACK:Def");
  } else {
    u8g2.drawStr(0, 63, "OK:Editar  BACK:Def  Enc:Menu");
  }
}

// -------- PINOUT (ESP32-S3 LOLIN Mini, 3 paginas) --------
void drawPinoutView() {
  char buf[28];
  u8g2.setFont(u8g2_font_6x10_tf);
  snprintf(buf, sizeof(buf), "Pinout S3 [%d/3]", pinoutPage + 1);
  u8g2.drawStr(15, 10, buf);
  u8g2.drawLine(0, 12, 128, 12);
  u8g2.setFont(u8g2_font_5x7_tf);

  if (pinoutPage == 0) {          // UI
    u8g2.drawStr(2, 22, "GP5  - Encoder A");
    u8g2.drawStr(2, 31, "GP6  - Encoder B");
    u8g2.drawStr(2, 40, "GP7  - BTN Back");
    u8g2.drawStr(2, 49, "GP8  - BTN Confirm");
    u8g2.drawStr(2, 58, "GP9  - BTN Encoder");
  } else if (pinoutPage == 1) {   // Comunicaciones + bateria
    u8g2.drawStr(2, 22, "GP11 - SDA I2C");
    u8g2.drawStr(2, 31, "GP12 - SCL I2C");
    u8g2.drawStr(2, 40, "GP13 - SDI-12 TX");
    u8g2.drawStr(2, 49, "GP10 - SDI-12 RX");
    u8g2.drawStr(2, 58, "GP1  - Bateria ADC1");
  } else {                        // Testers y LED
    u8g2.drawStr(2, 22, "GP2  - Pulse1");
    u8g2.drawStr(2, 31, "GP4  - Pulse2");
    u8g2.drawStr(2, 40, "GP16 - SHT10 Data");
    u8g2.drawStr(2, 49, "GP17 - SHT10 Clk");
    u8g2.drawStr(2, 58, "GP18 UART  GP47 RGB");
  }

  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.drawStr(25, 63, "OK/Gira:Cambiar  Enc:Menu");
}
