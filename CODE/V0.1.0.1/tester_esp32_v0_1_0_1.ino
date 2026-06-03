/*
 * QC LemBot - ESP32-C3
 * Version: 0.1.0.0
 * Implementacion completa de todos los modulos
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
// PIN DEFINITIONS
// ============================================
#define ENCODER_TRA 0
#define ENCODER_TRB 1
#define BTN_BACK    2
#define BTN_CONFIRM 3
#define BTN_ENCODER 5
#define SDA_PIN     8
#define SCL_PIN     9
#define PULSE1_PIN  6
#define PULSE2_PIN  7
#define UART_RX_PIN 20

#define SHT10_DATA_PIN  7
#define SHT10_CLOCK_PIN 10

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
  MENU_BATTERY_DISCHARGE,
  MENU_CONFIG,
  MENU_PINOUT
};

MenuState currentState   = MENU_MAIN;
int menuSelection        = 0;
const int menuItems      = 10;
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
void IRAM_ATTR encoderISR();
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

void reallocArrays();
void updateCurrentReadings();
void handleRecording();
void startRecording();
void stopRecording();
void recordSample(unsigned long ts);
void resetRecording();

void updateBattery();
void updateUART();
void parseSoilData(String& line);
void updateSA();
void updateWX();
void updateDischarge();

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
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderCounter++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderCounter--;
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

  lastEncoded = (digitalRead(ENCODER_TRA) << 1) | digitalRead(ENCODER_TRB);
  attachInterrupt(digitalPinToInterrupt(ENCODER_TRA), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_TRB), encoderISR, CHANGE);

  // Pantalla bienvenida
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB10_tr);
  u8g2.drawStr(20, 28, "QC LemBot");
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(30, 46, "v0.1.0.0");
  u8g2.sendBuffer();
  delay(2000);

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
    rawDataPage -= diff;
    if (rawDataPage < 0) rawDataPage = RAW_PAGES - 1;
    if (rawDataPage >= RAW_PAGES) rawDataPage = 0;
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
    if (pinoutPage < 0) pinoutPage = 1;
    if (pinoutPage > 1) pinoutPage = 0;
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
  } else {
    // Desde cualquier pantalla: volver al menu principal
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
    currentState = MENU_CURRENT_VIEW;
  }
  else if (currentState == MENU_GRAPH_VIEW) {
    currentState = MENU_CURRENT_VIEW;
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
    if (sampleCount > 0) currentState = MENU_GRAPH_VIEW;
  }
  else if (currentState == MENU_GRAPH_VIEW) {
    currentState      = MENU_TABLE_VIEW;
    tableScrollOffset = 0;
  }
  else if (currentState == MENU_TABLE_VIEW) {
    currentState = MENU_CURRENT_VIEW;
  }
  else if (currentState == MENU_RAW_DATA) {
    rawDataPage = (rawDataPage + 1) % RAW_PAGES;
  }
  else if (currentState == MENU_PINOUT) {
    pinoutPage = 1 - pinoutPage;
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
    case 7: currentState = MENU_BATTERY_DISCHARGE;    dischState = DISCH_IDLE; break;
    case 8: currentState = MENU_CONFIG;               configSelection = 0; configTopIndex = 0; break;
    case 9: currentState = MENU_PINOUT;               pinoutPage = 0; break;
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
  float charge_C = ina228.readCharge();
  currentMAh = charge_C / 3.6f;
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
  currentMAh        = 0;
  ina228.resetAccumulators();
}

// ============================================
// BATTERY UPDATE
// ============================================
void updateBattery() {
  int16_t adc3  = ads.readADC_SingleEnded(3);
  batteryVoltage = (adc3 * 0.000125f) * 2.0f;

  if      (batteryVoltage >= 4.10f) batteryPercent = 100;
  else if (batteryVoltage >= 3.95f) batteryPercent = 80;
  else if (batteryVoltage >= 3.80f) batteryPercent = 66;
  else if (batteryVoltage >= 3.65f) batteryPercent = 50;
  else if (batteryVoltage >= 3.50f) batteryPercent = 33;
  else if (batteryVoltage >= 3.30f) batteryPercent = 15;
  else batteryPercent = 0;

  if (batteryPercent < 33 && millis() - lastBatteryBlink > 500) {
    batteryBlink = !batteryBlink;
    lastBatteryBlink = millis();
  } else if (batteryPercent >= 33) {
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
  int idx = 0;
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
    "8: Descarga Bat",
    "9: Config",
    "10: Pinout"
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

// -------- VISTA CORRIENTE --------
void drawCurrentView() {
  char buf[32];

  // Corriente actual - numeros grandes + unidad bold
  u8g2.setFont(u8g2_font_oldwizard_tn);
  sprintf(buf, "%.3f", currentCurrent);
  u8g2.drawStr(5, 12, buf);
  u8g2.setFont(u8g2_font_ncenB12_tr);
  u8g2.drawStr(50, 12, "mA");

  // Corriente maxima
  u8g2.setFont(u8g2_font_oldwizard_tn);
  sprintf(buf, "%.2f", max_current_mA);
  u8g2.drawStr(5, 28, buf);
  u8g2.setFont(u8g2_font_ncenB12_tr);
  u8g2.drawStr(50, 28, "max");

  // Voltaje
  u8g2.setFont(u8g2_font_oldwizard_tn);
  sprintf(buf, "%.3f", currentVoltage);
  u8g2.drawStr(5, 44, buf);
  u8g2.setFont(u8g2_font_ncenB12_tr);
  u8g2.drawStr(50, 44, "V");

  // mAh
  u8g2.setFont(u8g2_font_oldwizard_tn);
  sprintf(buf, "%.2f", currentMAh);
  u8g2.drawStr(5, 60, buf);
  u8g2.setFont(u8g2_font_ncenB12_tr);
  u8g2.drawStr(50, 60, "mAh");

  // Estimador de autonomia (esquina derecha, fuente pequeña)
  unsigned long elapsed_ms = isRecording ? (millis() - recordStartTime) : 0;
  if (elapsed_ms > 10000 && currentMAh > 0.01f) {
    float hoursElapsed = elapsed_ms / 3600000.0f;
    float rateMAhPerH  = currentMAh / hoursElapsed;
    float hoursLeft    = (BAT_CAPACITY_MAH - currentMAh) / rateMAhPerH;
    u8g2.setFont(u8g2_font_4x6_tf);
    sprintf(buf, "~%.0fh%.0fm", floor(hoursLeft), fmod(hoursLeft * 60, 60));
    u8g2.drawStr(88, 60, buf);
  }

  // Indicador REC / DONE (arriba derecha)
  if (isRecording) {
    u8g2.setFont(u8g2_font_5x7_tf);
    u8g2.drawStr(95, 8, "REC");
    u8g2.drawDisc(120, 5, 3);
    if (endingRecording) {
      u8g2.setFont(u8g2_font_4x6_tf);
      unsigned long rem = END_DELAY - (millis() - endRecordingTime);
      sprintf(buf, "stop %lus", rem / 1000);
      u8g2.drawStr(88, 16, buf);
    }
  } else if (recordingComplete) {
    u8g2.setFont(u8g2_font_5x7_tf);
    u8g2.drawStr(90, 8, "DONE");
    u8g2.setFont(u8g2_font_4x6_tf);
    sprintf(buf, "%d smp", sampleCount);
    u8g2.drawStr(90, 16, buf);
  }

  // Alerta OC
  if (currentCurrent > OC_THRESHOLD) {
    u8g2.setFont(u8g2_font_5x7_tf);
    u8g2.drawStr(80, 44, "!!OC!!");
  }

  // Contador muestras (igual que 0.0.9.8)
  if (sampleCount > 0) {
    u8g2.setFont(u8g2_font_4x6_tf);
    sprintf(buf, "%d", sampleCount);
    u8g2.drawStr(105, 63, buf);
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

  for (int i = 0; i < 5 && (i + tableScrollOffset) < sampleCount; i++) {
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

  u8g2.setFont(u8g2_font_5x7_tf);
  char buf[24];
  sprintf(buf, "I:%s A0:%s A1:%s P1:%s P2:%s",
    sa_currentOk?"OK":"X",
    sa_a0Ok?"OK":"X",
    sa_a1Ok?"OK":"X",
    sa_pulse1Ok?"OK":"X",
    sa_pulse2Ok?"OK":"X");
  u8g2.drawStr(2, 58, buf);

  u8g2.setFont(u8g2_font_4x6_tf);
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
      sprintf(buf, "GPIO6 (Pulse1): %s", p1?"HIGH":"LOW"); u8g2.drawStr(2, 27, buf);
      sprintf(buf, "GPIO7 (Pulse2): %s", p2?"HIGH":"LOW"); u8g2.drawStr(2, 36, buf);
      sprintf(buf, "UART_RX(20):    %s", digitalRead(UART_RX_PIN)?"HIGH":"LOW"); u8g2.drawStr(2, 45, buf);
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
    float dT = s10t - bt, dH = s10h - bh;
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

// -------- PINOUT --------
void drawPinoutView() {
  if (pinoutPage == 0) {
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(20, 10, "Pinout v1.0");
    u8g2.drawLine(0, 12, 128, 12);
    u8g2.setFont(u8g2_font_5x7_tf);
    u8g2.drawStr(2, 22, "GP0  - Encoder A");
    u8g2.drawStr(2, 31, "GP1  - Encoder B");
    u8g2.drawStr(2, 40, "GP2  - BTN Back");
    u8g2.drawStr(2, 49, "GP3  - BTN Confirm");
    u8g2.drawStr(2, 58, "GP5  - BTN Encoder");
  } else {
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(20, 10, "Pinout v1.0");
    u8g2.drawLine(0, 12, 128, 12);
    u8g2.setFont(u8g2_font_5x7_tf);
    u8g2.drawStr(2, 22, "GP6  - Pulse1 / SHT10D");
    u8g2.drawStr(2, 31, "GP7  - Pulse2");
    u8g2.drawStr(2, 40, "GP8  - SDA I2C");
    u8g2.drawStr(2, 49, "GP9  - SCL I2C");
    u8g2.drawStr(2, 58, "GP10 - SHT10 CLK  GP20:UART");
  }

  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.drawStr(25, 63, "OK/Gira:Cambiar  Enc:Menu");
}
