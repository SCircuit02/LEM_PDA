/*
 * QC LemBot Nano - Medidor de corriente
 * Placa: Arduino Nano (ATmega328P)
 *
 * Es el mismo modulo de corriente del firmware QC LemBot v0.1.5:
 *   - Pagina 1: corriente actual, mediana, maxima y minima (> 0).
 *   - Pagina 2: voltaje, mAh, corriente promedio y estimado de uso.
 *   - Grafico y tabla de la grabacion.
 *   - La grabacion arranca sola cuando la corriente supera START_I_MA.
 * Ademas envia por Serial (115200) los 4 valores I, Imed, Imax, Imin [mA]
 * con formato "etiqueta:valor": se leen en el Monitor Serie y se grafican
 * en el Plotter Serie del IDE. "nan" = valor aun no disponible.
 *
 * Conexiones:
 *   INA228 y OLED SSD1306 128x64 por I2C: SDA = A4, SCL = A5 (fijos en el Nano)
 *   Boton en D5 a GND (pull-up interno):
 *     - pulsacion corta:  siguiente vista (pag1 -> pag2 -> grafico -> tabla)
 *     - mantener 1,5 s:   reiniciar la medicion
 *
 * Librerias: "U8g2" (olikraus) y "Adafruit INA228" (con Adafruit BusIO).
 *
 * Memoria (el Nano tiene 2 KB de RAM):
 *   - La OLED usa el modo pagina de U8g2 (buffer de 128 bytes).
 *   - Los textos de pantalla van en flash (F()).
 *   - Las muestras se guardan en "half float" (2 bytes, ~3 cifras
 *     significativas). La corriente actual, la maxima y la minima se
 *     muestran con precision completa.
 */
#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <Adafruit_INA228.h>

// ------------- Configuracion (mismos valores por defecto que el FW) -------------
#define BTN_PIN            5
#define START_I_MA         0.8f      // inicia la grabacion al superar esta corriente
#define END_I_MA           9.0f      // termina la grabacion al bajar de esta corriente
#define SAMPLE_PERIOD_MS   1000UL    // intervalo entre muestras grabadas
#define MAX_RECORD_MS      120000UL  // duracion maxima de una grabacion
#define END_DELAY_MS       3000UL    // espera antes de cerrar la grabacion
#define OC_MA              10.0f     // alerta de sobrecorriente
#define BAT_CAPACITY_MAH   3500.0f   // capacidad para el estimado de uso
#define MAX_SAMPLES        120       // igual que el FW
#define SHUNT_OHM          0.015f    // igual que el FW: setShunt(0.015, 20.0)
#define MAX_CURRENT_A      20.0f
#define SERIAL_PERIOD_MS   500UL     // envio de los 4 valores por Serial
#define DISPLAY_PERIOD_MS  150UL     // refresco de la pantalla
#define LONG_PRESS_MS      1500UL    // mantener para reiniciar
#define TABLE_ROWS         4
#define TABLE_PAGE_MS      2000UL    // la tabla avanza sola cada 2 s

U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
Adafruit_INA228 ina228;
bool inaOk = false;

// ------------- Medicion (igual que el FW) -------------
float currentVoltage = 0.0f;
float currentCurrent = 0.0f;
float max_current_mA = 0.0f;
float min_current_mA = 0.0f;       // minima > 0 observada (capta uA)
float estMAh         = 0.0f;       // carga integrada para el estimado de uso
unsigned long estStartTime = 0, estLastTime = 0;

uint16_t recI[MAX_SAMPLES];        // corriente de cada muestra [mA] en half float
uint16_t recMAh[MAX_SAMPLES];      // carga acumulada en cada muestra [mAh] en half float
int  sampleCount = 0;
bool isRecording = false, recordingComplete = false, endingRecording = false;
unsigned long recordStartTime = 0, nextSampleTime = 0, endRecordingTime = 0;

// ------------- Vistas -------------
enum View : uint8_t { VIEW_P1, VIEW_P2, VIEW_GRAPH, VIEW_TABLE, VIEW_COUNT };
uint8_t view = VIEW_P1;
int     tableOffset = 0;
unsigned long lastTablePage = 0;
unsigned long resetMsgUntil = 0;

// Valores preparados antes de dibujar (el modo pagina dibuja 8 veces por cuadro)
bool  statsOk = false;
float median = 0.0f, avgI = 0.0f;
bool  estOk = false;
float estVal = 0.0f;
bool  estDays = false;
float gMin, gMax, gMean, gSust, gRange;   // estadisticas del grafico

// ================= Half float (IEEE 754 de 16 bits) =================
uint16_t f2h(float f) {
  union { float f; uint32_t u; } v;
  v.f = f;
  uint32_t x = v.u;
  uint16_t sign = (x >> 16) & 0x8000;
  int16_t  e    = (int16_t)((x >> 23) & 0xFF) - 127 + 15;
  uint32_t m    = x & 0x7FFFFFUL;
  if (((x >> 23) & 0xFF) == 0xFF) return sign | 0x7C00;   // inf/nan
  if (e >= 31) return sign | 0x7C00;                       // fuera de rango
  if (e <= 0) {                                            // subnormal
    if (e < -10) return sign;
    m |= 0x800000UL;
    uint16_t h = (uint16_t)(m >> (14 - e));
    if ((m >> (13 - e)) & 1) h++;
    return sign | h;
  }
  uint16_t h = sign | ((uint16_t)e << 10) | (uint16_t)(m >> 13);
  if (m & 0x1000) h++;                                     // redondeo
  return h;
}

float h2f(uint16_t h) {
  uint16_t e = (h >> 10) & 0x1F, m = h & 0x3FF;
  if (e == 0) {                                            // subnormal: m * 2^-24
    float f = m * 5.9604645e-8f;
    return (h & 0x8000) ? -f : f;
  }
  union { float f; uint32_t u; } v;
  v.u = ((uint32_t)(h & 0x8000) << 16) |
        ((e == 31) ? 0x7F800000UL : ((uint32_t)(e + 112) << 23)) |
        ((uint32_t)m << 13);
  return v.f;
}

// ================= Formato (sin sprintf/dtostrf para ahorrar flash) =================
char* catU(char* p, unsigned long v) { ultoa(v, p, 10); return p + strlen(p); }

// Escribe v con 'dec' decimales en out
void fmtF(char* out, float v, uint8_t dec) {
  unsigned long mult = 1;
  for (uint8_t i = 0; i < dec; i++) mult *= 10;
  bool neg = v < 0;
  if (neg) v = -v;
  unsigned long x = (unsigned long)(v * mult + 0.5f);
  if (x == 0) neg = false;
  char* p = out;
  if (neg) *p++ = '-';
  p = catU(p, x / mult);
  if (dec) {
    *p++ = '.';
    char frac[8];
    ultoa(x % mult, frac, 10);
    for (uint8_t k = strlen(frac); k < dec; k++) *p++ = '0';
    strcpy(p, frac);
  }
}

// Alinea a la derecha dentro de 'width' caracteres (para la tabla)
void padLeft(char* s, uint8_t width) {
  uint8_t n = strlen(s);
  if (n >= width) return;
  memmove(s + (width - n), s, n + 1);
  memset(s, ' ', width - n);
}

void printAt(int x, int y, const __FlashStringHelper* s) { u8g2.setCursor(x, y); u8g2.print(s); }

// ================= INA228 =================
bool initINA() {
  if (!ina228.begin()) return false;
  ina228.setShunt(SHUNT_OHM, MAX_CURRENT_A);
  ina228.setAveragingCount(INA228_COUNT_128);
  return true;
}

void resetRecording() {
  sampleCount       = 0;
  isRecording       = false;
  recordingComplete = false;
  endingRecording   = false;
  max_current_mA    = 0;
  min_current_mA    = 0;
  estMAh            = 0;
  estStartTime      = millis();
  estLastTime       = 0;
  tableOffset       = 0;
  if (inaOk) ina228.resetAccumulators();
}

void updateCurrentReadings() {
  currentVoltage = ina228.getBusVoltage_V();
  currentCurrent = ina228.getCurrent_mA();
  if (currentCurrent > max_current_mA) max_current_mA = currentCurrent;
  if (currentCurrent > 0.0f && (min_current_mA <= 0.0f || currentCurrent < min_current_mA))
    min_current_mA = currentCurrent;

  // Integracion independiente para el estimado de uso (mAh = mA * h)
  unsigned long now = millis();
  if (estLastTime != 0 && currentCurrent > 0.0f)
    estMAh += currentCurrent * ((now - estLastTime) / 3600000.0f);
  estLastTime = now;
}

void startRecording() {
  isRecording     = true;
  recordStartTime = millis();
  nextSampleTime  = recordStartTime + SAMPLE_PERIOD_MS;
  sampleCount     = 0;
  endingRecording = false;
  ina228.resetAccumulators();
}

void recordSample() {
  if (sampleCount >= MAX_SAMPLES) return;
  recI[sampleCount]   = f2h(currentCurrent);
  recMAh[sampleCount] = f2h(ina228.readCharge() / 3.6f);
  sampleCount++;
}

void handleRecording() {
  unsigned long now = millis();
  if (!isRecording && !recordingComplete && currentCurrent > START_I_MA) startRecording();
  if (!isRecording) return;

  if (!endingRecording &&
      (currentCurrent < END_I_MA || now - recordStartTime >= MAX_RECORD_MS || sampleCount >= MAX_SAMPLES)) {
    endingRecording  = true;
    endRecordingTime = now;
  }
  if (endingRecording && now - endRecordingTime >= END_DELAY_MS) {
    isRecording       = false;
    recordingComplete = true;
    endingRecording   = false;
    return;
  }
  // Muestreo con horario fijo: la muestra k corresponde a (k+1) * SAMPLE_PERIOD_MS
  if ((long)(now - nextSampleTime) >= 0) { recordSample(); nextSampleTime += SAMPLE_PERIOD_MS; }
}

// Mediana (por histograma, sin ordenar) y promedio de las muestras grabadas
void computeCurrentStats() {
  statsOk = sampleCount > 0;
  if (!statsOk) { median = 0; avgI = 0; return; }
  float sum = 0, bmin = h2f(recI[0]), bmax = bmin;
  for (int i = 0; i < sampleCount; i++) {
    float v = h2f(recI[i]);
    sum += v;
    if (v < bmin) bmin = v;
    if (v > bmax) bmax = v;
  }
  avgI = sum / sampleCount;
  const int NB = 40;
  uint8_t hist[NB];                  // uint8_t alcanza: MAX_SAMPLES < 256
  memset(hist, 0, sizeof(hist));
  float range = bmax - bmin;
  if (range < 0.0001f) range = 0.0001f;
  float bs = range / NB;
  for (int i = 0; i < sampleCount; i++) hist[constrain((int)((h2f(recI[i]) - bmin) / bs), 0, NB - 1)]++;
  int half = (sampleCount + 1) / 2, cum = 0;
  median = bmax;
  for (int b = 0; b < NB; b++) {
    cum += hist[b];
    if (cum >= half) { median = bmin + (b + 0.5f) * bs; break; }
  }
}

// Min, max, media y nivel sostenido para el grafico (igual que el FW)
void computeGraphStats() {
  gMax = h2f(recI[0]); gMin = gMax;
  float sum = 0;
  for (int i = 0; i < sampleCount; i++) {
    float v = h2f(recI[i]);
    if (v > gMax) gMax = v;
    if (v < gMin) gMin = v;
    sum += v;
  }
  gMean  = sum / sampleCount;
  gRange = gMax - gMin;
  if (gRange < 0.1f) gRange = 0.1f;
  const int BINS = 40;
  uint8_t binTime[BINS];
  memset(binTime, 0, sizeof(binTime));
  float binSize = gRange / BINS;
  if (binSize < 0.01f) binSize = 0.01f;
  int curBin = -1, curCons = 0;
  for (int i = 0; i < sampleCount; i++) {
    int b = constrain((int)((h2f(recI[i]) - gMin) / binSize), 0, BINS - 1);
    if (b == curBin) curCons++;
    else { if (curBin >= 0) binTime[curBin] += curCons; curBin = b; curCons = 1; }
  }
  if (curBin >= 0) binTime[curBin] += curCons;
  int maxBT = 0, sustBin = 0;
  for (int i = 0; i < BINS; i++) if (binTime[i] > maxBT) { maxBT = binTime[i]; sustBin = i; }
  gSust = gMin + (sustBin * binSize) + (binSize / 2);
}

// Estimado de uso: capacidad / consumo medido
void computeEstimate() {
  estOk = false;
  unsigned long el = millis() - estStartTime;
  if (estMAh <= 0.002f || el <= 5000UL) return;
  float rate = estMAh / (el / 3600000.0f);          // mAh por hora
  if (rate <= 0.0001f) return;
  float h = BAT_CAPACITY_MAH / rate;
  estOk   = true;
  estDays = h >= 100.0f;
  estVal  = estDays ? h / 24.0f : h;
}

// ================= Dibujo =================
// Fila grande: numero (oldwizard) + etiqueta justo despues, sin solaparse
void drawCurRow(int y, bool ok, float v, uint8_t dec, const __FlashStringHelper* label) {
  char num[14];
  if (ok) { fmtF(num, v, dec); u8g2.setFont(u8g2_font_oldwizard_tn); }
  else    { strcpy_P(num, PSTR("--")); u8g2.setFont(u8g2_font_ncenB12_tr); }
  u8g2.drawStr(2, y, num);
  int w = u8g2.getStrWidth(num);
  u8g2.setFont(u8g2_font_ncenB12_tr);
  printAt(2 + w + 3, y, label);
}

// Pie de las paginas: ayuda a la izquierda, estado a la derecha
void drawFooter(const __FlashStringHelper* help) {
  char sb[12];
  u8g2.setFont(u8g2_font_4x6_tr);
  printAt(2, 63, millis() < resetMsgUntil ? F("Medicion reiniciada") : help);
  if (currentCurrent > OC_MA) {
    printAt(102, 63, F("!!OC"));
  } else if (isRecording) {
    if (endingRecording) {
      unsigned long passed = millis() - endRecordingTime;
      unsigned long rem = END_DELAY_MS > passed ? END_DELAY_MS - passed : 0;
      strcpy_P(sb, PSTR("fin"));
      strcpy_P(catU(sb + 3, rem / 1000UL), PSTR("s"));
      u8g2.drawStr(98, 63, sb);
    } else {
      printAt(104, 63, F("REC"));
      u8g2.drawDisc(101, 61, 1);
    }
  } else if (recordingComplete) {
    strcpy_P(catU(sb, sampleCount), PSTR("smp"));
    u8g2.drawStr(100, 63, sb);
  }
}

// Filas de las paginas: 12, 27, 42, 57 (dejan libre el pie en y = 63)
void drawPage1() {
  drawCurRow(12, true,                  currentCurrent, 3, F("mA"));
  drawCurRow(27, statsOk,               median,         2, F("med"));
  drawCurRow(42, max_current_mA > 0.0f, max_current_mA, 2, F("max"));
  drawCurRow(57, min_current_mA > 0.0f, min_current_mA, 3, F("min"));
  drawFooter(F("BTN>pag2 mant>reset"));
}

void drawPage2() {
  drawCurRow(12, true,    currentVoltage, 3, F("V"));
  drawCurRow(27, true,    estMAh,         2, F("mAh"));
  drawCurRow(42, statsOk, avgI,           2, F("prom"));
  if (!estOk)       drawCurRow(57, false, 0,      0, F("uso"));
  else if (estDays) drawCurRow(57, true,  estVal, 0, F("d uso"));
  else              drawCurRow(57, true,  estVal, 1, F("h uso"));
  drawFooter(F("BTN>graf mant>reset"));
}

void drawGraph() {
  char b[12];
  if (sampleCount < 2) {
    u8g2.setFont(u8g2_font_5x7_tr);
    printAt(30, 35, F("Sin datos aun"));
    u8g2.setFont(u8g2_font_4x6_tr);
    printAt(0, 63, F("BTN>tabla"));
    return;
  }
  const int gH = 50, gY = 58, gX = 25, gW = 100;
  u8g2.drawFrame(gX, gY - gH, gW, gH);
  int yPrev = gY - (int)((h2f(recI[0]) - gMin) * gH / gRange);
  for (int i = 1; i < sampleCount; i++) {
    int x1 = gX + ((i - 1) * gW / (sampleCount - 1));
    int x2 = gX + (i * gW / (sampleCount - 1));
    int y2 = gY - (int)((h2f(recI[i]) - gMin) * gH / gRange);
    u8g2.drawLine(x1, yPrev, x2, y2);
    yPrev = y2;
  }
  int meanY = gY - (int)((gMean - gMin) * gH / gRange);
  int sustY = gY - (int)((gSust - gMin) * gH / gRange);
  for (int x = gX; x < gX + gW; x += 3) u8g2.drawPixel(x, meanY);
  for (int x = gX; x < gX + gW; x += 5) { u8g2.drawPixel(x, sustY); u8g2.drawPixel(x + 1, sustY); }

  u8g2.setFont(u8g2_font_4x6_tr);
  fmtF(b, gMax, 1); u8g2.drawStr(1, 10, b);
  fmtF(b, gMin, 1); u8g2.drawStr(1, 58, b);
  b[0] = 'M'; fmtF(b + 1, gMean, 1); u8g2.drawStr(96, 63, b);
  b[0] = 'S'; fmtF(b + 1, gSust, 0); u8g2.drawStr(62, 63, b);
  catU(b, sampleCount); u8g2.drawStr(2, 63, b);
}

void drawTable() {
  char b[28], col[12];
  u8g2.setFont(u8g2_font_5x7_tr);
  printAt(52, 8, F("TABLA"));
  if (sampleCount == 0) {
    printAt(42, 35, F("Sin datos"));
    u8g2.setFont(u8g2_font_4x6_tr);
    printAt(0, 63, F("BTN>pag1 mant>reset"));
    return;
  }
  printAt(2, 18, F("Ts     mA    mAh"));
  u8g2.drawLine(0, 20, 128, 20);
  for (int i = 0; i < TABLE_ROWS && (i + tableOffset) < sampleCount; i++) {
    int k = i + tableOffset;
    fmtF(b, (k + 1) * (SAMPLE_PERIOD_MS / 1000.0f), 1); padLeft(b, 4);
    strcat_P(b, PSTR(" "));
    fmtF(col, h2f(recI[k]), 1);   padLeft(col, 6); strcat(b, col);
    strcat_P(b, PSTR(" "));
    fmtF(col, h2f(recMAh[k]), 2); padLeft(col, 5); strcat(b, col);
    u8g2.drawStr(2, 30 + i * 8, b);          // 30, 38, 46, 54
  }
  u8g2.setFont(u8g2_font_4x6_tr);
  int last = min(tableOffset + TABLE_ROWS, sampleCount);
  char* p = catU(b, tableOffset + 1); *p++ = '-';
  p = catU(p, last); *p++ = '/';
  catU(p, sampleCount);
  u8g2.drawStr(88, 63, b);
  printAt(2, 63, F("BTN>pag1 mant>reset"));
}

void drawINAError() {
  u8g2.setFont(u8g2_font_5x7_tr);
  printAt(4, 24, F("INA228 no responde"));
  u8g2.setFont(u8g2_font_4x6_tr);
  printAt(4, 40, F("Revise SDA=A4, SCL=A5,"));
  printAt(4, 48, F("alimentacion y direccion 0x40"));
  printAt(4, 60, F("Reintentando..."));
}

void updateDisplay() {
  if (inaOk) {
    if (view == VIEW_P1 || view == VIEW_P2) computeCurrentStats();
    if (view == VIEW_P2) computeEstimate();
    if (view == VIEW_GRAPH && sampleCount >= 2) computeGraphStats();
  }
  u8g2.firstPage();
  do {
    if (!inaOk)                   drawINAError();
    else if (view == VIEW_P1)     drawPage1();
    else if (view == VIEW_P2)     drawPage2();
    else if (view == VIEW_GRAPH)  drawGraph();
    else                          drawTable();
  } while (u8g2.nextPage());
}

// ================= Boton D5 =================
void handleButton() {
  static bool last = HIGH;
  static unsigned long tDown = 0;
  static bool longDone = false;
  bool now = digitalRead(BTN_PIN);
  unsigned long t = millis();
  if (now == LOW && last == HIGH) { tDown = t; longDone = false; }
  if (now == LOW && !longDone && t - tDown >= LONG_PRESS_MS) {
    longDone = true;                              // mantener: reiniciar
    resetRecording();
    resetMsgUntil = t + 1200;
  }
  if (now == HIGH && last == LOW && !longDone && t - tDown >= 30) {
    view = (view + 1) % VIEW_COUNT;               // corta: siguiente vista
    tableOffset = 0;
    lastTablePage = t;
  }
  last = now;
}

// ================= Serial: I, Imed, Imax, Imin =================
void printValue(const __FlashStringHelper* label, bool ok, float v, uint8_t dec) {
  Serial.print(label);
  if (ok) Serial.print(v, dec); else Serial.print(F("nan"));
}

void sendSerial() {
  computeCurrentStats();
  printValue(F("I:"),     true,                  currentCurrent, 3);
  printValue(F(",Imed:"), statsOk,               median,         2);
  printValue(F(",Imax:"), max_current_mA > 0.0f, max_current_mA, 2);
  printValue(F(",Imin:"), min_current_mA > 0.0f, min_current_mA, 3);
  Serial.println();
}

// ================= Setup / Loop =================
void setup() {
  Serial.begin(115200);
  pinMode(BTN_PIN, INPUT_PULLUP);

  u8g2.setBusClock(400000);
  u8g2.begin();

  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_ncenB12_tr);
    printAt(16, 28, F("QC LemBot"));
    u8g2.setFont(u8g2_font_5x7_tr);
    printAt(24, 46, F("Nano - Corriente"));
  } while (u8g2.nextPage());
  delay(1500);

  inaOk = initINA();
  if (!inaOk) Serial.println(F("INA228 no responde (SDA=A4, SCL=A5, dir 0x40)"));
  resetRecording();
}

void loop() {
  static unsigned long lastDisp = 0, lastSerial = 0, lastRetry = 0;
  unsigned long now = millis();

  if (!inaOk) {
    if (now - lastRetry >= 1000UL) {
      lastRetry = now;
      inaOk = initINA();
      if (inaOk) { Serial.println(F("INA228 OK")); resetRecording(); }
    }
  } else {
    updateCurrentReadings();
    handleRecording();
    if (now - lastSerial >= SERIAL_PERIOD_MS) { lastSerial = now; sendSerial(); }
  }

  handleButton();

  // La tabla avanza sola (el Nano tiene un solo boton)
  if (view == VIEW_TABLE && sampleCount > TABLE_ROWS && now - lastTablePage >= TABLE_PAGE_MS) {
    lastTablePage = now;
    tableOffset += TABLE_ROWS;
    if (tableOffset >= sampleCount) tableOffset = 0;
  }

  if (now - lastDisp >= DISPLAY_PERIOD_MS) { lastDisp = now; updateDisplay(); }
}
