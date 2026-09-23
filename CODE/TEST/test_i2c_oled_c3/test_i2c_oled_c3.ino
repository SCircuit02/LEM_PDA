/*
 * test_i2c_oled_c3 - Prueba de bus I2C + OLED para ESP32-C3 (QC LemBot)
 *
 * Que hace:
 *   1. Mide el nivel en reposo de SDA/SCL (detecta pull-ups faltantes o una
 *      linea tirada a GND).
 *   2. Libera el bus si un esclavo quedo sosteniendo SDA en bajo (9 pulsos SCL).
 *   3. Escanea el bus I2C y confirma el INA228 leyendo sus registros de ID.
 *   4. Muestra el resultado en la OLED y por el Monitor Serie, y repite cada
 *      3 s: sirve para mover cables/soldaduras y ver el efecto en vivo.
 *   5. Si el LED RGB NO esta en el bus I2C: parpadea R-G-B al arrancar (prueba
 *      de que el C3 corre el sketch) y luego hace de semaforo.
 *
 * Arduino IDE (menu Herramientas):
 *   Placa:           "ESP32C3 Dev Module"
 *   USB CDC On Boot: "Enabled"  (si no, el Monitor Serie no muestra nada)
 *   Flash Mode:      "DIO"      (varios C3 SuperMini no arrancan en QIO)
 *   Monitor Serie a 115200 baudios.
 */
#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>

// ---------------- Configuracion ----------------
#define SDA_PIN     8        // como esta soldado hoy
#define SCL_PIN     9        // recomendado para la placa nueva: SDA 10, SCL 9
#define RGB_PIN     8        // LED WS2812 onboard de los C3 con RGB
#define OLED_SH1106 0        // 0 = SSD1306 (0.96"), 1 = SH1106 (1.3")
#define I2C_HZ      100000   // 100 kHz: mas tolerante a pull-ups debiles

#if OLED_SH1106
U8G2_SH1106_128X64_NONAME_F_HW_I2C  u8g2(U8G2_R0, U8X8_PIN_NONE, SCL_PIN, SDA_PIN);
#else
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, SCL_PIN, SDA_PIN);
#endif

// Si el LED comparte pin con el I2C no se toca (romperia el bus)
const bool rgbOnBus = (RGB_PIN == SDA_PIN || RGB_PIN == SCL_PIN);

uint8_t found[24];
int     nFound    = 0;
uint8_t oledAddr  = 0;       // 0 = no encontrada
uint8_t inaAddr   = 0;       // 0 = no encontrado
bool    oledReady = false;
unsigned long scans = 0;

void rgb(uint8_t r, uint8_t g, uint8_t b) {
  if (rgbOnBus) return;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  rgbLedWrite(RGB_PIN, r, g, b);
#else
  neopixelWrite(RGB_PIN, r, g, b);
#endif
}

// Lee un registro de 16 bits (MSB primero). false si el dispositivo no responde.
bool readReg16(uint8_t addr, uint8_t reg, uint16_t &val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)addr, 2) != 2) return false;
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  val = ((uint16_t)hi << 8) | lo;
  return true;
}

// INA228: MANUFACTURER_ID (0x3E) = 0x5449 ("TI"), DEVICE_ID (0x3F) >> 4 = 0x228
bool isINA228(uint8_t addr) {
  uint16_t mfg, dev;
  if (!readReg16(addr, 0x3E, mfg) || !readReg16(addr, 0x3F, dev)) return false;
  return mfg == 0x5449 && (dev >> 4) == 0x228;
}

const char* guess(uint8_t a) {
  if (a == inaAddr)           return "INA228 (ID confirmado)";
  if (a == 0x3C || a == 0x3D) return "OLED";
  if (a == 0x44 || a == 0x45) return "SHT30?";
  if (a >= 0x48 && a <= 0x4B) return "ADS1115?";
  if (a == 0x76 || a == 0x77) return "BME280?";
  return "?";
}

// Si un esclavo quedo con SDA en bajo (reset a mitad de una transferencia),
// hasta 9 pulsos de SCL lo liberan. Devuelve true si hizo falta.
bool busRecovery() {
  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, INPUT_PULLUP);
  delay(2);
  if (digitalRead(SDA_PIN) == HIGH) return false;
  pinMode(SCL_PIN, OUTPUT_OPEN_DRAIN);
  for (int i = 0; i < 9 && digitalRead(SDA_PIN) == LOW; i++) {
    digitalWrite(SCL_PIN, LOW);  delayMicroseconds(10);
    digitalWrite(SCL_PIN, HIGH); delayMicroseconds(10);
  }
  pinMode(SCL_PIN, INPUT_PULLUP);
  return true;
}

void scanBus() {
  nFound = 0; oledAddr = 0; inaAddr = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      if (nFound < (int)sizeof(found)) found[nFound++] = a;
      if ((a == 0x3C || a == 0x3D) && !oledAddr) oledAddr = a;
      if (a >= 0x40 && a <= 0x4F && !inaAddr && isINA228(a)) inaAddr = a;
    }
  }
  scans++;
}

void printReport() {
  Serial.println();
  Serial.printf("==== Escaneo #%lu (SDA=%d SCL=%d, %d kHz) ====\n",
                scans, SDA_PIN, SCL_PIN, I2C_HZ / 1000);
  if (nFound == 0) Serial.println("  Ningun dispositivo responde en el bus I2C.");
  for (int i = 0; i < nFound; i++) Serial.printf("  0x%02X  %s\n", found[i], guess(found[i]));
  Serial.printf("OLED  : %s\n", oledAddr ? "OK" : "NO encontrada (esperada en 0x3C/0x3D)");
  Serial.printf("INA228: %s\n", inaAddr  ? "OK" : "NO encontrado (esperado en 0x40..0x4F)");
}

void printTips() {
  Serial.println("\nPosibles causas (revisar en este orden):");
  Serial.println(" - SDA y SCL invertidos (el error mas comun al soldar).");
  Serial.println(" - Sin pull-ups: SDA/SCL necesitan ~4.7k a 3V3 (los modulos suelen traerlas).");
  Serial.println(" - Falta 3V3 o GND en la OLED / INA228 (medir con multimetro).");
  Serial.println(" - OLED de 1.3\" suele ser SH1106: poner OLED_SH1106 en 1.");
  Serial.println(" - INA228 con A0/A1 soldados distinto: cambia su direccion (0x40..0x4F).");
  Serial.println(" - GPIO2, 8 y 9 son strapping: si alguno esta en BAJO al encender,");
  Serial.println("   el C3 puede quedar en modo descarga y parecer muerto.");
}

void showOLED() {
  if (!oledAddr) return;
  if (!oledReady) {
    u8g2.setI2CAddress(oledAddr << 1);   // u8g2 usa la direccion de 8 bits
    u8g2.setBusClock(I2C_HZ);
    oledReady = u8g2.begin();
    if (!oledReady) return;
  }
  char b[32];
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 10, "I2C TEST ESP32-C3");
  snprintf(b, sizeof(b), "SDA:%d SCL:%d  #%lu", SDA_PIN, SCL_PIN, scans);
  u8g2.drawStr(0, 22, b);
  snprintf(b, sizeof(b), "OLED 0x%02X OK", oledAddr);
  u8g2.drawStr(0, 34, b);
  if (inaAddr) snprintf(b, sizeof(b), "INA228 0x%02X OK", inaAddr);
  else         snprintf(b, sizeof(b), "INA228: NO");
  u8g2.drawStr(0, 46, b);
  int n = snprintf(b, sizeof(b), "Disp(%d):", nFound);
  for (int i = 0; i < nFound && n < (int)sizeof(b) - 4; i++)
    n += snprintf(b + n, sizeof(b) - n, " %02X", found[i]);
  u8g2.drawStr(0, 58, b);
  u8g2.sendBuffer();
}

// Semaforo en el RGB (si esta libre): verde = todo OK, amarillo = parcial, rojo = nada
void showStatusLED() {
  if (oledAddr && inaAddr)      rgb(0, 30, 0);
  else if (oledAddr || inaAddr) rgb(30, 20, 0);
  else                          rgb(30, 0, 0);
}

void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 2000) delay(10);   // esperar el Monitor Serie (USB CDC)

  // Prueba de vida: parpadeo R-G-B (solo si el LED no esta en el bus I2C)
  rgb(40, 0, 0); delay(250); rgb(0, 40, 0); delay(250); rgb(0, 0, 40); delay(250); rgb(0, 0, 0);

  Serial.println("\n=== Test I2C + OLED - ESP32-C3 ===");
  Serial.println("Si lees esto, el C3 arranca y corre el sketch.");
  if (rgbOnBus)
    Serial.printf("AVISO: el LED RGB (GPIO%d) esta en el bus I2C; puede encender raro con el trafico.\n", RGB_PIN);

  // Nivel en reposo SIN pull-up interno: ALTO = probablemente hay pull-up externo
  pinMode(SDA_PIN, INPUT);
  pinMode(SCL_PIN, INPUT);
  delay(5);
  int sdaIdle = digitalRead(SDA_PIN);
  int sclIdle = digitalRead(SCL_PIN);
  Serial.printf("Reposo: SDA=%s SCL=%s (ALTO = hay pull-up; BAJO = falta pull-up o algo tira a GND)\n",
                sdaIdle ? "ALTO" : "BAJO", sclIdle ? "ALTO" : "BAJO");

  if (busRecovery()) Serial.println("SDA estaba trabado en BAJO: se aplicaron pulsos de SCL para liberarlo.");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(I2C_HZ);
  scanBus();
  printReport();
  showOLED();
  showStatusLED();
  if (!oledAddr || !inaAddr) printTips();
}

void loop() {
  static unsigned long last = 0;
  if (millis() - last >= 3000) {
    last = millis();
    scanBus();
    printReport();
    showOLED();
    showStatusLED();
  }
}
