/*
 * LemPDA SDI-12 - Gestor web de sensores SDI-12
 * Placa: ESP32-S2 (por ejemplo LOLIN S2 Mini). Core arduino-esp32 3.x.
 *
 * Crea una red WiFi propia con portal cautivo:
 *   Red:   LemPDA-XXXX   (XXXX = ultimos 4 digitos hex de la MAC del AP)
 *   Clave: 12345678      (WPA2 exige al menos 8 caracteres)
 *   Web:   http://192.168.4.1  (el telefono la abre solo al conectarse)
 *
 * Desde la web se puede:
 *   - Escanear las 62 direcciones y ver los sensores conectados y su ID.
 *   - Tocar un sensor para ver su identificacion (aI!) y sus valores con
 *     aC! (concurrente) y aM! (estandar).
 *   - Cambiar el ID de un sensor por uno que no este en uso.
 *
 * Hardware: front-end SDI-12 del QC LemBot (bit-bang de 2 pines):
 *   TX = GPIO10 -> SN74LVC1G3157 -> SN74LVC1G240 (inversor) -> bus SDI-12
 *   RX = GPIO11 <- SN74AHC1G14 (inversor Schmitt) <- bus SDI-12
 *   Las dos inversiones cancelan la del bus: el MCU ve un UART normal
 *   1200 baud 7E1 (reposo = ALTO, start = BAJO).
 *
 * Las operaciones SDI-12 corren por pasos (un comando por vuelta del loop),
 * asi la web sigue respondiendo mientras un sensor mide.
 */
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>

// ---------------- Configuracion ----------------
#define SDI12_TX_PIN   10
#define SDI12_RX_PIN   11
#define STATUS_LED     15          // LED azul de la LOLIN S2 Mini (-1 = sin LED)
#define AP_PASS        "12345678"  // minimo 8 caracteres (WPA2)
#define MAX_SENSORS    16          // sensores que se guardan en la lista
#define MAX_WAIT_S     30          // tope de espera de una medicion [s]

const IPAddress AP_IP(192, 168, 4, 1);
const IPAddress AP_MASK(255, 255, 255, 0);

WebServer server(80);
DNSServer dnsServer;
String apName;

// =====================================================================
//  Tipos y estado (antes de la primera funcion: el IDE inserta ahi los
//  prototipos automaticos y necesitan conocer estos tipos)
// =====================================================================
#define SDI12_NUM_ADDR  62
struct Meas {
  int n = -1;            // -1 = sin medir, -2 = el sensor no soporta el comando
  int t = 0;             // segundos de espera que pidio el sensor
  uint32_t at = 0;       // segundo de uptime en que se midio
  String raw;            // valores concatenados: "+1.23-4.5+6"
};
struct Sensor {
  char   addr = 0;
  String info;           // respuesta de aI! sin la direccion
  Meas   c, m;           // resultados de aC! y aM!
};
Sensor sensors[MAX_SENSORS];
int nSensors = 0;

enum JobType : uint8_t { JOB_NONE, JOB_SCAN, JOB_INFO, JOB_MEASURE, JOB_CHID };
struct Job {
  JobType type = JOB_NONE;
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
} job;

char scanFound[SDI12_NUM_ADDR];
int  scanCount = 0;
String lastMsg = "";
bool   lastErr = false;

// =====================================================================
//  SDI-12 bit-bang (igual que el firmware QC LemBot 0.1.4 / 0.2.x)
// =====================================================================
#define SDI12_TX_MARK   HIGH   // reposo / bit 1 / stop
#define SDI12_TX_SPACE  LOW    // start / bit 0 / break
#define SDI12_RX_MARK   HIGH   // ALTO en RX = marca (1)
#define SDI12_BIT_US    833    // 1200 baud
#define SDI12_HALF_US   417

char sdi12AddrAt(int i) {
  if (i < 10) return '0' + i;
  if (i < 36) return 'a' + (i - 10);
  return 'A' + (i - 36);
}

int sdi12IndexOf(char a) {
  if (a >= '0' && a <= '9') return a - '0';
  if (a >= 'a' && a <= 'z') return 10 + (a - 'a');
  if (a >= 'A' && a <= 'Z') return 36 + (a - 'A');
  return -1;
}

// Un caracter: start + 7 datos (LSB primero) + paridad par + stop
void sdi12SendChar(uint8_t ch) {
  noInterrupts();
  digitalWrite(SDI12_TX_PIN, SDI12_TX_SPACE);
  delayMicroseconds(SDI12_BIT_US);
  uint8_t parity = 0;
  for (int i = 0; i < 7; i++) {
    uint8_t bit = (ch >> i) & 0x01;
    parity ^= bit;
    digitalWrite(SDI12_TX_PIN, bit ? SDI12_TX_MARK : SDI12_TX_SPACE);
    delayMicroseconds(SDI12_BIT_US);
  }
  digitalWrite(SDI12_TX_PIN, parity ? SDI12_TX_MARK : SDI12_TX_SPACE);
  delayMicroseconds(SDI12_BIT_US);
  digitalWrite(SDI12_TX_PIN, SDI12_TX_MARK);
  delayMicroseconds(SDI12_BIT_US);
  interrupts();
}

// Break (>= 12 ms) + marca (>= 8,33 ms) + comando
void sdi12SendBreakAndCommand(const String& cmd) {
  pinMode(SDI12_TX_PIN, OUTPUT);
  digitalWrite(SDI12_TX_PIN, SDI12_TX_SPACE);
  delay(13);
  digitalWrite(SDI12_TX_PIN, SDI12_TX_MARK);
  delay(9);
  for (int i = 0; i < (int)cmd.length(); i++) sdi12SendChar((uint8_t)cmd.charAt(i));
  digitalWrite(SDI12_TX_PIN, SDI12_TX_MARK);   // reposo: libera el bus
}

// Un caracter de RX (7 bits); -1 si no llega el start antes del timeout
int sdi12ReadChar(unsigned long timeoutMs) {
  unsigned long t0 = millis();
  while (digitalRead(SDI12_RX_PIN) == SDI12_RX_MARK) {
    if (millis() - t0 >= timeoutMs) return -1;
  }
  noInterrupts();
  delayMicroseconds(SDI12_BIT_US + SDI12_HALF_US);   // centro del bit 0
  uint8_t c = 0;
  for (int i = 0; i < 7; i++) {
    if (digitalRead(SDI12_RX_PIN) == SDI12_RX_MARK) c |= (1 << i);
    delayMicroseconds(SDI12_BIT_US);
  }
  delayMicroseconds(SDI12_BIT_US);                   // saltar la paridad
  interrupts();
  return c & 0x7F;
}

// Lee una linea de respuesta (sin CR/LF); "" si no llega nada a tiempo
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

String sdi12Command(const String& cmd, uint16_t waitMs) {
  sdi12SendBreakAndCommand(cmd);
  return sdi12ReadLine(waitMs);
}

bool sdi12Probe(char addr) {
  String r = sdi12Command(String(addr) + "!", 120);
  return r.length() > 0 && r.charAt(0) == addr;
}

// =====================================================================
//  Lista de sensores y tareas SDI-12 (por pasos)
// =====================================================================
int findSensor(char a) {
  for (int i = 0; i < nSensors; i++) if (sensors[i].addr == a) return i;
  return -1;
}

void finishJob(const String& msg, bool err) {
  lastMsg = msg;
  lastErr = err;
  job.type = JOB_NONE;
  job.target = 0;
  Serial.println(msg);
}

// Rearma la lista con lo encontrado, conservando lo ya medido
void rebuildSensorList() {
  Sensor old[MAX_SENSORS];
  int nOld = nSensors;
  for (int i = 0; i < nOld; i++) old[i] = sensors[i];
  nSensors = 0;
  for (int f = 0; f < scanCount && nSensors < MAX_SENSORS; f++) {
    Sensor s;
    s.addr = scanFound[f];
    for (int i = 0; i < nOld; i++) if (old[i].addr == s.addr) { s = old[i]; break; }
    sensors[nSensors++] = s;
  }
}

void readInfo(int idx) {
  String r = sdi12Command(String(sensors[idx].addr) + "I!", 400);
  if (r.length() > 1 && r.charAt(0) == sensors[idx].addr) sensors[idx].info = r.substring(1);
}

void storeMeasurement() {
  int idx = findSensor(job.target);
  if (idx < 0) return;
  Meas& m = (job.mode == 'C') ? sensors[idx].c : sensors[idx].m;
  m.n   = job.n;
  m.t   = job.t;
  m.at  = millis() / 1000;
  m.raw = job.raw;
}

// Un paso de la tarea en curso. Cada paso envia a lo mas unos pocos comandos.
void jobStep() {
  if (job.type == JOB_NONE) return;

  if (job.type == JOB_SCAN) {
    if (job.phase == 0) {                       // probar una direccion por paso
      char a = sdi12AddrAt(job.i);
      if (sdi12Probe(a)) scanFound[scanCount++] = a;
      if (++job.i >= SDI12_NUM_ADDR) { rebuildSensorList(); job.phase = 1; job.k = 0; }
      return;
    }
    if (job.k < nSensors) {                     // identificacion de cada sensor
      if (sensors[job.k].info.length() == 0) readInfo(job.k);
      job.k++;
      return;
    }
    String msg = nSensors == 1 ? "1 sensor encontrado" : String(nSensors) + " sensores encontrados";
    if (scanCount > MAX_SENSORS) msg += " (se muestran " + String(MAX_SENSORS) + ")";
    finishJob(msg, false);
    return;
  }

  if (job.type == JOB_INFO) {
    int idx = findSensor(job.target);
    if (idx >= 0) readInfo(idx);
    finishJob(idx >= 0 && sensors[idx].info.length() ? String("Identificación de ") + job.target + " leída"
                                                       : String("El sensor ") + job.target + " no respondió a aI!", idx < 0);
    return;
  }

  if (job.type == JOB_MEASURE) {
    if (job.phase == 0) {                       // enviar aC! o aM!
      String r = sdi12Command(String(job.target) + job.mode + "!", 400);
      bool ok = r.length() >= 5 && r.charAt(0) == job.target;
      if (!ok && job.mode == 'C') {
        int idx = findSensor(job.target);
        if (idx >= 0) { sensors[idx].c.n = -2; sensors[idx].c.raw = ""; }
        job.mode = 'M';                         // sin aC!: se mide con aM!
        return;
      }
      if (!ok) { finishJob(String("El sensor ") + job.target + " no respondió a a" + job.mode + "!", true); return; }
      job.t = r.substring(1, 4).toInt();
      job.n = r.substring(4).toInt();
      job.raw = "";
      job.waitStart = millis();
      job.waitMs = (unsigned long)min(job.t, MAX_WAIT_S) * 1000UL + 150UL;
      job.phase = 1;
      return;
    }
    if (job.phase == 1) {                       // esperar la medicion (sin bloquear)
      bool ready = millis() - job.waitStart >= job.waitMs;
      // Con aM! el sensor avisa cuando termina ("a<CR><LF>")
      if (!ready && job.mode == 'M' && digitalRead(SDI12_RX_PIN) != SDI12_RX_MARK) {
        String sr = sdi12ReadLine(60);
        ready = sr.length() > 0 && sr.charAt(0) == job.target;
      }
      if (ready) { job.phase = 2; job.k = 0; }
      return;
    }
    // phase 2: leer aD0!..aD9! (uno por paso) hasta que no haya mas datos
    String dr = sdi12Command(String(job.target) + "D" + String(job.k) + "!", 900);
    String body = (dr.length() > 1 && dr.charAt(0) == job.target) ? dr.substring(1) : "";
    bool hasValues = body.indexOf('+') >= 0 || body.indexOf('-') >= 0;
    if (hasValues) job.raw += body;
    if (hasValues && ++job.k <= 9) return;
    storeMeasurement();
    if (job.both && job.mode == 'C') { job.mode = 'M'; job.phase = 0; return; }
    finishJob(String("Medición de ") + job.target + " lista", false);
    return;
  }

  if (job.type == JOB_CHID) {
    char oldA = job.target, newA = job.newAddr;
    if (sdi12Probe(newA))  { finishJob(String("El ID ") + newA + " ya está en uso", true); return; }
    if (!sdi12Probe(oldA)) { finishJob(String("El sensor ") + oldA + " no responde", true); return; }
    sdi12Command(String(oldA) + "A" + newA + "!", 400);
    bool newOk = sdi12Probe(newA);
    bool oldGone = !sdi12Probe(oldA);
    if (!newOk) { finishJob(String("No se pudo cambiar el ID de ") + oldA, true); return; }
    int idx = findSensor(oldA);
    if (idx >= 0) sensors[idx].addr = newA;
    // mantener la lista ordenada por direccion
    for (int a = 0; a < nSensors; a++)
      for (int b = a + 1; b < nSensors; b++)
        if (sdi12IndexOf(sensors[b].addr) < sdi12IndexOf(sensors[a].addr)) { Sensor t = sensors[a]; sensors[a] = sensors[b]; sensors[b] = t; }
    finishJob(String("ID cambiado: ") + oldA + " -> " + newA + (oldGone ? "" : " (la dirección anterior aún responde)"), false);
    return;
  }
}

// Texto y avance de la tarea para la web
String jobText(int& p, int& t) {
  p = 0; t = 0;
  switch (job.type) {
    case JOB_SCAN:
      if (job.phase == 0) { p = job.i; t = SDI12_NUM_ADDR; return "Escaneando " + String(job.i) + "/62"; }
      p = job.k; t = nSensors;
      return "Leyendo identificación " + String(min(job.k + 1, nSensors)) + "/" + String(nSensors);
    case JOB_INFO:    return String("Leyendo aI! de ") + job.target;
    case JOB_CHID:    return String("Cambiando ID ") + job.target + " -> " + job.newAddr;
    case JOB_MEASURE: {
      String m = String("a") + job.mode + "!";
      if (job.phase == 0) return "Enviando " + m + " a " + job.target;
      if (job.phase == 1) {
        p = millis() - job.waitStart; t = job.waitMs;
        long rest = ((long)job.waitMs - p) / 1000;
        return "Midiendo " + m + " en " + job.target + ": faltan " + String(max(rest, 0L)) + " s";
      }
      return "Leyendo datos " + m + " (aD" + String(job.k) + "!) de " + job.target;
    }
    default: return "";
  }
}

// =====================================================================
//  Web
// =====================================================================
#include "index_html.h"   // pagina web (PAGE)

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

void measJson(String& j, const Meas& m) {
  j += "{\"n\":"; j += m.n;
  j += ",\"t\":"; j += m.t;
  j += ",\"at\":"; j += m.at;
  j += ",\"raw\":\""; j += jsonEsc(m.raw); j += "\"}";
}

void sendJson(int code, const String& body) {
  server.sendHeader("Cache-Control", "no-store");
  server.send(code, "application/json; charset=utf-8", body);
}

String buildStateJson() {
  int p, t;
  String txt = jobText(p, t);
  String j;
  j.reserve(1024 + nSensors * 400);
  j += "{\"up\":"; j += millis() / 1000;
  j += ",\"ap\":\""; j += apName; j += "\"";
  j += ",\"busy\":"; j += job.type != JOB_NONE ? "true" : "false";
  j += ",\"a\":\""; if (job.target) j += job.target; j += "\"";
  j += ",\"p\":"; j += p; j += ",\"t\":"; j += t;
  j += ",\"txt\":\""; j += jsonEsc(txt); j += "\"";
  j += ",\"msg\":\""; j += jsonEsc(lastMsg); j += "\"";
  j += ",\"err\":"; j += lastErr ? "true" : "false";
  j += ",\"sensors\":[";
  for (int i = 0; i < nSensors; i++) {
    if (i) j += ',';
    j += "{\"a\":\""; j += sensors[i].addr; j += "\"";
    j += ",\"i\":\""; j += jsonEsc(sensors[i].info); j += "\"";
    j += ",\"c\":"; measJson(j, sensors[i].c);
    j += ",\"m\":"; measJson(j, sensors[i].m);
    j += "}";
  }
  j += "]}";
  return j;
}

void handleState() { sendJson(200, buildStateJson()); }

// Valida y arranca una tarea; responde 409 si ya hay otra en curso
bool jobBusy() {
  if (job.type == JOB_NONE) return false;
  sendJson(409, "{\"ok\":false,\"err\":\"Ocupado: espere a que termine la operación en curso\"}");
  return true;
}

bool argSensor(char& a) {
  String s = server.arg("a");
  a = s.length() == 1 ? s.charAt(0) : 0;
  if (sdi12IndexOf(a) < 0 || findSensor(a) < 0) {
    sendJson(400, "{\"ok\":false,\"err\":\"Sensor no válido: escanee de nuevo\"}");
    return false;
  }
  return true;
}

void handleScan() {
  if (jobBusy()) return;
  job = Job();
  job.type = JOB_SCAN;
  scanCount = 0;
  lastMsg = "";
  sendJson(200, "{\"ok\":true}");
}

void handleInfo() {
  char a;
  if (jobBusy() || !argSensor(a)) return;
  job = Job();
  job.type = JOB_INFO;
  job.target = a;
  sendJson(200, "{\"ok\":true}");
}

void handleMeasure() {
  char a;
  if (jobBusy() || !argSensor(a)) return;
  String m = server.arg("m");
  job = Job();
  job.type = JOB_MEASURE;
  job.target = a;
  job.both = (m == "B");
  job.mode = (m == "M") ? 'M' : 'C';
  sendJson(200, "{\"ok\":true}");
}

void handleChid() {
  char a;
  if (jobBusy() || !argSensor(a)) return;
  String n = server.arg("n");
  char na = n.length() == 1 ? n.charAt(0) : 0;
  if (sdi12IndexOf(na) < 0 || findSensor(na) >= 0) {
    sendJson(400, "{\"ok\":false,\"err\":\"El ID nuevo no es válido o ya está en uso\"}");
    return;
  }
  job = Job();
  job.type = JOB_CHID;
  job.target = a;
  job.newAddr = na;
  sendJson(200, "{\"ok\":true}");
}

void handleRoot() {
  server.sendHeader("Cache-Control", "no-store");
  server.send_P(200, "text/html; charset=utf-8", PAGE);
}

// Portal cautivo: cualquier otra URL (pruebas de conexion de Android, iOS y
// Windows incluidas) redirige a la pagina del equipo
void redirectToPortal() {
  server.sendHeader("Location", String("http://") + AP_IP.toString() + "/", true);
  server.send(302, "text/plain", "");
}

// =====================================================================
//  Setup / Loop
// =====================================================================
void setup() {
  Serial.begin(115200);
  pinMode(SDI12_TX_PIN, OUTPUT);
  digitalWrite(SDI12_TX_PIN, SDI12_TX_MARK);   // bus en reposo
  pinMode(SDI12_RX_PIN, INPUT);
  if (STATUS_LED >= 0) pinMode(STATUS_LED, OUTPUT);

  WiFi.mode(WIFI_AP);
  uint8_t mac[6] = {0};
  WiFi.softAPmacAddress(mac);
  char ssid[16];
  snprintf(ssid, sizeof(ssid), "LemPDA-%02X%02X", mac[4], mac[5]);
  apName = ssid;
  WiFi.softAPConfig(AP_IP, AP_IP, AP_MASK, IPAddress((uint32_t)0), AP_IP);   // DNS = el equipo
  if (!WiFi.softAP(ssid, AP_PASS)) Serial.println("ERROR: no se pudo crear la red WiFi");

  dnsServer.start(53, "*", AP_IP);             // todas las consultas DNS -> 192.168.4.1

  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/state", HTTP_GET, handleState);
  server.on("/api/scan", HTTP_POST, handleScan);
  server.on("/api/info", HTTP_POST, handleInfo);
  server.on("/api/measure", HTTP_POST, handleMeasure);
  server.on("/api/chid", HTTP_POST, handleChid);
  server.onNotFound(redirectToPortal);
  server.begin();

  Serial.printf("Red %s (clave %s) -> http://%s/\n", ssid, AP_PASS, AP_IP.toString().c_str());
}

void loop() {
  dnsServer.processNextRequest();   // en el core 3.x el DNS es asincrono; no hace dano
  server.handleClient();
  jobStep();

  if (STATUS_LED >= 0) {            // LED: parpadeo rapido si trabaja, destello cada 2 s si espera
    unsigned long t = millis();
    bool on = job.type != JOB_NONE ? (t / 150) % 2 : (t % 2000) < 60;
    digitalWrite(STATUS_LED, on ? HIGH : LOW);
  }
  delay(1);
}
