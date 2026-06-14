/*
  RunCam camera-control bench test  —  FFCv3 -> RunCam Split
  Pitt SOAR

  Verifies the camera control the flight firmware uses, BEFORE trusting it in flight.
  It speaks the RunCam Device Protocol on the same UART/baud the FC uses and, crucially,
  reads GET_DEVICE_INFO so you can confirm the camera actually advertises the explicit
  START/STOP recording features (not just the power-button toggle).

  WIRING (matches FFCV3_PIN_DEFINITION.h: RX_1_RC_TX = PB12, TX_1_RC_RX = PB13):
     FC PB13 (TX) ---> camera RX
     FC PB12 (RX) <--- camera TX
     common GND, 115200 8N1
  Power the camera as you would in flight.

  USAGE: flash this, open the USB serial monitor at 115200, and use the menu.
     i  -> GET_DEVICE_INFO  (run FIRST: prints protocol version + feature support)
     r  -> START recording  (0xCC 0x01 0x03)
     s  -> STOP recording   (0xCC 0x01 0x04)
     p  -> Simulate POWER button toggle (0xCC 0x01 0x01) -- legacy/fallback compare
  Every frame sent and every byte received is printed in hex for logic-analyzer cross-check.
*/

#include <Arduino.h>

// ---- RunCam UART (same pins as the flight code) ----
#define RC_RX PB12   // FC RX  <- camera TX
#define RC_TX PB13   // FC TX  -> camera RX
HardwareSerial rcSerial(RC_RX, RC_TX);

// ---- RunCam Device Protocol constants ----
#define RC_HEADER                 0xCC
#define RC_CMD_GET_DEVICE_INFO    0x00
#define RC_CMD_CAMERA_CONTROL     0x01
#define RC_ACT_SIM_POWER          0x01   // toggle (legacy)
#define RC_ACT_START_RECORDING    0x03   // explicit start
#define RC_ACT_STOP_RECORDING     0x04   // explicit stop

// Feature bitmask (from the protocol's GET_DEVICE_INFO response)
#define FEAT_SIM_POWER   (1 << 0)
#define FEAT_SIM_WIFI    (1 << 1)
#define FEAT_CHANGE_MODE (1 << 2)
#define FEAT_START_REC   (1 << 6)
#define FEAT_STOP_REC    (1 << 7)

// ---- crc8 dvb-s2 (poly 0xD5) -- identical to the FC's calcCrc/crc8_calc ----
uint8_t crc8(uint8_t crc, uint8_t a) {
  crc ^= a;
  for (int i = 0; i < 8; i++)
    crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
  return crc;
}
uint8_t crc8Buf(const uint8_t *b, uint8_t n) {
  uint8_t c = 0;
  for (uint8_t i = 0; i < n; i++) c = crc8(c, b[i]);
  return c;
}

void printHex(const uint8_t *b, uint8_t n) {
  for (uint8_t i = 0; i < n; i++) {
    if (b[i] < 0x10) Serial.print('0');
    Serial.print(b[i], HEX);
    Serial.print(' ');
  }
  Serial.println();
}

// Send a 4-byte CAMERA_CONTROL frame (header, cmd, action, crc).
void sendCamAction(uint8_t action) {
  uint8_t f[4] = { RC_HEADER, RC_CMD_CAMERA_CONTROL, action, 0 };
  f[3] = crc8Buf(f, 3);
  rcSerial.write(f, 4);
  Serial.print(F("  TX: "));
  printHex(f, 4);
}

// Read whatever the camera sends back within a timeout (control cmds may be silent).
uint8_t readResponse(uint8_t *buf, uint8_t maxLen, uint16_t timeoutMs) {
  uint8_t n = 0;
  unsigned long t0 = millis();
  while (millis() - t0 < timeoutMs && n < maxLen) {
    if (rcSerial.available()) { buf[n++] = rcSerial.read(); t0 = millis(); }
  }
  return n;
}

// Send GET_DEVICE_INFO and decode the response: protocol version + feature support.
void getDeviceInfo() {
  uint8_t req[3] = { RC_HEADER, RC_CMD_GET_DEVICE_INFO, 0 };
  req[2] = crc8Buf(req, 2);

  while (rcSerial.available()) rcSerial.read();   // flush stale rx
  rcSerial.write(req, 3);
  Serial.print(F("  TX: "));
  printHex(req, 3);

  uint8_t resp[16];
  uint8_t n = readResponse(resp, sizeof(resp), 300);
  if (n == 0) {
    Serial.println(F("  RX: (nothing) -> check wiring/baud/power, and that TX/RX aren't swapped"));
    return;
  }
  Serial.print(F("  RX: "));
  printHex(resp, n);

  // Response: header(0xCC) | protocolVersion | feature(uint16) | crc8
  if (n >= 5 && resp[0] == RC_HEADER) {
    uint8_t  ver  = resp[1];
    uint16_t feat = (uint16_t)resp[2] | ((uint16_t)resp[3] << 8);  // little-endian
    bool crcOK = (crc8Buf(resp, 4) == resp[4]);

    Serial.print(F("  Protocol version: "));  Serial.println(ver);
    Serial.print(F("  Feature word: 0x"));     Serial.print(feat, HEX);
    Serial.println(crcOK ? F("  (crc OK)") : F("  (crc MISMATCH - byte order? see raw bytes)"));
    Serial.print(F("    Simulate Power : ")); Serial.println((feat & FEAT_SIM_POWER)   ? F("yes") : F("no"));
    Serial.print(F("    Simulate WiFi  : ")); Serial.println((feat & FEAT_SIM_WIFI)    ? F("yes") : F("no"));
    Serial.print(F("    Change Mode    : ")); Serial.println((feat & FEAT_CHANGE_MODE) ? F("yes") : F("no"));
    Serial.print(F("    START recording: ")); Serial.println((feat & FEAT_START_REC)   ? F("yes") : F("no"));
    Serial.print(F("    STOP recording : ")); Serial.println((feat & FEAT_STOP_REC)    ? F("yes") : F("no"));

    if ((feat & FEAT_START_REC) && (feat & FEAT_STOP_REC))
      Serial.println(F("  >> Explicit START/STOP supported. The FC implementation is correct for this camera."));
    else
      Serial.println(F("  >> START/STOP NOT advertised. Use the power-toggle fallback (track state in firmware)."));
  } else {
    Serial.println(F("  Unrecognized response. If feature flags look wrong, the camera may send MSB-first;"));
    Serial.println(F("  the raw RX bytes above are authoritative."));
  }
}

void menu() {
  Serial.println();
  Serial.println(F("=== RunCam control test ==="));
  Serial.println(F("  i = GET_DEVICE_INFO (run first: confirms START/STOP support)"));
  Serial.println(F("  r = START recording"));
  Serial.println(F("  s = STOP recording"));
  Serial.println(F("  p = Simulate POWER button (toggle; legacy/fallback compare)"));
  Serial.print  (F("select> "));
}

void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000) { }
  delay(300);

  rcSerial.begin(115200);
  delay(1000);   // let the camera UART come up (no control command sent here)

  Serial.println(F("FFCv3 <-> RunCam bench test"));
  Serial.println(F("Wiring: PB13(FC TX)->cam RX, PB12(FC RX)<-cam TX, common GND, 115200"));
  Serial.println(F("Tip: run 'i' first; then 'r' should start a recording you can see on the camera."));
  menu();
}

void loop() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c == '\n' || c == '\r' || c == ' ') return;

  switch (c) {
    case 'i':
      Serial.println(F("\n[GET_DEVICE_INFO]"));
      getDeviceInfo();
      break;
    case 'r': {
      Serial.println(F("\n[START recording  -> action 0x03]"));
      sendCamAction(RC_ACT_START_RECORDING);
      uint8_t b[16]; uint8_t n = readResponse(b, sizeof(b), 150);
      if (n) { Serial.print(F("  RX: ")); printHex(b, n); }
      break;
    }
    case 's': {
      Serial.println(F("\n[STOP recording   -> action 0x04]"));
      sendCamAction(RC_ACT_STOP_RECORDING);
      uint8_t b[16]; uint8_t n = readResponse(b, sizeof(b), 150);
      if (n) { Serial.print(F("  RX: ")); printHex(b, n); }
      break;
    }
    case 'p': {
      Serial.println(F("\n[Simulate POWER button -> action 0x01 (TOGGLE)]"));
      sendCamAction(RC_ACT_SIM_POWER);
      uint8_t b[16]; uint8_t n = readResponse(b, sizeof(b), 150);
      if (n) { Serial.print(F("  RX: ")); printHex(b, n); }
      break;
    }
    default:
      Serial.print(F("\nunknown key: "));
      Serial.println(c);
      break;
  }
  menu();
}
