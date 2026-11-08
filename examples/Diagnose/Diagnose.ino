/*!
 * @file Diagnose.ino
 * @brief Stand-alone WT61P / WT61PC link check for RP2040 (Earle Philhower core).
 *        Does NOT use the WT61P_YPR library, so it tells you whether the problem
 *        is wiring, the module, or the library.
 *
 * Open the Serial Monitor (any baud) and read the report. Then type:
 *   s  - scan again
 *   r  - try to restore the module's factory settings at every baud
 *        (back to 9600 baud, 10 Hz, default packets), then rescan.
 *        Note: this also clears any calibration saved on the module.
 *
 * Wiring (Pico): module TX -> GP1, module RX -> GP0, VCC -> 3V3, GND -> GND.
 */
#include <Arduino.h>

#define IMU_SERIAL  Serial1
#define IMU_RX_PIN  1      // RP2040 pin connected to the module's TX
#define IMU_TX_PIN  0      // RP2040 pin connected to the module's RX
#define LISTEN_MS   1000   // listen time per baud rate

static const uint32_t BAUDS[] = {9600, 115200, 921600, 460800, 230400, 57600, 38400, 19200, 4800};
static const uint8_t NBAUD = sizeof(BAUDS) / sizeof(BAUDS[0]);

struct Stats {
  uint32_t bytes, headers, valid, csErr;
  uint32_t perType[11];   // 0x50..0x5A
  uint8_t first[24];
  uint8_t nFirst;
};

static void openAt(uint32_t baud)
{
  IMU_SERIAL.end();
  IMU_SERIAL.begin(baud);
  delay(5);
  while (IMU_SERIAL.available()) IMU_SERIAL.read();
}

static void listen(uint32_t baud, Stats &s)
{
  memset(&s, 0, sizeof(s));
  openAt(baud);
  uint8_t buf[11];
  uint8_t idx = 0;
  const uint32_t t0 = millis();
  while (millis() - t0 < LISTEN_MS) {
    while (IMU_SERIAL.available()) {
      const uint8_t c = IMU_SERIAL.read();
      s.bytes++;
      if (s.nFirst < sizeof(s.first)) s.first[s.nFirst++] = c;
      if (c == 0x55) s.headers++;
      // Simple framer, good enough for counting.
      if (idx == 0 && c != 0x55) continue;
      if (idx == 1 && (c < 0x50 || c > 0x5A)) { idx = (c == 0x55) ? 1 : 0; continue; }
      buf[idx++] = c;
      if (idx == 11) {
        uint8_t sum = 0;
        for (uint8_t i = 0; i < 10; i++) sum += buf[i];
        if (sum == buf[10]) { s.valid++; s.perType[buf[1] - 0x50]++; }
        else s.csErr++;
        idx = 0;
      }
    }
  }
}

static void printStats(uint32_t baud, const Stats &s)
{
  Serial.printf("%7lu baud: %5lu bytes, %4lu x 0x55, %4lu valid pkts, %4lu bad checksums",
                (unsigned long)baud, (unsigned long)s.bytes, (unsigned long)s.headers,
                (unsigned long)s.valid, (unsigned long)s.csErr);
  if (s.valid) {
    Serial.print("  types:");
    for (uint8_t i = 0; i < 11; i++) {
      if (s.perType[i]) Serial.printf(" 0x%02X x%lu", 0x50 + i, (unsigned long)s.perType[i]);
    }
  }
  Serial.println();
  if (s.nFirst) {
    Serial.print("               first bytes:");
    for (uint8_t i = 0; i < s.nFirst; i++) Serial.printf(" %02X", s.first[i]);
    Serial.println();
  }
}

static void rxLineTest()
{
  // A UART line idles HIGH. With a pull-down on our RX pin, HIGH means the
  // module's TX is connected and powered; LOW means nothing is driving it.
  IMU_SERIAL.end();
  pinMode(IMU_RX_PIN, INPUT_PULLDOWN);
  delay(2);
  uint16_t highs = 0;
  for (uint16_t i = 0; i < 1000; i++) {
    highs += digitalRead(IMU_RX_PIN);
    delayMicroseconds(100);
  }
  Serial.printf("RX line test on GP%d (pull-down): HIGH %u/1000 samples -> ", IMU_RX_PIN, highs);
  if (highs > 900)      Serial.println("driven high: module TX looks connected and powered.");
  else if (highs < 10)  Serial.println("LOW: nothing is driving this pin. Check module TX -> this pin, GND, and power.");
  else                  Serial.println("toggling: data is arriving on this pin.");
}

static void scanAll()
{
  Serial.println();
  Serial.println("=== WT61P link check ===");
  rxLineTest();
  Serial.printf("Listening %d ms at each baud rate...\n", LISTEN_MS);

  uint32_t bestBaud = 0, bestValid = 0, totalBytes = 0;
  Stats s;
  for (uint8_t i = 0; i < NBAUD; i++) {
    listen(BAUDS[i], s);
    printStats(BAUDS[i], s);
    totalBytes += s.bytes;
    if (s.valid > bestValid) { bestValid = s.valid; bestBaud = BAUDS[i]; }
  }

  Serial.println();
  if (bestValid >= 2) {
    Serial.printf("RESULT: module found at %lu baud (%lu valid packets/s).\n",
                  (unsigned long)bestBaud, (unsigned long)bestValid);
    Serial.println("        Wiring is fine. If WT61P_YPR still fails, paste this report.");
  } else if (totalBytes == 0) {
    Serial.println("RESULT: no bytes at all at any baud rate.");
    Serial.println("        -> wiring/power (module TX must go to GP1, shared GND),");
    Serial.println("           or the module's output is switched off. Try 'r'.");
  } else {
    Serial.println("RESULT: bytes arrive but no valid packets at any baud rate.");
    Serial.println("        -> the module is probably at a baud the link can't carry cleanly");
    Serial.println("           (e.g. 921600 over long/loose wires). Try 'r' to restore 9600.");
  }
  Serial.println("Type 's' to scan again, 'r' to restore factory settings.");
}

static void sendCmd(uint8_t reg, uint8_t lo, uint8_t hi)
{
  const uint8_t c[5] = {0xFF, 0xAA, reg, lo, hi};
  IMU_SERIAL.write(c, 5);
  IMU_SERIAL.flush();
}

static void factoryRestoreAllBauds()
{
  Serial.println("Sending 'restore defaults' at every baud rate...");
  for (uint8_t i = 0; i < NBAUD; i++) {
    openAt(BAUDS[i]);
    sendCmd(0x69, 0x88, 0xB5);   // unlock
    delay(50);
    sendCmd(0x00, 0x01, 0x00);   // SAVE = 1: restore default settings
    delay(200);
  }
  Serial.println("Done. Power-cycle the module now (unplug its VCC for a second),");
  Serial.println("then type 's' to rescan. It should appear at 9600 baud.");
}

void setup()
{
  Serial.begin(115200);
  const uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 5000) {}   // wait for the Serial Monitor

  IMU_SERIAL.setRX(IMU_RX_PIN);
  IMU_SERIAL.setTX(IMU_TX_PIN);
  IMU_SERIAL.setFIFOSize(1024);

  scanAll();
}

void loop()
{
  if (Serial.available()) {
    const char c = Serial.read();
    if (c == 's' || c == 'S') scanAll();
    if (c == 'r' || c == 'R') factoryRestoreAllBauds();
  }
}
