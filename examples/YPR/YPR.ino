/*!
 * @file YPR.ino
 * @brief Non-blocking yaw / pitch / roll from a WitMotion WT61P (DFRobot WT61PC) on an
 *        RP2040 (Earle Philhower arduino-pico core).
 *
 * Wiring (Raspberry Pi Pico, Serial1 = UART0):
 *   WT61PC VCC -> 3V3      (3.3 V keeps the module's TX at RP2040-safe levels)
 *   WT61PC GND -> GND
 *   WT61PC TX  -> GP1 (Serial1 RX)
 *   WT61PC RX  -> GP0 (Serial1 TX)
 *
 * First boot: begin() finds the module at its factory 9600 baud, switches it to
 * angle-only output at 200 Hz and 921600 baud and saves that. Power-cycle the
 * module once afterwards in case its firmware only applies the rate on reboot.
 * Later boots detect it at 921600 straight away and write nothing.
 *
 * loop() never waits on the sensor: imu.update() only parses bytes that are
 * already buffered, and printing is skipped if the USB buffer is full.
 *
 * @license The MIT License (MIT)
 */
#include <WT61P_YPR.h>

#define IMU_SERIAL Serial1
#define IMU_RX_PIN 1
#define IMU_TX_PIN 0

WT61P_YPR imu(IMU_SERIAL);

// Format centi-degrees as "-123.45" without float printf.
static char *fmtCenti(char *p, int32_t cd)
{
  if (cd < 0) { *p++ = '-'; cd = -cd; }
  p += sprintf(p, "%ld.%02ld", (long)(cd / 100), (long)(cd % 100));
  return p;
}

static int32_t rawToCenti(int16_t raw)
{
  return ((int32_t)raw * 18000L) / 32768L;   // 180 deg full scale, 0.01 deg units
}

void setup()
{
  Serial.begin(115200);   // USB CDC: the number is ignored, it runs at USB speed

#if defined(ARDUINO_ARCH_RP2040)
  IMU_SERIAL.setRX(IMU_RX_PIN);
  IMU_SERIAL.setTX(IMU_TX_PIN);
  IMU_SERIAL.setFIFOSize(256);   // ~23 packets of slack if loop() ever stalls
#endif

  // 921600 baud, 200 Hz, angle packets only.
  while (!imu.begin(WT61P_DEFAULT_BAUD, WT61P_YPR::RATE_200HZ)) {
    Serial.println("WT61P not found, retrying...");
    delay(200);
  }
  Serial.print("WT61P ready at ");
  Serial.print(imu.baud());
  Serial.println(" baud");
}

void loop()
{
  if (imu.update()) {
    // CSV: yaw,pitch,roll (degrees)
    char line[40];
    char *p = line;
    p = fmtCenti(p, rawToCenti(imu.rawYaw()));   *p++ = ',';
    p = fmtCenti(p, rawToCenti(imu.rawPitch())); *p++ = ',';
    p = fmtCenti(p, rawToCenti(imu.rawRoll()));  *p++ = '\n';
    const size_t len = p - line;
    if ((size_t)Serial.availableForWrite() >= len) {
      Serial.write((const uint8_t *)line, len);
    }
    // Floats are also available: imu.yaw(), imu.pitch(), imu.roll()
  }

  if (imu.age() > 100) {
    // No angle packet for 100 ms: sensor unplugged / browned out.
  }

  // ... the rest of your application runs here at full speed ...
}
