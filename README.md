# DFRobot_WT61PC (V2: non-blocking, angle-only)
* [中文版](./README_CN.md)

Driver for the DFRobot WT61PC (SKU: SEN0386). The module is a WitMotion **WT61P**
AHRS IMU with a DFRobot sticker, so it follows the WitMotion WT61P serial protocol.

V2 is a rewrite focused on speed and keeping `loop()` responsive:

- **`update()` never blocks.** It only parses bytes already in the UART receive
  buffer, using a byte-wise state machine, and returns straight away. V1's
  `available()` could busy-wait for up to 3 × 5 s.
- **Angles (roll / pitch / yaw) only.** The module is set to send just the 0x53
  angle packet, and any other packet is validated and dropped.
- **921600 baud, 200 Hz by default.** `begin()` finds the module at whatever baud
  it currently uses (factory default 9600), sets it to angle-only / 200 Hz /
  921600 and saves that to the module. The module's flash is only written when
  something has to change, so a normal boot sends no commands at all.
- **Fixes negative angles on 32-bit MCUs.** On RP2040/ESP32, V1 combined the
  bytes into a 32-bit `int` without a signed 16-bit cast, so −1° read as 359°.
- Resynchronises after noise or dropped bytes without losing the next good packet.

![WT61PC](./resources/images/WT61PC.png)

Product link: https://www.dfrobot.com/product-2200.html

## Installation

1. Download [`dist/DFRobot_WT61PC-2.0.0.zip`](./dist/DFRobot_WT61PC-2.0.0.zip)
   (on GitHub: open the file, then click **Download raw file**).
2. Arduino IDE: **Sketch → Include Library → Add .ZIP Library…** and select the zip.
3. The example is under **File → Examples → DFRobot_WT61PC → getData**.

## Wiring (Raspberry Pi Pico / RP2040, Earle Philhower core)

| WT61PC | Pico |
| ------ | ---- |
| VCC    | 3V3 (keeps the module's TX at RP2040-safe 3.3 V) |
| GND    | GND |
| TX     | GP1 (Serial1 RX) |
| RX     | GP0 (Serial1 TX) |

## Usage

```C++
#include <DFRobot_WT61PC.h>
DFRobot_WT61PC imu(Serial1);

void setup() {
  Serial1.setRX(1);
  Serial1.setTX(0);
  Serial1.setFIFOSize(256);          // default is 32 bytes (~3 packets)
  while (!imu.begin()) delay(200);   // 921600 baud, 200 Hz, angle-only
}

void loop() {
  if (imu.update()) {                // non-blocking
    float y = imu.yaw(), p = imu.pitch(), r = imu.roll();
    // ...
  }
  // rest of the program runs at full speed
}
```

See `example/getData/getData.ino` for a complete sketch that streams
`yaw,pitch,roll` CSV over USB without ever blocking on the USB buffer either.

**First boot:** after `begin()` has reconfigured a factory module, power-cycle the
module once. The datasheet says a changed output rate only takes effect after a
re-power on some firmware.

## Methods

```C++
  DFRobot_WT61PC(HardwareSerial &port);   // also accepts &Serial1 (V1 style)

  // BLOCKING, call once from setup(). Takes ~250 ms when the module is already
  // configured and ~1.2 s on first-time setup. If nothing answers it gives up
  // after ~10 * probeMs.
  bool begin(uint32_t baud = 921600, uint8_t rate = FREQUENCY_200HZ, uint16_t probeMs = 250);

  bool update(void);       // NON-BLOCKING. true = new angle packet decoded
  bool available(void);    // V1 alias of update()

  float roll(void);        // X, degrees, -180..180
  float pitch(void);       // Y, degrees,  -90..90
  float yaw(void);         // Z, degrees, -180..180
  sSensorData_t Angle;     // V1 compatible: Angle.X/Y/Z = roll/pitch/yaw

  int16_t rawRoll(void), rawPitch(void), rawYaw(void);   // deg = raw * 180 / 32768

  uint32_t age(void);              // ms since last angle packet (0xFFFFFFFF if none yet)
  uint32_t lastUpdateMicros(void); // micros() at last angle packet
  uint32_t packetCount(void);      // good angle packets
  uint32_t errorCount(void);       // checksum failures
  uint32_t baud(void);             // current host UART baud

  // Low-level commands (5 bytes each, non-blocking)
  void unlock(void);
  void save(void);
  void setRate(uint8_t rateCode);               // FREQUENCY_xxx
  void modifyFrequency(uint8_t frequency);      // V1 alias of setRate()
  void setOutputContent(uint8_t rswl, uint8_t rswh = 0);
  bool setBaud(uint32_t baud);                  // module side only
```

### Migrating from V1

- `available()` is still there but is now non-blocking. It returns `false`
  straight away when no complete angle packet is buffered.
- `Acc` and `Gyro` have been removed (angle only). `Angle.X/Y/Z` still works.
- The constructor takes a `HardwareSerial` (Serial1/Serial2, or a SerialPIO on
  RP2040) because `begin()` has to change the host baud rate.

### Baud rate notes

At 200 Hz angle-only output the module sends 2200 bytes/s, which any baud from
38400 up can carry. A higher baud mostly reduces latency: one packet takes
0.12 ms on the wire at 921600, against 11.5 ms at 9600. If long or noisy wires
make `errorCount()` climb, use `imu.begin(460800)` or `imu.begin(230400)`.
`begin()` will find the module and move it to the new baud.

## Tests

`extras/test` holds a host-side test with a simulated WT61P:

```
cd extras/test
g++ -std=c++11 -Wall -Imock -I../.. test_wt61pc.cpp ../../DFRobot_WT61PC.cpp && ./a.out
```

## History

- 2026/11/08 - Version 2.0.0: non-blocking angle-only rewrite, auto baud / config, signed-angle fix.
- 2023/07/12 - Version 1.0.0 released.

## Credits

Originally written by huyujie(yujie.hu@dfrobot.com). (Welcome to our [website](https://www.dfrobot.com/))
