/*!
 * @file  DFRobot_WT61PC.h
 * @brief Non-blocking, angle-only (roll / pitch / yaw) driver for the
 *        DFRobot WT61PC (SEN0386) == WitMotion WT61P serial AHRS module.
 *
 * Design goals
 *  - update() never waits: it only consumes bytes already sitting in the
 *    UART receive buffer, parses them with a byte-wise state machine and
 *    returns immediately. Cost per call is O(bytes buffered), typically a few
 *    microseconds.
 *  - Only the 0x53 angle packet is decoded; every other packet type is
 *    checksum-validated and dropped.
 *  - begin() (call once from setup()) finds the module at whatever baud rate
 *    it is currently using, switches it to angle-only output at the requested
 *    rate and baud, saves that to the module's flash and re-opens the UART at
 *    the new baud. Flash is only written when the configuration actually has
 *    to change, so normal boots do not wear the module's flash.
 *
 * @copyright Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license   The MIT License (MIT)
 * @version   V2.0
 */
#ifndef __DFRobot_WT61PC_H__
#define __DFRobot_WT61PC_H__

#include <Arduino.h>

// Output-rate codes (register 0x03, RATE). Names kept from V1 for
// compatibility. NB: per the WitMotion datasheet code 0x01 is 0.2 Hz.
#define FREQUENCY_0_1HZ   0x01
#define FREQUENCY_0_2HZ   0x01
#define FREQUENCY_0_5HZ   0x02
#define FREQUENCY_1HZ     0x03
#define FREQUENCY_2HZ     0x04
#define FREQUENCY_5HZ     0x05
#define FREQUENCY_10HZ    0x06
#define FREQUENCY_20HZ    0x07
#define FREQUENCY_50HZ    0x08
#define FREQUENCY_100HZ   0x09
#define FREQUENCY_125HZ   0x0A
#define FREQUENCY_200HZ   0x0B

// Fastest baud rate the module supports (datasheet 5.2.10).
#ifndef WT61PC_DEFAULT_BAUD
#define WT61PC_DEFAULT_BAUD 921600UL
#endif

class DFRobot_WT61PC
{
public:
  typedef struct {
    float X;   // roll  (deg, -180..180)
    float Y;   // pitch (deg,  -90..90)
    float Z;   // yaw   (deg, -180..180)
  } sSensorData_t;

  /** Latest angles. X = roll, Y = pitch, Z = yaw (degrees). Kept for V1 code. */
  sSensorData_t Angle;

  /**
   * @param port Hardware (or PIO) serial port wired to the module, e.g. Serial1.
   *             On the Earle Philhower RP2040 core set the pins / FIFO size
   *             (Serial1.setRX(), setTX(), setFIFOSize()) before calling begin().
   */
  explicit DFRobot_WT61PC(HardwareSerial &port);
  explicit DFRobot_WT61PC(HardwareSerial *port);

  /**
   * @fn begin
   * @brief One-time setup (BLOCKING, call from setup()).
   *        Opens the port at @p baud. If the module is not already streaming
   *        angle-only data at that baud and rate, it scans all supported baud
   *        rates, then writes: angle-only output, @p rate, @p baud, and saves.
   * @param baud     Target baud rate (4800 ... 921600). Default 921600.
   * @param rate     Output-rate code, FREQUENCY_xxx. Default 200 Hz.
   * @param probeMs  Listen time per baud rate while searching. Must cover at
   *                 least two packets at the module's current output rate
   *                 (factory default 10 Hz -> 250 ms is plenty).
   * @return true if angle packets are being received at @p baud afterwards.
   *         Worst case (no module connected) takes ~10 * probeMs.
   */
  bool begin(uint32_t baud = WT61PC_DEFAULT_BAUD,
             uint8_t rate = FREQUENCY_200HZ,
             uint16_t probeMs = 250);

  /**
   * @fn update
   * @brief NON-BLOCKING. Parse whatever bytes are already buffered.
   *        Call as often as you like from loop().
   * @return true if at least one new angle packet was decoded since the last call.
   */
  bool update(void);

  /** V1-compatible alias of update(). Now non-blocking. */
  bool available(void) { return update(); }

  // Latest angles in degrees (Z-Y-X Euler order, see datasheet 3.3).
  float roll(void)  const { return Angle.X; }
  float pitch(void) const { return Angle.Y; }
  float yaw(void)   const { return Angle.Z; }

  // Raw signed register values; degrees = raw * 180 / 32768.
  int16_t rawRoll(void)  const { return _raw[0]; }
  int16_t rawPitch(void) const { return _raw[1]; }
  int16_t rawYaw(void)   const { return _raw[2]; }

  /** micros() timestamp at which the last angle packet completed. */
  uint32_t lastUpdateMicros(void) const { return _lastMicros; }
  /** Milliseconds since the last angle packet (large if none ever arrived). */
  uint32_t age(void) const;
  /** Number of good angle packets decoded. */
  uint32_t packetCount(void) const { return _packets; }
  /** Number of checksum failures seen (noise / UART overrun indicator). */
  uint32_t errorCount(void) const { return _errors; }
  /** Baud rate the host UART is currently running at. */
  uint32_t baud(void) const { return _baud; }

  // ---- Low-level configuration commands (non-blocking, 5 bytes each). ----
  // Newer WT61P firmware needs unlock() before a write and save() after it.
  void unlock(void);                                    // FF AA 69 88 B5
  void save(void);                                      // FF AA 00 00 00
  void setRate(uint8_t rateCode);                       // FF AA 03 rate 00
  void setOutputContent(uint8_t rswl, uint8_t rswh = 0); // FF AA 02 L H
  /** Tell the module to change baud. Does NOT change the host UART. */
  bool setBaud(uint32_t baud);                          // FF AA 04 code 00

  /** V1-compatible alias of setRate(). */
  void modifyFrequency(uint8_t frequency) { setRate(frequency); }

  /** Baud -> register code (0x01..0x09), or 0 if unsupported. */
  static uint8_t baudCode(uint32_t baud);

private:
  enum {
    PKT_LEN    = 11,
    HDR        = 0x55,
    PKT_ANGLE  = 0x53,
    TYPE_MIN   = 0x50,
    TYPE_MAX   = 0x5A,
    RSW_ANGLE  = 0x08,   // RSWL bit 3 -> 0x53 packet only
  };

  struct ProbeResult {
    uint16_t valid;      // any packet with a good checksum
    uint16_t angle;      // 0x53 packets
    uint16_t other;      // non-0x53 packets
  };

  HardwareSerial &_s;
  uint32_t _baud;

  // Parser state
  uint8_t  _buf[PKT_LEN];
  uint8_t  _idx;
  uint8_t  _lastType;

  // Results
  int16_t  _raw[3];
  uint32_t _lastMicros;
  uint32_t _lastMillis;
  uint32_t _packets;
  uint32_t _errors;
  bool     _everReceived;

  uint8_t feed(uint8_t c);      // returns packet type when one completes, else 0
  void    resync(void);
  void    decodeAngle(void);
  void    writeCmd(uint8_t reg, uint8_t lo, uint8_t hi);
  void    openPort(uint32_t baud);
  void    drainInput(void);
  void    probe(uint16_t ms, bool stopEarly, ProbeResult &r);
};

#endif
