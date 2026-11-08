/*!
 * @file  WT61P_YPR.cpp
 * @brief Non-blocking, angle-only driver for the WitMotion WT61P (DFRobot WT61PC).
 * @copyright Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license   The MIT License (MIT)
 * @version   V1.0
 */
#include "WT61P_YPR.h"
#include <string.h>

static const float ANGLE_SCALE = 180.0f / 32768.0f;

// Output rate in tenths of Hz, indexed by (rate code - 1).
static const uint16_t RATE_HZ10[] = {2, 5, 10, 20, 50, 100, 200, 500, 1000, 1250, 2000};

WT61P_YPR::WT61P_YPR(HardwareSerial &port)
  : _s(port), _baud(0), _idx(0), _lastType(0),
    _lastMicros(0), _lastMillis(0), _packets(0), _errors(0), _everReceived(false),
    _powerCycleNeeded(false)
{
  _raw[0] = _raw[1] = _raw[2] = 0;
  Angle.X = Angle.Y = Angle.Z = 0.0f;
}

WT61P_YPR::WT61P_YPR(HardwareSerial *port) : WT61P_YPR(*port) {}

/* ------------------------------------------------------------------------ */
/*  Hot path                                                                */
/* ------------------------------------------------------------------------ */

bool WT61P_YPR::update(void)
{
  // Only touch bytes that are already buffered -> never waits.
  int n = _s.available();
  bool fresh = false;
  while (n-- > 0) {
    int c = _s.read();
    if (c < 0) {
      break;
    }
    if (feed((uint8_t)c) == PKT_ANGLE) {
      fresh = true;
    }
  }
  if (fresh) {
    // Convert once, for the newest packet only.
    Angle.X = _raw[0] * ANGLE_SCALE;
    Angle.Y = _raw[1] * ANGLE_SCALE;
    Angle.Z = _raw[2] * ANGLE_SCALE;
  }
  return fresh;
}

// Byte-wise packet framer: 0x55 <type> <8 data bytes> <sum>.
uint8_t WT61P_YPR::feed(uint8_t c)
{
  if (_idx == 0) {
    if (c == HDR) {
      _buf[_idx++] = c;
    }
    return 0;
  }

  _buf[_idx++] = c;

  if (_idx == 2) {
    if (c < TYPE_MIN || c > TYPE_MAX) {
      resync();
    }
    return 0;
  }
  if (_idx < PKT_LEN) {
    return 0;
  }

  uint8_t sum = 0;
  for (uint8_t i = 0; i < PKT_LEN - 1; i++) {
    sum += _buf[i];
  }
  if (sum != _buf[PKT_LEN - 1]) {
    _errors++;
    resync();
    return 0;
  }

  _idx = 0;
  _lastType = _buf[1];
  if (_lastType == PKT_ANGLE) {
    decodeAngle();
  }
  return _lastType;
}

// Drop the current header byte and restart framing at the next plausible
// header already in the buffer, so a corrupted packet never costs us the
// packet that follows it.
void WT61P_YPR::resync(void)
{
  const uint8_t n = _idx;
  uint8_t k = 1;
  for (;;) {
    while (k < n && _buf[k] != HDR) {
      k++;
    }
    if (k >= n) {
      _idx = 0;
      return;
    }
    const uint8_t m = n - k;
    if (m >= 2 && (_buf[k + 1] < TYPE_MIN || _buf[k + 1] > TYPE_MAX)) {
      k++;
      continue;
    }
    memmove(_buf, _buf + k, m);
    _idx = m;
    return;
  }
}

void WT61P_YPR::decodeAngle(void)
{
  // Little-endian signed 16-bit. The int16_t cast matters on 32-bit MCUs
  // (RP2040, ESP32, ...): without it negative angles read as 180..360 deg.
  _raw[0] = (int16_t)(((uint16_t)_buf[3] << 8) | _buf[2]);   // roll  (X)
  _raw[1] = (int16_t)(((uint16_t)_buf[5] << 8) | _buf[4]);   // pitch (Y)
  _raw[2] = (int16_t)(((uint16_t)_buf[7] << 8) | _buf[6]);   // yaw   (Z)
  _lastMicros = micros();
  _lastMillis = millis();
  _packets++;
  _everReceived = true;
}

uint32_t WT61P_YPR::age(void) const
{
  if (!_everReceived) {
    return 0xFFFFFFFFUL;
  }
  return millis() - _lastMillis;
}

/* ------------------------------------------------------------------------ */
/*  Configuration                                                           */
/* ------------------------------------------------------------------------ */

void WT61P_YPR::writeCmd(uint8_t reg, uint8_t lo, uint8_t hi)
{
  const uint8_t cmd[5] = {0xFF, 0xAA, reg, lo, hi};
  _s.write(cmd, sizeof(cmd));
}

void WT61P_YPR::unlock(void)                          { writeCmd(0x69, 0x88, 0xB5); }
void WT61P_YPR::save(void)                            { writeCmd(0x00, 0x00, 0x00); }
void WT61P_YPR::setRate(uint8_t rateCode)             { writeCmd(0x03, rateCode, 0x00); }
void WT61P_YPR::setOutputContent(uint8_t l, uint8_t h) { writeCmd(0x02, l, h); }

bool WT61P_YPR::setBaud(uint32_t baud)
{
  const uint8_t code = baudCode(baud);
  if (!code) {
    return false;
  }
  writeCmd(0x04, code, 0x00);
  return true;
}

uint8_t WT61P_YPR::baudCode(uint32_t baud)
{
  switch (baud) {
    case 4800:   return 0x01;
    case 9600:   return 0x02;
    case 19200:  return 0x03;
    case 38400:  return 0x04;
    case 57600:  return 0x05;
    case 115200: return 0x06;
    case 230400: return 0x07;
    case 460800: return 0x08;
    case 921600: return 0x09;
    default:     return 0;
  }
}

void WT61P_YPR::drainInput(void)
{
  for (uint16_t i = 0; i < 2048 && _s.available() > 0; i++) {
    _s.read();
  }
  _idx = 0;
}

void WT61P_YPR::openPort(uint32_t baud)
{
  _s.end();
  _s.begin(baud);
  _baud = baud;
  drainInput();
}

void WT61P_YPR::probe(uint16_t ms, bool stopEarly, ProbeResult &r)
{
  r.valid = r.angle = r.other = 0;
  _idx = 0;
  const uint32_t t0 = millis();
  while ((uint32_t)(millis() - t0) < ms) {
    while (_s.available() > 0) {
      int c = _s.read();
      if (c < 0) {
        break;
      }
      const uint8_t t = feed((uint8_t)c);
      if (t) {
        r.valid++;
        if (t == PKT_ANGLE) {
          r.angle++;
        } else {
          r.other++;
        }
      }
    }
    if (stopEarly && r.valid >= 2) {
      return;
    }
    yield();
  }
}

bool WT61P_YPR::begin(uint32_t baud, uint8_t rate, uint16_t probeMs)
{
  if (!baudCode(baud) || rate < RATE_0_2HZ || rate > RATE_200HZ) {
    return false;
  }

  // 1) Is it already talking at the target baud?
  ProbeResult r;
  uint32_t found = 0;
  openPort(baud);
  probe(probeMs, false, r);
  if (r.valid >= 2) {
    found = baud;
  }

  // 2) Otherwise scan, factory default first.
  static const uint32_t SCAN[] = {9600, 921600, 460800, 230400, 115200, 57600, 38400, 19200, 4800};
  for (uint8_t i = 0; !found && i < sizeof(SCAN) / sizeof(SCAN[0]); i++) {
    if (SCAN[i] == baud) {
      continue;
    }
    openPort(SCAN[i]);
    probe(probeMs, true, r);
    if (r.valid >= 2) {
      found = SCAN[i];
    }
  }
  if (!found) {
    openPort(baud);
    return false;
  }

  // 3) Decide whether the module's stored config needs rewriting. Only do it
  //    when necessary so normal boots never write the module's flash.
  _powerCycleNeeded = false;
  const bool reconfigure = (found != baud) || rateTooLow(r, rate, baud, probeMs);

  if (reconfigure) {
    unlock();                       _s.flush(); delay(30);
    setOutputContent(RSW_ANGLE, 0); _s.flush(); delay(30);   // 0x53 packets only
    setRate(rate);                  _s.flush(); delay(30);
    save();                         _s.flush(); delay(150);

    if (found != baud) {
      unlock();                     _s.flush(); delay(30);
      setBaud(baud);                _s.flush(); delay(30);
      // Some firmware applies the new baud immediately, some only on save:
      // save at both bauds so either kind ends up persisted.
      save();                       _s.flush(); delay(150);
      openPort(baud);               delay(30);
      unlock();                     _s.flush(); delay(30);
      save();                       _s.flush(); delay(150);
    }
  }

  // 4) Verify (already verified by step 1 if nothing was written).
  if (reconfigure) {
    drainInput();
    probe(probeMs, false, r);

    if (r.angle == 0 && found != baud) {
      // Some firmware saves a new baud but keeps using the old one until it
      // is power-cycled. Keep talking at the old baud for this session; the
      // next begin() after a power cycle finds it at the new baud and writes
      // nothing.
      openPort(found);
      probe(probeMs, false, r);
      _powerCycleNeeded = r.angle > 0;
    } else if (r.angle > 0 && rateTooLow(r, rate, _baud, probeMs)) {
      // Rate / output content saved but not applied until a power cycle.
      _powerCycleNeeded = true;
    }
  }

  _packets = 0;
  _errors = 0;
  return r.angle > 0;
}

// True if fewer than 60 % of the angle packets expected at @p rate arrived
// (capped at what @p baud can carry).
bool WT61P_YPR::rateTooLow(const ProbeResult &r, uint8_t rate, uint32_t baud, uint16_t probeMs)
{
  uint32_t wantHz10 = RATE_HZ10[rate - 1];
  const uint32_t maxHz10 = baud / 11;              // 11-byte packets, 10 bits/byte
  if (wantHz10 > maxHz10) {
    wantHz10 = maxHz10;
  }
  return (uint64_t)r.angle * 100000ULL < (uint64_t)wantHz10 * probeMs * 6ULL;
}
