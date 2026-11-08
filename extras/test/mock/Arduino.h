// Minimal host-side Arduino shim for unit-testing the WT61PC parser.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

extern uint64_t g_us;   // virtual time in microseconds
inline uint32_t micros() { g_us += 2; return (uint32_t)g_us; }
inline uint32_t millis() { g_us += 2; return (uint32_t)(g_us / 1000); }
inline void delay(uint32_t ms) { g_us += (uint64_t)ms * 1000; }
inline void yield() { g_us += 20; }

class HardwareSerial {
public:
  virtual ~HardwareSerial() {}
  virtual void begin(unsigned long baud) = 0;
  virtual void end() = 0;
  virtual int available() = 0;
  virtual int read() = 0;
  virtual void flush() = 0;
  virtual size_t write(const uint8_t *buf, size_t n) = 0;
};
