// Host unit test: g++ -std=c++11 -Wall -Imock -I../.. test_wt61pc.cpp ../../WT61P_YPR.cpp
#include "WT61P_YPR.h"
#include <deque>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <cassert>

uint64_t g_us = 0;
static int g_fail = 0;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); g_fail++; } } while (0)

static std::vector<uint8_t> pkt(uint8_t type, int16_t a, int16_t b, int16_t c, int16_t d = 0)
{
  std::vector<uint8_t> p = {0x55, type,
    (uint8_t)a, (uint8_t)((uint16_t)a >> 8), (uint8_t)b, (uint8_t)((uint16_t)b >> 8),
    (uint8_t)c, (uint8_t)((uint16_t)c >> 8), (uint8_t)d, (uint8_t)((uint16_t)d >> 8), 0};
  uint8_t s = 0; for (int i = 0; i < 10; i++) s += p[i]; p[10] = s;
  return p;
}

// ---------------------------------------------------------------------------
// Simulated WT61P: streams packets in virtual time, understands FF AA commands.
// ---------------------------------------------------------------------------
struct FakeWT : public HardwareSerial {
  // persisted (flash) + live config
  uint32_t flashBaud = 9600, flashRate = 0x06, flashRsw = 0x1E;
  uint32_t baud = 9600, rate = 0x06, rsw = 0x1E;
  bool baudOnSave = false;          // firmware variant
  uint32_t pendingBaud = 0;
  bool unlocked = false;
  int saves = 0, cmds = 0;

  uint32_t hostBaud = 0; bool open = false;
  std::deque<uint8_t> rx;           // bytes waiting for host
  uint64_t nextPktUs = 0;
  std::vector<uint8_t> cmdBuf;
  int16_t roll = -1234, pitch = 4321, yaw = -32768;

  static uint32_t rateHz10(uint32_t r) { static const uint16_t t[] = {2,5,10,20,50,100,200,500,1000,1250,2000}; return t[r-1]; }
  static uint32_t codeBaud(uint8_t c) { static const uint32_t t[] = {4800,9600,19200,38400,57600,115200,230400,460800,921600}; return t[c-1]; }

  void powerCycle() { baud = flashBaud; rate = flashRate; rsw = flashRsw; unlocked = false; pendingBaud = 0; nextPktUs = g_us; }

  void pump() {
    while (nextPktUs <= g_us) {
      std::vector<uint8_t> frame;
      if (rsw & 0x02) { auto p = pkt(0x51, 1, 2, 2048); frame.insert(frame.end(), p.begin(), p.end()); }
      if (rsw & 0x04) { auto p = pkt(0x52, 3, 4, 5);    frame.insert(frame.end(), p.begin(), p.end()); }
      if (rsw & 0x08) { auto p = pkt(0x53, roll, pitch, yaw, 0x1234); frame.insert(frame.end(), p.begin(), p.end()); }
      // Bandwidth-limited: a frame can't be sent faster than the wire allows.
      uint64_t wireUs = (uint64_t)frame.size() * 10 * 1000000 / baud;
      uint64_t periodUs = 10000000ULL / rateHz10(rate);
      if (open) {
        if (hostBaud == baud) rx.insert(rx.end(), frame.begin(), frame.end());
        else for (size_t i = 0; i < frame.size() / 3; i++) rx.push_back(rand() & 0xFF);  // framing garbage
      }
      nextPktUs += periodUs > wireUs ? periodUs : wireUs;
    }
  }

  void onCmd(uint8_t reg, uint8_t lo, uint8_t hi) {
    cmds++;
    if (reg == 0x69 && lo == 0x88 && hi == 0xB5) { unlocked = true; return; }
    if (!unlocked) return;
    switch (reg) {
      case 0x02: rsw = lo | (hi << 8); break;
      case 0x03: rate = lo; break;
      case 0x04: if (baudOnSave) pendingBaud = codeBaud(lo); else baud = codeBaud(lo); break;
      case 0x00:
        if (pendingBaud) { baud = pendingBaud; pendingBaud = 0; }
        flashBaud = baud; flashRate = rate; flashRsw = rsw; saves++; break;
    }
  }

  // HardwareSerial
  void begin(unsigned long b) override { hostBaud = b; open = true; nextPktUs = nextPktUs < g_us ? g_us : nextPktUs; }
  void end() override { open = false; rx.clear(); }
  int available() override { pump(); return (int)rx.size(); }
  int read() override { if (rx.empty()) return -1; uint8_t c = rx.front(); rx.pop_front(); return c; }
  void flush() override {}
  size_t write(const uint8_t *b, size_t n) override {
    if (hostBaud != baud) return n;           // module can't decode it
    for (size_t i = 0; i < n; i++) {
      cmdBuf.push_back(b[i]);
      if (cmdBuf.size() == 1 && cmdBuf[0] != 0xFF) cmdBuf.clear();
      else if (cmdBuf.size() == 2 && cmdBuf[1] != 0xAA) cmdBuf.clear();
      else if (cmdBuf.size() == 5) { onCmd(cmdBuf[2], cmdBuf[3], cmdBuf[4]); cmdBuf.clear(); }
    }
    return n;
  }
};

// Plain byte source for parser tests.
struct ByteSrc : public HardwareSerial {
  std::deque<uint8_t> q;
  void begin(unsigned long) override {}
  void end() override {}
  int available() override { return (int)q.size(); }
  int read() override { if (q.empty()) return -1; uint8_t c = q.front(); q.pop_front(); return c; }
  void flush() override {}
  size_t write(const uint8_t *, size_t n) override { return n; }
  void push(const std::vector<uint8_t> &v) { q.insert(q.end(), v.begin(), v.end()); }
};

static bool near(float a, float b) { return std::fabs(a - b) < 0.01f; }

static void testParser()
{
  ByteSrc src;
  WT61P_YPR imu(src);

  // Nothing buffered -> returns immediately, false.
  CHECK(!imu.update());
  CHECK(imu.age() == 0xFFFFFFFFUL);

  // Negative angles must stay negative (V1 bug on 32-bit MCUs).
  src.push(pkt(0x53, -16384, 8192, -1));
  CHECK(imu.update());
  CHECK(near(imu.roll(), -90.0f));
  CHECK(near(imu.pitch(), 45.0f));
  CHECK(near(imu.yaw(), -180.0f / 32768.0f));
  CHECK(imu.rawYaw() == -1);

  // Other packet types are ignored.
  src.push(pkt(0x51, 100, 200, 300));
  src.push(pkt(0x52, 100, 200, 300));
  CHECK(!imu.update());

  // Packet split across many update() calls.
  auto p = pkt(0x53, 100, 200, 300);
  for (size_t i = 0; i < p.size(); i++) {
    src.q.push_back(p[i]);
    bool got = imu.update();
    CHECK(got == (i == p.size() - 1));
  }
  CHECK(imu.rawRoll() == 100 && imu.rawPitch() == 200 && imu.rawYaw() == 300);

  // Corrupted packet followed immediately by a good one: good one must survive.
  auto bad = pkt(0x53, 1, 2, 3); bad[10] ^= 0x5A;
  src.push(bad); src.push(pkt(0x53, 7, 8, 9));
  uint32_t e0 = imu.errorCount();
  CHECK(imu.update());
  CHECK(imu.rawRoll() == 7 && imu.rawYaw() == 9);
  CHECK(imu.errorCount() == e0 + 1);

  // Truncated packet (dropped bytes) followed by a good one.
  auto trunc = pkt(0x53, 1, 2, 3); trunc.resize(6);
  src.push(trunc); src.push(pkt(0x53, 11, 12, 13));
  src.push(pkt(0x53, 21, 22, 23));
  imu.update();
  CHECK(imu.rawRoll() == 21 && imu.rawYaw() == 23);   // latest one wins

  // Packet whose payload is full of 0x55 bytes (header look-alikes).
  src.push({0x55, 0x55, 0x55});
  src.push(pkt(0x53, 0x5555, 0x5355, 0x5555));
  CHECK(imu.update());
  CHECK(imu.rawRoll() == 0x5555 && imu.rawPitch() == 0x5355);

  // Fuzz: random noise interleaved with known packets; every good packet
  // separated by noise must be recovered with the right payload.
  srand(1);
  int recovered = 0, sent = 0;
  for (int k = 0; k < 20000; k++) {
    int nn = rand() % 30;
    for (int i = 0; i < nn; i++) src.q.push_back(rand() & 0xFF);
    int16_t r = (int16_t)(rand() & 0xFFFF);
    src.push(pkt(0x53, r, (int16_t)~r, (int16_t)(r ^ 0x5555)));
    sent++;
    if (imu.update() && imu.rawRoll() == r && imu.rawPitch() == (int16_t)~r && imu.rawYaw() == (int16_t)(r ^ 0x5555)) recovered++;
  }
  printf("fuzz: recovered %d / %d packets after random noise, checksum errors %lu\n",
         recovered, sent, (unsigned long)imu.errorCount());
  CHECK(recovered >= sent * 99 / 100);   // noise can, rarely, forge a valid packet
}

static void testBegin(bool baudOnSave)
{
  FakeWT wt;
  wt.baudOnSave = baudOnSave;
  wt.powerCycle();
  WT61P_YPR imu(wt);

  uint64_t t0 = g_us;
  bool ok = imu.begin();
  printf("begin (factory, baud applies %s): ok=%d took %.0f ms, saves=%d, module baud=%u rate=0x%02X rsw=0x%02X\n",
         baudOnSave ? "on save" : "immediately", ok, (g_us - t0) / 1000.0, wt.saves,
         (unsigned)wt.baud, (unsigned)wt.rate, (unsigned)wt.rsw);
  CHECK(ok);
  CHECK(imu.baud() == 921600);
  CHECK(wt.flashBaud == 921600 && wt.flashRate == 0x0B && wt.flashRsw == 0x08);

  // Normal operation: ~200 packets/s, only angle packets, update() is cheap.
  uint64_t start = g_us; uint32_t n = 0;
  while (g_us - start < 1000000) { if (imu.update()) n++; g_us += 100; }
  printf("  1 s of loop(): %u fresh angle updates\n", (unsigned)n);
  CHECK(n >= 190 && n <= 201);
  CHECK(near(imu.roll(), -1234 * 180.0f / 32768.0f));
  CHECK(near(imu.yaw(), -180.0f));

  // Reboot both: second begin() must find it at 921600 and NOT write flash.
  wt.powerCycle();
  int savesBefore = wt.saves, cmdsBefore = wt.cmds;
  t0 = g_us;
  ok = imu.begin();
  printf("  re-begin after power cycle: ok=%d took %.0f ms, new saves=%d, commands sent=%d\n",
         ok, (g_us - t0) / 1000.0, wt.saves - savesBefore, wt.cmds - cmdsBefore);
  CHECK(ok);
  CHECK(wt.saves == savesBefore && wt.cmds == cmdsBefore);
}

static void testNoSensor()
{
  ByteSrc src;
  WT61P_YPR imu(src);
  uint64_t t0 = g_us;
  CHECK(!imu.begin());
  printf("begin with nothing connected: false after %.0f ms\n", (g_us - t0) / 1000.0);
}

int main()
{
  testParser();
  testBegin(false);
  testBegin(true);
  testNoSensor();
  printf(g_fail ? "%d FAILURE(S)\n" : "ALL PASSED\n", g_fail);
  return g_fail ? 1 : 0;
}
