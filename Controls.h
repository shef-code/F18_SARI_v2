// Physical controls on the PCF8574 board (docs/sari-controls-pcb.svg), read over I2C.
//
//   P0 / P1 / P2  encoder 1: A, B, push switch   SAI_SET (attitude adjust), SAI_CAGE
//   P3            test button                    SAI_TEST_BTN
//   P6 / P5       encoder 2: A, B                STBY_PRESS_ALT (altimeter pressure setting); its push switch is not wired
//
// The PCF8574 has no interrupt wired here, so a small task on the other core reads it about every millisecond. The
// main loop (which also runs LVGL and the DCS-BIOS parser) only turns the counted detents and switch changes into
// DCS-BIOS messages, so a long frame cannot make a turn of the knob get lost.
#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <DcsBios.h>
#include <atomic>

namespace controls {

constexpr uint8_t PCF_ADDR = 0x21;           // A0 at 3V3, A1 and A2 at GND

constexpr uint8_t ENC1_A = 0, ENC1_B = 1, CAGE_SW = 2, TEST_BTN = 3;
constexpr uint8_t ENC2_A = 6, ENC2_B = 5;

// Quadrature transitions per click of the knob. EC11 style encoders with a click on every cycle need 4; ones with a
// click on every half cycle need 2.
constexpr int STEPS_PER_DETENT = 4;
// Swap to reverse a knob that turns the wrong way.
constexpr bool ENC1_REVERSE = false;
constexpr bool ENC2_REVERSE = false;
// Each click moves the DCS-BIOS control by this much (the suggested step; the full range is 65535).
constexpr const char *STEP_UP = "+3200";
constexpr const char *STEP_DOWN = "-3200";

constexpr uint8_t DEBOUNCE_POLLS = 5;        // a switch must read the same this many polls in a row

struct Knob {
  uint8_t last = 0;
  int8_t sub = 0;                            // transitions not yet making up a whole click
  std::atomic<int> clicks{0};                // +1 per click one way, -1 the other; the loop drains it

  void update(uint8_t a, uint8_t b, bool reverse) {
    // Gray code: each valid transition is +1 or -1; a skipped (invalid) transition is ignored.
    static const int8_t table[16] = {0, +1, -1, 0, -1, 0, 0, +1, +1, 0, 0, -1, 0, -1, +1, 0};
    const uint8_t state = (a << 1) | b;
    const int8_t step = table[(last << 2) | state];
    last = state;
    sub += reverse ? -step : step;
    if (sub >= STEPS_PER_DETENT) { clicks.fetch_add(1); sub = 0; }
    else if (sub <= -STEPS_PER_DETENT) { clicks.fetch_sub(1); sub = 0; }
  }
};

struct Switch {
  bool state = false;                        // true = pressed
  uint8_t count = 0;
  std::atomic<bool> stable{false};

  void update(bool pressed) {
    if (pressed == state) { count = 0; return; }
    if (++count >= DEBOUNCE_POLLS) { state = pressed; count = 0; stable.store(pressed); }
  }
};

inline Knob enc1, enc2;
inline Switch cage, test;
inline bool lastCage = false, lastTest = false;
inline bool present = false;

inline bool readPins(uint8_t &v) {
  if (Wire.requestFrom(PCF_ADDR, (uint8_t)1) != 1) return false;
  v = Wire.read();
  return true;
}

inline void process(uint8_t v) {
  enc1.update((v >> ENC1_A) & 1, (v >> ENC1_B) & 1, ENC1_REVERSE);
  enc2.update((v >> ENC2_A) & 1, (v >> ENC2_B) & 1, ENC2_REVERSE);
  cage.update(!((v >> CAGE_SW) & 1));        // switches pull the pin to ground
  test.update(!((v >> TEST_BTN) & 1));
}

inline void pollTask(void *) {
  for (;;) {
    uint8_t v;
    if (readPins(v)) process(v);
#ifdef SARI_CONTROLS_DEBUG
    { static uint32_t t0 = 0; if (millis() - t0 > 500) { t0 = millis(); Serial.printf("CTRL pins=0x%02X\n", v); } }
#endif
    vTaskDelay(1);
  }
}

// Call after I2C_Init(). Does nothing (and the gauge still works) if the board is not connected.
inline void begin() {
#ifdef SARI_CONTROLS_DEBUG
  delay(1500);
  Serial.printf("CTRL begin: scanning I2C\n");
  for (uint8_t a = 8; a < 120; a++) { Wire.beginTransmission(a); if (Wire.endTransmission() == 0) Serial.printf("CTRL found 0x%02X\n", a); }
  Serial.printf("CTRL scan done\n");
#endif
  Wire.beginTransmission(PCF_ADDR);
  Wire.write(0xFF);                          // all pins as inputs with the weak pull-up
  if (Wire.endTransmission() != 0) return;
  present = true;

  uint8_t v;
  if (!readPins(v)) v = 0xFF;
  enc1.last = ((v >> ENC1_A) & 1) << 1 | ((v >> ENC1_B) & 1);
  enc2.last = ((v >> ENC2_A) & 1) << 1 | ((v >> ENC2_B) & 1);
  // Start from the switches as they are now, so nothing is sent at power-up.
  cage.state = lastCage = !((v >> CAGE_SW) & 1);
  test.state = lastTest = !((v >> TEST_BTN) & 1);
  cage.stable.store(cage.state);
  test.stable.store(test.state);

  // The panel interrupt runs on the Arduino loop core, so poll on the other one.
  const BaseType_t other = xPortGetCoreID() == 0 ? 1 : 0;
  xTaskCreatePinnedToCore(pollTask, "controls", 3072, nullptr, 1, nullptr, other);
}

inline void sendClicks(Knob &k, const char *control) {
  int n = k.clicks.load();
  while (n != 0) {
    if (!DcsBios::tryToSendDcsBiosMessage(control, n > 0 ? STEP_UP : STEP_DOWN)) return;   // retry next loop
    if (n > 0) { k.clicks.fetch_sub(1); n--; } else { k.clicks.fetch_add(1); n++; }
  }
}

inline void sendSwitch(Switch &s, bool &last, const char *control) {
  const bool now = s.stable.load();
  if (now != last && DcsBios::tryToSendDcsBiosMessage(control, now ? "1" : "0")) last = now;
}

// Call from loop().
inline void service() {
  if (!present) return;
  sendClicks(enc1, "SAI_SET");
  sendClicks(enc2, "STBY_PRESS_ALT");
  sendSwitch(cage, lastCage, "SAI_CAGE");
  sendSwitch(test, lastTest, "SAI_TEST_BTN");
}

}  // namespace controls
