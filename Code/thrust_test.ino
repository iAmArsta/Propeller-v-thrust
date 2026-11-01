#include <Arduino.h>
#include <HX711.h>
#include <Preferences.h>
#include <esp_arduino_version.h>
#include <math.h>
#include <string.h>
#include "config.h"

struct Calibration { uint32_t magic; float countsPerGram; int32_t offset; };
constexpr uint32_t CAL_MAGIC = 0x54485231;
struct Sample {
  uint32_t tUs, rpmAgeUs;
  int32_t raw;
  float rpm, grams;
  uint16_t escUs;
  uint8_t rpmPeriods;
};
HX711 hx;
Calibration cal = {0, 0, 0};
Sample samples[MAX_SAMPLES];
uint16_t sampleCount = 0;
uint32_t runId = 0, startUs = 0, lastHxUs = 0, bootMs = 0, stopElapsedUs = 0;
uint16_t commandedUs = ESC_MIN_US;
bool running = false, escReady = false;
char outcome[32] = "NO_RUN";
char command[48];
size_t commandLength = 0;

portMUX_TYPE rpmMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t pulseLastUs = 0, periodSumUs = 0, acceptedPulses = 0, rejectedPulses = 0;
volatile uint32_t periods[RPM_AVERAGE_PERIODS] = {};
volatile uint8_t periodCount = 0, periodIndex = 0;
volatile bool havePulse = false;

void ARDUINO_ISR_ATTR onPulse() {
  uint32_t now = micros();
  portENTER_CRITICAL_ISR(&rpmMux);
  if (havePulse) {
    uint32_t dt = now - pulseLastUs;
    if (dt < MIN_PULSE_US) {
      ++rejectedPulses;
      portEXIT_CRITICAL_ISR(&rpmMux);
      return;
    }
    if (dt > RPM_TIMEOUT_US) {
      periodCount = periodIndex = 0;
      periodSumUs = 0;
    } else {
      if (periodCount == RPM_AVERAGE_PERIODS) periodSumUs -= periods[periodIndex];
      else ++periodCount;
      periods[periodIndex] = dt;
      periodSumUs += dt;
      periodIndex = (periodIndex + 1) % RPM_AVERAGE_PERIODS;
    }
  }
  havePulse = true;
  pulseLastUs = now;
  ++acceptedPulses;
  portEXIT_CRITICAL_ISR(&rpmMux);
}

void resetRpm() {
  portENTER_CRITICAL(&rpmMux);
  pulseLastUs = periodSumUs = acceptedPulses = rejectedPulses = 0;
  periodCount = periodIndex = 0;
  havePulse = false;
  portEXIT_CRITICAL(&rpmMux);
}

void readRpm(uint32_t now, float &rpm, uint32_t &age, uint8_t &n) {
  portENTER_CRITICAL(&rpmMux);
  uint32_t last = pulseLastUs, sum = periodSumUs;
  n = periodCount;
  bool seen = havePulse;
  now = micros();
  portEXIT_CRITICAL(&rpmMux);
  age = seen ? now - last : RPM_TIMEOUT_US;
  rpm = (seen && n && sum && age <= RPM_TIMEOUT_US)
      ? 60000000.0f * n / (float(sum) * PULSES_PER_REV) : 0.0f;
}

void writeEsc(uint16_t us) {
  commandedUs = constrain(us, ESC_MIN_US, ESC_MAX_US);
  uint32_t duty = uint32_t((uint64_t(commandedUs) * ESC_HZ * ((1UL << ESC_BITS) - 1)
                          + 500000) / 1000000);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(ESC_PIN, duty);
#else
  ledcWrite(0, duty);
#endif
}

bool averageRaw(int32_t &result, uint8_t count = 30) {
  int64_t sum = 0;
  for (uint8_t i = 0; i < count; ++i) {
    uint32_t waitStarted = millis();
    while (!hx.is_ready()) {
      if (millis() - waitStarted > 700 || digitalRead(STOP_PIN) == LOW) return false;
      delay(1);
    }
    int32_t raw = hx.read();
    if (raw >= 8388600 || raw <= -8388600) return false;
    sum += raw;
  }
  result = int32_t(sum / count);
  return true;
}

void dumpCsv() {
  Serial.println("# CSV_BEGIN");
  Serial.printf("# result=%s\n# run_id=%lu\n# stop_elapsed_ms=%.3f\n", outcome,
                (unsigned long)runId, stopElapsedUs / 1000.0);
  Serial.printf("# counts_per_gram=%.9g\n# tare_raw=%ld\n# thrust_sign=%.0f\n",
                cal.countsPerGram, (long)cal.offset, THRUST_SIGN);
  Serial.printf("# pulses_per_rev=%u\n# rpm_average_periods=%u\n# rpm_timeout_ms=%lu\n",
                PULSES_PER_REV, RPM_AVERAGE_PERIODS, (unsigned long)(RPM_TIMEOUT_US / 1000));
  Serial.printf("# accepted_pulses=%lu\n# rejected_pulses=%lu\n",
                (unsigned long)acceptedPulses, (unsigned long)rejectedPulses);
  Serial.println("run_id,time_s,phase,throttle_pct,esc_us,rpm,rpm_age_ms,rpm_periods,hx_raw,load_g,thrust_N");
  for (uint16_t i = 0; i < sampleCount; ++i) {
    const Sample &s = samples[i];
    float pct = 100.0f * (s.escUs - ESC_MIN_US) / (ESC_MAX_US - ESC_MIN_US);
    Serial.printf("%lu,%.6f,%s,%.3f,%u,%.3f,%.3f,%u,%ld,%.6f,%.6f\n",
      (unsigned long)runId, s.tUs / 1000000.0, s.tUs < RAMP_US ? "ramp" : "hold",
      pct, s.escUs, s.rpm, s.rpmAgeUs / 1000.0, s.rpmPeriods, (long)s.raw,
      s.grams, THRUST_SIGN * s.grams * GRAVITY / 1000.0f);
  }
  Serial.println("# CSV_END");
}

void finishRun(const char *reason) {
  writeEsc(ESC_MIN_US);
  stopElapsedUs = micros() - startUs;
  running = false;
  detachInterrupt(digitalPinToInterrupt(IR_PIN));
  snprintf(outcome, sizeof(outcome), "%s", reason);
  dumpCsv();
  Serial.println("START for another trial; DUMP to retransmit.");
}

void startRun() {
  if (!escReady || millis() - bootMs < ARM_MS) {
    Serial.println("Wait at 5 seconds after boot for ESC."); return;
  }
  if (cal.magic != CAL_MAGIC || !isfinite(cal.countsPerGram) || fabsf(cal.countsPerGram) < 0.001f) {
    Serial.println("Calibrate and SAVE first."); return;
  }
  if (digitalRead(STOP_PIN) == LOW) {
    Serial.println("Release stop button."); return;
  }
  Serial.println("Taring 30 samples. Don't touch anything");
  writeEsc(ESC_MIN_US);
  int32_t zero;
  if (!averageRaw(zero)) { Serial.println("HX711 tare failed."); return; }
  cal.offset = zero;
  sampleCount = 0;
  ++runId;
  resetRpm();
  attachInterrupt(digitalPinToInterrupt(IR_PIN), onPulse, IR_EDGE);
  Serial.println("10-second ramp then 5-second hold; ! to abort.");
  Serial.flush();
  startUs = lastHxUs = micros();
  running = true;
}

void handleCommand(const char *cmd) {
  if (running) {
    if (!strcmp(cmd, "STOP")) finishRun("ABORTED");
    return;
  }
  if (!strcmp(cmd, "START")) startRun();
  else if (!strcmp(cmd, "DUMP")) dumpCsv();
  else if (!strcmp(cmd, "TARE")) {
    int32_t zero;
    if (averageRaw(zero)) { cal.offset = zero; Serial.printf("# tare_raw=%ld\n", (long)zero); }
    else Serial.println("# ERROR: Tare failed.");
  } else if (!strcmp(cmd, "READ")) {
    int32_t raw;
    if (cal.magic == CAL_MAGIC && averageRaw(raw, 10)) {
      float grams = (double(raw) - cal.offset) / cal.countsPerGram;
      Serial.printf("# load_g=%.6f, thrust_N=%.6f\n", grams, THRUST_SIGN * grams * GRAVITY / 1000);
    } else Serial.println("# ERROR: Need valid calibration and connected HX711.");
  } else Serial.println("# Commands: START, STOP, !, DUMP, TARE, READ, HELP (newline after words).");
}

void serviceSerial() {
  for (uint8_t i = 0; i < 32 && Serial.available(); ++i) {
    char c = char(Serial.read());
    if (c == '!') {
      commandLength = 0;
      if (running) finishRun("ABORTED");
    } else if (c == '\n' || c == '\r') {
      if (commandLength) {
        command[commandLength] = '\0';
        commandLength = 0;
        handleCommand(command);
      }
    } else if (commandLength < sizeof(command) - 1) command[commandLength++] = c;
    else commandLength = 0;
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  pinMode(STOP_PIN, INPUT_PULLUP);
  pinMode(IR_PIN, INPUT_PULLUP);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  escReady = ledcAttach(ESC_PIN, ESC_HZ, ESC_BITS);
#else
  escReady = ledcSetup(0, ESC_HZ, ESC_BITS) > 0;
  if (escReady) ledcAttachPin(ESC_PIN, 0);
#endif
  if (escReady) writeEsc(ESC_MIN_US);
  else { pinMode(ESC_PIN, OUTPUT); digitalWrite(ESC_PIN, LOW); }
  bootMs = millis();
  hx.begin(HX_DOUT_PIN, HX_SCK_PIN, 128);
  Preferences prefs;
  if (prefs.begin("thrust-ia", true)) {
    if (prefs.getBytesLength("cal") == sizeof(cal)) prefs.getBytes("cal", &cal, sizeof(cal));
    prefs.end();
  }
  Serial.println("THRUST_IA_READY v1.0");
  Serial.println("START begins one trial. HELP lists commands.");
}

void loop() {
  if (running) {
    uint32_t now = micros(), elapsed = now - startUs;
    if (digitalRead(STOP_PIN) == LOW) finishRun("STOP_BUTTON");
    else if (elapsed >= RAMP_US + HOLD_US) finishRun("COMPLETE");
    else {
      uint16_t pulse = elapsed < RAMP_US
        ? ESC_MIN_US + uint64_t(ESC_MAX_US - ESC_MIN_US) * elapsed / RAMP_US : ESC_MAX_US;
      if (pulse != commandedUs) writeEsc(pulse);
      if (hx.is_ready()) {
        int32_t raw = hx.read();
        now = micros(); elapsed = now - startUs; lastHxUs = now;
        if (elapsed >= RAMP_US + HOLD_US) { finishRun("COMPLETE"); return; }
        float grams = (double(raw) - cal.offset) / cal.countsPerGram;
        if (raw >= 8388600 || raw <= -8388600) { finishRun("HX_SATURATED"); return; }
        if (!isfinite(grams) || fabsf(grams) > MAX_ABS_NET_G) { finishRun("LOAD_LIMIT"); return; }
        if (sampleCount == MAX_SAMPLES) { finishRun("BUFFER_FULL"); return; }
        Sample &s = samples[sampleCount++];
        s.tUs = elapsed; s.raw = raw; s.grams = grams; s.escUs = commandedUs;
        readRpm(now, s.rpm, s.rpmAgeUs, s.rpmPeriods);
      } else if (now - lastHxUs > HX_TIMEOUT_US) finishRun("HX_TIMEOUT");
    }
  }
  serviceSerial();
  delay(1);
}
