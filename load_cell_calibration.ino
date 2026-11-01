
#include <Arduino.h>
#include <HX711.h>
#include <Preferences.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
constexpr uint8_t HX_DOUT_PIN = 19, HX_SCK_PIN = 18, ESC_PIN = 25;
constexpr uint32_t CAL_MAGIC = 0x54485231;
struct Calibration { uint32_t magic; float countsPerGram; int32_t offset; };
Calibration cal = {0, 0, 0};
HX711 hx;
bool zeroTaken = false, factorReady = false;
char command[64];
size_t commandLength = 0;

bool averageRaw(int32_t &mean, double &sd, uint8_t count = 40) {
  double sum = 0, sumSq = 0;
  for (uint8_t i = 0; i < count; ++i) {
    if (!hx.wait_ready_timeout(700, 1)) {
      Serial.println("HX711 not responding."); return false;
    }
    int32_t x = hx.read();
    if (x >= 8388600 || x <= -8388600) {
      Serial.println("ADC saturated."); return false;
    }
    sum += x; sumSq += double(x) * x;
  }
  mean = lround(sum / count);
  sd = sqrt(fmax(0.0, (sumSq - sum * sum / count) / (count - 1)));
  return true;
}

void handleCommand(const char *cmd) {
  int32_t mean; double sd;
  if (!strcmp(cmd, "TARE")) {
    Serial.println("# Remove added force/weights; keep the assembled rig still.");
    if (averageRaw(mean, sd)) {
      cal.offset = mean; zeroTaken = true;
      Serial.printf("# tare_raw=%ld, raw_sd=%.3f\n", (long)mean, sd);
      Serial.println("# Apply a known axial load, let it settle, then send CAL <grams>.");
    }
  } else if (!strncmp(cmd, "CAL ", 4)) {
    char *end;
    double grams = strtod(cmd + 4, &end);
    while (*end == ' ') ++end;
    if (!zeroTaken || end == cmd + 4 || *end || !isfinite(grams) || grams <= 0 || grams > 4000) {
      Serial.println("TARE first; enter known mass"); return;
    }
    if (averageRaw(mean, sd)) {
      double delta = double(mean) - cal.offset;
      if (fabs(delta) < fmax(100.0, 10.0 * sd)) {
        Serial.println("Signal too noisy."); return;
      }
      cal.countsPerGram = delta / grams; cal.magic = CAL_MAGIC; factorReady = true;
      Serial.printf("# counts_per_gram=%.9g, loaded_raw=%ld, raw_sd=%.3f\n",
                    cal.countsPerGram, (long)mean, sd);
      Serial.println("Read with other known loads.");
    }
  } else if (!strcmp(cmd, "SAVE")) {
    if (!factorReady) { Serial.println("# ERROR: Complete TARE and CAL first."); return; }
    Preferences prefs;
    if (!prefs.begin("thrust-ia", false)) { Serial.println("Cannot open calibration storage."); return; }
    size_t written = prefs.putBytes("cal", &cal, sizeof(cal));
    prefs.end();
    Serial.println(written == sizeof(cal) ? "SAVED."
                                         : "Calibration save failed.");
  } else if (!strcmp(cmd, "READ")) {
    if (!factorReady) { Serial.println("Need a saved calibration first."); return; }
    if (averageRaw(mean, sd, 20)) {
      double g = (double(mean) - cal.offset) / cal.countsPerGram;
      Serial.printf("# load_g=%.6f, force_N=%.6f, sample_sd_g=%.6f, raw=%ld\n",
                    g, g * 9.80665 / 1000.0, sd / fabs(cal.countsPerGram), (long)mean);
    }
  } else Serial.println("# Commands: TARE | CAL <grams> (e.g. CAL 500) | READ | SAVE | HELP");
}

void setup() {
  pinMode(ESC_PIN, OUTPUT); digitalWrite(ESC_PIN, LOW);
  Serial.begin(115200);
  hx.begin(HX_DOUT_PIN, HX_SCK_PIN, 128);
  Preferences prefs;
  if (prefs.begin("thrust-ia", true)) {
    if (prefs.getBytesLength("cal") == sizeof(cal)) prefs.getBytes("cal", &cal, sizeof(cal));
    prefs.end();
  }
  factorReady = cal.magic == CAL_MAGIC && isfinite(cal.countsPerGram) && fabsf(cal.countsPerGram) > 0.001f;
  delay(1500);
  Serial.println("CALIBRATION_READY. Disconnect ESC battery.");
  Serial.println("Send TARE, apply known axial load, CAL, READ, SAVE.");
}

void loop() {
  while (Serial.available()) {
    char c = char(Serial.read());
    if (c == '\n' || c == '\r') {
      if (commandLength) {
        command[commandLength] = '\0'; commandLength = 0; handleCommand(command);
      }
    } else if (commandLength < sizeof(command) - 1) command[commandLength++] = c;
    else commandLength = 0;
  }
  delay(1);
}
