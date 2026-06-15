// ============================================================
// BuzzerPWM — bench test for driving the GP4 buzzer with PWM
//             so you can find a comfortable (quieter) volume.
//
// Board : Heltec ESP32 (Arduino-ESP32 core 3.x — uses the new LEDC API)
// Wiring: buzzer + → GP4,  buzzer − → GND
//
// HOW VOLUME WORKS
//   PWM volume = duty cycle. We attach GP4 to an LEDC PWM channel and write a
//   duty between 0 (silent) and 255 (full, 8-bit).
//
//   • ACTIVE buzzer (has its own oscillator, the default on this proto board):
//     keep BUZZER_FREQ high (>=25 kHz) so the buzzer sees a reduced *average*
//     voltage. Lower duty = quieter, but below ~30 it may go silent or raspy,
//     and a few active buzzers ignore PWM entirely.
//
//   • PASSIVE buzzer (no oscillator): set BUZZER_FREQ to the tone you want
//     (e.g. 2000–4000 Hz); duty then sets loudness (≈128 = 50% is loudest).
//
// SERIAL COMMANDS (115200 baud, send a single character)
//     + / -   volume up / down (step 10)
//     [ / ]   carrier frequency down / up
//     1       warning beep   (1 s)        at current volume
//     2       danger pattern (~2 s rapid) at current volume
//     3       gps double-chirp            at current volume
//     0       off
//     w       sweep volume 0 → 255 so you can hear the usable range
// ============================================================

#define BUZZER_PIN      4
#define BUZZER_RES_BITS 8        // 8-bit duty → 0..255
int  buzzerFreq   = 20000;       // PWM carrier (Hz)  — high for an active buzzer
int  buzzerVolume = 210;          // duty when ON (0..255) — tune this

// ---- low-level helpers -------------------------------------------------------
void buzzerOn()  { ledcWrite(BUZZER_PIN, buzzerVolume); }
void buzzerOff() { ledcWrite(BUZZER_PIN, 0); }

// Blocking beep at the current volume (fine for a test sketch).
void beep(int onMs, int offMs = 0) {
  buzzerOn();  delay(onMs);
  buzzerOff(); if (offMs) delay(offMs);
}

// ---- the three patterns from the firmware, at the current volume -------------
void patternWarning() { beep(1000); }                       // single 1 s
void patternDanger()  { for (int i = 0; i < 9; i++) beep(120, 100); } // ~2 s rapid
void patternGps()     { beep(150, 120); beep(150); }        // double chirp

void printStatus() {
  Serial.printf("[buzzer] freq=%d Hz   volume=%d/255 (%.0f%%)\n",
                buzzerFreq, buzzerVolume, buzzerVolume * 100.0 / 255.0);
}

void volumeSweep() {
  Serial.println("[buzzer] sweeping volume 0 -> 255 ...");
  for (int v = 0; v <= 255; v += 15) {
    ledcWrite(BUZZER_PIN, v);
    Serial.printf("  duty=%3d (%.0f%%)\n", v, v * 100.0 / 255.0);
    delay(400);
  }
  buzzerOff();
  Serial.println("[buzzer] sweep done");
}

void setup() {
  Serial.begin(115200);
  delay(300);

  // Attach GP4 to an LEDC PWM channel (core 3.x API).
  ledcAttach(BUZZER_PIN, buzzerFreq, BUZZER_RES_BITS);
  buzzerOff();

  Serial.println("\nBuzzerPWM ready. Commands: + - [ ] 1 2 3 0 w");
  printStatus();
  patternWarning();   // confirm it makes sound at startup volume
}

void loop() {
  if (!Serial.available()) return;
  char c = Serial.read();

  switch (c) {
    case '+': buzzerVolume = min(255, buzzerVolume + 10); printStatus(); break;
    case '-': buzzerVolume = max(0,   buzzerVolume - 10); printStatus(); break;

    case ']': buzzerFreq = min(80000, buzzerFreq + 2000);
              ledcChangeFrequency(BUZZER_PIN, buzzerFreq, BUZZER_RES_BITS);
              printStatus(); break;
    case '[': buzzerFreq = max(500,   buzzerFreq - 2000);
              ledcChangeFrequency(BUZZER_PIN, buzzerFreq, BUZZER_RES_BITS);
              printStatus(); break;

    case '1': Serial.println("warning"); patternWarning(); break;
    case '2': Serial.println("danger");  patternDanger();  break;
    case '3': Serial.println("gps");     patternGps();     break;
    case '0': buzzerOff(); Serial.println("off"); break;
    case 'w': volumeSweep(); break;
    default: break;   // ignore newlines / other chars
  }
}
