// #include <Arduino.h>


// const int pinsToTest[] = {
//   0, 2, 4, 5,   
//   12, 13, 14, 15, 16, 17,
//   18, 19, 21, 22, 23,
//   25, 26, 27,          // 25,26 = DAC
//   32, 33, 34, 35, 36, 37, 38, 39
// };
// const int nPins = sizeof(pinsToTest) / sizeof(pinsToTest[0]);

// // ADC-capable pins on many modules (tùy board): 32-39, 36-39 là ADC1/ADC2 etc.
// // We'll do analogRead if pin number hợp lệ cho ADC.
// bool isADCpin(int gpio) {
//   // common ADC pins (may vary by module). Adjust if necessary.
//   int adcPins[] = {32, 33, 34, 35, 36, 37, 38, 39, 25, 26, 27}; // 25/26/27 có ADC on some chips
//   for (int p: adcPins) if (p == gpio) return true;
//   return false;
// }

// // Touch-capable pins (t0..t9) (common): 0,2,4,12,13,14,15,27,32,33 etc.
// // We'll attempt touchRead where supported.
// bool isTouchPin(int gpio) {
//   int touchPins[] = {0,2,4,12,13,14,15,27,32,33};
//   for (int p: touchPins) if (p == gpio) return true;
//   return false;
// }

// // DAC pins: 25, 26 on ESP32
// bool isDACpin(int gpio) {
//   return (gpio == 25 || gpio == 26);
// }

// // Pins that are input-only (can't set OUTPUT)
// bool isInputOnly(int gpio) {
//   // GPIO34..GPIO39 are input-only
//   return (gpio >= 34 && gpio <= 39);
// }

// void setup() {
//   Serial.begin(115200);
//   delay(100);
//   Serial.println();
//   Serial.println("=== ESP32 PIN SELF-TEST BEGIN ===");
//   Serial.println("Warnings: disconnect external circuits before test.");
//   Serial.println();

//   for (int i = 0; i < nPins; ++i) {
//     int pin = pinsToTest[i];
//     Serial.printf("\n--- Testing GPIO%d ---\n", pin);

//     if (isInputOnly(pin)) {
//       Serial.println("Mode: INPUT-ONLY pin. Setting INPUT_PULLUP and reading value.");
//       pinMode(pin, INPUT_PULLUP);
//       delay(50);
//       int v = digitalRead(pin);
//       Serial.printf("Digital read (with INTERNAL_PULLUP): %d (0 if grounded)\n", v);
//       if (isADCpin(pin)) {
//         int a = analogRead(pin);
//         Serial.printf("Analog read: %d\n", a);
//       }
//       if (isTouchPin(pin)) {
//         int t = touchRead(pin);
//         Serial.printf("Touch read: %d\n", t);
//       }
//       continue;
//     }

//     // 1) Digital output test: toggle and readback
//     Serial.println("Digital OUT test: write HIGH, LOW and readback.");
//     pinMode(pin, OUTPUT);
//     digitalWrite(pin, HIGH);
//     delay(50);
//     int r1 = digitalRead(pin);
//     Serial.printf("Wrote HIGH, digitalRead => %d\n", r1);
//     digitalWrite(pin, LOW);
//     delay(50);
//     int r2 = digitalRead(pin);
//     Serial.printf("Wrote LOW, digitalRead => %d\n", r2);

//     // 2) Input test with pullup: switch to input_pullup and read
//     Serial.println("Switching to INPUT_PULLUP; read value (expect 1 unless externally grounded).");
//     pinMode(pin, INPUT_PULLUP);
//     delay(20);
//     int rin = digitalRead(pin);
//     Serial.printf("INPUT_PULLUP read => %d\n", rin);

//     // 3) ADC test if applicable
//     if (isADCpin(pin)) {
//       Serial.println("Analog (ADC) test: analogRead 10 samples.");
//       long sum = 0;
//       for (int k = 0; k < 10; ++k) {
//         int a = analogRead(pin);
//         sum += a;
//         delay(10);
//       }
//       Serial.printf("Analog avg: %ld\n", sum / 10);
//     }

//     // 4) Touch test if applicable
//     if (isTouchPin(pin)) {
//       Serial.println("Touch test (touchRead): 5 samples");
//       long tsum = 0;
//       for (int k = 0; k < 5; ++k) {
//         int t = touchRead(pin);
//         tsum += t;
//         delay(10);
//       }
//       Serial.printf("Touch avg: %ld\n", tsum / 5);
//     }

//     // 5) DAC test if applicable
//     if (isDACpin(pin)) {
//       Serial.println("DAC test: output mid-level (128), then 0.");
//       dacWrite(pin, 128); // 0..255
//       delay(100);
//       dacWrite(pin, 0);
//       delay(50);
//       Serial.println("DAC write done.");
//     } else {
//       // PWM quick test via ledc on non-DAC pins: generate small duty for a short time
//       Serial.println("PWM quick test via LEDC for 200ms (if pin supports).");
//       const int chan = 0;
//       const int freq = 5000;
//       const int resolution = 8;
//       // attach to pin (works on most gpio)
//       ledcSetup(chan, freq, resolution);
//       ledcAttachPin(pin, chan);
//       ledcWrite(chan, 128); // half duty
//       delay(200);
//       ledcWrite(chan, 0);
//       ledcDetachPin(pin);
//     }

//     Serial.printf("--- GPIO%d test complete ---\n", pin);
//     delay(150);
//   }

//   Serial.println("\n=== ALL DONE. Review logs above. ===");
// }

// void loop() {
//   // nothing repeating
//   delay(1000);
// }
