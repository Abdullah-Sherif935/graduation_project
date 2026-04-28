// ======================= 2-Channel Encoder Test (ESP32-S3) =======================
// Channel A → GPIO16
// Channel B → GPIO17

#define ENC_A 36   // قناة A
#define ENC_B 37   // قناة B

volatile long position = 0;   // العداد الكلي (x4 counts)

// دالة تحديث العداد عند تغير A
void IRAM_ATTR updateA() {
  bool A = digitalRead(ENC_A);
  bool B = digitalRead(ENC_B);

  if (A == B)
    position++;
  else
    position--;
}

// دالة تحديث العداد عند تغير B
void IRAM_ATTR updateB() {
  bool A = digitalRead(ENC_A);
  bool B = digitalRead(ENC_B);

  if (A != B)
    position++;
  else
    position--;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // إعداد القنوات A و B
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);

  // ربط الـ interrupts على التغيّر في A و B
  attachInterrupt(digitalPinToInterrupt(ENC_A), updateA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), updateB, CHANGE);

  Serial.println("\n[ENC TEST] Start counting 2-channel encoder on ESP32-S3...");
  Serial.println("Rotate the wheel slowly and watch 'Position' change...");
}

void loop() {
  static unsigned long lastTime = 0;
  unsigned long now = millis();

  if (now - lastTime >= 500) {   // كل نصف ثانية نطبع القيمة
    lastTime = now;

    noInterrupts();
    long pos = position;   // Snapshot آمن من العداد
    interrupts();

    Serial.print("Position (x4 counts) = ");
    Serial.println(pos);
  }
}
