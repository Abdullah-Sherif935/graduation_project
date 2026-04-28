// ======================= Dual Encoder Test (ESP32-S3) =======================
// Encoder 1: A → GPIO36 , B → GPIO37
// Encoder 2: A → GPIO38 , B → GPIO39
// Printed value = Real PPR (x4 / 4)

#define ENC1_A 36
#define ENC1_B 37

#define ENC2_A 16
#define ENC2_B 17

volatile long pos1 = 0;   // Encoder 1 (x4)
volatile long pos2 = 0;   // Encoder 2 (x4)

// -------------------- Encoder 1 ISR --------------------
void IRAM_ATTR enc1_A_ISR() {
  bool A = digitalRead(ENC1_A);
  bool B = digitalRead(ENC1_B);

  if (A == B) pos1++;
  else        pos1--;
}

void IRAM_ATTR enc1_B_ISR() {
  bool A = digitalRead(ENC1_A);
  bool B = digitalRead(ENC1_B);

  if (A != B) pos1++;
  else        pos1--;
}

// -------------------- Encoder 2 ISR --------------------
void IRAM_ATTR enc2_A_ISR() {
  bool A = digitalRead(ENC2_A);
  bool B = digitalRead(ENC2_B);

  if (A == B) pos2++;
  else        pos2--;
}

void IRAM_ATTR enc2_B_ISR() {
  bool A = digitalRead(ENC2_A);
  bool B = digitalRead(ENC2_B);

  if (A != B) pos2++;
  else        pos2--;
}

// -------------------- Setup --------------------
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(ENC1_A, INPUT_PULLUP);
  pinMode(ENC1_B, INPUT_PULLUP);

  pinMode(ENC2_A, INPUT_PULLUP);
  pinMode(ENC2_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC1_A), enc1_A_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC1_B), enc1_B_ISR, CHANGE);

  attachInterrupt(digitalPinToInterrupt(ENC2_A), enc2_A_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_B), enc2_B_ISR, CHANGE);

  Serial.println("\n[DUAL ENC TEST] ESP32-S3 Quadrature Encoders");
  Serial.println("Counts shown = REAL PPR (x4 / 4)\n");
}

// -------------------- Loop --------------------
void loop() {
  static uint32_t lastTime = 0;
  uint32_t now = millis();

  if (now - lastTime >= 500) {
    lastTime = now;

    noInterrupts();
    long c1 = pos1;
    long c2 = pos2;
    interrupts();

    // Convert x4 counts → real PPR
    long ppr1 = c1 / 4;
    long ppr2 = c2 / 4;

    Serial.print("ENC1 PPR = ");
    Serial.print(ppr1);
    Serial.print(" | ENC2 PPR = ");
    Serial.println(ppr2);
  }
}
