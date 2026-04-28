// ===================== HARDWARE SERIAL FOR ESP32-S3 ======================
#include <HardwareSerial.h>

// نستخدم UART1 على ESP32-S3
HardwareSerial HoverSerial(1);

// Pins المتوصلة بالـ Hoverboard
// لو حبيت تعكس RX/TX في التوصيل، غير الأرقام هنا بدل ما تغيّر الأسلاك
#define HOVER_RX 20   // ESP32 RX  ← Hoverboard TX
#define HOVER_TX 19   // ESP32 TX  → Hoverboard RX

// ===================== USER CONFIG ======================
#define SERIAL_BAUD 115200     // USB Serial مع الـ PC
#define HOVER_BAUD  115200     // UART مع لوحة الـ Hoverboard

#define START_FRAME 0xABCD     // لازم يطابق الـ firmware
#define MAX_PWR     1000
#define SEND_HZ     50         // 50Hz → كل 20ms
// ========================================================================


// ===================== DATA STRUCTS =====================
struct __attribute__((packed)) SerialCommand {
  uint16_t start;
  int16_t  steer;
  int16_t  speed;
  uint16_t checksum;
};
SerialCommand Command;

struct __attribute__((packed)) SerialFeedback {
  uint16_t start;
  int16_t  cmd1;
  int16_t  cmd2;
  int16_t  speedR_meas;
  int16_t  speedL_meas;
  int16_t  batVoltage;
  int16_t  boardTemp;
  uint16_t cmdLed;
  uint16_t checksum;
};
SerialFeedback Feedback;
SerialFeedback NewFeedback;

// ===================== RX STATE (لـ Feedback – ممكن نحتاجه بعدين) ========
uint8_t   rxIdx = 0;
uint16_t  bufStartFrame = 0;
uint8_t  *rxPtr;
uint8_t   inByte = 0, prevByte = 0;

// ===================== TIMERS ==========================
const unsigned long SEND_INTERVAL_MS = 1000UL / SEND_HZ;
unsigned long lastSendMs = 0;
unsigned long lastFBms   = 0;   // يتحدّث في Receive() لو فيه Feedback

// Targets & ramp
int16_t tgtR = 0;     // target power للعجلة اليمين
int16_t tgtL = 0;     // target power للعجلة الشمال
int16_t curR = 0;     // القيمة الفعلية اللي بتتبعت (بعد الـ ramp)
int16_t curL = 0;

const int16_t rampStep = 20;   // سرعة تغيّر الأمر (نعومة الحركة)
// ========================================================================


// ===================== HELPERS ==========================
static inline int16_t clampPwr(int32_t v) {
  if (v >  MAX_PWR) v =  MAX_PWR;
  if (v < -MAX_PWR) v = -MAX_PWR;
  return (int16_t)v;
}

void SendRaw(int16_t uSteer, int16_t uSpeed) {
  Command.start    = START_FRAME;
  Command.steer    = uSteer;
  Command.speed    = uSpeed;
  Command.checksum = Command.start ^ Command.steer ^ Command.speed;
  HoverSerial.write((uint8_t*)&Command, sizeof(Command));
}

/*
   نحول Right/Left wheel power إلى speed & steer:
      speed = (L + R) / 2
      steer = (R - L) / 2
*/
void SEND_R_L(int16_t PWR_R, int16_t PWR_L) {
  int32_t speed = ((int32_t)PWR_L + (int32_t)PWR_R) / 2;
  int32_t steer = ((int32_t)PWR_R - (int32_t)PWR_L) / 2;

  int16_t uSpeed = clampPwr(speed);
  int16_t uSteer = clampPwr(steer);

  SendRaw(uSteer, uSpeed);
}
// ========================================================================


// ===================== PC COMMAND PARSER =======================
// Format متوقَّع من الـ PC (Serial Monitor أو ROS):
//   R L\n
// مثال: "100 100\n"  → Right=100 ، Left=100
// المجال المسموح: -250 .. +250 لكل عجلة
void readPcCommand()
{
  if (Serial.available() > 0) {
    // نقرأ Right ثم Left
    int16_t cmdR = (int16_t)Serial.parseInt();
    int16_t cmdL = (int16_t)Serial.parseInt();

    // نتخلص من الـ newline لو موجود
    if (Serial.available()) {
      Serial.read();
    }

    // نقيّد المجال
    cmdR = constrain(cmdR, -250, 250);
    cmdL = constrain(cmdL, -250, 250);

    // نعمل scaling من ±250 إلى ±MAX_PWR (±1000)
    long scaledR = (long)cmdR * MAX_PWR / 250;
    long scaledL = (long)cmdL * MAX_PWR / 250;

    tgtR = clampPwr(scaledR);
    tgtL = clampPwr(scaledL);

    // Debug
    Serial.print("New targets → R=");
    Serial.print(tgtR);
    Serial.print("  L=");
    Serial.println(tgtL);
  }
}
// ========================================================================


// ===================== RECEIVE FEEDBACK (اختياري حاليًا) ================
void Receive() {
  while (HoverSerial.available()) {
    inByte = HoverSerial.read();
    bufStartFrame = (uint16_t(inByte) << 8) | prevByte;

    if (bufStartFrame == START_FRAME) {
      rxPtr = (uint8_t*)&NewFeedback;
      *rxPtr++ = prevByte;
      *rxPtr++ = inByte;
      rxIdx = 2;
    }
    else if (rxIdx >= 2 && rxIdx < sizeof(SerialFeedback)) {
      *rxPtr++ = inByte;
      rxIdx++;
    }

    if (rxIdx == sizeof(SerialFeedback)) {
      uint16_t cs =
        NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^
        NewFeedback.speedR_meas ^ NewFeedback.speedL_meas ^
        NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed;

      if (NewFeedback.start == START_FRAME && cs == NewFeedback.checksum) {
        Feedback = NewFeedback;
        lastFBms = millis();
        // دلوقتي مش بنستخدم الـ Feedback في الكنترول
      }
      rxIdx = 0;
    }

    prevByte = inByte;
  }
}
// ========================================================================


// ===================== SETUP =======================
void setup() {
  // USB مع الـ PC
  Serial.begin(SERIAL_BAUD);
  Serial.setTimeout(10);     // يخلي parseInt سريع

  // UART1 مع لوحة الهفر بورد على Pins (19 RX, 20 TX)
  HoverSerial.begin(HOVER_BAUD, SERIAL_8N1, HOVER_RX, HOVER_TX);

  Serial.println("\n[ESP32-S3] Hoverboard Controller READY");
  Serial.println("Send: R L  (e.g. 100 100)");
}
// ========================================================================


// ===================== LOOP (OPEN-LOOP CONTROL) =================
void loop() {
  // 1) اقرأ أوامر السرعة من الـ PC
  readPcCommand();

  // 2) (اختياري) اقرأ Feedback – مش مؤثر على الحركة دلوقتي
  // Receive();

  // 3) كل 20ms ابعت أوامر العجل الحالية (بعد الـ ramp)
  unsigned long now = millis();
  if (now - lastSendMs >= SEND_INTERVAL_MS) {
    lastSendMs = now;

    // Ramp بسيط علشان الحركة متبقاش فجائية
    int16_t dR = tgtR - curR;
    int16_t dL = tgtL - curL;

    if (dR >  rampStep) dR =  rampStep;
    if (dR < -rampStep) dR = -rampStep;
    if (dL >  rampStep) dL =  rampStep;
    if (dL < -rampStep) dL = -rampStep;

    curR += dR;
    curL += dL;

    SEND_R_L(curR, curL);

    // Debug: كل 20ms اطبع القيم اللي فعلاً رايحة للهفر بورد
    Serial.print("CMD → R=");
    Serial.print(curR);
    Serial.print("  L=");
    Serial.println(curL);
  }
}
// ========================================================================
