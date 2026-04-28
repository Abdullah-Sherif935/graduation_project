#include "FastAccelStepper.h"

// ==========================================
// 1. تعريف دبابيس المواتير (Pin Mapping)
// ==========================================
#define M1_STEP 10
#define M1_DIR  11
#define M1_EN   12

#define M2_STEP 20
#define M2_DIR  21
#define M2_EN   47

#define M3_STEP 35
#define M3_DIR  36
#define M3_EN   37

#define M4_STEP 40
#define M4_DIR  41
#define M4_EN   42

#define M5_STEP 15
#define M5_DIR  16
#define M5_EN   17

#define M6_STEP 4
#define M6_DIR  5
#define M6_EN   6

// ==========================================
// 2. إعدادات الميكانيكا (نسب التخفيض)
// ==========================================
// ضع هنا نسبة التخفيض لكل موتور من الـ 6 مواتير
// الموتور الأول 4.5 كمثال، الباقي 1.0 (يعني مفيش تخفيض)
const float GEAR_RATIOS[6] = {6.4, 25.0, 18.0952381, 4.0, 1.0, 10.0};

// ==========================================

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *steppers[6];

// 1600 نبضة لعمل لفة واحدة للموتور نفسه (بافتراض 1/8 Microstep)
const float STEPS_PER_REV = 1600.0; 
const float STEPS_PER_DEGREE = STEPS_PER_REV / 360.0;

void setup() {
  Serial.begin(115200);
  delay(1000); 
  Serial.println("ESP32-S3 Booting... 6-Axis Initializing with Gear Ratios");

  engine.init();

  steppers[0] = engine.stepperConnectToPin(M1_STEP);
  steppers[1] = engine.stepperConnectToPin(M2_STEP);
  steppers[2] = engine.stepperConnectToPin(M3_STEP);
  steppers[3] = engine.stepperConnectToPin(M4_STEP);
  steppers[4] = engine.stepperConnectToPin(M5_STEP);
  steppers[5] = engine.stepperConnectToPin(M6_STEP);

  int dirPins[] = {M1_DIR, M2_DIR, M3_DIR, M4_DIR, M5_DIR, M6_DIR};
  int enPins[] = {M1_EN, M2_EN, M3_EN, M4_EN, M5_EN, M6_EN};

  for (int i = 0; i < 6; i++) {
    if (steppers[i]) {
      steppers[i]->setDirectionPin(dirPins[i]);
      steppers[i]->setEnablePin(enPins[i], true); 
      steppers[i]->setAutoEnable(false); 
      steppers[i]->enableOutputs(); // تفعيل الكلبشة
      
      // ملاحظة: قد تحتاج لزيادة السرعة هنا لأن التروس تبطئ حركة الذراع النهائية
      steppers[i]->setSpeedInUs(500); // تقليل الرقم = زيادة السرعة   
      steppers[i]->setAcceleration(5000); 
      steppers[i]->setCurrentPosition(0); 
    }
  }

  Serial.println("System Ready!");
  Serial.println("Format: d1 d2 d3 d4 d5 d6 (e.g. 90 45 -30 0 180 -90)");
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n'); 
    input.trim(); 

    if (input.length() > 0) {
      float angles[6];
      
      int parsed = sscanf(input.c_str(), "%f %f %f %f %f %f", 
                          &angles[0], &angles[1], &angles[2], 
                          &angles[3], &angles[4], &angles[5]);

      if (parsed == 6) {
        Serial.print("Target Joint Angles -> ");
        for (int i = 0; i < 6; i++) {
          Serial.print(angles[i]); Serial.print("  ");
          
          // السر هنا: نضرب الزاوية المطلوبة × خطوات الدرجة × نسبة التخفيض
          long targetSteps = (long)(angles[i] * STEPS_PER_DEGREE * GEAR_RATIOS[i]);
          
          if (steppers[i]) {
            steppers[i]->moveTo(targetSteps);
          }
        }
        Serial.println();
      } else {
        Serial.println("Error: Format must be 6 space-separated numbers.");
      }
    }
  }
}