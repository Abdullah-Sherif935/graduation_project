#include "FastAccelStepper.h"
#include "driver/pulse_cnt.h" 

// ==========================================
// 1. تعريف دبابيس المواتير
// ==========================================
#define M1_STEP 10
#define M1_DIR 11
#define M1_EN 12

#define M2_STEP 20
#define M2_DIR 21
#define M2_EN 47

#define M3_STEP 35
#define M3_DIR 36
#define M3_EN 37

#define M4_STEP 40
#define M4_DIR 41
#define M4_EN 42

#define M5_STEP 15
#define M5_DIR 16
#define M5_EN 17

#define M6_STEP 4
#define M6_DIR 5
#define M6_EN 6

// ==========================================
// 2. إعدادات الإنكودر (Hardware PCNT)
// ==========================================
#define ENC2_A 9 
#define ENC2_B 3 

pcnt_unit_handle_t pcnt_unit = NULL; 

// ==========================================
// 3. إعدادات الميكانيكا
// ==========================================
const float GEAR_RATIOS[6] = {6.4, 25.0, 18.0952381, 4.0, 1.0, 10.0};
const bool DIR_STATES[6] = {true, false, true, true, true, true}; // موفت DIR

const int ENCODER_SIGN = 1; // إشارة الإنكودر المضبوطة

const float STEPS_PER_REV = 1600.0;
const float STEPS_PER_DEGREE = STEPS_PER_REV / 360.0;
const float ENC_TICKS_PER_REV = 1200.0; 

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *steppers[6];

float target_angle_j2 = 0.0; 

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Booting... Closed-Loop System (Absolute Coordinate Sync)");

  pinMode(ENC2_A, INPUT_PULLUP);
  pinMode(ENC2_B, INPUT_PULLUP);

  pcnt_unit_config_t unit_config = {
      .low_limit = -32768,
      .high_limit = 32767,
  };
  pcnt_new_unit(&unit_config, &pcnt_unit);

  pcnt_chan_config_t chan_config = {
      .edge_gpio_num = ENC2_A,
      .level_gpio_num = ENC2_B,
  };
  pcnt_channel_handle_t pcnt_chan = NULL;
  pcnt_new_channel(pcnt_unit, &chan_config, &pcnt_chan);

  pcnt_channel_set_edge_action(pcnt_chan, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
  pcnt_channel_set_level_action(pcnt_chan, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

  pcnt_unit_enable(pcnt_unit);
  pcnt_unit_clear_count(pcnt_unit);
  pcnt_unit_start(pcnt_unit);

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
      steppers[i]->setDirectionPin(dirPins[i], DIR_STATES[i]);
      steppers[i]->setEnablePin(enPins[i], true);
      steppers[i]->setAutoEnable(false);
      steppers[i]->enableOutputs(); 

      if (i == 1) { 
        steppers[i]->setSpeedInUs(600);  
        steppers[i]->setAcceleration(3000); 
      } else {
        steppers[i]->setSpeedInUs(500); 
        steppers[i]->setAcceleration(5000);
      }
      steppers[i]->setCurrentPosition(0);
    }
  }
}

void loop() {
  // 1. استقبال الأوامر من MoveIt
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() > 0) {
      float angles[6];
      int parsed = sscanf(input.c_str(), "%f %f %f %f %f %f",
                          &angles[0], &angles[1], &angles[2],
                          &angles[3], &angles[4], &angles[5]);

      if (parsed == 6) {
        target_angle_j2 = angles[1]; 
        
        for (int i = 0; i < 6; i++) {
          long targetSteps = (long)(angles[i] * STEPS_PER_DEGREE * GEAR_RATIOS[i]);
          if (steppers[i]) {
            steppers[i]->moveTo(targetSteps);
          }
        }
      }
    }
  }

  // 2. 🖨️ المراقبة الحية
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 250) {
    lastPrint = millis();
    
    int currentTicksPrint = 0;
    pcnt_unit_get_count(pcnt_unit, &currentTicksPrint);
    
    float live_actual_angle = ((float)currentTicksPrint * ENCODER_SIGN / ENC_TICKS_PER_REV) * 360.0;
    float live_error = target_angle_j2 - live_actual_angle;
    
    Serial.print("🎯 Target: "); Serial.print(target_angle_j2, 2);
    Serial.print("° | 🤖 Actual: "); Serial.print(live_actual_angle, 2);
    Serial.print("° | 📉 Error: "); Serial.print(live_error, 2);
    Serial.println("°");
  }

  // 3. 🛡️ حارس الإنكودر (التصحيح الموثوق)
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck > 50 && !steppers[1]->isRunning()) {
    lastCheck = millis();
    
    int currentTicks = 0;
    pcnt_unit_get_count(pcnt_unit, &currentTicks);

    float actual_angle_j2 = ((float)currentTicks * ENCODER_SIGN / ENC_TICKS_PER_REV) * 360.0;
    float error_angle = target_angle_j2 - actual_angle_j2;

    if (abs(error_angle) > 1.0) {
       // 🛠️ الحل السحري: مزامنة عقل الموتور مع الواقع الفيزيائي أولاً
       long actual_steps = (long)(actual_angle_j2 * STEPS_PER_DEGREE * GEAR_RATIOS[1]);
       steppers[1]->setCurrentPosition(actual_steps); 

       // 🛠️ استخدام الأمر المطلق للوصول للهدف بشكل آمن ومتوافق مع MoveIt
       long target_steps = (long)(target_angle_j2 * STEPS_PER_DEGREE * GEAR_RATIOS[1]);
       steppers[1]->moveTo(target_steps); 
    }
  }
}