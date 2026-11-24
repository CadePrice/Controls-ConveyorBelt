/*
 * Conveyor-Actuation.ino  —  FreeRTOS firmware for Arduino Mega 2560
 *
 * Hardware
 *   Motor   : 12 V TS25GA gearmotor through an H-bridge, IN1 = D6 (PWM), IN2 = D7
 *   Encoder : Hall quadrature, A = D2 (INT4), B = D3 (INT5), decoded x4
 *   ToF     : VL53L0X on I2C, SDA = D20, SCL = D21 (XSHUT / GPIO1 unconnected)
 *
 * Behaviour
 *   1. Calibration: drives the belt at 100 % duty, waits for it to settle, then
 *      measures the steady encoder speed. That speed is the "maximum velocity".
 *      The encoder sign is detected automatically at the same time.
 *   2. Closed loop: a PI controller holds the belt at TARGET_FRACTION (50 %) of
 *      that maximum.
 *   3. Logging: each new VL53L0X reading (~20 Hz) prints one CSV line on Serial
 *      (115200 baud):
 *        time_ms,tof_mm,speed_mms,target_mms,duty_pct
 *      Lines that start with '#' are status messages, not data.
 *
 * Libraries (Arduino IDE -> Library Manager)
 *   - "FreeRTOS" by Phillip Stevens (feilipu)
 *   - "VL53L0X"  by Pololu
 *
 * Note on timing: by default the feilipu port runs the FreeRTOS tick from the
 * watchdog timer, so one tick is about 15 ms. The control loop runs once per
 * tick and uses micros() for the real dt, so the PI maths stays correct even
 * though the watchdog clock isn't exact.
 */

#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <Wire.h>
#include <VL53L0X.h>

// ============================== Configuration ===============================

// --- Pins ---
static const uint8_t PIN_ENC_A  = 2;   // PE4 / INT4
static const uint8_t PIN_ENC_B  = 3;   // PE5 / INT5
static const uint8_t PIN_MOT_IN1 = 6;  // Timer4 OC4A (PWM)
static const uint8_t PIN_MOT_IN2 = 7;  // Timer4 OC4B (PWM)

// Set true if the belt runs the wrong way (swaps which H-bridge input gets the PWM).
static const bool MOTOR_REVERSE = true;

// --- Belt / encoder geometry (from conveyor_belt_params.m) ---
// These only set the mm/s units in the log and the PI gain units. The 50 % target
// is relative to the measured maximum, so it is correct even if these are off.
static const float ENC_PPR      = 11.0f;    // [ASSUMED] pulses per motor rev, one channel
static const float ENC_DECODE   = 4.0f;     // x4 quadrature
static const float GEAR_RATIO   = 10.0f;    // [ASSUMED]
static const float ROLLER_R_MM  = 15.9f;    // effective roller radius incl. half belt
static const float MM_PER_COUNT =
    2.0f * PI * ROLLER_R_MM / (ENC_PPR * ENC_DECODE * GEAR_RATIO);   // ~0.227 mm

// --- Setpoint ---
static const float TARGET_FRACTION = 0.40f;  // 50 % of measured max belt speed

// --- PI gains (same units as conveyor_belt_params.m) ---
static const float KP = 0.0719f;  // % duty per (mm/s)
static const float KI = 0.933f;   // % duty per mm  (= %/(mm/s·s); Ki/Kp ≈ 13 rad/s cancels the plant pole)

// --- Calibration ---
static const uint16_t CAL_SETTLE_MS  = 500;    // motor off, let things settle
static const uint16_t CAL_SPINUP_MS  = 1500;   // full duty, wait to reach steady speed
static const uint16_t CAL_MEASURE_MS = 1000;   // measuring window at full duty
static const float    CAL_MIN_CPS    = 50.0f;  // below this counts/s -> treat as fault

// --- Speed estimate ---
static const uint8_t SPEED_WINDOW = 4;  // speed = delta counts over the last 4 control periods (~60 ms)

// --- Logging ---
static const uint32_t SERIAL_BAUD     = 115200;
static const uint16_t TOF_PERIOD_MS   = 50;    // VL53L0X inter-measurement period (20 Hz)
static const uint16_t TOF_TIMEOUT_MS  = 500;   // print a warning if no reading for this long
static const uint16_t TOF_RETRY_MS    = 2000;  // retry VL53L0X init this often if it isn't found
static const uint16_t STACK_REPORT_MS = 5000;  // print free stack per task this often

// --- RTOS task stacks (bytes on AVR). The Mega has 8 KB of RAM; ~7.5 KB is free. ---
static const uint16_t STACK_CTRL = 640;
static const uint16_t STACK_TOF  = 1024;   // VL53L0X init + Serial float printing are stack-hungry

// ================================ Globals ===================================

static VL53L0X tof;
static SemaphoreHandle_t serialMutex;
static TaskHandle_t hCtrl = NULL;
static TaskHandle_t hTof  = NULL;

// Encoder
static volatile int32_t encCount = 0;
static volatile uint8_t encPrev  = 0;
static int8_t encSign = 1;  // set during calibration so forward = positive

// Index = (prevState << 2) | newState, where state = (B << 1) | A
static const int8_t QDEC[16] = {
   0, -1, +1,  0,
  +1,  0,  0, -1,
  -1,  0,  0, +1,
   0, +1, -1,  0
};

// Status shared from the control task to the logger
struct Status {
  float speed_mms;
  float target_mms;
  float duty_pct;
};
static Status g_status = {0, 0, 0};

// ============================ Encoder ISR ===================================

static inline uint8_t encReadPins() {
  // D2 = PE4 (A), D3 = PE5 (B)  ->  state = (B << 1) | A
  return (PINE >> 4) & 0x03;
}

static void encoderISR() {
  uint8_t s = encReadPins();
  encCount += QDEC[(encPrev << 2) | s];
  encPrev = s;
}

static int32_t encoderRead() {
  uint8_t sreg = SREG;
  cli();
  int32_t c = encCount;
  SREG = sreg;
  return c;
}

// ============================== Motor =======================================

static void motorSetDuty(float pct) {
  if (pct < 0.0f)   pct = 0.0f;
  if (pct > 100.0f) pct = 100.0f;
  uint8_t pwm = (uint8_t)(pct * 2.55f + 0.5f);
  if (!MOTOR_REVERSE) {
    digitalWrite(PIN_MOT_IN2, LOW);
    analogWrite(PIN_MOT_IN1, pwm);
  } else {
    digitalWrite(PIN_MOT_IN1, LOW);
    analogWrite(PIN_MOT_IN2, pwm);
  }
}

// ======================= RTOS fault hooks (fail safe) =======================
// The library's default hook halts the scheduler with the L LED on/slow-blinking but
// leaves the PWM timer running at whatever duty it last had. Stop the motor instead,
// say which task overflowed, then blink. Interrupts are off here; Serial polls.

static void faultHalt(const __FlashStringHelper *what, const char *task) {
  digitalWrite(PIN_MOT_IN1, LOW);   // digitalWrite also disconnects the PWM timer
  digitalWrite(PIN_MOT_IN2, LOW);
  Serial.print(F("# FATAL: "));
  Serial.print(what);
  if (task) { Serial.print(F(" in task ")); Serial.print(task); }
  Serial.println(F(". Motor stopped. Reset to restart."));
  Serial.flush();
  pinMode(LED_BUILTIN, OUTPUT);
  for (;;) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    for (volatile uint32_t i = 0; i < 150000UL; i++) { }   // ~0.1 s, no timers needed
  }
}

void vApplicationStackOverflowHook(TaskHandle_t, char *pcTaskName) {
  faultHalt(F("stack overflow"), pcTaskName);
}

// ============================ Helpers =======================================

static void publishStatus(float speed, float target, float duty) {
  taskENTER_CRITICAL();
  g_status.speed_mms  = speed;
  g_status.target_mms = target;
  g_status.duty_pct   = duty;
  taskEXIT_CRITICAL();
}

static Status readStatus() {
  taskENTER_CRITICAL();
  Status s = g_status;
  taskEXIT_CRITICAL();
  return s;
}

static void logMsg(const __FlashStringHelper *msg) {
  if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
    Serial.print(F("# "));
    Serial.println(msg);
    xSemaphoreGive(serialMutex);
  }
}

// ============================ Control task ==================================

static void taskControl(void *) {
  // ---------- Calibration ----------
  motorSetDuty(0);
  vTaskDelay(pdMS_TO_TICKS(CAL_SETTLE_MS));

  logMsg(F("Calibrating: running belt at 100% duty to measure max speed..."));
  motorSetDuty(100.0f);
  publishStatus(0.0f, 0.0f, 100.0f);
  vTaskDelay(pdMS_TO_TICKS(CAL_SPINUP_MS));

  int32_t  c0 = encoderRead();
  uint32_t t0 = micros();
  vTaskDelay(pdMS_TO_TICKS(CAL_MEASURE_MS));
  int32_t  c1 = encoderRead();
  uint32_t t1 = micros();

  float cps = (float)(c1 - c0) * 1.0e6f / (float)(t1 - t0);

  if (fabs(cps) < CAL_MIN_CPS) {
    motorSetDuty(0);
    publishStatus(0.0f, 0.0f, 0.0f);
    logMsg(F("FAULT: no encoder motion at full duty. Check motor power, driver, and encoder wiring. Motor stopped."));
    vTaskSuspend(NULL);
  }

  encSign = (cps < 0.0f) ? -1 : 1;
  const float maxSpeed = fabs(cps) * MM_PER_COUNT;
  const float target   = TARGET_FRACTION * maxSpeed;

  if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
    Serial.print(F("# max_speed_mms="));  Serial.print(maxSpeed, 1);
    Serial.print(F(" max_counts_per_s=")); Serial.print(fabs(cps), 0);
    Serial.print(F(" target_mms="));      Serial.print(target, 1);
    Serial.print(F(" enc_sign="));        Serial.println(encSign);
    xSemaphoreGive(serialMutex);
  }

  // ---------- Closed-loop PI ----------
  // Start the integrator at the matching open-loop duty so the hand-off is smooth.
  float integ = TARGET_FRACTION * 100.0f;

  int32_t  cntBuf[SPEED_WINDOW];
  uint32_t tBuf[SPEED_WINDOW];
  int32_t  cNow = encoderRead();
  uint32_t tNow = micros();
  for (uint8_t i = 0; i < SPEED_WINDOW; i++) { cntBuf[i] = cNow; tBuf[i] = tNow; }
  uint8_t  idx   = 0;
  uint32_t tPrev = tNow;

  TickType_t lastWake = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&lastWake, 1);   // one tick (~15 ms)

    cNow = encoderRead();
    tNow = micros();

    // Speed over the sliding window
    int32_t  cOld = cntBuf[idx];
    uint32_t tOld = tBuf[idx];
    cntBuf[idx] = cNow;
    tBuf[idx]   = tNow;
    idx = (idx + 1) % SPEED_WINDOW;

    float winDt = (float)(tNow - tOld) * 1.0e-6f;
    float speed = (winDt > 0.0f)
                  ? (float)(encSign * (cNow - cOld)) * MM_PER_COUNT / winDt
                  : 0.0f;

    float dt = (float)(tNow - tPrev) * 1.0e-6f;
    tPrev = tNow;

    // PI with conditional integration (anti-windup)
    float err = target - speed;
    float uUnsat = KP * err + integ;
    bool satHigh = (uUnsat >= 100.0f) && (err > 0.0f);
    bool satLow  = (uUnsat <= 0.0f)   && (err < 0.0f);
    if (!satHigh && !satLow) {
      integ += KI * err * dt;
      if (integ > 100.0f) integ = 100.0f;
      if (integ < 0.0f)   integ = 0.0f;
    }
    float u = KP * err + integ;
    if (u > 100.0f) u = 100.0f;
    if (u < 0.0f)   u = 0.0f;

    motorSetDuty(u);
    publishStatus(speed, target, u);
  }
}

// ========================= ToF + logging task ===============================

// Print every address that ACKs on the bus. The VL53L0X should show up at 0x29.
static void i2cScan() {
  uint8_t found = 0;
  if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
    Serial.print(F("# I2C scan:"));
    for (uint8_t a = 1; a < 127; a++) {
      Wire.beginTransmission(a);
      if (Wire.endTransmission() == 0) {
        Serial.print(F(" 0x"));
        if (a < 16) Serial.print('0');
        Serial.print(a, HEX);
        found++;
      }
    }
    if (!found) Serial.print(F(" nothing found (VL53L0X should be 0x29: check wiring/power/XSHUT)"));
    Serial.println();
    xSemaphoreGive(serialMutex);
  }
}

static bool tofStart() {
  tof.setTimeout(100);
  if (!tof.init()) return false;
  tof.setMeasurementTimingBudget(33000);   // 33 ms, fits inside the 50 ms period
  tof.startContinuous(TOF_PERIOD_MS);
  return true;
}

static void printRow(uint32_t now, int16_t tofMm) {
  Status s = readStatus();
  if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
    Serial.print(now);
    Serial.print(',');
    Serial.print(tofMm);                  // -1 = no sensor / timeout, 8190/8191 = out of range
    Serial.print(',');
    Serial.print(s.speed_mms, 1);
    Serial.print(',');
    Serial.print(s.target_mms, 1);
    Serial.print(',');
    Serial.println(s.duty_pct, 1);
    xSemaphoreGive(serialMutex);
  }
}

static void taskTofLog(void *) {
  Wire.begin();
  Wire.setClock(100000);              // 100 kHz: more tolerant of long wires / weak pull-ups
  Wire.setWireTimeout(25000, true);   // recover if the I2C bus locks up

  i2cScan();
  bool tofOK = tofStart();
  if (tofOK) logMsg(F("VL53L0X ready."));
  else       logMsg(F("VL53L0X init failed. Logging speed anyway (tof_mm = -1), retrying every 2 s."));

  if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
    Serial.println(F("time_ms,tof_mm,speed_mms,target_mms,duty_pct"));
    xSemaphoreGive(serialMutex);
  }

  uint32_t lastReading = millis();
  uint32_t lastRow     = lastReading;
  uint32_t lastRetry   = lastReading;
  uint32_t lastStack   = lastReading;

  for (;;) {
    uint32_t now = millis();

    if (tofOK) {
      // Check the data-ready bit without blocking, so this task never busy-waits on I2C.
      bool ready = (tof.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) != 0;
      if (ready) {
        uint16_t mm = tof.readRangeContinuousMillimeters();   // returns immediately when ready
        printRow(now, tof.timeoutOccurred() ? -1 : (int16_t)mm);
        lastReading = lastRow = now;
      } else if (now - lastReading > TOF_TIMEOUT_MS) {
        logMsg(F("VL53L0X: no reading within timeout"));
        lastReading = now;
      }
    } else {
      // No sensor: still log the control loop at the same rate.
      if (now - lastRow >= TOF_PERIOD_MS) {
        printRow(now, -1);
        lastRow = now;
      }
      if (now - lastRetry >= TOF_RETRY_MS) {
        lastRetry = now;
        tofOK = tofStart();
        if (tofOK) { logMsg(F("VL53L0X ready.")); lastReading = millis(); }
      }
    }

    if (now - lastStack >= STACK_REPORT_MS) {
      lastStack = now;
      if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
        Serial.print(F("# free stack bytes: ctrl="));
        Serial.print(uxTaskGetStackHighWaterMark(hCtrl));
        Serial.print(F(" tof="));
        Serial.println(uxTaskGetStackHighWaterMark(hTof));
        xSemaphoreGive(serialMutex);
      }
    }

    vTaskDelay(1);   // poll once per tick (~15 ms)
  }
}

// ============================ Arduino entry =================================

void setup() {
  Serial.begin(SERIAL_BAUD);

  pinMode(PIN_MOT_IN1, OUTPUT);
  pinMode(PIN_MOT_IN2, OUTPUT);
  digitalWrite(PIN_MOT_IN1, LOW);
  digitalWrite(PIN_MOT_IN2, LOW);

  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  encPrev = encReadPins();
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), encoderISR, CHANGE);

  serialMutex = xSemaphoreCreateMutex();

  // Stack sizes are in bytes on the AVR port.
  if (xTaskCreate(taskControl, "Ctrl", STACK_CTRL, NULL, 3, &hCtrl) != pdPASS ||
      xTaskCreate(taskTofLog,  "ToF",  STACK_TOF,  NULL, 1, &hTof)  != pdPASS) {
    faultHalt(F("not enough RAM to create tasks"), NULL);
  }

  Serial.println(F("# Conveyor-Actuation starting"));
  // The feilipu library starts the scheduler automatically after setup() returns.
}

void loop() {
  // Runs in the FreeRTOS idle task. Leave empty.
}
