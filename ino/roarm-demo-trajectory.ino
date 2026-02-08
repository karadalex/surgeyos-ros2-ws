#include <Arduino.h>
#include <SCServo.h>

// Bus-servo object (ST3215 / SMS_STS protocol)
SMS_STS st;

// ESP32 RoArm-M1 / Waveshare examples commonly use these pins:
static constexpr int S_RXD = 18;
static constexpr int S_TXD = 19;
static constexpr uint32_t SERVO_BAUD = 1000000; // 1 Mbps

// RoArm-M1 servo IDs (as shipped): 1..5
static constexpr uint8_t IDS[5] = {1, 2, 3, 4, 5};

// Motion tuning
static constexpr uint16_t SPEED = 1200; // lower = gentler
static constexpr uint8_t  ACC   = 30;   // 0..150, lower = gentler

// Update rate
static constexpr uint32_t DT_MS = 25;   // 40 Hz

// Helper: clamp to ST3215 tick range
static int16_t clampTick(int32_t t) {
  if (t < 0) return 0;
  if (t > 4095) return 4095;
  return (int16_t)t;
}

int16_t home[5] = {2048, 2048, 2048, 2048, 2048}; // will be overwritten if reads succeed

void setup() {
  Serial.begin(115200);
  delay(200);

  // Serial1 controls the bus servos
  Serial1.begin(SERVO_BAUD, SERIAL_8N1, S_RXD, S_TXD);
  st.pSerial = &Serial1;

  delay(500);
  Serial.println("RoArm-M1 wave trajectory starting...");

  // Read initial (current) positions so we oscillate around whatever pose the arm is in
  for (int i = 0; i < 5; i++) {
    int p = st.ReadPos(IDS[i]);     // returns -1 on failure
    if (p >= 0 && p <= 4095) {
      home[i] = (int16_t)p;
    }
    Serial.print("Home ID "); Serial.print(IDS[i]);
    Serial.print(" = "); Serial.println(home[i]);
    delay(20);
  }

  // Optional: ensure gripper stays put by commanding it to its current position once
  st.WritePosEx(4, home[3], SPEED, ACC);
}

void loop() {
  static uint32_t last = 0;
  uint32_t now = millis();
  if (now - last < DT_MS) return;
  last = now;

  // Time in seconds
  const float t = now * 0.001f;

  // Small “wave” motions (ticks). Keep amplitudes modest for safety.
  // Base (ID1): slow yaw wiggle
  int16_t p1 = clampTick(home[0] + (int32_t)(120.0f * sinf(2.0f * PI * 0.10f * t)));

  // Shoulder (ID2) + Elbow (ID3): out-of-phase to make an up/down wave
  int16_t p2 = clampTick(home[1] + (int32_t)(180.0f * sinf(2.0f * PI * 0.18f * t)));
  int16_t p3 = clampTick(home[2] + (int32_t)(180.0f * sinf(2.0f * PI * 0.18f * t + PI)));

  // Gripper (ID4): keep fixed (safer)
  int16_t p4 = home[3];

  // Wrist rotate / EoAT (ID5): gentle wiggle
  int16_t p5 = clampTick(home[4] + (int32_t)(140.0f * sinf(2.0f * PI * 0.22f * t)));

  // Send commands
  st.WritePosEx(1, p1, SPEED, ACC);
  st.WritePosEx(2, p2, SPEED, ACC);
  st.WritePosEx(3, p3, SPEED, ACC);
  st.WritePosEx(4, p4, SPEED, ACC);
  st.WritePosEx(5, p5, SPEED, ACC);
}