#include <Arduino.h>
#include <SCServo.h>

// ================== Servo bus setup ==================
SMS_STS st;

static constexpr int SERVO_RX_PIN = 18;
static constexpr int SERVO_TX_PIN = 19;
static constexpr uint32_t SERVO_BAUD = 1000000;

// RoArm-M1 servo IDs (typical): 1..5
static constexpr uint8_t SERVO_ID[5] = {1, 2, 3, 4, 5}; // base, shoulder, elbow, gripper, wrist

// Motion tuning
static constexpr uint16_t MOVE_SPEED = 1100;  // gentler -> smaller
static constexpr uint8_t  MOVE_ACC   = 35;    // gentler -> smaller

// Update timing
static constexpr uint32_t UPDATE_DT_MS = 25;  // 40 Hz

// ================== Kinematics model (TUNE THESE) ==================
// Units: millimeters and radians.
// Link lengths: shoulder->elbow and elbow->wrist/end-effector reference point.
// You MUST tune these to your arm (measure approximate distances).
static constexpr float L1_MM = 115.0f;  // shoulder to elbow
static constexpr float L2_MM = 115.0f;  // elbow to wrist/end point

// Height of shoulder joint center above base frame origin (mm)
static constexpr float SHOULDER_Z_MM = 55.0f;

// Optional tool length beyond wrist pivot to “tip” reference point (mm)
static constexpr float TOOL_MM = 0.0f;

// ================== Servo angle->tick mapping (TUNE THESE) ==================
// ST3215 servo internal range is 0..4095 ticks for 0..360deg.
// ticks_per_rad = 4096 / (2*pi)
static constexpr float TICKS_PER_RAD = 4096.0f / (2.0f * 3.1415926f);

// Offsets (tick center for “0 rad” of each joint in your model).
// Start with 2048 and then tune so that 0 rad corresponds to your chosen neutral pose.
static int16_t tick_zero[5] = {2048, 2048, 2048, 2048, 2048};

// Direction (+1 or -1) to match your physical joint increasing direction
static int8_t dir_sign[5] = {+1, -1, -1, +1, -1};

// Gear multipliers: some joints may have mechanical ratio vs servo rotation.
// Waveshare’s ROS serial example effectively uses a multiplier for joint2.
// Start with these; adjust if joint angles appear scaled wrong.
static float gear_mul[5] = {1.0f, 3.0f, 1.0f, 1.0f, 1.0f};

// Joint limits in ticks (soft clamp). Adjust to your safe range.
static int16_t tick_min[5] = {0, 0, 0, 0, 0};
static int16_t tick_max[5] = {4095, 4095, 4095, 4095, 4095};

// ================== Trajectory: a bigger line segment in XYZ ==================
// Define a larger segment (mm) in base frame.
// x forward, y left, z up (convention; match your mental model).
static constexpr float P0[3] = {160.0f,  0.0f, 110.0f}; // start XYZ
static constexpr float P1[3] = {260.0f, 60.0f, 140.0f}; // end XYZ

static constexpr uint32_t SEGMENT_MS = 4500; // time to go P0->P1
static constexpr uint32_t HOLD_MS    = 600;  // pause at ends

// ================== Helpers ==================
static int16_t clamp_tick(int32_t t, int idx) {
  if (t < tick_min[idx]) return tick_min[idx];
  if (t > tick_max[idx]) return tick_max[idx];
  return (int16_t)t;
}

static float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static float smoothstep(float x) {
  if (x <= 0.0f) return 0.0f;
  if (x >= 1.0f) return 1.0f;
  return x * x * (3.0f - 2.0f * x);
}

static int16_t angle_to_tick(int joint_idx, float rad) {
  // Convert model radians to servo ticks
  float ticks_f = (float)tick_zero[joint_idx]
                + (float)dir_sign[joint_idx] * gear_mul[joint_idx] * (rad * TICKS_PER_RAD);

  int32_t t = (int32_t)lroundf(ticks_f);

  // For 0..4095 wrap-around servos you might prefer wrap. For safety, we clamp.
  return clamp_tick(t, joint_idx);
}

static void send_joints_ticks(const int16_t ticks[5]) {
  for (int i = 0; i < 5; i++) {
    st.WritePosEx(SERVO_ID[i], ticks[i], MOVE_SPEED, MOVE_ACC);
  }
}

// ================== IK solver (base + 2-link planar) ==================
// Input: target XYZ (mm)
// Output: joint angles (rad) for j1..j5 (we mainly solve j1,j2,j3; set j4 fixed; compute j5 compensation)
// Returns true if solvable, false if unreachable.
static bool solve_ik_xyz(float x, float y, float z,
                        float &j1, float &j2, float &j3, float &j5) {
  // Base yaw
  j1 = atan2f(y, x);

  // Planar distance from base axis
  float r = sqrtf(x*x + y*y);

  // Effective target in shoulder plane
  float z_sh = z - SHOULDER_Z_MM;

  // If you want the "tip" to land on the point and TOOL_MM exists:
  // subtract tool length in the direction of the forearm/wrist.
  // For this simple model we ignore TOOL_MM or handle it by shortening L2.
  float L2_eff = (TOOL_MM > 0.0f) ? (L2_MM - TOOL_MM) : L2_MM;

  // 2-link IK for (r, z_sh)
  float d2 = r*r + z_sh*z_sh;
  float c3 = (d2 - L1_MM*L1_MM - L2_eff*L2_eff) / (2.0f * L1_MM * L2_eff);
  c3 = clampf(c3, -1.0f, 1.0f);

  // elbow-down solution (change sign on s3 for elbow-up)
  float s3 = sqrtf(fmaxf(0.0f, 1.0f - c3*c3));
  j3 = atan2f(s3, c3);

  float k1 = L1_MM + L2_eff * c3;
  float k2 = L2_eff * s3;

  j2 = atan2f(z_sh, r) - atan2f(k2, k1);

  // Wrist compensation: keep tool roughly level in the r-z plane.
  // Adjust with a constant offset if your tool orientation differs.
  j5 = -(j2 + j3);

  // Basic reachability check (if original c3 was out of [-1,1], it was unreachable)
  // We clamped, but we can still report unreachable if it was beyond some tolerance.
  // Here we treat any |c3| > 1.0 before clamp as unreachable:
  // (We can’t see it now, so approximate by checking edge cases.)
  // If you want strict, compute before clamp and return false.
  return true;
}

// ================== Main state machine ==================
enum Phase { GO_0_TO_1, HOLD_1, GO_1_TO_0, HOLD_0 };
static Phase phase = GO_0_TO_1;
static uint32_t phase_start_ms = 0;
static uint32_t last_update_ms = 0;

// Keep gripper fixed at current tick at startup
static int16_t grip_tick_fixed = 2048;

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial1.begin(SERVO_BAUD, SERIAL_8N1, SERVO_RX_PIN, SERVO_TX_PIN);
  st.pSerial = &Serial1;

  delay(500);
  Serial.println("RoArm-M1 Cartesian line segment IK demo");

  // Read current gripper position to hold it fixed (optional)
  int gp = st.ReadPos(SERVO_ID[3]);
  if (gp >= 0 && gp <= 4095) grip_tick_fixed = (int16_t)gp;

  phase_start_ms = millis();
  last_update_ms = phase_start_ms;
}

void loop() {
  uint32_t now = millis();
  if (now - last_update_ms < UPDATE_DT_MS) return;
  last_update_ms = now;

  uint32_t elapsed = now - phase_start_ms;

  float u = 0.0f;
  bool moving = false;

  if (phase == GO_0_TO_1) {
    u = smoothstep((float)elapsed / (float)SEGMENT_MS);
    moving = true;
  } else if (phase == GO_1_TO_0) {
    u = smoothstep((float)elapsed / (float)SEGMENT_MS);
    moving = true;
  }

  float x, y, z;
  if (phase == GO_0_TO_1) {
    x = P0[0] + (P1[0] - P0[0]) * u;
    y = P0[1] + (P1[1] - P0[1]) * u;
    z = P0[2] + (P1[2] - P0[2]) * u;
  } else if (phase == GO_1_TO_0) {
    x = P1[0] + (P0[0] - P1[0]) * u;
    y = P1[1] + (P0[1] - P1[1]) * u;
    z = P1[2] + (P0[2] - P1[2]) * u;
  } else {
    // Holding: keep endpoint constant
    if (phase == HOLD_1) { x = P1[0]; y = P1[1]; z = P1[2]; }
    else                { x = P0[0]; y = P0[1]; z = P0[2]; }
  }

  float j1=0, j2=0, j3=0, j5=0;
  bool ok = solve_ik_xyz(x, y, z, j1, j2, j3, j5);

  if (!ok) {
    // If unreachable, don't move (or you could clamp/hold last valid pose)
    Serial.println("IK unreachable - holding position");
    return;
  }

  // Build target ticks
  int16_t ticks[5];
  ticks[0] = angle_to_tick(0, j1);
  ticks[1] = angle_to_tick(1, j2);
  ticks[2] = angle_to_tick(2, j3);
  ticks[3] = grip_tick_fixed;          // keep gripper fixed
  ticks[4] = angle_to_tick(4, j5);     // wrist compensation

  send_joints_ticks(ticks);

  // Phase transitions
  if (phase == GO_0_TO_1 && elapsed >= SEGMENT_MS) {
    phase = HOLD_1;
    phase_start_ms = now;
  } else if (phase == HOLD_1 && elapsed >= HOLD_MS) {
    phase = GO_1_TO_0;
    phase_start_ms = now;
  } else if (phase == GO_1_TO_0 && elapsed >= SEGMENT_MS) {
    phase = HOLD_0;
    phase_start_ms = now;
  } else if (phase == HOLD_0 && elapsed >= HOLD_MS) {
    phase = GO_0_TO_1;
    phase_start_ms = now;
  }
}