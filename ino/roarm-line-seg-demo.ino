#include <Arduino.h>
#include <SCServo.h>

// ================== Servo bus setup ==================
SMS_STS st;

static constexpr int SERVO_RX_PIN = 18;
static constexpr int SERVO_TX_PIN = 19;
static constexpr uint32_t SERVO_BAUD = 1000000;

// RoArm-M1 servo IDs: 1..5 (base, shoulder, elbow, gripper, wrist rotate)
static constexpr uint8_t SERVO_ID[5] = {1, 2, 3, 4, 5};

// Motion tuning
static constexpr uint16_t MOVE_SPEED = 1200; // lower = gentler
static constexpr uint8_t  MOVE_ACC   = 35;   // lower = gentler

// Update timing
static constexpr uint32_t UPDATE_DT_MS = 25; // 40 Hz

// ================== Kinematics model (TUNE THESE) ==================
// Units: millimeters and radians.
// L1: shoulder->elbow, L2: elbow->wrist-center (NOT tip).
static constexpr float L1_MM = 115.0f;
static constexpr float L2_MM = 115.0f;

// Height of shoulder joint center above base origin
static constexpr float SHOULDER_Z_MM = 55.0f;

// Tool length: wrist-center -> TIP distance (IMPORTANT for tip tracking)
static constexpr float TOOL_MM = 70.0f; // <-- measure & tune (mm)

// Desired tool pitch in the r-z plane (0 = pointing straight outward in +r, positive lifts up)
static constexpr float TOOL_PITCH_RAD = 0.0f;

// ================== Servo mapping (TUNE THESE) ==================
// ST3215: 0..4095 ticks = 0..2π rad (one turn)
static constexpr float TICKS_PER_RAD = 4096.0f / (2.0f * 3.1415926f);

// tick_zero = servo tick when model joint angle = 0 rad
static int16_t tick_zero[5] = {2048, 2048, 2048, 2048, 2048};

// dir_sign: flip if joint moves opposite direction
static int8_t dir_sign[5] = {+1, -1, -1, +1, -1};

// gear_mul: mechanical ratio; shoulder often differs. Start here, tune later.
static float gear_mul[5] = {1.0f, 3.0f, 1.0f, 1.0f, 1.0f};

// Soft tick limits (set tighter after you test safely)
static int16_t tick_min[5] = {0, 0, 0, 0, 0};
static int16_t tick_max[5] = {4095, 4095, 4095, 4095, 4095};

// ================== Trajectory definition ==================
// We center a line segment in the "middle" of reachable workspace automatically.
// Segment length (bigger segment)
static constexpr float LINE_LEN_MM = 140.0f;   // total length of segment (mm)
static constexpr uint32_t SEGMENT_MS = 5000;   // time tip takes to go end->end
static constexpr uint32_t HOLD_MS    = 700;    // pause at ends

// Direction of the line in XY plane (unit vector). Here: along +X axis.
static constexpr float LINE_DIR_X = 1.0f;
static constexpr float LINE_DIR_Y = 0.0f;

// Keep tip Z constant at workspace-center Z
// (You can change to make diagonal lines by also varying Z.)

// ================== Helpers ==================
static float clampf(float v, float lo, float hi) { return (v < lo) ? lo : (v > hi) ? hi : v; }

static int16_t clamp_tick(int32_t t, int idx) {
  if (t < tick_min[idx]) return tick_min[idx];
  if (t > tick_max[idx]) return tick_max[idx];
  return (int16_t)t;
}

static float smoothstep(float x) {
  if (x <= 0.0f) return 0.0f;
  if (x >= 1.0f) return 1.0f;
  return x * x * (3.0f - 2.0f * x);
}

static int16_t angle_to_tick(int joint_idx, float rad) {
  float ticks_f = (float)tick_zero[joint_idx]
                + (float)dir_sign[joint_idx] * gear_mul[joint_idx] * (rad * TICKS_PER_RAD);
  return clamp_tick((int32_t)lroundf(ticks_f), joint_idx);
}

static void send_ticks(const int16_t ticks[5]) {
  for (int i = 0; i < 5; i++) {
    st.WritePosEx(SERVO_ID[i], ticks[i], MOVE_SPEED, MOVE_ACC);
  }
}

// ================== IK: base + 2-link planar to wrist ==================
// We solve for wrist-center, then compute wrist joint so tool pitch matches TOOL_PITCH_RAD.
// Input: tip xyz (mm)
// Output: joint angles (rad): j1(base), j2(shoulder), j3(elbow), j5(wrist pitch/rotate proxy)
// Return false if unreachable.
static bool solve_ik_tip_xyz(float tip_x, float tip_y, float tip_z,
                            float &j1, float &j2, float &j3, float &j5) {
  // Base yaw
  j1 = atan2f(tip_y, tip_x);

  // Convert tip target to cylindrical in base frame
  float tip_r = sqrtf(tip_x * tip_x + tip_y * tip_y);

  // Compute wrist-center target by subtracting tool vector in r-z plane
  // Tool direction is defined by TOOL_PITCH_RAD in r-z plane.
  float wrist_r = tip_r - TOOL_MM * cosf(TOOL_PITCH_RAD);
  float wrist_z = tip_z - TOOL_MM * sinf(TOOL_PITCH_RAD);

  // Shoulder plane coordinates
  float z_sh = wrist_z - SHOULDER_Z_MM;
  float r_sh = wrist_r;

  // Reject obviously impossible wrist_r (behind base axis)
  if (r_sh < 10.0f) return false;

  // 2-link IK
  float d2 = r_sh*r_sh + z_sh*z_sh;
  float c3_raw = (d2 - L1_MM*L1_MM - L2_MM*L2_MM) / (2.0f * L1_MM * L2_MM);

  if (c3_raw < -1.0f || c3_raw > 1.0f) return false; // unreachable

  float c3 = clampf(c3_raw, -1.0f, 1.0f);

  // elbow-down solution
  float s3 = sqrtf(fmaxf(0.0f, 1.0f - c3*c3));
  j3 = atan2f(s3, c3);

  float k1 = L1_MM + L2_MM * c3;
  float k2 = L2_MM * s3;

  j2 = atan2f(z_sh, r_sh) - atan2f(k2, k1);

  // Enforce desired tool pitch: pitch = j2 + j3 + j5
  j5 = TOOL_PITCH_RAD - (j2 + j3);

  return true;
}

// ================== Workspace-centered line segment ==================
// We compute a "mid-workspace" center automatically using link lengths.
// r_center is mid of [r_min, r_max] => equals max(L1, L2) for equal-ish links.
// Then we ensure tip target stays safely reachable given TOOL and Z.
static void compute_workspace_center(float &cx, float &cy, float &cz) {
  float r_min = fabsf(L1_MM - L2_MM);
  float r_max = (L1_MM + L2_MM);

  // Middle of reachable annulus for the wrist-center
  float r_center_wrist = 0.5f * (r_min + r_max);

  // Put center slightly above shoulder height to avoid low collisions
  float z_center = SHOULDER_Z_MM + 0.35f * (L1_MM + L2_MM);

  // Convert wrist-center to tip-center by adding tool vector back
  float tip_r_center = r_center_wrist + TOOL_MM * cosf(TOOL_PITCH_RAD);
  float tip_z_center = z_center + TOOL_MM * sinf(TOOL_PITCH_RAD);

  // Place the center on +X axis by default
  cx = tip_r_center;
  cy = 0.0f;
  cz = tip_z_center;
}

// ================== Trajectory state machine ==================
enum Phase { GO_0_TO_1, HOLD_1, GO_1_TO_0, HOLD_0 };
static Phase phase = GO_0_TO_1;
static uint32_t phase_start_ms = 0;
static uint32_t last_update_ms = 0;

// Keep gripper fixed
static int16_t gripper_fixed_tick = 2048;

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial1.begin(SERVO_BAUD, SERIAL_8N1, SERVO_RX_PIN, SERVO_TX_PIN);
  st.pSerial = &Serial1;

  delay(500);
  Serial.println("RoArm-M1 TIP line segment IK demo (centered in workspace)");

  // Read current gripper tick so we keep it fixed
  int gp = st.ReadPos(SERVO_ID[3]);
  if (gp >= 0 && gp <= 4095) gripper_fixed_tick = (int16_t)gp;

  phase_start_ms = millis();
  last_update_ms = phase_start_ms;
}

void loop() {
  uint32_t now = millis();
  if (now - last_update_ms < UPDATE_DT_MS) return;
  last_update_ms = now;

  uint32_t elapsed = now - phase_start_ms;

  // Compute workspace-centered segment endpoints (in tip XYZ)
  float cx, cy, cz;
  compute_workspace_center(cx, cy, cz);

  // Direction normalize (just in case)
  float dir_norm = sqrtf(LINE_DIR_X*LINE_DIR_X + LINE_DIR_Y*LINE_DIR_Y);
  float dx = (dir_norm > 1e-6f) ? (LINE_DIR_X / dir_norm) : 1.0f;
  float dy = (dir_norm > 1e-6f) ? (LINE_DIR_Y / dir_norm) : 0.0f;

  // Endpoints centered at (cx,cy,cz)
  float half = 0.5f * LINE_LEN_MM;
  float p0x = cx - half * dx;
  float p0y = cy - half * dy;
  float p0z = cz;

  float p1x = cx + half * dx;
  float p1y = cy + half * dy;
  float p1z = cz;

  // Phase interpolation
  float u = 0.0f;
  if (phase == GO_0_TO_1 || phase == GO_1_TO_0) {
    u = smoothstep((float)elapsed / (float)SEGMENT_MS);
  }

  float tip_x, tip_y, tip_z;
  if (phase == GO_0_TO_1) {
    tip_x = p0x + (p1x - p0x) * u;
    tip_y = p0y + (p1y - p0y) * u;
    tip_z = p0z + (p1z - p0z) * u;
  } else if (phase == GO_1_TO_0) {
    tip_x = p1x + (p0x - p1x) * u;
    tip_y = p1y + (p0y - p1y) * u;
    tip_z = p1z + (p0z - p1z) * u;
  } else if (phase == HOLD_1) {
    tip_x = p1x; tip_y = p1y; tip_z = p1z;
  } else {
    tip_x = p0x; tip_y = p0y; tip_z = p0z;
  }

  // IK solve
  float j1, j2, j3, j5;
  if (!solve_ik_tip_xyz(tip_x, tip_y, tip_z, j1, j2, j3, j5)) {
    // If unreachable, just don't send (holds last position)
    Serial.println("IK unreachable at this tip target. Reduce LINE_LEN_MM or adjust center/geometry.");
    return;
  }

  // Convert to ticks
  int16_t ticks[5];
  ticks[0] = angle_to_tick(0, j1);
  ticks[1] = angle_to_tick(1, j2);
  ticks[2] = angle_to_tick(2, j3);
  ticks[3] = gripper_fixed_tick;
  ticks[4] = angle_to_tick(4, j5);

  send_ticks(ticks);

  // Phase transitions
  if (phase == GO_0_TO_1 && elapsed >= SEGMENT_MS) {
    phase = HOLD_1; phase_start_ms = now;
  } else if (phase == HOLD_1 && elapsed >= HOLD_MS) {
    phase = GO_1_TO_0; phase_start_ms = now;
  } else if (phase == GO_1_TO_0 && elapsed >= SEGMENT_MS) {
    phase = HOLD_0; phase_start_ms = now;
  } else if (phase == HOLD_0 && elapsed >= HOLD_MS) {
    phase = GO_0_TO_1; phase_start_ms = now;
  }
}