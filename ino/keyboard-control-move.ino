#include <Arduino.h>
#include <SCServo.h>

// ===================== Servo bus =====================
SMS_STS st;

static constexpr int SERVO_RX_PIN = 18;
static constexpr int SERVO_TX_PIN = 19;
static constexpr uint32_t SERVO_BAUD = 1000000;

// Servo IDs (typical RoArm-M1): 1..5
static constexpr uint8_t SERVO_ID[5] = {1, 2, 3, 4, 5};

// Motion tuning
static constexpr uint16_t MOVE_SPEED = 1100;
static constexpr uint8_t  MOVE_ACC   = 35;

// Update rate
static constexpr uint32_t UPDATE_DT_MS = 25;

// ===================== Kinematics model (TUNE THESE) =====================
// L1: shoulder->elbow, L2: elbow->wrist-center (mm)
static constexpr float L1_MM = 115.0f;
static constexpr float L2_MM = 115.0f;

// shoulder joint center height above base origin (mm)
static constexpr float SHOULDER_Z_MM = 55.0f;

// wrist-center -> TIP (mm)
static constexpr float TOOL_MM = 70.0f;  // <-- measure and tune

// Tool pitch in r-z plane (rad): 0 = pointing outward
static constexpr float TOOL_PITCH_RAD = 0.0f;

// ===================== Servo mapping (TUNE THESE) =====================
static constexpr float TICKS_PER_RAD = 4096.0f / (2.0f * 3.1415926f);

// tick at 0 rad for each joint in your model
static int16_t tick_zero[5] = {2048, 2048, 2048, 2048, 2048};

// direction sign per joint
static int8_t dir_sign[5] = {+1, -1, -1, +1, -1};

// gear ratio multiplier (shoulder often differs)
static float gear_mul[5] = {1.0f, 3.0f, 1.0f, 1.0f, 1.0f};

// soft limits (tighten later)
static int16_t tick_min[5] = {0, 0, 0, 0, 0};
static int16_t tick_max[5] = {4095, 4095, 4095, 4095, 4095};

// ===================== Target tip position =====================
static float tip_x = 0.0f; // mm
static float tip_y = 0.0f;
static float tip_z = 0.0f;

// Step size for key commands (mm)
static float step_mm = 10.0f;

// Keep gripper fixed at startup
static int16_t gripper_fixed_tick = 2048;

// ===================== Helpers =====================
static float clampf(float v, float lo, float hi) { return (v < lo) ? lo : (v > hi) ? hi : v; }

static int16_t clamp_tick(int32_t t, int idx) {
  if (t < tick_min[idx]) return tick_min[idx];
  if (t > tick_max[idx]) return tick_max[idx];
  return (int16_t)t;
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

// workspace center (in tip space)
static void compute_workspace_center_tip(float &cx, float &cy, float &cz) {
  float r_min = fabsf(L1_MM - L2_MM);
  float r_max = (L1_MM + L2_MM);
  float r_center_wrist = 0.5f * (r_min + r_max);

  float z_center_wrist = SHOULDER_Z_MM + 0.35f * (L1_MM + L2_MM);

  float tip_r = r_center_wrist + TOOL_MM * cosf(TOOL_PITCH_RAD);
  float tip_zc = z_center_wrist + TOOL_MM * sinf(TOOL_PITCH_RAD);

  cx = tip_r;
  cy = 0.0f;
  cz = tip_zc;
}

// ===================== IK: TIP xyz -> joints =====================
static bool solve_ik_tip_xyz(float tip_x_in, float tip_y_in, float tip_z_in,
                            float &j1, float &j2, float &j3, float &j5) {
  j1 = atan2f(tip_y_in, tip_x_in);

  float tip_r = sqrtf(tip_x_in * tip_x_in + tip_y_in * tip_y_in);

  // Convert to wrist-center target
  float wrist_r = tip_r - TOOL_MM * cosf(TOOL_PITCH_RAD);
  float wrist_z = tip_z_in - TOOL_MM * sinf(TOOL_PITCH_RAD);

  float r_sh = wrist_r;
  float z_sh = wrist_z - SHOULDER_Z_MM;

  if (r_sh < 10.0f) return false;

  float d2 = r_sh*r_sh + z_sh*z_sh;
  float c3_raw = (d2 - L1_MM*L1_MM - L2_MM*L2_MM) / (2.0f * L1_MM * L2_MM);
  if (c3_raw < -1.0f || c3_raw > 1.0f) return false;

  float c3 = clampf(c3_raw, -1.0f, 1.0f);
  float s3 = sqrtf(fmaxf(0.0f, 1.0f - c3*c3));     // elbow-down
  j3 = atan2f(s3, c3);

  float k1 = L1_MM + L2_MM * c3;
  float k2 = L2_MM * s3;

  j2 = atan2f(z_sh, r_sh) - atan2f(k2, k1);

  // enforce tool pitch: pitch = j2 + j3 + j5
  j5 = TOOL_PITCH_RAD - (j2 + j3);

  return true;
}

// ===================== Serial command parsing =====================
static void print_help() {
  Serial.println("\nCommands:");
  Serial.println("  x <mm>   : add mm to X (forward/back)");
  Serial.println("  y <mm>   : add mm to Y (left/right)");
  Serial.println("  z <mm>   : add mm to Z (up/down)");
  Serial.println("  step <mm>: set step for keys (w/a/s/d/r/f)");
  Serial.println("  w/s      : +X / -X by step");
  Serial.println("  a/d      : +Y / -Y by step");
  Serial.println("  r/f      : +Z / -Z by step");
  Serial.println("  home     : reset target to centered workspace");
  Serial.println("  where    : print current target");
  Serial.println("  help     : show this help\n");
}

static void print_where() {
  Serial.print("Target tip (mm): X=");
  Serial.print(tip_x, 1);
  Serial.print(" Y=");
  Serial.print(tip_y, 1);
  Serial.print(" Z=");
  Serial.println(tip_z, 1);
}

static bool read_line(String &out) {
  static String buf;
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      out = buf;
      buf = "";
      out.trim();
      return out.length() > 0;
    } else {
      buf += c;
      if (buf.length() > 120) buf.remove(0, buf.length() - 120);
    }
  }
  return false;
}

static void handle_command(const String &line) {
  if (line.length() == 0) return;

  // single-key commands
  if (line.length() == 1) {
    char k = line[0];
    if (k == 'w') tip_x += step_mm;
    else if (k == 's') tip_x -= step_mm;
    else if (k == 'a') tip_y += step_mm;
    else if (k == 'd') tip_y -= step_mm;
    else if (k == 'r') tip_z += step_mm;
    else if (k == 'f') tip_z -= step_mm;
    else if (k == 'h') { // quick home
      compute_workspace_center_tip(tip_x, tip_y, tip_z);
    } else if (k == '?') {
      print_help();
      return;
    }
    print_where();
    return;
  }

  // tokenized commands: "x 10", "step 5", "home", "where"
  String cmd = line;
  cmd.toLowerCase();

  if (cmd == "help") { print_help(); return; }
  if (cmd == "where") { print_where(); return; }
  if (cmd == "home") {
    compute_workspace_center_tip(tip_x, tip_y, tip_z);
    print_where();
    return;
  }

  // Parse: <letter> <value>
  // We'll split by first space
  int sp = cmd.indexOf(' ');
  if (sp < 0) {
    Serial.println("Unknown command. Type 'help'.");
    return;
  }

  String key = cmd.substring(0, sp);
  String val = cmd.substring(sp + 1);
  val.trim();

  float v = val.toFloat(); // works for +/- floats

  if (key == "x") tip_x += v;
  else if (key == "y") tip_y += v;
  else if (key == "z") tip_z += v;
  else if (key == "step") {
    step_mm = fabsf(v);
    Serial.print("step_mm = "); Serial.println(step_mm, 1);
    return;
  } else {
    Serial.println("Unknown command. Type 'help'.");
    return;
  }

  print_where();
}

// ===================== Setup / loop =====================
void setup() {
  Serial.begin(115200);
  delay(200);

  Serial1.begin(SERVO_BAUD, SERIAL_8N1, SERVO_RX_PIN, SERVO_TX_PIN);
  st.pSerial = &Serial1;

  delay(500);
  Serial.println("RoArm-M1 serial XYZ teleop (TIP IK)");
  print_help();

  // Hold gripper at current position
  int gp = st.ReadPos(SERVO_ID[3]);
  if (gp >= 0 && gp <= 4095) gripper_fixed_tick = (int16_t)gp;

  // Start at centered target
  compute_workspace_center_tip(tip_x, tip_y, tip_z);
  print_where();
}

void loop() {
  // ---- Handle input lines ----
  String line;
  if (read_line(line)) {
    handle_command(line);
  }

  // ---- Periodic IK + command ----
  static uint32_t last_ms = 0;
  uint32_t now = millis();
  if (now - last_ms < UPDATE_DT_MS) return;
  last_ms = now;

  float j1, j2, j3, j5;
  if (!solve_ik_tip_xyz(tip_x, tip_y, tip_z, j1, j2, j3, j5)) {
    Serial.println("IK unreachable. Try smaller steps or 'home'.");
    return; // hold last position
  }

  int16_t ticks[5];
  ticks[0] = angle_to_tick(0, j1);
  ticks[1] = angle_to_tick(1, j2);
  ticks[2] = angle_to_tick(2, j3);
  ticks[3] = gripper_fixed_tick;
  ticks[4] = angle_to_tick(4, j5);

  send_ticks(ticks);
}