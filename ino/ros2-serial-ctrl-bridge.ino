#include <Arduino.h>
#include <SCServo.h>
#include <stdlib.h>
#include <string.h>

/*
  Standalone serial commands (newline-terminated):
  - PING
  - HOME
  - STATUS
  - DEMO
  - POSE P1 P2 P3 P4 P5 [S] [A]
    Example: POSE 2048 1900 2200 2048 2048 600 40

  ROS2 command format is still supported:
  {"T":3,"P1":2048,"P2":2048,"P3":2048,"P4":2048,"P5":2048,"S1":0,"S2":0,"S3":0,"S4":0,"S5":0,"A1":60,"A2":60,"A3":60,"A4":60,"A5":60}
*/

// ROS2 node serial link:
// /dev/ttyUSB0 @ 115200
static constexpr uint32_t PC_BAUD = 115200;

// ST3215/SMS_STS bus-servo link (ESP32 default pins used in this repo)
static constexpr int SERVO_RX_PIN = 18;
static constexpr int SERVO_TX_PIN = 19;
static constexpr uint32_t SERVO_BAUD = 1000000;

static constexpr size_t FRAME_MAX = 256;
static constexpr size_t LINE_BUF_MAX = 128;

SMS_STS st;

char frameBuf[FRAME_MAX];
size_t frameLen = 0;
bool inFrame = false;
int braceDepth = 0;
char lineBuf[LINE_BUF_MAX];
size_t lineLen = 0;

int16_t currentPos[5] = {2048, 2048, 2048, 2048, 2048};
uint16_t currentSpeed[5] = {600, 600, 600, 600, 600};
uint8_t currentAcc[5] = {60, 60, 60, 60, 60};

static int16_t clampPos(long v) {
  if (v < 0) return 0;
  if (v > 4095) return 4095;
  return static_cast<int16_t>(v);
}

static uint16_t clampSpeed(long v) {
  if (v < 0) return 0;
  if (v > 3400) return 3400;
  return static_cast<uint16_t>(v);
}

static uint8_t clampAcc(long v) {
  if (v < 0) return 0;
  if (v > 150) return 150;
  return static_cast<uint8_t>(v);
}

static bool extractLong(const char *json, const char *key, long &out) {
  const char *p = strstr(json, key);
  if (!p) return false;

  p = strchr(p, ':');
  if (!p) return false;
  p++;  // Move to value start

  char *endPtr = nullptr;
  out = strtol(p, &endPtr, 10);
  return endPtr != p;
}

static void writePose(const int16_t p[5], const uint16_t s[5], const uint8_t a[5]) {
  for (int i = 0; i < 5; i++) {
    st.WritePosEx(i + 1, p[i], s[i], a[i]);
    currentPos[i] = p[i];
    currentSpeed[i] = s[i];
    currentAcc[i] = a[i];
  }
}

static void moveAll(int16_t p1, int16_t p2, int16_t p3, int16_t p4, int16_t p5, uint16_t speed, uint8_t acc) {
  int16_t p[5] = {p1, p2, p3, p4, p5};
  uint16_t s[5] = {speed, speed, speed, speed, speed};
  uint8_t a[5] = {acc, acc, acc, acc, acc};
  writePose(p, s, a);
}

static void runDemo() {
  moveAll(2048, 2048, 2048, 2048, 2048, 500, 40);
  delay(900);
  moveAll(2350, 1850, 2300, 2048, 2048, 500, 40);
  delay(900);
  moveAll(1750, 2350, 1800, 2048, 2048, 500, 40);
  delay(900);
  moveAll(2048, 2048, 2048, 2300, 2048, 500, 40);
  delay(700);
  moveAll(2048, 2048, 2048, 1750, 2048, 500, 40);
  delay(700);
  moveAll(2048, 2048, 2048, 2048, 2048, 500, 40);
  delay(700);
}

static void processFrame(const char *json) {
  long cmdType = 0;
  if (!extractLong(json, "\"T\"", cmdType) || cmdType != 3) {
    return;
  }

  long p[5];
  long s[5] = {0, 0, 0, 0, 0};
  long a[5] = {60, 60, 60, 60, 60};

  if (!extractLong(json, "\"P1\"", p[0])) return;
  if (!extractLong(json, "\"P2\"", p[1])) return;
  if (!extractLong(json, "\"P3\"", p[2])) return;
  if (!extractLong(json, "\"P4\"", p[3])) return;
  if (!extractLong(json, "\"P5\"", p[4])) return;

  extractLong(json, "\"S1\"", s[0]);
  extractLong(json, "\"S2\"", s[1]);
  extractLong(json, "\"S3\"", s[2]);
  extractLong(json, "\"S4\"", s[3]);
  extractLong(json, "\"S5\"", s[4]);

  extractLong(json, "\"A1\"", a[0]);
  extractLong(json, "\"A2\"", a[1]);
  extractLong(json, "\"A3\"", a[2]);
  extractLong(json, "\"A4\"", a[3]);
  extractLong(json, "\"A5\"", a[4]);

  int16_t pp[5];
  uint16_t ss[5];
  uint8_t aa[5];
  for (int i = 0; i < 5; i++) {
    pp[i] = clampPos(p[i]);
    ss[i] = clampSpeed(s[i]);
    aa[i] = clampAcc(a[i]);
  }
  writePose(pp, ss, aa);

  Serial.println("OK");
}

static void resetFrameParser() {
  frameLen = 0;
  inFrame = false;
  braceDepth = 0;
}

static void processLine(char *line) {
  while (*line == ' ' || *line == '\t') line++;
  if (*line == '\0') return;

  size_t n = strlen(line);
  while (n > 0 && (line[n - 1] == ' ' || line[n - 1] == '\t')) {
    line[n - 1] = '\0';
    n--;
  }

  if (strcmp(line, "PING") == 0) {
    Serial.println("OK");
    return;
  }

  if (strcmp(line, "HOME") == 0) {
    moveAll(2048, 2048, 2048, 2048, 2048, 500, 40);
    Serial.println("OK");
    return;
  }

  if (strcmp(line, "DEMO") == 0) {
    Serial.println("OK");
    runDemo();
    return;
  }

  if (strcmp(line, "STATUS") == 0) {
    Serial.print("POS ");
    for (int i = 0; i < 5; i++) {
      Serial.print(currentPos[i]);
      if (i < 4) Serial.print(' ');
    }
    Serial.println();
    return;
  }

  if (strncmp(line, "POSE", 4) == 0) {
    long vals[7];
    int count = 0;

    char *tok = strtok(line, " ");
    while (tok != nullptr) {
      if (count > 0 && count <= 7) {
        vals[count - 1] = strtol(tok, nullptr, 10);
      }
      count++;
      tok = strtok(nullptr, " ");
    }

    // POSE P1 P2 P3 P4 P5 [S] [A]
    if (count >= 6) {
      int16_t p[5] = {
        clampPos(vals[0]),
        clampPos(vals[1]),
        clampPos(vals[2]),
        clampPos(vals[3]),
        clampPos(vals[4])
      };
      uint16_t s = (count >= 7) ? clampSpeed(vals[5]) : 500;
      uint8_t a = (count >= 8) ? clampAcc(vals[6]) : 40;
      uint16_t ss[5] = {s, s, s, s, s};
      uint8_t aa[5] = {a, a, a, a, a};
      writePose(p, ss, aa);
      Serial.println("OK");
      return;
    }
  }

  Serial.println("ERR unknown_cmd");
}

void setup() {
  Serial.begin(PC_BAUD);
  delay(200);

  Serial1.begin(SERVO_BAUD, SERIAL_8N1, SERVO_RX_PIN, SERVO_TX_PIN);
  st.pSerial = &Serial1;

  Serial.println("READY");
}

void loop() {
  while (Serial.available() > 0) {
    char c = static_cast<char>(Serial.read());

    if (inFrame) {
      if (frameLen >= FRAME_MAX - 1) {
        resetFrameParser();
        Serial.println("ERR frame_overflow");
        continue;
      }

      frameBuf[frameLen++] = c;

      if (c == '{') braceDepth++;
      if (c == '}') braceDepth--;

      if (braceDepth == 0) {
        frameBuf[frameLen] = '\0';
        processFrame(frameBuf);
        resetFrameParser();
      }
      continue;
    }

    if (c == '{') {
      inFrame = true;
      braceDepth = 1;
      frameLen = 0;
      frameBuf[frameLen++] = c;
      continue;
    }

    if (c == '\n' || c == '\r') {
      if (lineLen > 0) {
        lineBuf[lineLen] = '\0';
        processLine(lineBuf);
        lineLen = 0;
      }
      continue;
    }

    if (lineLen < LINE_BUF_MAX - 1) {
      lineBuf[lineLen++] = c;
    }
  }
}
