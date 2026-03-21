#include <AccelStepper.h>

// NodeMCU ESP8266 gantry controller.
// Commands over USB serial:
//   PING
//   HOME
//   STOP
//   G0 X<mm> Y<mm> Z<mm> F<mm_per_s>
//
// This sketch assumes the two physical X drivers share the same STEP/DIR
// lines so both X motors always move in lock-step. That keeps the controller
// within the NodeMCU GPIO budget while still driving 2X + 1Y + 1Z motors.

constexpr unsigned long SERIAL_BAUD = 115200;
constexpr unsigned long STATUS_PERIOD_MS = 100;
constexpr float DEFAULT_FEED_MM_S = 40.0f;
constexpr float DEFAULT_ACCEL_MM_S2 = 120.0f;

// Calibrate these values for your mechanics and microstepping.
constexpr float X_STEPS_PER_MM = 80.0f;
constexpr float Y_STEPS_PER_MM = 80.0f;
constexpr float Z_STEPS_PER_MM = 400.0f;

// Safe default NodeMCU pins that leave the USB serial pins free.
constexpr uint8_t X_STEP_PIN = D1;
constexpr uint8_t X_DIR_PIN = D2;
constexpr uint8_t Y_STEP_PIN = D5;
constexpr uint8_t Y_DIR_PIN = D6;
constexpr uint8_t Z_STEP_PIN = D7;
constexpr uint8_t Z_DIR_PIN = D0;

AccelStepper xAxis(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
AccelStepper yAxis(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);
AccelStepper zAxis(AccelStepper::DRIVER, Z_STEP_PIN, Z_DIR_PIN);

char serialBuffer[96];
size_t serialLength = 0;
String lastError = "";

long mmToSteps(float mm, float stepsPerMm) {
  return lroundf(mm * stepsPerMm);
}

float stepsToMm(long steps, float stepsPerMm) {
  return static_cast<float>(steps) / stepsPerMm;
}

float currentXmm() {
  return stepsToMm(xAxis.currentPosition(), X_STEPS_PER_MM);
}

float currentYmm() {
  return stepsToMm(yAxis.currentPosition(), Y_STEPS_PER_MM);
}

float currentZmm() {
  return stepsToMm(zAxis.currentPosition(), Z_STEPS_PER_MM);
}

bool isBusy() {
  return xAxis.distanceToGo() != 0 || yAxis.distanceToGo() != 0 || zAxis.distanceToGo() != 0;
}

void configureAxis(AccelStepper& axis, float stepsPerMm) {
  axis.setMinPulseWidth(2);
  axis.setMaxSpeed(DEFAULT_FEED_MM_S * stepsPerMm);
  axis.setAcceleration(DEFAULT_ACCEL_MM_S2 * stepsPerMm);
}

void stopAllAxes() {
  const long x = xAxis.currentPosition();
  const long y = yAxis.currentPosition();
  const long z = zAxis.currentPosition();
  xAxis.moveTo(x);
  yAxis.moveTo(y);
  zAxis.moveTo(z);
}

void zeroAllAxes() {
  stopAllAxes();
  xAxis.setCurrentPosition(0);
  yAxis.setCurrentPosition(0);
  zAxis.setCurrentPosition(0);
}

float parseField(const String& line, char prefix, float fallbackValue) {
  const int start = line.indexOf(prefix);
  if (start < 0) {
    return fallbackValue;
  }

  int end = line.indexOf(' ', start);
  if (end < 0) {
    end = line.length();
  }

  return line.substring(start + 1, end).toFloat();
}

void applyCoordinatedSpeeds(float targetXmm, float targetYmm, float targetZmm, float feedMmS) {
  const float dx = fabsf(targetXmm - currentXmm());
  const float dy = fabsf(targetYmm - currentYmm());
  const float dz = fabsf(targetZmm - currentZmm());
  const float path = sqrtf(dx * dx + dy * dy + dz * dz);

  if (path < 0.001f) {
    stopAllAxes();
    return;
  }

  const float xSpeed = max(1.0f, feedMmS * (dx / path) * X_STEPS_PER_MM);
  const float ySpeed = max(1.0f, feedMmS * (dy / path) * Y_STEPS_PER_MM);
  const float zSpeed = max(1.0f, feedMmS * (dz / path) * Z_STEPS_PER_MM);

  xAxis.setMaxSpeed(xSpeed);
  yAxis.setMaxSpeed(ySpeed);
  zAxis.setMaxSpeed(zSpeed);

  xAxis.setAcceleration(DEFAULT_ACCEL_MM_S2 * X_STEPS_PER_MM);
  yAxis.setAcceleration(DEFAULT_ACCEL_MM_S2 * Y_STEPS_PER_MM);
  zAxis.setAcceleration(DEFAULT_ACCEL_MM_S2 * Z_STEPS_PER_MM);
}

void queueMove(float targetXmm, float targetYmm, float targetZmm, float feedMmS) {
  const float safeFeed = feedMmS > 0.0f ? feedMmS : DEFAULT_FEED_MM_S;
  applyCoordinatedSpeeds(targetXmm, targetYmm, targetZmm, safeFeed);
  xAxis.moveTo(mmToSteps(targetXmm, X_STEPS_PER_MM));
  yAxis.moveTo(mmToSteps(targetYmm, Y_STEPS_PER_MM));
  zAxis.moveTo(mmToSteps(targetZmm, Z_STEPS_PER_MM));
}

void publishStatus() {
  Serial.printf("BUSY %d\n", isBusy() ? 1 : 0);
  Serial.printf("POS X%.3f Y%.3f Z%.3f\n", currentXmm(), currentYmm(), currentZmm());
}

void reportError(const String& error) {
  lastError = error;
  Serial.print("ERR ");
  Serial.println(lastError);
}

void processCommand(const String& line) {
  if (line == "PING") {
    Serial.println("OK");
    return;
  }

  if (line == "HOME") {
    zeroAllAxes();
    Serial.println("OK");
    return;
  }

  if (line == "STOP") {
    stopAllAxes();
    Serial.println("OK");
    return;
  }

  if (line.startsWith("G0")) {
    const float targetX = parseField(line, 'X', currentXmm());
    const float targetY = parseField(line, 'Y', currentYmm());
    const float targetZ = parseField(line, 'Z', currentZmm());
    const float feed = parseField(line, 'F', DEFAULT_FEED_MM_S);

    if (feed <= 0.0f) {
      reportError("bad_feed");
      return;
    }

    queueMove(targetX, targetY, targetZ, feed);
    Serial.println("OK");
    return;
  }

  reportError("unknown_cmd");
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    const char ch = static_cast<char>(Serial.read());
    if (ch == '\r') {
      continue;
    }

    if (ch == '\n') {
      serialBuffer[serialLength] = '\0';
      String line = String(serialBuffer);
      line.trim();
      serialLength = 0;
      if (line.length() > 0) {
        processCommand(line);
      }
      continue;
    }

    if (serialLength < sizeof(serialBuffer) - 1) {
      serialBuffer[serialLength++] = ch;
    } else {
      serialLength = 0;
      reportError("cmd_too_long");
    }
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial.setTimeout(5);

  configureAxis(xAxis, X_STEPS_PER_MM);
  configureAxis(yAxis, Y_STEPS_PER_MM);
  configureAxis(zAxis, Z_STEPS_PER_MM);

  delay(200);
  Serial.println("OK");
}

void loop() {
  readSerialCommands();

  xAxis.run();
  yAxis.run();
  zAxis.run();

  static unsigned long lastStatusAt = 0;
  const unsigned long now = millis();
  if (now - lastStatusAt >= STATUS_PERIOD_MS) {
    lastStatusAt = now;
    publishStatus();
  }
}
