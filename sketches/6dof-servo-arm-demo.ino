#include <Servo.h>

// Define servo pins
const int SERVO_PINS[] = {3, 5, 6, 9, 10, 11};
const int NUM_SERVOS = 6;

// Create servo objects
Servo servos[NUM_SERVOS];

// Current and target angles for each servo
int currentAngles[NUM_SERVOS];
int targetAngles[NUM_SERVOS];

// Movement parameters
const int STEP_DELAY = 15;  // Delay between steps in ms
const int ANGLE_STEP = 1;   // Degrees to move per step

void setup() {
  Serial.begin(9600);
  
  // Initialize all servos
  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].attach(SERVO_PINS[i]);
    currentAngles[i] = 90;  // Start at middle position
    servos[i].write(currentAngles[i]);
  }
}

void moveToAngles(int newAngles[]) {
  // Set target angles
  for (int i = 0; i < NUM_SERVOS; i++) {
    targetAngles[i] = constrain(newAngles[i], 0, 180);
  }
  
  // Move all servos in parallel until they reach their targets
  bool moving = true;
  while (moving) {
    moving = false;
    
    for (int i = 0; i < NUM_SERVOS; i++) {
      if (currentAngles[i] != targetAngles[i]) {
        moving = true;
        
        if (currentAngles[i] < targetAngles[i]) {
          currentAngles[i] += ANGLE_STEP;
        } else {
          currentAngles[i] -= ANGLE_STEP;
        }
        
        servos[i].write(currentAngles[i]);
      }
    }
    
    delay(STEP_DELAY);
  }
}

void loop() {
  // Example usage - move to specific angles
  if (Serial.available() > 0) {
    int newAngles[NUM_SERVOS];
    
    // Read 6 comma-separated angles from Serial
    for (int i = 0; i < NUM_SERVOS; i++) {
      newAngles[i] = Serial.parseInt();
      if (i < NUM_SERVOS - 1) {
        char comma = Serial.read(); // Skip comma
      }
    }
    
    moveToAngles(newAngles);
  }
}