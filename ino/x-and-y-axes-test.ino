/* Example sketch to control a stepper motor with 
   A4988/DRV8825 stepper motor driver and 
   Arduino without a library. 
   More info: https://www.makerguides.com */

// Define stepper motor connections and steps per revolution:
#define dirPin 2
#define stepPin 3
#define stepsPerRevolution 200

#define x_stepPin 5
#define x_dirPin 6
#define x_enblPin 7


void setup() {
  // Declare y-axis pins as output:
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  // Declare x-axis pins as output
  pinMode(x_stepPin, OUTPUT);
  pinMode(x_dirPin, OUTPUT);
  pinMode(x_enblPin, OUTPUT);

  digitalWrite(x_enblPin, HIGH);
}


void loop() {
  // =================================================
  // START X-AXIS
  // =================================================
  // Set the spinning direction clockwise:
  digitalWrite(x_dirPin, HIGH);

  // Spin the stepper motor 5 revolutions fast:
  for (int i = 0; i < 100 * stepsPerRevolution; i++) {
    // These four lines result in 1 step:
    digitalWrite(x_stepPin, HIGH);
    delayMicroseconds(100);
    digitalWrite(x_stepPin, LOW);
    delayMicroseconds(100);
  }

  delay(1000);

  // Set the spinning direction counterclockwise:
  digitalWrite(x_dirPin, LOW);

  //Spin the stepper motor 5 revolutions fast:
  for (int i = 0; i < 100 * stepsPerRevolution; i++) {
    // These four lines result in 1 step:
    digitalWrite(x_stepPin, HIGH);
    delayMicroseconds(100);
    digitalWrite(x_stepPin, LOW);
    delayMicroseconds(100);
  }

  delay(1000);

  // =================================================
  // START Y-AXIS
  // =================================================
  // Set the spinning direction clockwise:
  digitalWrite(dirPin, HIGH);

  // Spin the stepper motor 5 revolutions fast:
  for (int i = 0; i < 5 * stepsPerRevolution; i++) {
    // These four lines result in 1 step:
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
  }

  delay(1000);

  // Set the spinning direction counterclockwise:
  digitalWrite(dirPin, LOW);

  //Spin the stepper motor 5 revolutions fast:
  for (int i = 0; i < 5 * stepsPerRevolution; i++) {
    // These four lines result in 1 step:
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
  }

  delay(1000);
}