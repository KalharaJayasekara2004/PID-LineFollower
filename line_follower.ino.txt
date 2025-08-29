#include <BeeLineSensorPro.h>

// Motor M1 (Left)
int enA = 10;
int in1 = 11;
int in2 = 9;

// Motor M2 (Right)
int enB = 5;
int in3 = 7;
int in4 = 6;

// Line Sensor Configuration
BeeLineSensorPro sensor = BeeLineSensorPro((unsigned char[]) {
  A0, A1, A2, A3, A4, A5, A6, A7
}, LINE_BLACK);

// PID Constants
float kp = 0.12;    // Proportional Gain (Adjust for correction strength)
float ki = 0.002;  // Integral Gain (Small value to avoid overshooting)
float kd = 0.15;   // Derivative Gain (Adjust for smoothness)

// PID Variables
float integral = 0; 
int last_value = 0;
const float max_integral = 1000; // Prevent Integral Windup

// Timer for Discontinuity Handling
unsigned long lastSeenTime = 0;
const int maxSearchTime = 2000; // Max search time (2s)

void mdrive(int m1, int m2);  // Function prototype

void setup() {
  // Motor pins
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
  delay(1000);

  // Sensor Calibration
  for (int i = 0; i < 100; i++) {
    sensor.calibrate();
    mdrive(-220, 220);  // Rotate left
  }
  mdrive(0, 0);
  delay(100);

  for (int i = 0; i < 100; i++) {
    sensor.calibrate();
    mdrive(220, -220);  // Rotate right
  }
  mdrive(0, 0);
  delay(1000);

  lastSeenTime = millis(); // Start the timer
}

void loop() {
  int baseSpeed = 170;
  int err = sensor.readSensor();
  
  if (err != 0) {
    // PID CONTROL
    integral += err; // Accumulate integral error
    integral = constrain(integral, -max_integral, max_integral); // Prevent Windup

    int diff = err * kp + integral * ki + (err - last_value) * kd;
    last_value = err;

    mdrive(baseSpeed + diff, baseSpeed - diff);
    lastSeenTime = millis(); // Reset the timer when line is found
  } 
  else {
    // HANDLE DISCONTINUOUS PATH
    unsigned long currentTime = millis();
    if (currentTime - lastSeenTime < 1000) {
      // Small Gap → Keep moving forward
      mdrive(baseSpeed, baseSpeed);
    } 
    else if (currentTime - lastSeenTime < maxSearchTime) {
      // Search Mode → Turn Left-Right
      if ((currentTime / 500) % 2 == 0) {
        mdrive(-100, 100); // Turn Left
      } else {
        mdrive(100, -100); // Turn Right
      }
    }
  }
}

// Motor Control Function
void mdrive(int m1, int m2) {
  // Left Motor
  if (m1 > 0) {
    m1 = constrain(m1, 0, 255);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enA, m1);
  } else {
    m1 = constrain(m1, -255, 0);
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enA, -m1);
  }

  // Right Motor
  if (m2 > 0) {
    m2 = constrain(m2, 0, 255);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enB, m2);
  } else {
    m2 = constrain(m2, -255, 0);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enB, -m2);
  }
}
