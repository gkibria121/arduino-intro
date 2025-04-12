#include <Servo.h>

// Motor pin definitions (IN1 and IN2 for each motor)
const int M1IN[2] = {9, 8};  
const int M2IN[2] = {7, 6}; 
const int M3IN[2] = {2, 3}; 
const int M4IN[2] = {5, 4};  

// Array of pointers to each motor pin pair for convenient iteration
const int* MT[4] = {M1IN, M2IN, M3IN, M4IN}; 

// Sensor pins
const int leftIRSensorPin = 13;
const int rightIRSensorPin = 12;
const int ultrasonicTriggerPin = 10;
const int ultrasonicEchoPin = 11;

// Servo pin (used for robot head movement)
const int servoPin = A3;   

// Constants for robot behavior
const int MIN_DISTANCE_CM = 5;   // Minimum safe distance from obstacle (cm)
const int MAX_DISTANCE_CM = 100;    // Maximum safe distance from obstacle (cm)
const int TURN_DELAY_MS = 300;      // Time to turn in milliseconds
const int SERVO_PULSE_MS = 500;     // Time to move servo during initialization
const int LOOP_DELAY_MS = 100;      // Delay between each loop iteration
bool servoInitialized = false;      // Flag for tracking servo initialization

// Servo instance for robot head movement
Servo robotHead;

/**
 * Measures the distance using the ultrasonic sensor.
 * @return Distance in centimeters.
 */
long measureDistance() {
  // Clear the trigger pin
  digitalWrite(ultrasonicTriggerPin, LOW);
  delayMicroseconds(2);

  // Trigger the ultrasonic pulse
  digitalWrite(ultrasonicTriggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(ultrasonicTriggerPin, LOW);

  // Measure pulse duration from echo pin
  long duration = pulseIn(ultrasonicEchoPin, HIGH);

  // Convert duration to distance (cm)
  long distance = (duration * 0.0343) / 2;

  return distance;
}

/**
 * Arduino setup function. Initializes serial, sensors, motor pins, and servo.
 */
void setup() {
  Serial.begin(9600);

  // Initialize motor pins as outputs
  for (int i = 0; i < 4; i++) {
    pinMode(MT[i][0], OUTPUT);
    pinMode(MT[i][1], OUTPUT);
  }

  // Set up IR and ultrasonic sensor pins
  pinMode(leftIRSensorPin, INPUT);
  pinMode(rightIRSensorPin, INPUT);
  pinMode(ultrasonicTriggerPin, OUTPUT); 
  pinMode(ultrasonicEchoPin, INPUT);

  // Set servo pin (analog pin for PWM output)
  pinMode(servoPin, OUTPUT); 

  delay(1000);  // Allow time for everything to settle
  Serial.println("Human-Following Robot Initialized");

  // Optional: Uncomment to run servo initialization at startup
  // initializeServo();
}

/**
 * Initializes the servo by sweeping it through a range of motion.
 */
void initializeServo() {
  Serial.println("Rotating servo...");
  robotHead.attach(servoPin);

  for (int i = 90; i < 180; i++) {
    robotHead.write(i);
    delay(15);
  }
  for (int i = 180; i > 0; i--) {
    robotHead.write(i);
    delay(15);
  }
  for (int i = 0; i <= 90; i++) {
    robotHead.write(i);
    delay(15);
  }

  Serial.println("Servo initialization complete");
  servoInitialized = true;
}

/**
 * Main loop. Reads sensor input and controls motors accordingly.
 */
void loop() {
  // Read IR sensor data (active low)
  int leftIRDetected = !digitalRead(leftIRSensorPin);
  int rightIRDetected = !digitalRead(rightIRSensorPin);

  // Measure distance from ultrasonic sensor
  long distance = measureDistance();

  // Display current distance
  Serial.print("Distance: ");
  Serial.println(distance);
  if ((leftIRDetected && rightIRDetected) || (distance < MIN_DISTANCE_CM) ) {
 
     Serial.println("Obstacle detected! Avoiding...");
    stopAll();
  } 
  // Priority 3: Turn left if human is on left side
 else  if (leftIRDetected) {
    Serial.println("Human detected on left - Turning left");
    turnLeft();
  }
  // Priority 4: Turn right if human is on right side
  else if (rightIRDetected) {
    Serial.println("Human detected on right - Turning right");
    turnRight();
  }
  // Priority 1: Avoid obstacles
 else if (distance > MAX_DISTANCE_CM) {
     Serial.println("No human detected - stopping");
    stopAll();
    // Optional avoidance routine:
    // delay(500);
    // moveBackward();
    // delay(500);
    // turnRight();
    // delay(700);
  } 
  // Default behavior: follow human
  else {
    Serial.println("No Obstacle detected - follow human!");
    moveForward();
  }

  // Optional delay between iterations
  // delay(LOOP_DELAY_MS);
}

///////////////////////////
// MOTOR CONTROL SECTION //
///////////////////////////

/**
 * Stops a single motor.
 * @param motorPins The 2-pin array for the motor.
 */
void motorStop(const int motorPins[2]) {
  digitalWrite(motorPins[0], LOW);
  digitalWrite(motorPins[1], LOW); 
}

/**
 * Drives a motor forward.
 * @param motorPins The 2-pin array for the motor.
 */
void motorGoForward(const int motorPins[2]) {
  digitalWrite(motorPins[0], LOW);
  digitalWrite(motorPins[1], HIGH);
}

/**
 * Drives a motor backward.
 * @param motorPins The 2-pin array for the motor.
 */
void motorGoBackward(const int motorPins[2]) {
  digitalWrite(motorPins[0], HIGH);
  digitalWrite(motorPins[1], LOW);
}

//////////////////////////
// MOVEMENT FUNCTIONS   //
//////////////////////////

/**
 * Stops all motors.
 */
void stopAll() {
  for (int i = 0; i < 4; i++) {
    motorStop(MT[i]);
  }
}

/**
 * Moves the robot forward.
 */
void moveForward() {
  for (int i = 0; i < 4; i++) {
    motorGoForward(MT[i]);
  }
}

/**
 * Moves the robot backward.
 */
void moveBackward() {
  for (int i = 0; i < 4; i++) {
    motorGoBackward(MT[i]);
  }
}

/**
 * Turns the robot left.
 */
void turnLeft() {
  motorGoBackward(MT[0]); // M1
  motorGoBackward(MT[2]); // M3
  motorGoForward(MT[1]);  // M2
  motorGoForward(MT[3]);  // M4
}

/**
 * Turns the robot right.
 */
void turnRight() {
  motorGoForward(MT[0]); // M1
  motorGoForward(MT[2]); // M3
  motorGoBackward(MT[1]); // M2
  motorGoBackward(MT[3]); // M4
}

/**
 * Rotates the robot clockwise (alias for right turn).
 */
void rotateClockwise() {
  turnRight();
}

/**
 * Rotates the robot counter-clockwise (alias for left turn).
 */
void rotateCounterClockwise() {
  turnLeft();
}
