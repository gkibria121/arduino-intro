#include <Servo.h>

const int M1IN[2] = {2, 3};  
const int M2IN[2] = {6, 7}; 
const int M3IN[2] = {4, 5}; 
const int M4IN[2] = {8, 9};  
const int* MT[4] = {M1IN, M2IN, M3IN, M4IN}; // Array of pointers to motor pin arrays

const int leftIRSensorPin = 13;
const int rightIRSensorPin = 12;
const int ultrasonicTriggerPin = A2;
const int ultrasonicEchoPin = A1;
const int servoPin = A3;   

// Constants for better readability
const int MIN_DISTANCE_CM = 50;  // Minimum safe distance in cm
const int TURN_DELAY_MS = 300;   // Delay for turns in milliseconds
const int SERVO_PULSE_MS = 500; // Servo pulse time in milliseconds
const int LOOP_DELAY_MS = 100;   // Main loop delay in milliseconds
bool servoInitialized = false;  // Flag to track if servo has been initialized

Servo robotHead;

// Function to measure distance using ultrasonic sensor
long measureDistance() {
  // Clear the trigger pin
  digitalWrite(ultrasonicTriggerPin, LOW);
  delayMicroseconds(2);
  
  // Set trigger pin high for 10 microseconds
  digitalWrite(ultrasonicTriggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(ultrasonicTriggerPin, LOW);
  
  // Read the echo pin - returns the sound wave travel time in microseconds
  long duration = pulseIn(ultrasonicEchoPin, HIGH);
  
  // Calculate the distance in centimeters
  // Speed of sound is approximately 343 meters/second or 0.0343 cm/microsecond
  // Distance = (Time x Speed) / 2 (round trip)
  long distance = (duration * 0.0343) / 2;
  
  return distance;
}

void setup() {
    // Initialize serial communication
    Serial.begin(9600);
    
    // Set motor pins as outputs
    for (int i = 0; i < 4; i++) {
        pinMode(MT[i][0], OUTPUT);
        pinMode(MT[i][1], OUTPUT);
    }
    
    // Set sensor pins with correct modes
    pinMode(leftIRSensorPin, INPUT);
    pinMode(rightIRSensorPin, INPUT);
    pinMode(ultrasonicTriggerPin, OUTPUT); 
    pinMode(ultrasonicEchoPin, INPUT);
    pinMode(servoPin, OUTPUT); 
    
    // Wait for serial to connect
    delay(1000);
    Serial.println("Human-Following Robot Initialized");
    
    // Initialize servo by rotating it to starting position
    initializeServo();
}

// Function to properly initialize the servo at startup
void initializeServo() {
    Serial.println("Rotating servo...");
    robotHead.attach(servoPin);
    // Sequence of pulses to ensure servo initializes properly
    for (int i = 90; i < 180; i++) {
       robotHead.write(i);
       delay(15);
    }
    for (int i = 180; i > 0; i--) {
       robotHead.write(i);
       delay(15);
    }
    for (int i = 0; i > 90; i--) {
       robotHead.write(i);
       delay(15);
    }
    
    
    Serial.println("Servo initialization complete");
    servoInitialized = true;
}

void loop() {
    // Read digital values from IR sensors
    int leftIRDetected = digitalRead(leftIRSensorPin);
    int rightIRDetected = digitalRead(rightIRSensorPin); 
    
    // Measure distance with ultrasonic sensor
    long distance = measureDistance();
    
    // Debug sensor readings
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
    
    // Obstacle avoidance has highest priority
    if (distance < MIN_DISTANCE_CM) {
        Serial.println("Obstacle detected! Avoiding...");
        stopAll();
        delay(500);
        moveBackward();
        delay(500);
        turnRight();
        delay(700);
    }
    // Human following logic based on IR sensors
    else if (leftIRDetected && rightIRDetected) {
        // Human detected by both sensors - move forward
        Serial.println("Human detected ahead - Following!");
        moveForward();
         
    }
    else if (leftIRDetected) {
        // Human detected on left side only - turn left
        Serial.println("Human detected on left - Turning left");
        turnLeft();
    }
    else if (rightIRDetected) {
        // Human detected on right side only - turn right
        Serial.println("Human detected on right - Turning right");
        turnRight();
    }
    else {
        // No human detected - stop and wait
        Serial.println("No human detected - Stopping");
        stopAll();
    }
    
    delay(LOOP_DELAY_MS); // Short delay between readings
}
 

// Motor control functions
void motorStop(const int motorPins[2]) { 
    digitalWrite(motorPins[0], LOW);
    digitalWrite(motorPins[1], LOW); 
}

void motorGoForward(const int motorPins[2]) {
    digitalWrite(motorPins[0], LOW);
    digitalWrite(motorPins[1], HIGH);
}

void motorGoBackward(const int motorPins[2]) {
    digitalWrite(motorPins[0], HIGH);
    digitalWrite(motorPins[1], LOW);
}

// Robot movement functions
void stopAll() {
    for (int i = 0; i < 4; i++) {
        motorStop(MT[i]);
    }
}

void moveForward() {
    for (int i = 0; i < 4; i++) {
        motorGoForward(MT[i]);
    }
}

void moveBackward() {
    for (int i = 0; i < 4; i++) {
        motorGoBackward(MT[i]);
    }
}

void turnLeft() {
    // Left side motors (M1, M3) go backward
    motorGoBackward(MT[0]); // M1
    motorGoBackward(MT[2]); // M3
    
    // Right side motors (M2, M4) go forward
    motorGoForward(MT[1]); // M2
    motorGoForward(MT[3]); // M4
}

void turnRight() {
    // Left side motors (M1, M3) go forward
    motorGoForward(MT[0]); // M1
    motorGoForward(MT[2]); // M3
    
    // Right side motors (M2, M4) go backward
    motorGoBackward(MT[1]); // M2
    motorGoBackward(MT[3]); // M4
}

// Kept for completeness but not used in current logic
void rotateClockwise() {
    turnRight();
}

void rotateCounterClockwise() {
    turnLeft();
}