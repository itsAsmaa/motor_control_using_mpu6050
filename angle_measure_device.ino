#include <Wire.h>
#include <LiquidCrystal.h>
#include <MPU6050.h>

// Initialize the LCD
LiquidCrystal lcd(13, 12, 11, 10, 9, 8);

// Initialize the MPU6050
MPU6050 mpu;

// Pin definitions
const byte motorPin1 = 6;
const byte motorPin2 = 5;
const byte sensorPin1 = 2;
const byte sensorPin2 = 3;

const int threshold = 512;

// Variables
byte motorDirection = 0;
float targetPosition = 0, newTargetPosition = 0;
volatile long sensorCount1 = 0;
float currentPitch = 0;
float encoderPosition = 0;

// Function declarations
void sensorInterrupt1();
void readMPU();
void startMotors();
void stopMotors();
void controlMotorPosition();

void setup() {
    // Initialize the LCD
    lcd.begin(16, 2);
    lcd.setCursor(0, 0);
    lcd.print("MPU=");
    
    Serial.begin(115200);
    
    // Initialize MPU6050
    Wire.begin();
    mpu.initialize();
    
    // Configure sensor pins as input
    pinMode(sensorPin1, INPUT_PULLUP);
    pinMode(sensorPin2, INPUT_PULLUP);
    pinMode(motorPin1, OUTPUT);
    pinMode(motorPin2, OUTPUT);
    
    // Attach interrupts for encoder sensor
    attachInterrupt(digitalPinToInterrupt(sensorPin1), sensorInterrupt1, CHANGE);
}

void loop() { 
  readMPU();
  controlMotorPosition();
  
  // Print values to the Serial Monitor for debugging
  Serial.print("Encoder Position: ");
  Serial.println(encoderPosition);
  Serial.print("MPU Pitch: ");
  Serial.println(currentPitch);
  
  delay(100);  // Small delay to reduce serial printing rate
}

void sensorInterrupt1() {
    sensorCount1++;
    encoderPosition = sensorCount1 /4.5 ;  //  4.5 counts per degree 
}

// Function to read MPU6050 accelerometer data
void readMPU() {
    // Read accelerometer data
    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az);
    
    // Calculate pitch from accelerometer data (simplified)
    currentPitch = atan2(ay, az) * 180.0 / PI;
    
    // Update target position based on MPU pitch
    if (currentPitch < 0) currentPitch = 0; // Limit pitch to 0
    if (currentPitch > 90) currentPitch = 90; // Limit pitch to 90
    
    newTargetPosition = currentPitch;
    
    lcd.setCursor(0, 1);
    lcd.print("Pitch: ");
    lcd.print(currentPitch);
    lcd.print(" ");
}

void controlMotorPosition() {
    // Calculate the error between the current encoder position and target position
    float error = newTargetPosition - encoderPosition;
    
    if (abs(error) > 0.5) {  // Only correct if the error is large enough
        if (error > 0) {
            motorDirection = 1;  // Move forward
            startMotors();
        } else {
            motorDirection = 0;  // Move backward
            startMotors();
        }
    } else {
        stopMotors();  // If the error is small, stop the motor
    }
}

void startMotors() {
    if (motorDirection == 0) {
        analogWrite(motorPin1, 125);  // Forward direction
        analogWrite(motorPin2, 0);
    } else {
        analogWrite(motorPin1, 0);    // Reverse direction
        analogWrite(motorPin2, 125);
    }
}

void stopMotors() {
    analogWrite(motorPin1, 0);
    analogWrite(motorPin2, 0);
}
