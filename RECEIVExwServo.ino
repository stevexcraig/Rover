#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>

Servo steeringServo;

/*—–( Declare Constants and Pin Numbers )—–*/
#define CE_PIN 9
#define CSN_PIN 10

// NOTE: the "LL" at the end of the constant is "LongLong" type
const uint64_t pipe = 0xE8E8F0F0E1LL;  // Define the transmit pipe

/*—–( Declare objects )—–*/
RF24 radio(CE_PIN, CSN_PIN);  // Create a Radio

int joystickData[4];               // Array holding Joystick readings (only using [0] and [1])

// Motor pins
const int motor1Pin = 4;  // RIGHT MOTOR
const int motor2Pin = 2;  //
const int enablePin = 3;  //

const int motor1BPin = 7;  // LEFT MOTOR
const int motor2BPin = 8;  //
const int enableBPin = 6;  //

int motorSpeed = 0;
int motorSpeedB = 0;

// Motor speed limits
int potMax = 130;   //left
int potMaxB = 130;  //right
int potMin = 10;
int potMinB = 10;

// Steering constants
const int SERVO_CENTER = 90;
const int SERVO_LEFT_MAX = 60;
const int SERVO_RIGHT_MAX = 120;
const int JOYSTICK_DEADZONE = 50;

void setup() {
  Serial.begin(9600);
  Serial.println("Nrf24L01 Receiver Starting");
  
  radio.begin();
  radio.openReadingPipe(1, pipe);
  radio.startListening();

  // set all the other pins you're using as outputs:
  pinMode(motor1Pin, OUTPUT);
  pinMode(motor2Pin, OUTPUT);
  pinMode(enablePin, OUTPUT);

  // set all the other pins you're using as outputs:
  pinMode(motor1BPin, OUTPUT);
  pinMode(motor2BPin, OUTPUT);
  pinMode(enableBPin, OUTPUT);

  // set enablePin high so that motor can turn on:
  digitalWrite(enablePin, HIGH);
  digitalWrite(enableBPin, HIGH);

  steeringServo.attach(5);
  steeringServo.write(SERVO_CENTER);
}

void loop() {
  if (radio.available()) {
    // Read the data payload
    radio.read(joystickData, sizeof(joystickData));

    // Handle forward/backward movement (Y-axis) - joystickData[0]
    handleMotorControl(joystickData[0]);
    
    // Handle steering (X-axis) - joystickData[1]
    handleSteering(joystickData[1]);
    
    // Debug output - only show the values we're using
    Serial.print("Forward/Back: ");
    Serial.print(joystickData[0]);
    Serial.print(" | Left/Right: ");
    Serial.println(joystickData[1]);
    
    delay(20);
  }
}

void handleMotorControl(int yValue) {
  if (yValue > 564) {
    // Forward movement
    digitalWrite(motor1Pin, HIGH);  // Left Forward
    digitalWrite(motor2Pin, LOW);   //

    motorSpeed = map(yValue, 564, 1023, potMin, potMax);
    analogWrite(enablePin, motorSpeed);

    digitalWrite(motor1BPin, HIGH);  // Right Forward
    digitalWrite(motor2BPin, LOW);   //

    motorSpeedB = map(yValue, 564, 1023, potMinB, potMaxB);
    analogWrite(enableBPin, motorSpeedB);
  }
  else if (yValue < 460) {
    // Backward movement
    digitalWrite(motor1Pin, LOW);   // Left Backward
    digitalWrite(motor2Pin, HIGH);  //

    motorSpeed = map(yValue, 460, 0, potMin, potMax);
    analogWrite(enablePin, motorSpeed);

    digitalWrite(motor1BPin, LOW);   // Right Backward
    digitalWrite(motor2BPin, HIGH);  //

    motorSpeedB = map(yValue, 460, 0, potMinB, potMaxB);
    analogWrite(enableBPin, motorSpeedB);
  }
  else {
    // Stop motors (deadzone)
    digitalWrite(motor1Pin, LOW);
    digitalWrite(motor2Pin, LOW);
    digitalWrite(motor1BPin, LOW);
    digitalWrite(motor2BPin, LOW);
  }
}

void handleSteering(int xValue) {
  int servoPosition;
  
  // Apply deadzone around center
  if (abs(xValue - 512) < JOYSTICK_DEADZONE) {
    servoPosition = SERVO_CENTER;
  }
  else {
    // Map joystick X value to servo position
    servoPosition = map(xValue, 0, 1023, SERVO_LEFT_MAX, SERVO_RIGHT_MAX);
    
    // Constrain to valid servo range
    servoPosition = constrain(servoPosition, SERVO_LEFT_MAX, SERVO_RIGHT_MAX);
  }
  
  steeringServo.write(servoPosition);
}
