// Include the required libraries
#include <PID_v1.h>

// Define the pin connections for L293D motor driver
int motorPin1 = 2;    // Pin 2 connected to L293D IN1
int motorPin2 = 3;    // Pin 3 connected to L293D IN2
int enablePin = 9;    // Pin 9 connected to L293D Enable1

// Define the analog input pin for the potentiometer
int potPin = A0;

// Define the PID parameters
double Setpoint, Input, Output;
double Kp = 1.0, Ki = 0.0, Kd = 0.0;    // PID constants

// Create the PID instance
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  // Set the motor control pins as output
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(enablePin, OUTPUT);
  
  // Initialize the serial communication
  Serial.begin(9600);
  
  // Set the initial PID parameters
  Setpoint = 0;  // Set the desired speed
  
  // Set the input range of the potentiometer
  myPID.SetOutputLimits(0, 255);  // Set the output range for motor speed
  
  // Set the tuning parameters for the PID controller
  myPID.SetMode(AUTOMATIC);  // Set the controller to automatically adjust
  myPID.SetSampleTime(10);   // Set the time interval for PID computation
}

void loop() {
  // Read the potentiometer value
  int potValue = analogRead(potPin);
  
  // Map the potentiometer value to the desired speed range
  Setpoint = map(potValue, 0, 1023, 0, 255);
  
  // Read the actual speed of the motor
  Input = analogRead(A1);
  
  // Compute the PID control output
  myPID.Compute();
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);
  
  // Set the motor speed based on the PID output
  analogWrite(enablePin, Output);
  //analogWrite(enablePin, abs(Output));
  
  // Print the values for debugging
  Serial.print("Setpoint: ");
  Serial.print(Setpoint);
  Serial.print(", Input: ");
  Serial.print(Input);
  Serial.print(", Output: ");
  Serial.println(Output);
  
  delay(10);
}
