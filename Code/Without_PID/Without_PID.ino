// Define the pins for motor control
int motorPin1 = 2;
int motorPin2 = 3;
int enablePin = 9;

// Define the potentiometer pin
int potPin = A0;

void setup() {
  // Set the motor control pins as outputs
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(enablePin, OUTPUT);

  // Initialize the potentiometer pin
  pinMode(potPin, INPUT);
  
  // Set the serial baud rate
  Serial.begin(9600);
}

void loop() {
  // Read the potentiometer value
  int potValue = analogRead(potPin);

  // Map the potentiometer value to the motor speed range
  int motorSpeed = map(potValue, 0, 1023, 0, 255);

  // Set the motor direction
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);

  // Set the motor speed
  analogWrite(enablePin, motorSpeed);

  // Print the motor speed on the serial monitor
  Serial.print("Motor Speed: ");
  Serial.println(motorSpeed);

  delay(100);
}
