// PendTech  
// Anti sleep alarm for drivers
const int sensorPin = 2;     // Pin connected to the IR sensor (or eye detection sensor)
const int motorPin = 8;      // Pin connected to the motor
const int buzzerPin = 9;     // Pin connected to the buzzer
long time;  // Variable to store time
void setup() {
  pinMode(motorPin, OUTPUT);   // Set motorPin as an OUTPUT
    pinMode(buzzerPin, OUTPUT);  // Set buzzerPin as an OUTPUT
    pinMode(sensorPin, INPUT);   // Set sensorPin as an INPUT
    digitalWrite(motorPin, HIGH); // Turn on the motor initially
}
void loop() {
    // Check if the IR sensor (sensorPin) is triggered (LOW means it's triggered)
  if (!digitalRead(sensorPin)) {
        time = millis();  // Record the current time
        // Keep checking if the IR sensor is still triggered
    while (!digitalRead(sensorPin)) {
      // When the IR sensor is triggered, turn off the buzzer and turn on the motor and wait for 1 second
      digitalWrite(buzzerPin, LOW);
      digitalWrite(motorPin, HIGH);
      delay(1000);
    }
  } else {
    // If the IR sensor is not triggered (eyes open)

    // Check the time elapsed since the sensor was last triggered
    if (TimeDelay() >= 3) {
      digitalWrite(buzzerPin, HIGH);  // If 3 seconds have passed, turn on the buzzer
    }
    if (TimeDelay() >= 4) {
      digitalWrite(motorPin, LOW);   // If 4 seconds have passed, turn off the motor
    }
  }
}

int TimeDelay() {
  long t = millis() - time;  // Calculate the time elapsed since the IR sensor was triggered
  t = t / 1000;  // Convert milliseconds to seconds
  return t;  // Return the elapsed time in seconds
}
  
