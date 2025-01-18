const int potPin = A3;
const int motorPWM = 4;
const int motorDir1 = 1;
const int motorDir2 = 12;
const int tpsPin = A2;

int potValue = 0;
int tpsValue = 0;
int targetPosition = 0;
int currentPosition = 0;

unsigned long previousMillis = 0;
const unsigned long interval = 1.5; // Shorter interval

// setup the gains for the stabilization 
float kp = 7;
float ki = 0.02;
float kd = 0.05;

float integral = 0;
float previousError = 0;
float filteredDerivative = 0;

void setup() {
  pinMode(motorPWM, OUTPUT);
  pinMode(motorDir1, OUTPUT);
  pinMode(motorDir2, OUTPUT);
  pinMode(tpsPin, INPUT);
  Serial.begin(9600);
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    potValue = analogRead(potPin);
    targetPosition = map(potValue, 0, 1023, 0, 255);

    tpsValue = analogRead(tpsPin);
    currentPosition = map(tpsValue, 0, 1023, 0, 255);

    int error = targetPosition - currentPosition;

    if (abs(error) < 3) {
      error = 0;
      integral = 0;
    }

    integral += error * interval / 1000.0;
    integral = constrain(integral, 0, 255); 

    float derivative = (error - previousError) / (interval / 1000.0);
    filteredDerivative = 0.9 * filteredDerivative + 0.1 * derivative;

    float output = kp * error + ki * integral + kd * filteredDerivative;

    previousError = error;

    int motorSpeed = constrain(abs(output), 0, 255);

    if (error > 5) {
      digitalWrite(motorDir1, HIGH);
      digitalWrite(motorDir2, LOW);
    } else if (error < -5) {
      digitalWrite(motorDir1, LOW);
      digitalWrite(motorDir2, HIGH);
    } else {
      motorSpeed = 0;
      digitalWrite(motorDir1, LOW);
      digitalWrite(motorDir2, LOW);
    }
    analogWrite(motorPWM, motorSpeed);
  }
}




