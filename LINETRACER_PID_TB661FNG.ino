// PID ni Jyags (TB6612FNG version - SMOOTH)

#define PWMA 5
#define AIN1 7
#define AIN2 8

#define PWMB 6
#define BIN1 9
#define BIN2 10

#define STBY 4

// Sensors
int sensorPins[5] = {A1, A2, A3, A4, A5};

float Kp = 60;
float Kd = 45;
float Ki = 0;

int baseSpeed = 200;
int maxSpeed = 255;

int error = 0;
int lastError = 0;

unsigned long lastLineTime = 0;
const int lineLostDelay = 120;

// ---------- Motor control helpers ----------

void setMotorA(int speed, bool forward) {
  digitalWrite(AIN1, forward ? HIGH : LOW);
  digitalWrite(AIN2, forward ? LOW : HIGH);
  analogWrite(PWMA, speed);
}

void setMotorB(int speed, bool forward) {
  digitalWrite(BIN1, forward ? HIGH : LOW);
  digitalWrite(BIN2, forward ? LOW : HIGH);
  analogWrite(PWMB, speed);
}

void stopMotors() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}

// ---------- Line reading ----------

int readLine(bool &lineDetected) {
  int weights[5] = {-3, -1, 0, 1, 3};
  int sum = 0;
  int count = 0;

  for (int i = 0; i < 5; i++) {
    int val = analogRead(sensorPins[i]);

    if (val < 500) {
      sum += weights[i];
      count++;
    }
  }

  if (count == 0) {
    lineDetected = false;
    return (lastError >= 0) ? 3 : -3;
  }

  lineDetected = true;
  return sum / count;
}

// ---------- Setup ----------

void setup() {
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  pinMode(STBY, OUTPUT);

  digitalWrite(STBY, HIGH);

  delay(800);
}

// ---------- Loop ----------

void loop() {

  int s0 = analogRead(A1);
  int s1 = analogRead(A2);
  int s2 = analogRead(A3);
  int s3 = analogRead(A4);
  int s4 = analogRead(A5);

  // sharp left
  if (s0 < 500 && s1 < 500) {
    setMotorA(255, false);
    setMotorB(255, true);
    lastError = -3;
    return;
  }

  // sharp right
  if (s4 < 500 && s3 < 500) {
    setMotorA(255, true);
    setMotorB(255, false);
    lastError = 3;
    return;
  }

  bool lineDetected;
  error = readLine(lineDetected);

  if (lineDetected) {
    lastLineTime = millis();
  }

  // line lost recovery
  if (millis() - lastLineTime > lineLostDelay) {

    if (lastError > 0) {
      setMotorA(255, true);
      setMotorB(200, false);
    } else {
      setMotorA(200, false);
      setMotorB(255, true);
    }

    return;
  }

  // -------- SMOOTH PID (no clamp) --------
  int correction = (Kp * error) + (Kd * (error - lastError));

  int leftSpeed = baseSpeed + correction;
  int rightSpeed = baseSpeed - correction;

  leftSpeed = constrain(leftSpeed, 0, maxSpeed);
  rightSpeed = constrain(rightSpeed, 0, maxSpeed);

  setMotorA(leftSpeed, true);
  setMotorB(rightSpeed, true);

  lastError = error;
}