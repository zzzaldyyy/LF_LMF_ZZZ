// ===========================
// KONFIGURASI SENSOR CD4051
// ===========================
const int S0 = 2;
const int S1 = 3;
const int S2 = 4;
const int muxOut = A0;
const int numSensors = 8;

const int physicalOrder[8] = {3, 0, 1, 2, 4, 6, 7, 5};
int threshold[8] = {700, 700, 700, 700, 700, 700, 700, 700};
byte sensor_values[numSensors];
bool isLineDetected[numSensors];

bool finish = false;

// ===========================
// KONFIGURASI MOTOR
// ===========================
#define MRF 6   // KANAN MAJU
#define MRB 5   // KANAN MUNDUR
#define MLF 10  // KIRI MAJU
#define MLB 9   // KIRI MUNDUR

int BaseSpeedL = 100;
int BaseSpeedR = 110;
int kp = 20, ki = 0, kd = 5;
int lastError = 0;
float integral = 0;

// ===========================
// KONTROL WAKTU
// ===========================
unsigned long lastMoveTime = 0;
unsigned long moveInterval = 10;

void setup() {
  Serial.begin(9600);
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(muxOut, INPUT);

  pinMode(MRF, OUTPUT);
  pinMode(MRB, OUTPUT);
  pinMode(MLF, OUTPUT);
  pinMode(MLB, OUTPUT);
}

byte readSensor() {
  byte dataSensor = 0;
  for (int i = 0; i < 8; i++) {
    int sensorIndex = physicalOrder[i];
    digitalWrite(S0, bitRead(sensorIndex, 0));
    digitalWrite(S1, bitRead(sensorIndex, 1));
    digitalWrite(S2, bitRead(sensorIndex, 2));
    delayMicroseconds(20);
    int val = analogRead(A0);

    if (val < threshold[sensorIndex]) {
      dataSensor |= (0x80 >> i);
    }
  }
  for (int i = 0; i < 8; i++) {
    sensor_values[i] = (dataSensor >> (7 - i)) & 0x01;
  }
  return dataSensor;
}

int calcError(byte sensorData) {
  int weights[8] = {-4, -2, -1, 0, 0, 1, 2, 4};
  int sum = 0, count = 0;
  for (int i = 0; i < 8; i++) {
    if (sensorData & (0x80 >> i)) {
      sum += weights[i];
      count++;
    }
  }
  if (count == 0) return 0;
  return sum / count;
}

void followLine(byte s) {

  int error = calcError(s);

  integral += error;
  int derivative = error - lastError;
  lastError = error;

  float correction = kp * error + ki * integral + kd * derivative;

  int leftSpeed = constrain(BaseSpeedL + correction, 0, 255);
  int rightSpeed = constrain(BaseSpeedR - correction, 0, 255);

  setMotorSpeed(leftSpeed, rightSpeed);

}

byte readSensorInverted() {
  byte dataSensor = 0;
  for (int i = 0; i < 8; i++) {
    int sensorIndex = physicalOrder[i];
    digitalWrite(S0, bitRead(sensorIndex, 0));
    digitalWrite(S1, bitRead(sensorIndex, 1));
    digitalWrite(S2, bitRead(sensorIndex, 2));
    delayMicroseconds(20);
    int val = analogRead(A0);

    if (val > threshold[sensorIndex]) {
      dataSensor |= (0x80 >> i);
    }
  }
  for (int i = 0; i < 8; i++) {
    sensor_values[i] = (dataSensor >> (7 - i)) & 0x01;
  }
  return dataSensor;
}

void followLineInverted(byte s) {

  int error = calcError(s);

  integral += error;
  int derivative = error - lastError;
  lastError = error;

  float correction = kp * error + ki * integral + kd * derivative;

  int leftSpeed = constrain(BaseSpeedL + correction, 0, 255);
  int rightSpeed = constrain(BaseSpeedR - correction, 0, 255);

  setMotorSpeed(leftSpeed, rightSpeed);

}

void setMotorSpeed(int leftSpeed, int rightSpeed) {
  if(leftSpeed >= 0) {
    analogWrite(MLF, leftSpeed);
    analogWrite(MLB, 0);
  } else {
    analogWrite(MLF, 0);
    analogWrite(MLB, -leftSpeed);
  }

  if(rightSpeed >= 0) {
    analogWrite(MRF, rightSpeed);
    analogWrite(MRB, 0);
  } else {
    analogWrite(MRF, 0);
    analogWrite(MRB, -rightSpeed);
  }
}

void stopMotor() {
  setMotorSpeed(0, 0);
}

void turnLeftUntilLine() {
  setMotorSpeed(-90, 90);
  while (true) {
    byte s = readSensor();
    if (s & 0b00011100) break;
  }
}

void turnRightUntilLine() {
  setMotorSpeed(90, -90);
  while (true) {
    byte s = readSensor();
    if (s & 0b00111000) break;
  }
}

void moveForwardUntilTime(unsigned long duration) {
  unsigned long startTime = millis();
  setMotorSpeed(BaseSpeedL, BaseSpeedR);
  while (millis() - startTime < duration) {
    byte s = readSensor();  // Tetap bisa deteksi perubahan sensor
    // Optional: bisa keluar lebih awal kalau muncul garis
  }
}

void stopMotorUntilTime(unsigned long duration) {
  unsigned long startTime = millis();
  setMotorSpeed(0, 0);
  while (millis() - startTime < duration) {
    byte s = readSensor();  // Tetap bisa deteksi perubahan sensor
    // Optional: bisa keluar lebih awal kalau muncul garis
  }
}

void loop() {
  while(!finish) {
    byte s = readSensor();
    byte si = readSensorInverted();
    if (s == 0b00001111 || s == 0b00011111 || s == 0b11110000 || s == 0b11111000
    ){
      moveForwardUntilTime(500);
      // stopMotorUntilTime(300);
    }
    if (s == 0b11100111 || s == 0b11101111 || s == 0b11110111 ||
        s == 0b11001111 || s == 0b11011111 || s == 0b11110011 || 
        s == 0b11111011 || s == 0b10011111 || s == 0b10111111 ||
        s == 0b11111001 || s == 0b11111101){
      followLine(si);
    } else {
      followLine(s);
    }
  }
}