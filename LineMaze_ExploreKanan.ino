// ===========================
// KONFIGURASI OLED DAN TOMBOL
// ===========================
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define BTN_UP     12 //ATAS
#define BTN_DOWN   8  //BAWAH
#define BTN_LEFT   11 //KIRI
#define BTN_RIGHT  7  //KANAN
const int debounceDelay = 200;
unsigned long lastButtonPress = 0;

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

// ===========================
// KONFIGURASI MOTOR
// ===========================
#define MRF 6   // KANAN MAJU
#define MRB 5   // KANAN MUNDUR
#define MLF 10  // KIRI MAJU
#define MLB 9   // KIRI MUNDUR

int BaseSpeedL = 70;
int BaseSpeedR = 75;
int kp = 20, ki = 0, kd = 5;
int lastError = 0;
float integral = 0;

// =========================
// VARIABEL LAINNYA
// =========================
bool isTurning = false;
bool isShortPath = false;
bool finish = false;
bool LL, L, CS, R, RR;
int pathlength; 
int  readpath;
char path[25];

// ===========================
// KONTROL WAKTU
// ===========================
unsigned long lastMoveTime = 0;
unsigned long moveInterval = 10;

// ===========================
// KONTROL OLED 
// ===========================
void highlightText(int x, int y, const char* text) {
  display.setTextSize(2);
  int w = strlen(text) * 6;
  int h = 8;

  display.fillRect(x - 1, y - 1, w + 2, h + 2, WHITE);

  display.setTextColor(BLACK);
  display.setCursor(x, y);
  display.print(text);

  display.setTextColor(WHITE);
}

void normalText(int x, int y, const char* text) {
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(x, y);
  display.print(text);
}

void highlightValue(int x, int y, int value) {
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(x, y);
  display.print(" < ");
  display.print(value);
  display.print(" > ");
}

void normalValue(int x, int y, int value) {
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(x, y);
  display.print(value);
}

// ===========================
// KONTROL MOTOR
// ===========================
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
  setMotorSpeed(-90, 100);
  while (true) {
    byte s = readSensor();
    if (s & 0b00011100) break;
  }
}

void turnRightUntilLine() {
  setMotorSpeed(90, -100);
  while (true) {
    byte s = readSensor();
    if (s & 0b00111000) break;
  }
}

void uTurnUntilLine() {
  setMotorSpeed(-100, 100);
  while (true) {
    byte s = readSensor();
    if (bitRead(s, 2) || bitRead(s, 5)) break;
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

void moveLeftUntilTime(unsigned long duration) {
  unsigned long startTime = millis();
  setMotorSpeed(-90, 90);
  while (millis() - startTime < duration) {
    byte s = readSensor();  // Tetap bisa deteksi perubahan sensor
    // Optional: bisa keluar lebih awal kalau muncul garis
  }
}

void moveLeftwithDelay(unsigned long duration) {
  setMotorSpeed(-90, 90);
  delay(duration);
}

void moveRightUntilTime(unsigned long duration) {
  unsigned long startTime = millis();
  setMotorSpeed(90, -90);
  while (millis() - startTime < duration) {
    byte s = readSensor();  // Tetap bisa deteksi perubahan sensor
    // Optional: bisa keluar lebih awal kalau muncul garis
  }
}

// ===========================
// FOLLOW LINE
// ===========================
byte readSensor() {
  byte dataSensor = 0;
  for (int i = 0; i < 8; i++) {
    int sensorIndex = physicalOrder[i];
    digitalWrite(S0, bitRead(sensorIndex, 0));
    digitalWrite(S1, bitRead(sensorIndex, 1));
    digitalWrite(S2, bitRead(sensorIndex, 2));
    delayMicroseconds(20);
    int val = analogRead(A0);
    isLineDetected[i] = val < threshold[sensorIndex];
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

void followLine(byte sensorData) {
  if (sensorData == 0x00) {
    stopMotor();
    return;
  }
  int error = calcError(sensorData);
  integral += error;
  int derivative = error - lastError;
  lastError = error;
  float correction = kp * error + ki * integral + kd * derivative;
  int leftSpeed = constrain(BaseSpeedL + correction, 0, 255);
  int rightSpeed = constrain(BaseSpeedR - correction, 0, 255);
  setMotorSpeed(leftSpeed, rightSpeed);
}

// =========================
// DETEKSI KONDISI JALAN
// =========================
bool isDeadEnd(byte s) { // DEAD END
  return s == 0b00000000;
}

bool isStraight(byte s) { // STRAIGHT
  return (s == 0b00011000 || s == 0b00010000 || s == 0b00001000);
}

bool isLeft(byte s) { // LEFT ONLY
  return (s == 0b10000000 || s == 0b11000000 || s == 0b11100000);
}

bool isLeftOnly(byte s) { // LEFT ONLY
  return (s == 0b00000000);
}

bool isRight(byte s) { // RIGHT ONLY
  return (s == 0b00000011 || s == 0b00000111);
}

bool isRightOnly(byte s) { // LEFT ONLY
  return (s == 0b00000000);
}

bool isTJunction(byte s) { // T-JUNCTION
  return (s == 0b00000000 || s == 0b10000001);
}

bool isCrossJunction(byte s) { // CROSS-JUNCTION
  return (s == 0b10011001 || s == 0b10010001 || s == 0b10001001 || 
          s == 0b00011000 || s == 0b00010000 || s == 0b00001000 || 
          s == 0b00110000 || s == 0b00100000 || s == 0b00001100 ||
          s == 0b00000100 || s == 0b10100001 || s == 0b10110001 ||
          s == 0b10000101 || s == 0b10001101);
}

bool isEndMaze(byte s) {
  return (s == 0b11111111 || s == 0b01111110 || s == 0b01111100 ||
          s == 0b00111110 || s == 0b00111100);
}

void updateFlags() {
  LL = sensor_values[0] || sensor_values[1];
  L  = sensor_values[2];
  CS = sensor_values[3] || sensor_values[4];
  R  = sensor_values[5];
  RR = sensor_values[6] || sensor_values[7];
}

// ===========================
// MAZE SOLVER
// ===========================
void simplifyPath() {
  bool changed;
  do {
    changed = false;
    for (int i = 1; i < pathlength - 1; i++) {
      if (path[i] == 'U') {
        char before = path[i - 1];
        char after = path[i + 1];

        if (before == 'R' && after == 'R') {
          path[i - 1] = 'S';
        }
        else if (before == 'R' && after == 'S') {
          path[i - 1] = 'L';
        }
        else if (before == 'L' && after == 'L') {
          path[i - 1] = 'S';
        }
        else if (before == 'L' && after == 'S') {
          path[i - 1] = 'R';
        }
        else if (before == 'S' && after == 'L') {
          path[i - 1] = 'R';
        }
        else if (before == 'S' && after == 'R') {
          path[i - 1] = 'L';
        }
        else if ((before == 'R' && after == 'L') || (before == 'L' && after == 'R')) {
          path[i - 1] = 'U';
        }
        else if (before == 'S' && after == 'S') {
          path[i - 1] = 'U';
        }
        else {
          continue;
        }

        // Geser array
        for (int j = i; j < pathlength - 2; j++) {
          path[j] = path[j + 2];
        }
        pathlength -= 2;
        changed = true;
        break;
      }
    }
  } while (changed);
}

void choosePath() {
  if (path[readpath] == 'F') {
    isShortPath = false;
    return;
  } else if (path[readpath] == 'R') {
    isTurning = true;
    stopMotorUntilTime(200);
    moveRightUntilTime(300);
    turnRightUntilLine();
    isTurning = false;
  } else if (path[readpath] == 'S') {
    //
  } else if (path[readpath] == 'L') {
    isTurning = true;
    moveLeftUntilTime(300);
    turnLeftUntilLine();
    isTurning = false;
  }
  readpath++;
}


// ===========================
// SETUP
// ===========================
void setup() {
  Wire.begin();
  Serial.begin(9600);
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(muxOut, INPUT);

  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  pinMode(BTN_LEFT, INPUT_PULLUP);
  pinMode(BTN_RIGHT, INPUT_PULLUP);

  pinMode(MRF, OUTPUT);
  pinMode(MRB, OUTPUT);
  pinMode(MLF, OUTPUT);
  pinMode(MLB, OUTPUT);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
}

// =========================
// LOOP UTAMA
// =========================
void loop() {
  if (!finish) {
    if (isTurning) {
      return; // Skip semua logic, tunggu belok selesai
    }

    unsigned long currentTime = millis();
    if (currentTime - lastMoveTime >= moveInterval) {
      lastMoveTime = currentTime;
      byte sensorData = readSensor();
      updateFlags();

      Serial.print("Sensor: ");
      Serial.println(sensorData, BIN);

      if (isDeadEnd(sensorData)) {
        isTurning = true;
        moveForwardUntilTime(400);
        stopMotorUntilTime(200); 
        uTurnUntilLine();
        path[pathlength] = 'U';
        pathlength++;
        isTurning = false;
      }
      else if (isEndMaze(sensorData)) {
        moveForwardUntilTime(300);
        stopMotorUntilTime(200);
        sensorData = readSensor();
        if (isEndMaze(sensorData)) {
          stopMotor();
          path[pathlength] = 'F';
          pathlength++;
          finish = true;
        }
        else if (isTJunction(sensorData)) {
          isTurning = true;
          turnRightUntilLine();
          path[pathlength] = 'R';
          pathlength++;
          isTurning = false;
        }
        else if (isCrossJunction(sensorData)) {
          isTurning = true;
          moveRightUntilTime(300);
          turnRightUntilLine();
          path[pathlength] = 'R';
          pathlength++;
          isTurning = false;
        }
      }
      else if (!LL && CS && RR) {
        moveForwardUntilTime(300);
        stopMotorUntilTime(200);
        sensorData = readSensor();
        updateFlags();
        if (isEndMaze(sensorData)) {
          stopMotor();
          path[pathlength] = 'F';
          pathlength++;
          finish = true;
        }
        else if (isRightOnly(sensorData)) {
          isTurning = true;
          moveRightUntilTime(300);
          turnRightUntilLine();
          isTurning = false;
        }
        else {
          isTurning = true;
          moveRightUntilTime(300);
          turnRightUntilLine();
          path[pathlength] = 'R';
          pathlength++;
          isTurning = false;
        }
      }
      else if (LL && CS && !RR) {
        moveForwardUntilTime(300);
        // stopMotorUntilTime(200);
        sensorData = readSensor();
        updateFlags();
        if (isEndMaze(sensorData)) {
          stopMotor();
          path[pathlength] = 'F';
          pathlength++;
          finish = true;
        }
        else if (isLeftOnly(sensorData)) {
          isTurning = true;
          stopMotorUntilTime(200);
          moveLeftUntilTime(300);
          turnLeftUntilLine();
          isTurning = false;
        }
        else {
          followLine(sensorData);
          path[pathlength] = 'S';
          pathlength++;
          delay(200);
        }
      }
      // else if (isRight(sensorData)) {
      //   isTurning = true;
      //   moveForwardUntilTime(300);
      //   stopMotorUntilTime(200);
      //   moveRightUntilTime(300);
      //   turnRightUntilLine();
      //   isTurning = false;
      // }
      else if (isStraight(sensorData)) {
        followLine(sensorData);
      }
      else {
        followLine(sensorData);
      }
    }
  }

  simplifyPath();

  if (finish){
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(WHITE); // Tambahan biar pasti tampil
    display.setCursor(0, 0);
    display.print("Path:");

    int x = 0;
    int y = 20;

    for (int i = 0; i < pathlength; i++) {
      if (x > 100) { // Biar nggak keluar layar
        x = 0;
        y += 16;
      }
      display.setCursor(x, y);
      display.print(path[i]);
      x += 12;
    }

    display.display();

    if (digitalRead(BTN_LEFT) == LOW) {
      isShortPath = true;
      lastButtonPress = millis();
    }
  }

  if (isShortPath) {
    if (isTurning) {
      return; // Skip semua logic, tunggu belok selesai
    }

    unsigned long currentTime = millis();
    if (currentTime - lastMoveTime >= moveInterval) {
      lastMoveTime = currentTime;
      byte sensorData = readSensor();
      updateFlags();

      Serial.print("Sensor: ");
      Serial.println(sensorData, BIN);

      if (isDeadEnd(sensorData)) {
        isTurning = true;
        moveForwardUntilTime(400);
        stopMotorUntilTime(200); 
        uTurnUntilLine();
        isTurning = false;
      }
      else if (isEndMaze(sensorData)) {
        moveForwardUntilTime(300);
        stopMotorUntilTime(200);
        sensorData = readSensor();
        if (isEndMaze(sensorData)) {
          stopMotor();
          isShortPath = false;
        }
        else if (isTJunction(sensorData)) {
          choosePath();
        }
        else if (isCrossJunction(sensorData)) {
          choosePath();
        }
      }
      else if (!LL && CS && RR) {
        moveForwardUntilTime(300);
        stopMotorUntilTime(200);
        sensorData = readSensor();
        updateFlags();
        if (isEndMaze(sensorData)) {
          stopMotor();
          isShortPath = false;
        }
        else if (isRightOnly(sensorData)) {
          isTurning = true;
          moveRightUntilTime(300);
          turnRightUntilLine();
          isTurning = false;
        }
        else {
          choosePath();
        }
      }
      else if (LL && CS && !RR) {
        moveForwardUntilTime(300);
        sensorData = readSensor();
        updateFlags();
        if (isEndMaze(sensorData)) {
          stopMotor();
          isShortPath = false;
        }
        else if (isLeftOnly(sensorData)) {
          isTurning = true;
          stopMotorUntilTime(200);
          moveLeftUntilTime(300);
          turnLeftUntilLine();
          isTurning = false;
        }
        else {
          choosePath();
        }
      }
      // else if (isRight(sensorData)) {
      //   isTurning = true;
      //   moveForwardUntilTime(300);
      //   stopMotorUntilTime(200);
      //   moveRightUntilTime(300);
      //   turnRightUntilLine();
      //   isTurning = false;
      // }
      else if (isStraight(sensorData)) {
        followLine(sensorData);
      }
      else {
        followLine(sensorData);
      }
    }
  }

}

