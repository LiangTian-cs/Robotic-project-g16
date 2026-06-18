#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

const int enablePin1 = 9;   
const int enablePin2 = 10;   

const int in1Pin = 2;        
const int in2Pin = 3;
const int in3Pin = 4;      
const int in4Pin = 5;

const int maxSpeed = 250;
const int motorRunSpeed = 180;   

Adafruit_MPU6050 mpu;
Adafruit_BME280 bme;

bool bmeAvailable = false;

enum RobotState {
  STATE_UP,
  STATE_STABLE,
  STATE_DOWN
};

RobotState currentState = STATE_STABLE;

bool motorShouldRun = false;

float yRest = 0.0;
float yFiltered = 0.0;

const float yThreshold = 1.2;   

const bool upIsPositiveY = false;

const int requiredConfirmCount = 1;

int upCount = 0;
int downCount = 0;

unsigned long lastSensorTime = 0;

const unsigned long sensorInterval = 50;

unsigned long lastBmeTime = 0;
const unsigned long bmeInterval = 100;

float temperatureC = 0.0;
float humidityPercent = 0.0;
float pressureHpa = 0.0;

void scanI2C();
void setupMPU();
void setupBME280();
void calibrateYRest();

RobotState detectStateFromY(float yValue);
void applyStateAction(RobotState state);

void Drive(int leftSpeed, int rightSpeed);
void stopMotors();

void updateBME280();
void printData(float yRaw, float yFilteredValue, float yDelta, RobotState state);
const char* stateToText(RobotState state);

void setup() {
  Serial.begin(9600);
  delay(500);


  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  pinMode(in3Pin, OUTPUT);
  pinMode(in4Pin, OUTPUT);
  pinMode(enablePin1, OUTPUT);
  pinMode(enablePin2, OUTPUT);
  stopMotors();

  Wire.begin();

  Serial.println("========================================");
  Serial.println("I2C bus scan before sensor setup");
  Serial.println("========================================");
  scanI2C();

  setupMPU();
  setupBME280();

  calibrateYRest();

  Serial.println("========================================");
  Serial.println("Robot state detection started");
  Serial.println("MPU output rate: 20 times per second");
  Serial.println("BME280 output rate: 10 times per second");
  Serial.println("UP     -> motors start running");
  Serial.println("STABLE -> keep previous motor state");
  Serial.println("DOWN   -> motors stop");
  Serial.println("========================================");
}

void loop() {
  unsigned long now = millis();

  if (now - lastSensorTime >= sensorInterval) {
    lastSensorTime = now;

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float yRaw = a.acceleration.y;

    yFiltered = 0.25 * yFiltered + 0.75 * yRaw;

    float yDelta = yFiltered - yRest;

    currentState = detectStateFromY(yFiltered);

    applyStateAction(currentState);

    updateBME280();

    printData(yRaw, yFiltered, yDelta, currentState);
  }
}

void scanI2C() {
  byte error, address;
  int deviceCount = 0;

  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
      deviceCount++;
    }
  }

  Serial.print("Total I2C devices found: ");
  Serial.println(deviceCount);
  Serial.println();
}

void setupMPU() {
  Serial.println("Starting MPU sensor...");

  if (!mpu.begin()) {
    Serial.println("MPU sensor failed. Check SDA/SCL, VCC, GND.");
    while (1) {
      stopMotors();
      delay(100);
    }
  }

  Serial.println("MPU sensor detected.");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  delay(200);
}

void setupBME280() {
  Serial.println("Starting BME280 sensor...");

  if (bme.begin(0x76)) {
    bmeAvailable = true;
    Serial.println("BME280 detected at address 0x76.");
  } else if (bme.begin(0x77)) {
    bmeAvailable = true;
    Serial.println("BME280 detected at address 0x77.");
  } else {
    bmeAvailable = false;
    Serial.println("BME280 not detected.");
    Serial.println("The robot will continue with MPU + motor logic only.");
  }

  if (bmeAvailable) {
    bme.setSampling(
      Adafruit_BME280::MODE_NORMAL,
      Adafruit_BME280::SAMPLING_X2,    
      Adafruit_BME280::SAMPLING_X8,    
      Adafruit_BME280::SAMPLING_X2,    
      Adafruit_BME280::FILTER_X2,
      Adafruit_BME280::STANDBY_MS_0_5
    );

    updateBME280();
  }

  Serial.println();
}

void calibrateYRest() {
  Serial.println("Calibrating Y-axis rest value...");
  Serial.println("Keep the robot still.");

  float sum = 0.0;
  const int samples = 100;

  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    sum += a.acceleration.y;
    delay(10);
  }

  yRest = sum / samples;
  yFiltered = yRest;

  Serial.print("Y-axis rest value: ");
  Serial.print(yRest, 3);
  Serial.println(" m/s^2");
  Serial.println();
}

RobotState detectStateFromY(float yValue) {
  float yDelta = yValue - yRest;

  bool upDetected = false;
  bool downDetected = false;

  if (upIsPositiveY) {

    upDetected = yDelta > yThreshold;
    downDetected = yDelta < -yThreshold;
  } else {

    upDetected = yDelta < -yThreshold;
    downDetected = yDelta > yThreshold;
  }

  if (upDetected) {
    upCount++;
    downCount = 0;
  } else if (downDetected) {
    downCount++;
    upCount = 0;
  } else {
    upCount = 0;
    downCount = 0;
  }

  if (upCount >= requiredConfirmCount) {
    return STATE_UP;
  }

  if (downCount >= requiredConfirmCount) {
    return STATE_DOWN;
  }

  return STATE_STABLE;
}

void applyStateAction(RobotState state) {
  if (state == STATE_UP) {
    motorShouldRun = true;
  } else if (state == STATE_DOWN) {
    motorShouldRun = false;
  }


  if (motorShouldRun) {
    Drive(motorRunSpeed, motorRunSpeed);
  } else {
    stopMotors();
  }
}

void Drive(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -maxSpeed, maxSpeed);
  rightSpeed = constrain(rightSpeed, -maxSpeed, maxSpeed);

  if (rightSpeed > 0) {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
  } else if (rightSpeed < 0) {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
  } else {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
  }

  if (leftSpeed > 0) {
    digitalWrite(in3Pin, HIGH);
    digitalWrite(in4Pin, LOW);
  } else if (leftSpeed < 0) {
    digitalWrite(in3Pin, LOW);
    digitalWrite(in4Pin, HIGH);
  } else {
    digitalWrite(in3Pin, LOW);
    digitalWrite(in4Pin, LOW);
  }

  analogWrite(enablePin1, abs(rightSpeed));
  analogWrite(enablePin2, abs(leftSpeed));
}

void stopMotors() {
  analogWrite(enablePin1, 0);
  analogWrite(enablePin2, 0);

  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, LOW);
  digitalWrite(in3Pin, LOW);
  digitalWrite(in4Pin, LOW);
}

void updateBME280() {
  if (!bmeAvailable) return;

  unsigned long now = millis();

  if (now - lastBmeTime >= bmeInterval) {
    lastBmeTime = now;

    temperatureC = bme.readTemperature();
    humidityPercent = bme.readHumidity();
    pressureHpa = bme.readPressure() / 100.0;
  }
}

void printData(float yRaw, float yFilteredValue, float yDelta, RobotState state) {
  if (bmeAvailable) {
    Serial.print("Temperature: ");
    Serial.print(temperatureC, 1);
    Serial.print(" C, ");

    Serial.print("Humidity: ");
    Serial.print(humidityPercent, 1);
    Serial.print(" %, ");

    Serial.print("Air pressure: ");
    Serial.print(pressureHpa, 1);
    Serial.print(" hPa, ");
  } else {
    Serial.print("Temperature: BME280 not found, ");
    Serial.print("Humidity: BME280 not found, ");
    Serial.print("Air pressure: BME280 not found, ");
  }

  Serial.print("Y raw: ");
  Serial.print(yRaw, 3);
  Serial.print(" m/s^2, ");

  Serial.print("Y filtered: ");
  Serial.print(yFilteredValue, 3);
  Serial.print(" m/s^2, ");

  Serial.print("Y delta: ");
  Serial.print(yDelta, 3);
  Serial.print(" m/s^2, ");

  Serial.print("State: ");
  Serial.print(stateToText(state));
  Serial.print(", ");

  Serial.print("Motor: ");
  if (motorShouldRun) {
    Serial.println("RUNNING");
  } else {
    Serial.println("STOPPED");
  }
}

const char* stateToText(RobotState state) {
  switch (state) {
    case STATE_UP:
      return "UP";
    case STATE_STABLE:
      return "STABLE";
    case STATE_DOWN:
      return "DOWN";
    default:
      return "UNKNOWN";
  }
}
