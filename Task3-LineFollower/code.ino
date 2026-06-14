#include <Wire.h>
#include <ADS1115_WE.h>

const int enablePin1 = 9;   
const int enablePin2 = 10;  

const int in1Pin = 2;  
const int in2Pin = 3;  
const int in3Pin = 4;  
const int in4Pin = 5;  

const int maxSpeed = 250;      

#define I2C_ADDRESS 0x48
ADS1115_WE ads(I2C_ADDRESS);

const ADS1115_MUX adcMux[3] = {
  ADS1115_COMP_0_GND,   
  ADS1115_COMP_1_GND,   
  ADS1115_COMP_2_GND    
};

const char* sensorName[3] = {"LEFT", "CENTER", "RIGHT"};

int16_t adcRaw[3] = {0, 0, 0};
int irSensorDigital[3] = {0, 0, 0};

int16_t threshold[3] = {
  28500,   
  23900,   
  28400    
};

const bool blackLineIsHigh = true;   
const byte averageSamples = 4;       

int irSensors = B000;                

float error = 0.0;          
float lastError = 0.0;      
float integral = 0.0;       

float Kp = 1.5;           
float Ki = 0.0;            
float Kd = 0.4;            

int leftMotorSpeed = 0;
int rightMotorSpeed = 0;

int leftBaseSpeed = 210;   
int rightBaseSpeed = 210;  

float leftGain = 1.0;   
float rightGain = 1.0;  

const int searchSpeed = 120;  

int lastTurnDirection = 0;

const bool debugMode = true;
unsigned long lastPrintTime = 0;
const unsigned long printInterval = 200;

void Scan();
int16_t readAdsRaw(ADS1115_MUX channel);
int16_t readAverageRaw(ADS1115_MUX channel);
void UpdateDirection();
void Drive(int leftSpeed, int rightSpeed);
void stopMotors();
void printHeader();
void printDebug();

void setup() {
  Serial.begin(9600);

  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  pinMode(in3Pin, OUTPUT);
  pinMode(in4Pin, OUTPUT);
  pinMode(enablePin1, OUTPUT);
  pinMode(enablePin2, OUTPUT);
  stopMotors();

  Wire.begin();
  if (!ads.init()) {
    Serial.println("ADS1115 init failed!");
    stopMotors();
    while (1) delay(100);
  }
  Serial.println("ADS1115 init success!");

  ads.setVoltageRange_mV(ADS1115_RANGE_4096);
  ads.setMeasureMode(ADS1115_SINGLE);
  ads.setConvRate(ADS1115_860_SPS);
  ads.setAlertPinMode(ADS1115_DISABLE_ALERT);

  if (debugMode) printHeader();
}

void loop() {
  Scan();               
  UpdateDirection();    
  Drive(leftMotorSpeed, rightMotorSpeed);

  if (debugMode) printDebug();
}

void Scan() {
  irSensors = B000;

  for (int i = 0; i < 3; i++) {
    adcRaw[i] = readAverageRaw(adcMux[i]);

    if (blackLineIsHigh) {
      irSensorDigital[i] = (adcRaw[i] >= threshold[i]) ? 1 : 0;
    } else {
      irSensorDigital[i] = (adcRaw[i] <= threshold[i]) ? 1 : 0;
    }

    irSensors |= irSensorDigital[i] << (2 - i);
  }
}

int16_t readAverageRaw(ADS1115_MUX channel) {
  long sum = 0;
  for (byte i = 0; i < averageSamples; i++) {
    sum += readAdsRaw(channel);
    delayMicroseconds(300);
  }
  return (int16_t)(sum / averageSamples);
}

int16_t readAdsRaw(ADS1115_MUX channel) {
  ads.setCompareChannels(channel);
  ads.startSingleMeasurement();

  unsigned long startTime = micros();
  while (ads.isBusy()) {
    if ((micros() - startTime) > 5000UL) break;
  }
  return ads.getRawResult();
}

void UpdateDirection() {
  switch (irSensors) {
    case B100:  
      error = 0.6;
      break;
    case B110:  
      error = 0.3;
      break;
    case B010:  
      error = 0.0;
      break;
    case B011:  
      error = -0.3;
      break;
    case B001:  
      error = -0.6;
      break;
    case B111:  
      error = 0.0;
      break;
    case B000:  
      handleLineLost();
      return;   
    default:
      error = 0.0;
      break;
  }

  if (error < 0) lastTurnDirection = -1;
  else if (error > 0) lastTurnDirection = 1;
  else lastTurnDirection = 0;   

  float proportional = error;
  integral += error;
  integral = constrain(integral, -30.0, 30.0);
  float derivative = error - lastError;
  lastError = error;

  float correction = Kp * proportional + Ki * integral + Kd * derivative;


float maxCorrectionLeft  = maxSpeed - abs(leftBaseSpeed);
float maxCorrectionRight = maxSpeed - abs(rightBaseSpeed);
float maxCorrection = min(maxCorrectionLeft, maxCorrectionRight);
correction = constrain(correction, -maxCorrection, maxCorrection);

leftMotorSpeed = leftBaseSpeed - correction;
rightMotorSpeed = rightBaseSpeed + correction;


  leftMotorSpeed = constrain(leftMotorSpeed, -maxSpeed, maxSpeed);
  rightMotorSpeed = constrain(rightMotorSpeed, -maxSpeed, maxSpeed);
}

void handleLineLost() {
  if (lastTurnDirection <= 0) {   
    leftMotorSpeed = -searchSpeed;
    rightMotorSpeed = searchSpeed;
  } else {                         
    leftMotorSpeed = searchSpeed;
    rightMotorSpeed = -searchSpeed;
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

  analogWrite(enablePin1, abs(rightSpeed)* rightGain);
  analogWrite(enablePin2, abs(leftSpeed) * leftGain);
}

void stopMotors() {
  analogWrite(enablePin1, 0);
  analogWrite(enablePin2, 0);
  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, LOW);
  digitalWrite(in3Pin, LOW);
  digitalWrite(in4Pin, LOW);
  leftMotorSpeed = 0;
  rightMotorSpeed = 0;
}

void printHeader() {
  Serial.println("====================================");
  Serial.println("PID Line Tracking Debug");
  Serial.println("====================================");
  for (int i = 0; i < 3; i++) {
    Serial.print(sensorName[i]);
    Serial.print("\t");
  }
  Serial.print("BIN\t");
  Serial.print("Err\t");
  Serial.print("Corr\t");
  Serial.print("L_PWM\t");
  Serial.print("R_PWM\t");
  Serial.println();
}

void printDebug() {
  if (millis() - lastPrintTime < printInterval) return;
  lastPrintTime = millis();

  for (int i = 0; i < 3; i++) {
    Serial.print(adcRaw[i]);
    Serial.print("(");
    Serial.print(irSensorDigital[i]);
    Serial.print(")");
    Serial.print("\t");
  }

  Serial.print(irSensorDigital[0]);
  Serial.print(irSensorDigital[1]);
  Serial.print(irSensorDigital[2]);
  Serial.print("\t");

  Serial.print(error, 2);
  Serial.print("\t");
  float correction = (rightMotorSpeed - leftMotorSpeed) / 2.0;
  Serial.print(correction, 1);
  Serial.print("\t");

  Serial.print(leftMotorSpeed);
  Serial.print("\t");
  Serial.print(rightMotorSpeed);
  Serial.println();
}
