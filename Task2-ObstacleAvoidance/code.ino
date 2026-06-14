#include <Servo.h>

int MotorLeft1 = 3;
int MotorLeft2 = 2;
int MotorRight1 = 5;
int MotorRight2 = 4;

int inputPin = 12;  
int outputPin = 11;
const int servo = 6;
Servo myservo;


float Fspeedd = 0;   
float Lspeedd = 0;   
float Rspeedd = 0;   

const float NO_ECHO_DIST = 200.0;   


const int Fgo = 8;  
const int Rgo = 6;   
const int Lgo = 4;   
const int Bgo = 2;   

unsigned long lastScanTime = 0;      
int scanAngleIndex = 0;             


int scanState = 0;               
unsigned long scanStateStartTime = 0;   
int currentAngleIndex = 0;       
const int servoMoveTime = 150;    
const int servoStableTime = 250;  
const int angles[3] = {177, 70, 5};       
unsigned long lastActionStart = 0;    
int currentDirection = Fgo;           
int afterBackStep = 0;   
bool isTurning = false;               
unsigned long turnDuration = 300;    
unsigned long turnStartTime = 0;      


const unsigned long TURN_DUR_NORMAL = 300;   
const unsigned long TURN_DUR_ADJUST = 250;   
const unsigned long TURN_DUR_BACK = 500;     
const unsigned long TURN_DUR_BACK_ROTATE = 300; 


const int FRONT_AVOID_DIST = 40;      
const int SIDE_AVOID_DIST = 40;       
const int BACK_DIST = 40;

void setMotorStop();
void setMotorForward();
void setMotorBackward();
void setMotorTurnLeft();
void setMotorTurnRight();
void measureFront();
void measureLeft();
void measureRight();
void updateDecision();
void executeMotion();

void setup() {
  Serial.begin(9600);

  pinMode(MotorLeft1, OUTPUT);
  pinMode(MotorLeft2, OUTPUT);
  pinMode(MotorRight1, OUTPUT);
  pinMode(MotorRight2, OUTPUT);

  pinMode(inputPin, INPUT);
  pinMode(outputPin, OUTPUT);

  myservo.attach(servo);
  myservo.write(70);   
  delay(300);

scanState = 1;                      
scanStateStartTime = millis();
currentAngleIndex = 0;
myservo.write(angles[currentAngleIndex]);


  Serial.println("System ready - roaming with continuous scanning");
}


void loop() {
  switch (scanState) {
    case 0:
      break;

    case 1:  
      if (millis() - scanStateStartTime >= servoMoveTime) {
        scanState = 2;
        scanStateStartTime = millis();
      }
      break;

    case 2:  
      if (millis() - scanStateStartTime >= servoStableTime) {
        scanState = 3;
        scanStateStartTime = millis();
      }
      break;

    case 3:  
      {
        int angle = angles[currentAngleIndex];
        if (angle == 70) {
          measureFront();
        } else if (angle == 177) {
          measureLeft();
        } else if (angle == 5) {
          measureRight();
        }
        Serial.print("F:");
        Serial.print(Fspeedd);
        Serial.print(" L:");
        Serial.print(Lspeedd);
        Serial.print(" R:");
        Serial.println(Rspeedd);

        currentAngleIndex = (currentAngleIndex + 1) % 3;
        int nextAngle = angles[currentAngleIndex];
        myservo.write(nextAngle);

        scanState = 1;
        scanStateStartTime = millis();
      }
      break;
  }

  updateDecision();

  executeMotion();
}


void setMotorStop() {
  digitalWrite(MotorRight1, LOW);
  digitalWrite(MotorRight2, LOW);
  digitalWrite(MotorLeft1, LOW);
  digitalWrite(MotorLeft2, LOW);
}

void setMotorForward() {
  digitalWrite(MotorRight1, LOW);
  digitalWrite(MotorRight2, HIGH);
  digitalWrite(MotorLeft1, LOW);
  digitalWrite(MotorLeft2, HIGH);
}

void setMotorBackward() {
  digitalWrite(MotorRight1, HIGH);
  digitalWrite(MotorRight2, LOW);
  digitalWrite(MotorLeft1, HIGH);
  digitalWrite(MotorLeft2, LOW);
}

void setMotorTurnLeft() {
  
  digitalWrite(MotorRight1, LOW);
  digitalWrite(MotorRight2, HIGH);   
  digitalWrite(MotorLeft1, LOW);
  digitalWrite(MotorLeft2, LOW);     
}

void setMotorTurnRight() {
  digitalWrite(MotorLeft1, LOW);
  digitalWrite(MotorLeft2, HIGH);    
  digitalWrite(MotorRight1, LOW);
  digitalWrite(MotorRight2, LOW);    
}

void measureFront() {
  digitalWrite(outputPin, LOW);
  delayMicroseconds(2);
  digitalWrite(outputPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(outputPin, LOW);
  float duration = pulseIn(inputPin, HIGH);
 if (duration == 0) {
    Fspeedd = NO_ECHO_DIST;
  } else {
    Fspeedd = duration / 5.8 / 10;
  }
}

void measureLeft() {
  digitalWrite(outputPin, LOW);
  delayMicroseconds(2);
  digitalWrite(outputPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(outputPin, LOW);
  float duration = pulseIn(inputPin, HIGH);
  if (duration == 0) {
    Lspeedd = NO_ECHO_DIST;
  } else {
    Lspeedd = duration / 5.8 / 10;
  }
}

void measureRight() {
  digitalWrite(outputPin, LOW);
  delayMicroseconds(2);
  digitalWrite(outputPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(outputPin, LOW);
  float duration = pulseIn(inputPin, HIGH);
  if (duration == 0) {
    Rspeedd = NO_ECHO_DIST;
  } else {
    Rspeedd = duration / 5.8 / 10;
  }
}

void updateDecision() {
  if (isTurning) {
    if (millis() - turnStartTime >= turnDuration) {
    if (currentDirection == Bgo && afterBackStep == 0) {
      afterBackStep = 1;
      currentDirection = Rgo;   
      turnDuration = 300;       
      turnStartTime = millis();
      Serial.println("Back finished, start rotate");
    }
    else if (afterBackStep == 1) {
      afterBackStep = 0;
      isTurning = false;
      currentDirection = Fgo;
      turnDuration = TURN_DUR_NORMAL;   
      Serial.println("Rotate finished, resume forward");
    }
    else {
      isTurning = false;
      currentDirection = Fgo;
      turnDuration = TURN_DUR_NORMAL;
      Serial.println("Turn finished, resume forward");
      }
    }
    return;
  }


  if (Fspeedd < FRONT_AVOID_DIST) {
    if (Fspeedd < BACK_DIST && (Lspeedd < BACK_DIST && Rspeedd < BACK_DIST)) {
      currentDirection = Bgo;
      afterBackStep = 0; 
      turnDuration = TURN_DUR_BACK;   // 500 ms
      Serial.println("Decision: Backward (stuck)");
    } 

    else {
      if (Lspeedd > Rspeedd) {
        currentDirection = Lgo;
        afterBackStep = 0; 
        turnDuration = TURN_DUR_NORMAL;  
        Serial.println("Decision: Turn Left");
      } else {
        currentDirection = Rgo;
        afterBackStep = 0; 
        turnDuration = TURN_DUR_NORMAL;
        Serial.println("Decision: Turn Right");
      }
    }

    if (currentDirection != Fgo) {
      isTurning = true;
      turnStartTime = millis();
    }
  } 

  else if (Lspeedd < SIDE_AVOID_DIST && Rspeedd >= SIDE_AVOID_DIST) {
    currentDirection = Rgo;  
    afterBackStep = 0;
     turnDuration = TURN_DUR_ADJUST;
    isTurning = true;
    turnStartTime = millis();
    Serial.println("Decision: Adjust Right (left side close)");
  }
  else if (Rspeedd < SIDE_AVOID_DIST && Lspeedd >= SIDE_AVOID_DIST) {
    currentDirection = Lgo; 
    afterBackStep = 0;
     turnDuration = TURN_DUR_ADJUST;
    isTurning = true;
    turnStartTime = millis();
    Serial.println("Decision: Adjust Left (right side close)");
  }
  else {
    currentDirection = Fgo;
  }
}


void executeMotion() {
  switch (currentDirection) {
    case Fgo:
      setMotorForward();
      break;
    case Bgo:
      setMotorBackward();
      break;
    case Lgo:
      setMotorTurnLeft();
      break;
    case Rgo:
      setMotorTurnRight();
      break;
    default:
      setMotorStop();
      break;
  }
}
