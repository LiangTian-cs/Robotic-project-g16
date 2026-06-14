int enablePin1 = 9;
int enablePin2 = 10;
int in1Pin = 2;
int in2Pin = 3;
int in3Pin = 4;
int in4Pin = 5;
int sensDiffThresh = 50;
int SensorLeft = 0;
int SensorRight = 0;

int LeftSensor = A1; 
int RightSensor = A2; 
const int slowSpeed = 100; 
const int maxSpeed = 250; 


int SensorDifference = 0; 
int error = 40;

int leftServoSpeed = 0;
int rightServoSpeed = 0;

void setup()
{
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  pinMode(in3Pin, OUTPUT);
  pinMode(in4Pin, OUTPUT);
  pinMode(enablePin1, OUTPUT);
  pinMode(enablePin2, OUTPUT);
}

void loop()
{
 boolean reverse = 0; 
 SensorLeft = 1023 - analogRead(LeftSensor); 
 delay(1); 

 SensorRight = 1023 - analogRead(RightSensor); 
 delay(1); // delay to let adc settle

 SensorDifference = abs(SensorLeft - SensorRight); 


 if (SensorLeft > SensorRight && SensorDifference > sensDiffThresh) { 
 leftServoSpeed = slowSpeed; 
 rightServoSpeed = maxSpeed; 
 }

 if (SensorLeft < SensorRight && SensorDifference > sensDiffThresh) { 
 leftServoSpeed = maxSpeed; 
 rightServoSpeed = slowSpeed; 
 }

 else if (SensorDifference < sensDiffThresh) { 
 leftServoSpeed = maxSpeed; 
 rightServoSpeed = maxSpeed; 
 }
  setMotor(leftServoSpeed, rightServoSpeed, reverse);
}

void setMotor(int leftServoSpeed, int rightServoSpeed, boolean reverse)
{
  analogWrite(enablePin1, rightServoSpeed); 
  digitalWrite(in1Pin, ! reverse);
  digitalWrite(in2Pin, reverse);
  analogWrite(enablePin2, leftServoSpeed); 
  digitalWrite(in3Pin, ! reverse);
  digitalWrite(in4Pin, reverse);
}
