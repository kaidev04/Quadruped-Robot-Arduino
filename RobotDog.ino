#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <MPU6050_light.h>
#include <math.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  70 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  450 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

#define PI 3.14159265358979323846

// our servo # counter
uint8_t servonum = 12;

MPU6050 mpu(Wire);

struct Leg {
  String name;

  int upperLegServo;
  int lowerLegServo;
  int hipServo;

  double upperLegLength;
  double lowerLegLength;
  
  int legHeight;

  double upperLegPos;
  double lowerLegPos;
  double hipServoPos;

  double defaultUpperLegPos;
  double defaultLowerLegPos;
};

int defaultLegHeight = 15;

Leg frontLeftLeg = {"frontLeftLeg", 0, 1, 2, 12.5, 13, defaultLegHeight, 90, 90, 90, 0, 0};
Leg frontRightLeg = {"frontRightLeg", 3, 4, 5, 12.5, 13, defaultLegHeight, 90, 90, 90, 0, 0};
Leg backLeftLeg = {"backLeftLeg", 6, 7, 8, 12.5, 13, defaultLegHeight, 90, 90, 90, 0, 0};
Leg backRightLeg = {"backRightLeg", 9, 10, 11, 12.5, 13, defaultLegHeight, 90, 90, 90, 0, 0};

int walkRadius = 5;
int bodyWidth = 20;
int bodyLength = 35;

unsigned long prevTime = millis();
long interval = 60;
int counter1 = 1;
int counter2 = 1;

bool addition1 = true;  // If true = add 1 step to x axis for frontLeftLeg and backRightLeg, else minus 1 step
bool addition2 = false;  // If true = add 1 step to x axis for frontRightLeg and backLeftLeg, else minus 1 step
bool initial = true;  // If true = do not move frontRightLeg and backLeftLeg until addition1 has completed the first cycle

bool balanceMode = false;

int kp = 1;
int ki = 0.2;
int kd = 100;

float PID_p, PID_i, PID_d;

float previousErrorX;
int previousLegHeight = defaultLegHeight;

unsigned long timer = 0;
int period = 50;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  Serial.println("8 channel Servo test!");

  pwm.begin();

  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");

  frontLeftLeg.hipServoPos = 120;
  backLeftLeg.hipServoPos = 120;
  frontRightLeg.hipServoPos = 60;
  backRightLeg.hipServoPos = 60;

  if(true) {
    setLegHeight(frontLeftLeg, frontLeftLeg.legHeight);
    setLegHeight(frontRightLeg, frontRightLeg.legHeight);
    setLegHeight(backLeftLeg, backLeftLeg.legHeight);
    setLegHeight(backRightLeg, backRightLeg.legHeight);
  }

  delay(10);
}

// You can use this function if you'd like to set the pulse length in seconds
// e.g. setServoPulse(0, 0.001) is a ~1 millisecond pulse width. It's not precise!
void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= SERVO_FREQ;   // Analog servos run at ~60 Hz updates
  Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000000;  // convert input seconds to us
  pulse /= pulselength;
  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}



void loop() {
  if(false) {
    mpu.update();
  
    if((millis()-timer)>10){ // print data every 10ms
      Serial.print("X : ");
      Serial.print(mpu.getAngleX());
      Serial.print("\tY : ");
      Serial.print(mpu.getAngleY());
      Serial.print("\tZ : ");
      Serial.println(mpu.getAngleZ());
      timer = millis();  
    }
  }
  if(false) {
    pwm.setPWM(frontLeftLeg.hipServo, 0, angletoPulse(frontLeftLeg.hipServoPos));
    pwm.setPWM(frontRightLeg.hipServo, 0, angletoPulse(frontRightLeg.hipServoPos));
    pwm.setPWM(backLeftLeg.hipServo, 0, angletoPulse(backLeftLeg.hipServoPos));
    pwm.setPWM(backRightLeg.hipServo, 0, angletoPulse(backRightLeg.hipServoPos));
  }

  if(false) {
    walkMotion();
  }

  if(false) {
    staticBalance();
  }

  if(false) {
    PID_balance();
  }

  if(false) {
    for(int height = 10; height <= 18; height++) {
      setLegHeight(frontLeftLeg, height);
      setLegHeight(frontRightLeg, height);
      setLegHeight(backLeftLeg, height);
      setLegHeight(backRightLeg, height);
      delay(50);
    }

    for(int height = 18; height >= 10; height--) {
      setLegHeight(frontLeftLeg, height);
      setLegHeight(frontRightLeg, height);
      setLegHeight(backLeftLeg, height);
      setLegHeight(backRightLeg, height);
      delay(50);
    }
  }

}

int angletoPulse(int angle) {
  int pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  return pulse;
}

void setAngleZero() {
  for(int servo = 0; servo < servonum; servo++) {
    pwm.setPWM(servo, 0, angletoPulse(0));
  }
}

void resetAngle() {
  for(int servo = 0; servo < servonum; servo++) {
    pwm.setPWM(servo, 0, angletoPulse(90));
  }
}

void setLegHeight(Leg leg, int height) {
  

  leg.upperLegPos = acos((pow(leg.upperLegLength, 2) + pow(height, 2) - pow(leg.lowerLegLength, 2)) / (2 * leg.upperLegLength * height));
  leg.lowerLegPos = acos((pow(leg.upperLegLength, 2) + pow(leg.lowerLegLength, 2) - pow(height, 2)) / (2 * leg.upperLegLength * leg.lowerLegLength));

  leg.upperLegPos *= 180/PI;
  leg.lowerLegPos *= 180/PI;
  
  // If statements below seem redundant but are required due to issues modifying struct object attributes

  if(leg.name.equals("frontLeftLeg")) {
    leg.upperLegPos = 90 + leg.upperLegPos;
    leg.lowerLegPos = 180 - leg.lowerLegPos;

    frontLeftLeg.defaultUpperLegPos = round(leg.upperLegPos);
    frontLeftLeg.defaultLowerLegPos = round(leg.lowerLegPos);
    if(!balanceMode) {
      frontLeftLeg.legHeight = height;
    }
  }
  
  else if(leg.name.equals("frontRightLeg")) {
    leg.upperLegPos = 90 - leg.upperLegPos;

    frontRightLeg.defaultUpperLegPos = round(leg.upperLegPos);
    frontRightLeg.defaultLowerLegPos = round(leg.lowerLegPos);
    if(!balanceMode) {
      frontRightLeg.legHeight = height;
    }
  }

  else if(leg.name.equals("backLeftLeg")) {
    leg.upperLegPos = 90 + leg.upperLegPos;
    leg.lowerLegPos = 180 - leg.lowerLegPos;

    backLeftLeg.defaultUpperLegPos = round(leg.upperLegPos);
    backLeftLeg.defaultLowerLegPos = round(leg.lowerLegPos);
    if(!balanceMode) {
      backLeftLeg.legHeight = height;
    }
  }
  
  else if(leg.name.equals("backRightLeg")) {
    leg.upperLegPos = 90 - leg.upperLegPos;

    backRightLeg.defaultUpperLegPos = round(leg.upperLegPos);
    backRightLeg.defaultLowerLegPos = round(leg.lowerLegPos);
    if(!balanceMode) {
      backRightLeg.legHeight = height;
    }
  }

  pwm.setPWM(leg.upperLegServo, 0, angletoPulse(round(leg.upperLegPos)));
  pwm.setPWM(leg.lowerLegServo, 0, angletoPulse(round(leg.lowerLegPos)));
}

void walk(Leg leg, int stepLength, bool circular) {
  balanceMode = false;

  double deltaAngle = atan((double) stepLength/leg.legHeight);
  double newHeight = stepLength / deltaAngle;

  if(circular) {
    newHeight = newHeight - circularMotion(leg, stepLength, walkRadius, leg.legHeight);
    double newVertical = sqrt(pow(newHeight, 2) - pow(stepLength, 2));
    deltaAngle = atan((double) stepLength/newVertical);
    newHeight = stepLength / deltaAngle;
  }

  leg.lowerLegPos = acos((pow(leg.upperLegLength, 2) + pow(leg.lowerLegLength, 2) - pow(newHeight, 2)) / (2 * leg.upperLegLength * leg.lowerLegLength));
  leg.lowerLegPos *= 180/PI;

  deltaAngle *= 180/PI;

  // If statements below seem redundant but are required due to issues modifying struct object attributes

  if(leg.name.equals("frontLeftLeg")) {
    leg.upperLegPos = leg.defaultUpperLegPos - deltaAngle;
    leg.lowerLegPos = 180 - leg.lowerLegPos;
  }
  
  else if(leg.name.equals("frontRightLeg")) {
    leg.upperLegPos = leg.defaultUpperLegPos - deltaAngle;
    leg.upperLegPos = 90 - leg.upperLegPos;
  }

  else if(leg.name.equals("backLeftLeg")) {
    leg.upperLegPos = leg.defaultUpperLegPos - deltaAngle;
    leg.lowerLegPos = 180 - leg.lowerLegPos;
  }

  else if(leg.name.equals("backRightLeg")) {
    leg.upperLegPos = leg.defaultUpperLegPos - deltaAngle;
    leg.upperLegPos = 90 - leg.upperLegPos;
  }


  pwm.setPWM(leg.upperLegServo, 0, angletoPulse(round(leg.upperLegPos)));
  pwm.setPWM(leg.lowerLegServo, 0, angletoPulse(round(leg.lowerLegPos)));
  
}

int circularMotion(Leg leg, int stepLength, int radius, int height) {
  double verticalAxis = sqrt(pow(radius, 2) - pow((stepLength - radius), 2));
  double upperLegPos = acos((pow(leg.upperLegLength, 2) + pow(height, 2) - pow(leg.lowerLegLength, 2)) / (2 * leg.upperLegLength * height));
  double deltaHorizontal = verticalAxis * upperLegPos;
  double deltaHeight = sqrt(pow(deltaHorizontal, 2) + pow(verticalAxis, 2));
  
  return deltaHeight;

}

void walkMotion() {
  unsigned long currentTime = millis();

  if(currentTime - prevTime > interval) {
    if(addition1) {
      if(!initial) {
        counter2 -= 1;
        walk(frontRightLeg, counter2, false);
        walk(backLeftLeg, counter2, false);
      }
      counter1 += 1;

      walk(frontLeftLeg, counter1, true);
      walk(backRightLeg, counter1, true);
    }
    else if(addition2) {
      counter1 -= 1;
      counter2 += 1;
      
      walk(frontLeftLeg, counter1, false);
      walk(backRightLeg, counter1, false);

      walk(frontRightLeg, counter2, true);
      walk(backLeftLeg, counter2, true);
    }

    if(counter1 == 10) {
      addition1 = false;
      addition2 = true;
    }

    if(counter2 == 10) {
      addition1 = true;
      addition2 = false;
      initial = false;
    }

    prevTime = currentTime;
  }

}

void staticBalance() {
  balanceMode = true;

  mpu.update();

  float ax = mpu.getAngleX();
  float ay = mpu.getAngleY();

  int heightLimit = 25;

  if(true) {
    double deltaHeightX = bodyWidth * sin(ax * (PI/180));
    double deltaHeightY = bodyLength * sin(ay * (PI/180));

    int leftLegHeight = frontLeftLeg.legHeight - (int) deltaHeightX;
    int rightLegHeight = frontRightLeg.legHeight + (int) deltaHeightX;

    if((leftLegHeight + (int) deltaHeightY) < heightLimit && (rightLegHeight + (int) deltaHeightY) < heightLimit) {
      setLegHeight(frontLeftLeg, leftLegHeight + (int) deltaHeightY);
      setLegHeight(backLeftLeg, leftLegHeight - (int) deltaHeightY);
      setLegHeight(frontRightLeg, rightLegHeight + (int) deltaHeightY);
      setLegHeight(backRightLeg, rightLegHeight - (int) deltaHeightY);
    }
  }
}

void PID_balance() {
  balanceMode = true;

  if(millis() > timer + period) {    
    timer = millis();

    mpu.update();
    float errorX = mpu.getAngleX();

    PID_p = kp * errorX;

    float errorDifference = errorX - previousErrorX;
    PID_d = kd * (errorDifference/period);

    PID_i += ki * errorX;

    float outputX = PID_p + PID_i + PID_d;

    // Calculate leg height according to output
    double deltaHeightX = bodyWidth * sin(outputX * (PI/180));

    int heightLimit = 25;  // Height limit for each leg to avoid trigonometric error for impossible heights in triangles

    if((10 <= frontLeftLeg.legHeight - (int) deltaHeightX && frontLeftLeg.legHeight - (int) deltaHeightX <= heightLimit) && (10 <= frontRightLeg.legHeight + (int) deltaHeightX && frontRightLeg.legHeight + (int) deltaHeightX <= heightLimit)) {
      frontLeftLeg.legHeight -= (int) deltaHeightX;
      backLeftLeg.legHeight -= (int) deltaHeightX;
      frontRightLeg.legHeight += (int) deltaHeightX;
      backRightLeg.legHeight += (int) deltaHeightX;
    }

    Serial.print("frontLeftLeg: ");
    Serial.println(frontLeftLeg.legHeight);
    Serial.print("frontRightLeg: ");
    Serial.println(frontRightLeg.legHeight);

    if(true) {
      setLegHeight(frontLeftLeg, frontLeftLeg.legHeight);
      setLegHeight(backLeftLeg, backLeftLeg.legHeight);
      setLegHeight(frontRightLeg, frontRightLeg.legHeight);
      setLegHeight(backRightLeg, backRightLeg.legHeight);
    }

    previousErrorX = errorX;
  }
  

  
}
