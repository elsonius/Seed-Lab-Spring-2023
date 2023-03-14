#include "DualMC33926MotorShield.h"
#include "Encoder.h"

//Speed of our loop
#define sampleTime 10
//Maximum angular error used in our Hysteresis
#define angleMaxError 0.4
#define angleMaxErrorI 0.02

//Position PID Values
#define pos_max 0.30
#define umax_ang 0.3


#define pi 3.1415
#define wheelDiameter 6  //Inches
#define full_rotation 3200
#define wheelBase 11.5

DualMC33926MotorShield ms;
Encoder rightMotor(2, 5);
Encoder leftMotor(3, 6);

// test values
#define pos_Kp 0.1
#define pos_Ki 0.5
#define ang_Kp 1
#define ang_Ki 0.5

// Original values
// # define pos_Kp 0.0432
// # define pos_Ki 0.0023
// # define ang_Kp 0.9
// # define ang_Kp 0.00076





//Target setpoint
double pos_Int_Setpoint = 20;  //In Inches
double ang_Setpoint = pi/2;    //In radians

double pos_Setpoint = pos_Int_Setpoint * 1.007;  //Fudges our target distance to get better results
//Variables to hold all of localization
double Tc = 0, Ts = 0, e = 0;
double currentRightMotorEncoder = 0, prevRightMotorEncoder = 0, differenceEncoderRight = 0, rotationDifferenceRight = 0;
double currentLeftMotorEncoder = 0;
double prevLeftMotorEncoder = 0;
double differenceEncoderLeft = 0; 
double rotationDifferenceLeft = 0;
double directionalVelocity = 0;
double angularVelocity = 0;
double robotX = 0;
double robotY = 0;
double robotAngle = 0;
double targetPointX = 0;
double targetPointY = 0;
double positionalError = 10;
double angularError = 0;

double sysEnc = 0, pos_I, ang_I;
double sysAngVel = 0, sysPos = 0, sysPosVel = 0, sysAng = 0;
double pos_u = 0, ang_u = 0;

bool state = false;  
void setup() {
  Tc = millis();  
  ms.init();
  pinMode(4, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  digitalWrite(7, HIGH);
  digitalWrite(8, HIGH);
  Serial.begin(115200);
  delay(3000);
}

int sgn(float num) {
  int sign = 1;
  if (num >= 0) {
    sign = 1;
  } else {
    sign = -1;
  }
  return sign;
}

void loop() {

  //This statement will determine if the robot has reached the setpoint by seeing if
  // the postitional error is small enough to stop it.

  if ((positionalError > (pos_Setpoint / 29)) || ((positionalError > 0.1))) {

    //This makes sure that the code is run fro the sample time of 10 ms.

    if (millis() >= Tc + sampleTime) {

      //At first, the previous values will be 0 and as the program goes on, the next value becomes 
      // the previous and so on.

      prevRightMotorEncoder = currentRightMotorEncoder;
      currentRightMotorEncoder = rightMotor.read();
      prevLeftMotorEncoder = currentLeftMotorEncoder;
      currentLeftMotorEncoder = leftMotor.read();
      Serial.print("Right encoder ");
      Serial.println(currentRightMotorEncoder);
      Serial.print("left encoder " );
      Serial.println(currentLeftMotorEncoder);
      
      // This is the difference in counts for the left and right motor by finding the difference between
      // the previous and current values.

      differenceEncoderLeft = -1 * (currentLeftMotorEncoder - prevLeftMotorEncoder);
      differenceEncoderRight = -1 * (currentRightMotorEncoder - prevRightMotorEncoder);

      Serial.print("difference_left " );
      Serial.println(differenceEncoderLeft);
      Serial.print("difference_right " );
      Serial.println(differenceEncoderRight);

      // The rotation difference is taking the difference in motor count, multiplying it by the pi and wheel diameter. 
      // We divide it by the total counts which is 3200. This will get us the angle difference for each increment.
      // 
      
      rotationDifferenceLeft = (pi * wheelDiameter * differenceEncoderLeft) / full_rotation;
      rotationDifferenceRight = (pi * wheelDiameter * differenceEncoderRight) / full_rotation;

      Serial.print("rotation difference left " );
      Serial.println(rotationDifferenceLeft);
      Serial.print("rotation difference right " );
      Serial.println(rotationDifferenceRight);

      // We can calculate the angular velocity by finding the difference between the two wheels divided by the wheel base
      // which we have set at 11.5 inches.

      angularVelocity = (rotationDifferenceRight - rotationDifferenceLeft) / wheelBase;

      Serial.print("angularvelocity " );
      Serial.println(angularVelocity);

      // To get the direction velocity in which hte bot travels, is the rotation added from the left and right, multiplied by 1/2 for the two wheels.

      directionalVelocity = (rotationDifferenceRight + rotationDifferenceLeft) * 0.5;

      Serial.print("directionalVelocity " );
      Serial.println(directionalVelocity);
      
      // From all this math for the direction and angular velocity, we can determine the current x, y, and angle position of the robot.
      
      robotX += directionalVelocity * cos(robotAngle);
      robotY += directionalVelocity * sin(robotAngle);
      robotAngle += angularVelocity;

      Serial.print("robotX " );
      Serial.println(robotX);
      Serial.print("robotY " );
      Serial.println(robotY);
      Serial.print("robotAngle " );
      Serial.println(robotAngle);
     

      // This will check the angels, if the angle of the robot is more than pi, that will subtract 2pi
      // the opposite will happen if it is less than negative pi.
      
      if (robotAngle > (pi)) {
        robotAngle -= 2 * pi;
      }
      if (robotAngle < (-1 * pi)) {
        robotAngle += 2 * pi;
      }

      //Calculate Target Point
      targetPointX = pos_Setpoint * cos(ang_Setpoint);
      targetPointY = pos_Setpoint * sin(ang_Setpoint);


      //Calculate Postion Error
      positionalError = sqrt(sq(targetPointX - robotX) + sq(targetPointY - robotY));


      //Calculate Angular Error
      angularError = atan((targetPointY - robotY) / (targetPointX - robotX)) - robotAngle;


      //Correct Tangent
      if ((targetPointX - robotX) < 0) {
        angularError += pi;
      }


      //Correct Anglular error
      if (angularError > (pi)) {
        angularError -= 2 * pi;
      }
      if (angularError < (-1 * pi)) {
        angularError += 2 * pi;
      }


      if (state) {
        pos_I += Ts * positionalError;
        pos_u = pos_Kp * positionalError + pos_I * pos_Ki;
        ang_I += Ts * angularError;
        ang_u = ang_Kp * angularError + ang_I * ang_Ki;

        if (abs(pos_u) > pos_max) {  //low ang error
          pos_u = sgn(pos_u) * pos_max;
          pos_I = (pos_u - pos_Kp * positionalError) / pos_Ki;
        }

        if (abs(ang_u) > umax_ang) {
          ang_u = sgn(ang_u) * umax_ang;
          ang_I = (ang_I - ang_Ki * angularError) / ang_Ki;
        }

        if (pos_u > 0) {
          //analogWrite(9,0);
          //analogWrite(10,0);
          digitalWrite(7, LOW);
          digitalWrite(8, LOW);
        }


        else {
          //analogWrite(9,0);
          //analogWrite(10,0);
          digitalWrite(7, HIGH);
          digitalWrite(8, HIGH);
        }
        if (true) {
          analogWrite(9, abs(pos_u + (ang_u * 1.5)) * 255);  //12.34
          analogWrite(10, abs(pos_u - (ang_u * 1.5)) * 255);
        }


      } else {  //high angle difference, turn mode
        //ang_u = ang_Kp*angularError;
        ang_I += Ts * angularError;
        ang_u = ang_Kp * angularError + ang_I * ang_Ki;
        if (abs(ang_u) > umax_ang) {
          ang_u = sgn(ang_u) * umax_ang;
          ang_I = (ang_I - ang_Ki * angularError) / ang_Ki;
        }
        //Perform different motor commands depending on if error is positive or negative
        if (angularError < 0) {
          analogWrite(9, (abs(ang_u) * 255));  //12.34
          analogWrite(10, (abs(ang_u) * 255));
          digitalWrite(7, HIGH);
          digitalWrite(8, LOW);
        } else {
          analogWrite(9, (abs(ang_u) * 255));  //12.34
          analogWrite(10, (abs(ang_u) * 255));
          digitalWrite(7, LOW);
          digitalWrite(8, HIGH);

        }
      }


      Ts = millis() - Tc;
      Tc = millis();
    }


    //Serial.println(angularError);
  } else {


    analogWrite(9, (0));  //12.34
    analogWrite(10, (0));


  }

}
