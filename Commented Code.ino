#include "DualMC33926MotorShield.h"
#include "Encoder.h"

#include "Wire.h"

#define sampleTime 10
#define ADDR 0x8

DualMC33926MotorShield ms; 
Encoder encoder(2,5);

//Variables for holding received data
byte data[32] = {0};
byte value = 0;

//const double Ki = 0.156633991202905;
double Ki = .07, Kp = 15.8; //Not the same from matlab
//const double Kp = 3.18865312201695;
double setpoint = 0, I = 0, e = 0, Tc = 0, Ts = 0, u =0;
double curEnc = 0, prevEnc = 0;
double const umax = 7;

void setup() {
  ms.init(); 
  Serial.begin(115200);

  Wire.begin(ADDR);
  delay(1000);
  Serial.print("------------ I am Arduino Uno I2C at address 0x");
  Serial.print(ADDR);
  Serial.println(" ------------");
  delay(1000);

  Wire.onRequest(requestData1);
  Wire.onReceive(receiveData1);
  
  pinMode(4,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);
  pinMode(9,OUTPUT);
  pinMode(10,OUTPUT);
  digitalWrite(7,HIGH);
}

void loop() {



  // This if statement is so that it only runs for a set period of time, that value is sampleTime.

  if(millis() >= Tc + sampleTime){
    
    //encoder.read() will read the current position of the encoder, it does it by counts, the counts per rotation is 3200, so if you move it by pi/2, it will count 800.

    curEnc = encoder.read();

    // e = error
    // setpoint is where we are at. Setpoint is determined by the function void receiveData1 below. This function will occur when the arduino senses data being transferred
    // from the raspberry pi, dont worry about it. This is chris's portion. The current encoder is being converted to radians, instead of counts. Subtracting the setpoint by the 
    // current encoder in radians will give us the error.

    e = setpoint-curEnc/3200*6.28;

      // This is feedback equations

      I += Ts*e;
      u = Kp*e+Ki*I;

      // u is the actual position in radians. If the absolute value of u is greater than umax which is 7?, then it will be multiplied by umax.

      if(abs(u) > umax){
        u = sgn(u)*umax;
        I = (u-Kp*e)/Ki;
      }

      // If the desired position in radians is less than 0 (negative),
      // analogWrite(9,0) will make the wheel stop, since it is analog and the max PWM is 0-255. This controls the speed
      // digitalWrite(7,LOW) makes the direction negative, so clockwise.
      // This means that this section only changes the direction and gets it ready for the code after the if statements.

      if( u < 0){
        analogWrite(9,0);
        digitalWrite(7,LOW);
      }

      // If the desired position in radians is greater than 0 (positive),
      // analogWrite(9,0) will make the wheel stop, since it is analog and the max PWM is 0-255. This controls the speed
      // digitalWrite(7,HIGH) makes the direction positive, so counter-clockwise.
      // This means that this section only changes the direction and gets it ready for the code after the if statements.

      if( u >= 0){
        analogWrite(9,0);
        digitalWrite(7,HIGH);
      }

      //analogWrite(9, u*12.34);

            
      // This code is where the wheel actually starts moving. Pin 9 controls the speed of the motor, so the speed of the 
      // motor is absolute value of u * 255 / 12.34. Not sure why they divided by 12.34

      
      analogWrite(9,abs(u)*255/12.34);
      Ts = millis()-Tc;
      Tc = millis();
    }
}

void requestData1() {
  Serial.println("---> Recieved Requests");
  Serial.println("Sending Back New Position");
  Serial.print("Sending value : ");
  Serial.println(value);
  Wire.write(value);
}

void receiveData1(int numBytes) {
  if(numBytes > 1) {
    int i = 0;
    while (Wire.available()) {
      data[1] = Wire.read();
      i++;
    }
  
    Serial.println(F("---> Recieved Events"));
    Serial.print(F("Recieved value : "));
    Serial.println(data[1]);
  
    value = int(data[1]);

    switch(value){
      case 1:
        setpoint = 0;
        Serial.println(setpoint);
        break;
      case 2:
        setpoint = 1.57;
        Serial.println(setpoint);
        break;
      case 3:
        setpoint = 3.14;
        Serial.println(setpoint);
        break;
      case 4:
        setpoint = 4.71;
        Serial.println(setpoint);
        break;
    } 
  }  
}

int sgn(float num){
  int sign = 1;
  if(num >= 0){
    sign = 1;
  } else {
    sign = -1;
  }
  return sign;
}