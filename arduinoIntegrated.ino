//this program integrates the controller design with the localization program

//library declarations
# include <Encoder.h>
#include "Wire.h"

// This is a fcuntion from the encoder library, it sets up an interrupt automatically
Encoder knobLeft(3, 5);
Encoder knobRight(7, 8);

//type and variable declerations
const int period = 10;
long positionLeft = -999;
long positionRight = -999;
float angular_position = 0.5;
float difference = 0;
float previous_state_velocity = 0;
float beginning = 0;
float angular_velocity = 0.0;
float time_previous = 0.0;
float milliseconds;
float previous_increment = 0.0;
float new_angular_velocity = 0;
float counter_per_rotation = 3200.0;
float ms_to_s = 1000.0;
int time_now = 0;
int time_now_now = 0;
int difference_2 = 0;
int counter = 0;
long newLeft, newRight;


//type and variable declerations for feedback control
// PID Controller Constants
double Kp = 8.05114690297919;  // Proportional gain
double Ki = 0.716404922216221;  // Integral gain
double Kd = 0;  // Derivative gain

// Variables for PID controller
double setpoint = 0;  // Desired position/setpoint
double input; //actual position
double outputControl; //angular motor position
double error, last_error;
double integral, derivative;
double I = 0;
double e_past = 0;
double Tc = 0; //current time
double current_time = Tc;
double Ts = 0; //elapsed time

// Other variables
unsigned long last_time;
double sample_time = 100;  // Sample time in milliseconds
double out_min = 0;
double out_max = 255;

//sign function for controlling the sign of the controller output
int sgn(float num){
  int sign = 1;
  if(num >= 0){
    sign = 1;
  } else {
    sign = -1;
  }
  return sign;
}

//Determine setpoint
void rasberryData(int numberOfBytes) {
  if(numberOfBytes > 1) {
    int i = 0;
    while (Wire.available()) {
      data[1] = Wire.read();
      i++;
    }
    value = int(data[1]);
    switch(value){
      case 1:
        setpoint = 0;
        //Serial.println(setpoint);
        break;
      case 2:
        setpoint = 1.57;
        //Serial.println(setpoint);
        break;
      case 3:
        setpoint = 3.14;
        //Serial.println(setpoint);
        break;
      case 4:
        setpoint = 4.71;
        //Serial.println(setpoint);
        break;
    } 
  }  
}

void setup() {
  Serial.begin(9600); // Use a baud rate of 9600 or else the program breaks
  pinMode(4,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);
  pinMode(9,OUTPUT);
  pinMode(10,OUTPUT);
  pinMode(12,INPUT);
  digitalWrite(4,HIGH);
}

void loop() {

  // Calculate time since last sample
  unsigned long now = millis();
  double elapsed_time = (double)(now - last_time);

  if(now >= current_time + period){
    input = knobLeft.read();
    // Calculate error: desired position minus actual position
    error = setpoint - input/3200*6.28;
     // Calculate integral terms
    integral += (error * elapsed_time);
     // Calculate controller output (actual position)
    output = (Kp * error) + (Ki * integral) + (Kd * derivative); 

    if(abs(outputControl) > out_max){
      output = sgn(outputControl) * out_max;
      integral = (outputControl - Kp*error)/Ki;      
    }
    if( outputControl < 0){
      analogWrite(9,0);
      digitalWrite(7,LOW);
    }
    if( outputControl >= 0){
      analogWrite(9,0);
      digitalWrite(7,HIGH);
    }
    analogWrite(9,abs(outputControl)*255/12.34);
    elapsed_time = millis()-Tc;
    current_time = millis();
    }
  }



}