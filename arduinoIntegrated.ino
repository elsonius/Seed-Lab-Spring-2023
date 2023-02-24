//this program integrates the controller design with the localization program

//library declarations
# include <Encoder.h>

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

//type and variable declerations for feedback control
// PID Controller Constants
double Kp = 8.05114690297919;  // Proportional gain
double Ki = 0.716404922216221;  // Integral gain
double Kd = 0;  // Derivative gain

// Variables for PID controller
double setpoint = 0;  // Desired position/setpoint
double input; //actual position
double output; //angular motor position
double error, last_error;
double integral, derivative;
double I = 0;
double e_past = 0;
double Tc = 0;
double Ts = 0; //elapsed time

// Other variables
unsigned long last_time;
double sample_time = 100;  // Sample time in milliseconds
double out_min = 0;
double out_max = 255;



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

//control portion
  // Calculate time since last sample
  unsigned long now = millis();
  double elapsed_time = (double)(now - last_time);

  // Read output: Read input data, current value of system output
  input = analogRead(A0);

  // Calculate error: desired position minus actual position
  error = setpoint - input;

if(elapsed_time > 0){

  // Calculate derivative terms
  derivative = (error - last_error) / elapsed_time;

  // Save current error for next loop
  last_error = error;
  
} else{
  derivative = 0;
}

  // Calculate integral terms
  integral += (error * elapsed_time);

  // Calculate controller output
  output = (Kp * error) + (Ki * integral) + (Kd * derivative);

  // Constrain output within minimum and maximum values
  output = constrain(output, out_min, out_max);

  // Output to the system (Set output to u (for example, setting a voltage to u using a digital to analog element)
  analogWrite(3, output);

  //set elapsed time to current time minus 

  // Save current time for next loop
  last_time = now;

  // Wait for next sample time
  delay(sample_time);








  if(millis() >= 1000 && millis() <= 1100){
    analogWrite(9,255);
    digitalWrite(7,LOW);
  }

  // This will make it so that it outputs data and runs for the 10ms period.
  if (millis() > time_now + period) {

    time_now = millis();

    // motor code for later

    // The .read functions will read the encoder in its current state

    long newLeft, newRight;
    newLeft = knobLeft.read();
    newRight = knobRight.read();

    // difference is the new position it is as at minus the old position of the motor

    difference = 2*PI*(newLeft - positionLeft)/3200 ;    
    counter = counter + difference;

    // To find the angular velocity, we multiply the difference of the two positions by 1000, since we are dividing by
    // ms. We divide by the time the new position is at minus the time the old position was at to get the angualar velocity
    // create a tab so that it can easily be copied into matlab.

    new_angular_velocity = 1000 * (difference) / (millis() - time_previous);
    Serial.print(millis()/ms_to_s);
    Serial.print("\t");    
    Serial.println(new_angular_velocity);    

    // We then set the old positions to the current one so we can get ready to loop again.

    positionLeft = newLeft;
    positionRight = newRight;
    time_previous = time_now;   
  }

  // This is the code that resets the angular_position of the wheel, changing the reference if not set to 0pi.
  if (Serial.available()) {
    Serial.read();
    Serial.println("Reset both knobs to zero");
    knobLeft.write(0);
    knobRight.write(0);
  }
}

















/*
  if (counter >= 800) {

    counter = 0;
    if (angular_position == 0) {
      angular_position = 1.5;
    } else {
      angular_position = angular_position - 0.5;
    }

    //Serial.print("Angular Position = ");
    //Serial.print(angular_position);
    //Serial.println("pi");

    angular_velocity = (abs((angular_position * PI) - (previous_increment * PI))) / (difference_2 / 1000);

    //Serial.print("rad/s = ");
    //Serial.println(angular_velocity);
    //previous_increment = angular_position;
    //angular_velocity = 0;
  }

  if (counter <= -800) {
    time_now_now = millis();
    difference_2 = time_now_now - time_previous;
    counter = 0;
    angular_position = angular_position + 0.5;
    if (angular_position == 2) {
      angular_position = 0;
      previous_increment = 0.5;
    }

    //Serial.print("Angular Position = ");
    //Serial.print(angular_position);
    //Serial.println("pi");

    //angular_velocity = (abs((angular_position*PI)-(previous_increment*PI)))/(difference_2/1000);

    //Serial.print("rad/s = ");
    //Serial.println(angular_velocity);
    //previous_increment = angular_position;
    //angular_velocity = 0;



    // if a character is sent from the serial monitor,
    // reset both back to zero.
  }
  
  angular_velocity = 0;
}
*/