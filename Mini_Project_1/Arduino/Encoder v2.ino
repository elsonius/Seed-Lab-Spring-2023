/* Encoder Library - TwoKnobs Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * This example code is in the public domain.
 */

#include <Encoder.h>

// Change these pin numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder knobLeft(3, 4);
Encoder knobRight(7, 8);
//   avoid using pins with LEDs attached

void setup() {
  Serial.begin(9600);
  Serial.println("TwoKnobs Encoder Test:");
}

#define PI 3.1415926535897932384626433832795


long positionLeft  = -999;
long positionRight = -999;
float angular_position = 0.5;
int difference;
float previous_state_velocity = 0;
float beginning = 0;
const int period = 10000;
int time_now = 0;
int time_now_now = 0;
int difference_2 = 0;
float angular_velocity = 0.0;
float time_previous = 0.0;
float milliseconds;
float previous_increment = 0.0;
int counter = 0;


void loop() {

  time_now = millis();

  while(millis() < time_now + period) {

  long newLeft, newRight;
  newLeft = knobLeft.read();
  newRight = knobRight.read();
  
  if (newLeft != positionLeft || newRight != positionRight) {
    
    Serial.print("Left = ");
    Serial.print(newLeft);
    Serial.print(", Right = ");
    Serial.print(newRight);    
    Serial.println();

    difference = newLeft - positionLeft;
    counter = counter + difference;

    positionLeft = newLeft;
    positionRight = newRight;
  
  }

  if (Serial.available()) {
    Serial.read();
    Serial.println("Reset both knobs to zero");
    knobLeft.write(0);
    knobRight.write(0);
  }

    if (counter >= 800) {

      counter = 0;
      if (angular_position == 0) {
        angular_position = 1.5;  
      } else {
      angular_position = angular_position - 0.5;
      }

      Serial.print("Angular Position = ");
      Serial.print(angular_position);
      Serial.println("pi");

      angular_velocity = (abs((angular_position*PI)-(previous_increment*PI)))/(difference_2/1000);

      Serial.print("rad/s = ");
      Serial.println(angular_velocity);
      previous_increment = angular_position;
      angular_velocity = 0;
      
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
      
      Serial.print("Angular Position = ");
      Serial.print(angular_position);
      Serial.println("pi"); 
      
      angular_velocity = (abs((angular_position*PI)-(previous_increment*PI)))/(difference_2/1000);

      Serial.print("rad/s = ");
      Serial.println(angular_velocity);
      previous_increment = angular_position;
      angular_velocity = 0;
  
    
  
  // if a character is sent from the serial monitor,
  // reset both back to zero.
    }
  time_previous = time_now;
  angular_velocity = 0;  
}
}
