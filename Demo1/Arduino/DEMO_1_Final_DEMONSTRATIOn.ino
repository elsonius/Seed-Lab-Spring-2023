#include <DualMC33926MotorShield.h>
#include <Encoder.h>

DualMC33926MotorShield motorShield;
Encoder leftEncoder(3, 5);
Encoder rightEncoder(2, 6);

//double position_inches = 24;
//double angle_radians = -1.57;


double angle_radians = 0;
double position_inches = 80;

double rotation_time = 0;
double rotation_time_counter = 0;
double previous_millis = 0;

int negative_check = 0;

double straight_line_previous = 0;
double straight_line_current = 0;
double straight_line_counter = 0;


double straight_line_time;

void setup() {

  angle_radians = angle_radians;
  pinMode(4, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(12, INPUT);
  digitalWrite(4, HIGH);
  digitalWrite(7, HIGH);
  digitalWrite(8, HIGH);
  Serial.begin(9600);
  Serial.println("Begin");
  if (angle_radians < 0) {
    negative_check = 1;
    angle_radians = angle_radians * -1;
  }
  delay(10);
}

void loop() {
    while (rotation_time_counter <= (angle_radians - 0.03)) {
      rotation_time = (millis() / 1000.0);
      rotation_time_counter = rotation_time_counter + (rotation_time - previous_millis);
      Serial.println(rotation_time_counter);
      if (negative_check == 0) {
        digitalWrite(7, HIGH);
        digitalWrite(8, LOW);
        analogWrite(9, 57);
        analogWrite(10, 57);


      }

      if (negative_check == 1) {
        digitalWrite(7, LOW);
        digitalWrite(8, HIGH);
        analogWrite(9, 60);
        analogWrite(10, 60);
      }
      previous_millis = rotation_time;
    }
    analogWrite(9, 0);
    analogWrite(10, 0);
    delay(1000);

  straight_line_previous = (millis() / 1000.0);

  while (straight_line_counter <= (position_inches / 8)) {
    straight_line_current = (millis() / 1000.0);
    straight_line_counter = straight_line_counter + (straight_line_current - straight_line_previous);
    Serial.println(straight_line_counter);
    digitalWrite(7, LOW);
    digitalWrite(8,LOW);
    analogWrite(9, 90);
    analogWrite(10, 68);
    straight_line_previous = straight_line_current;
  }
  analogWrite(9, 0);
  analogWrite(10, 0);
}
