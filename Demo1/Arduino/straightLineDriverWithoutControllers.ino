#include <DualMC33926MotorShield.h>
#include <Encoder.h>

DualMC33926MotorShield motorShield;
Encoder leftEncoder(3, 6);
Encoder rightEncoder(2, 5);

const float WHEEL_DIAMETER = 6.0; // inches
const float WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * PI;
const float DISTANCE_TO_MOVE = 6.0; // inches
const float ENCODER_TICKS_PER_REV = 3200.0; // 360 * 4 for 4x decoding

void setup() {
    Serial.begin(9600);

  motorShield.init();
  leftEncoder.write(0);
  rightEncoder.write(0);
      //Serial.println("in setup ");

}

void loop() {
  float distanceMoved = (leftEncoder.read() + rightEncoder.read()) / 2.0 * (WHEEL_CIRCUMFERENCE / ENCODER_TICKS_PER_REV);
  if (distanceMoved < DISTANCE_TO_MOVE) {
    Serial.println("Motor started");
    motorShield.setSpeeds(100, 100); // set motor speeds to move robot forward
  } else {
    motorShield.setSpeeds(0, 0); // stop motors once distance moved is reached
    Serial.println("Motor stoped");
  }
}
