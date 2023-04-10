#define MAX_VOLTAGE       8.1       
#define CIRCLE_RADIUS     0.4       
#define CIRCLE_TIME       3.0              
#define CIRCLE_THRESH_DIS 0.05      
#define CIRCLE_THRESH_ROT 2.00      
#define CIRCLE            2.5233    
#define SLAVE_ADDRESS     0x04

#include <Encoder.h>
#include <Wire.h>

#define SAMPLE_TIME 30.0     
#define WHEEL_RADIUS 0.07485   

#define WHEEL_DISTANCE  0.312
double WHEEL_DISTANCE1 = 0.312; 
double WHEEL_DISTANCE2 = 0.312; //circle

#define RAD_IN_DEG      0.01745329
#define METERS_IN_FEET  0.3048 

double KP_DIS = 0.00;
double KI_DIS = 25.0;
double KP_ROT = 0.00;
double KI_ROT = 0.20;
#define KP_DIS1         0.00
#define KI_DIS1         25.0
#define KP_ROT1         0.00
#define KI_ROT1         0.20 
#define KP_DIS2         0.00
#define KI_DIS2         30.0
#define KP_ROT2         0.00
#define KI_ROT2         0.10

#define KPO_DIS         2.00
#define KDO_DIS         0.88
#define SPEED_SAT_DIS   0.75
#define KPO_ROT         1.50
#define KDO_ROT         0.00
#define SPEED_SAT_ROT   90.0
double actPos_dis = 0;
double actPos_rot = 0;
double desPos_dis = 0;
double desPos_rot = 0;
double errorPos_dis = 0;
double errorPos_rot = 0;
double actSpeed_dis = 0;
double actSpeed_rot = 0;
double desSpeed_dis = 0;
double desSpeed_rot = 0;
double errorSpeed_dis = 0;
double errorSpeed_rot = 0;
double errorSpeedSum_dis = 0;
double errorSpeedSum_rot = 0;
double errorPosOld_dis = 0;
double errorPosOld_rot = 0;
double errorPosChange_dis = 0;
double errorPosChange_rot = 0;
#define CHANNEL_RA      2     
#define CHANNEL_LA      3     
#define CHANNEL_RB      5     
#define CHANNEL_LB      6     
#define ENABLE          4     
#define DIRECTION_R     7    
#define DIRECTION_L     8    
#define SPEED_R         9   
#define SPEED_L         10    
Encoder rightEnc(CHANNEL_RA, CHANNEL_RB);
Encoder leftEnc(CHANNEL_LA, CHANNEL_LB); 
double distance_count;
int state = 0;
int finState = 0;
byte dataRec[10] = {0};
byte outVal[2] = {0};
bool control[5] = {false, false, false, false, false};
long angle = 0;
long distance = 0;
// EXPERIMENTAL
void rotate() {
  errorSpeedSum_dis = 0;
  errorSpeedSum_rot = 0;
  control[0] = true;
  control[1] = true;
  control[2] = true;
  control[3] = true;
  control[4] = false;
  desPos_rot = actPos_rot - 90;
}
void aim(long aimAng) {
  //errorSpeedSum_dis = 0;
  //errorSpeedSum_rot = 0;
  control[0] = true;
  control[1] = true;
  control[2] = true;
  control[3] = true;
  control[4] = false;
  desPos_rot = (((double)aimAng) / 50);
}
void search(long searchSpeed) {
  //errorSpeedSum_dis = 0;
  //errorSpeedSum_rot = 0;
  control[0] = true;
  control[1] = false;
  control[2] = true;
  control[3] = true;
  control[4] = false;
  desSpeed_rot = ((double)searchSpeed) / 100;
}


// aim - fine tunes the angle of the robot to match the marker
// input is in deg*100
// Ex: input of 10 means 0.1 degrees


// drive - drives the robot a specific distance
// input is in millimeters
void drive (long driveDis) {
  distance_count = distance_count + 1;  
  if (distance_count >= 2) {
    kill();
  }  
  errorSpeedSum_dis = 0;
  errorSpeedSum_rot = 0;
  control[0] = true;
  control[1] = true;
  control[2] = true;
  control[3] = true;
  control[4] = false;
  desPos_dis = actPos_dis + (((double)driveDis) / 1000);
  desPos_rot = actPos_rot;    
     
  Serial.println("ASDASDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD");                       //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
}

void rotate() {
  errorSpeedSum_dis = 0;
  errorSpeedSum_rot = 0;
  control[0] = true;
  control[1] = true;
  control[2] = true;
  control[3] = true;
  control[4] = false;
  desPos_rot = actPos_rot - 90;
}


// circle - drives the robot in a predifined circle
// no input
void circle() {
  KP_DIS = KP_DIS2;
  KI_DIS = KI_DIS2;
  KP_ROT = KP_ROT2;
  KI_ROT = KI_ROT2;
  errorSpeedSum_dis = 0;
  errorSpeedSum_rot = 0;
  control[0] = false;
  control[1] = false;
  control[2] = true;
  control[3] = true;
  control[4] = true;
  desSpeed_dis = 3.14159265 * (((double)CIRCLE_RADIUS) / ((double)CIRCLE_TIME));
  desSpeed_rot = 180.0 / ((double)CIRCLE_TIME);
}

// correct - corrects robot position to past desired distance and rotation
// no input
void correct() {
  KP_DIS = KP_DIS1;
  KI_DIS = KI_DIS1;
  KP_ROT = KP_ROT1;
  KI_ROT = KI_ROT1;

  errorSpeedSum_dis = 0;
  errorSpeedSum_rot = 0;
  control[0] = true;
  control[1] = true;
  control[2] = true;
  control[3] = true;
  control[4] = false;
  desPos_dis += (CIRCLE - 0.05);
  desPos_rot += 350;
}

// kill - stops robot from doing anything else
// no input
void kill() {
  control[0] = false;
  control[1] = false;
  control[2] = false;
  control[3] = false;
  control[4] = false;
  digitalWrite(ENABLE, LOW);
}


void setup() {

  // serial communication initialization
  Serial.begin(115200);

  
  //initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);
  
  // assigns pins as either inputs or outputs
  pinMode(CHANNEL_RA, INPUT);
  pinMode(CHANNEL_LA, INPUT);
  pinMode(CHANNEL_RB, INPUT);
  pinMode(CHANNEL_LB, INPUT);
  pinMode(ENABLE, OUTPUT);
  pinMode(DIRECTION_R, OUTPUT);
  pinMode(DIRECTION_L, OUTPUT);
  pinMode(SPEED_R, OUTPUT);
  pinMode(SPEED_L, OUTPUT);

  // enables motors
  digitalWrite(ENABLE, HIGH);

  // writes direction to motor
  digitalWrite(DIRECTION_R, LOW);
  digitalWrite(DIRECTION_L, LOW);

  // EXPERIMENTALLLLLLLLLL
  //analogWrite(SPEED_R,25);
  //analogWrite(SPEED_L,25);

  static unsigned long currentTime = 0;
  static unsigned long correctTime = 0;
  static double newDeg_R = 0;
  static double newDeg_L = 0;
  static double oldDeg_R = 0;
  static double oldDeg_L = 0;
  static double angVel_R = 0;
  static double angVel_L = 0;
  static double sumVoltage = 0;
  static double difVoltage = 0;
  static double rightVoltage = 0;
  static double leftVoltage = 0;

  // measures time for delay
  currentTime = millis();

double experimental_past_time;
double experimental_now_time;

} 


void loop() {

switch(state){
  case 1:

    search(2000);
    break;
  case 2:

    // original = aim(angle + 500)

    aim(angle*0.8); // input is desired angle in degress*100
    //if (abs(desPos_rot) <= 1){ 



    finState = 1;

    //break;
  case 3:

Serial.println("33333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333");  
    finState = 0;
    //experimental
    drive(distance/1.95); //distance in mm
    //experimental = drive(distance - 1650);
    // original = state = 4;
   
    state = 4;


    break;
    
    
  case 4:

Serial.println("4444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444");



    // original error = 0.01
    if(abs(errorPos_dis) <= 0.01){

      rotate();
      state = 5;
    }
    break;
  case 5:

Serial.println("55555555555555555555555555555555555555555555555555555555555555555555555555555555555555");

    // original error = 1.0
    if(abs(errorPos_rot) <= 1.0){
      kill();
      circle();
      state = 0;
    }
    break;
};

  newDeg_R = ((double)rightEnc.read() * 360) / 3200;
  newDeg_L = -((double)leftEnc.read() * 360) / 3200;
  actPos_dis = WHEEL_RADIUS * 0.5 * (newDeg_R + newDeg_L) * RAD_IN_DEG;
  actPos_rot = (WHEEL_RADIUS / WHEEL_DISTANCE) * (newDeg_R - newDeg_L);
  angVel_R = (1000 * (newDeg_R - oldDeg_R)) / SAMPLE_TIME;
  angVel_L = (1000 * (newDeg_L - oldDeg_L)) / SAMPLE_TIME;
  actSpeed_dis = WHEEL_RADIUS * 0.5 * (angVel_R + angVel_L) * RAD_IN_DEG;
  actSpeed_rot = (WHEEL_RADIUS / WHEEL_DISTANCE) * (angVel_R - angVel_L);

  // distance outer-loop control
  if (control[0]) {
    errorPos_dis = desPos_dis - actPos_dis;
    errorPosChange_dis = ((errorPos_dis - errorPosOld_dis) * 1000.0) / SAMPLE_TIME;
    errorPosOld_dis = errorPos_dis;
    desSpeed_dis = (errorPos_dis * KPO_DIS) + (errorPosChange_dis * KDO_DIS);
    if (desSpeed_dis > SPEED_SAT_DIS) desSpeed_dis = SPEED_SAT_DIS;
    else if (desSpeed_dis < -SPEED_SAT_DIS) desSpeed_dis = -SPEED_SAT_DIS;
  } 
  
  // rotation outer-loop control
  if (control[1]) {
    errorPos_rot = desPos_rot - actPos_rot;
    errorPosChange_rot = ((errorPos_rot - errorPosOld_rot) * 1000.0) / SAMPLE_TIME;
    errorPosOld_rot = errorPos_dis;
    desSpeed_rot = (errorPos_rot * KPO_ROT) + (errorPosChange_rot * KDO_ROT);
    if (desSpeed_rot > SPEED_SAT_ROT) desSpeed_rot = SPEED_SAT_ROT;
    else if (desSpeed_rot < -SPEED_SAT_ROT) desSpeed_rot = -SPEED_SAT_ROT; 
  }

  // distance inner-loop control
  if (control[2]) {
    errorSpeed_dis = desSpeed_dis - actSpeed_dis;
    errorSpeedSum_dis += (errorSpeed_dis * SAMPLE_TIME) / 1000.0;
    sumVoltage = (errorSpeedSum_dis * KI_DIS) + (errorSpeed_dis * KP_DIS);
  }

  // rotation inner-loop control
  if (control[3]) {
    errorSpeed_rot = desSpeed_rot - actSpeed_rot;
    errorSpeedSum_rot += (errorSpeed_rot * SAMPLE_TIME) / 1000.0;
    difVoltage = (errorSpeedSum_rot * KI_ROT) + (errorSpeed_rot * KP_ROT);
  }

  // circle control
  if (control[4]) {
    if (abs(actPos_rot - (desPos_rot + 360.0)) <= CIRCLE_THRESH_ROT){
      correct();
    }
  }

  if (state == 1 && (actPos_rot >= 360)) {
    rightEnc.write(0);
    leftEnc.write(0);
    newDeg_R = 0;
    newDeg_L = 0;
    oldDeg_R = 0;
    oldDeg_L = 0;
    angVel_R = 0;
    angVel_L = 0;
    sumVoltage = 0;
    difVoltage = 0;
  }
  
  // determines individual voltages
  rightVoltage = 0.5 * (sumVoltage + difVoltage);
  leftVoltage = 0.5 * (sumVoltage - difVoltage);
  if (leftVoltage > MAX_VOLTAGE) leftVoltage = MAX_VOLTAGE;
  else if (leftVoltage < -MAX_VOLTAGE) leftVoltage = -MAX_VOLTAGE;
  if (rightVoltage > MAX_VOLTAGE) rightVoltage = MAX_VOLTAGE;
  else if (rightVoltage < -MAX_VOLTAGE) rightVoltage = -MAX_VOLTAGE;

  // determines direction of each motorr and then writes to the motor
  if(rightVoltage < 0) digitalWrite(DIRECTION_R, HIGH);
  else digitalWrite(DIRECTION_R, LOW);
  if(leftVoltage < 0) digitalWrite(DIRECTION_L, HIGH);
  else digitalWrite(DIRECTION_L, LOW); 
  analogWrite(SPEED_R, abs(rightVoltage*255/ MAX_VOLTAGE));
  analogWrite(SPEED_L, abs(leftVoltage*255/ MAX_VOLTAGE));

  // displays samples for debugging purposes
  Serial.print((double)currentTime / 1000); // sample time in seconds
  Serial.print("\t");
  Serial.print(actPos_dis, 4);
  Serial.print("\t");
  Serial.print(actPos_rot, 2);
  Serial.print("\t\t");
  Serial.print((double)distance/1000, 4);
  Serial.print("\t");
  Serial.print((double)angle/100, 2);
  Serial.print("\t\t");
  Serial.print(state);
  Serial.print("\n\r");

  // reassigns old degree variables
  oldDeg_R = newDeg_R;
  oldDeg_L = newDeg_L;

  // ensures function isn't taking too long
  if (millis() > (currentTime + SAMPLE_TIME)) Serial.println("ERROR: Under Sampling!");
  
  // creates delay of SAMPLE_TIME ms
  while(millis() < (currentTime + SAMPLE_TIME));
  
} // end of loop



//i2c communication functions
void receiveData(int byteCount) {
  int k = 0;
  while (Wire.available()) {
    dataRec[k] = Wire.read();
   
    k++;
  }
  state = dataRec[1];
  distance = (dataRec[2] << 8 | dataRec[3]);
  angle = (dataRec[4] << 8 | dataRec[5]);
  if(dataRec[6] == 1){
    angle = -1* angle;
  }
  
}
void sendData(){

 if (state < 2){
  int sendRot = int(actPos_rot *100);
  outVal[0] = sendRot >>8;
  outVal[1] = sendRot & 0x00FF;

  Wire.write(outVal,2);
 }
 if (state == 2){
  outVal[0] = finState;
  outVal[1] = 0;

  //Serial.println();
  Wire.write(outVal, 1);
 }
}

