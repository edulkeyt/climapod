#include <Servo.h>
#include <Adafruit_PWMServoDriver.h>


/*
* 
************************************************************ 
*      https://goo.gl/photos/xvHwKM44gh7wN5UK6             *
*      https://www.youtube.com/watch?v=PL4174oBPTs         *
*************************************************************
*
*/



float TODEG = 57.3; // Conversion from rad to deg


// ======================== Servos & distances ===================================

// Distances of the leg
float coxa =  3.7;  // cm
float femur = 5.9;  // cm
float tibia = 9.45;  // cm

// Servos
// Servo S_RF1;
// Servo S_RF2;
// Servo S_RF3;

// Servo S_LF1;
// Servo S_LF2;
// Servo S_LF3;

// Servo S_RB1;
// Servo S_RB2;
// Servo S_RB3;

// Servo S_LB1;
// Servo S_LB2;
// Servo S_LB3;

// Coords of each leg
double RFx, RFy, RFz;
double LFx, LFy, LFz;
double RBx, RBy, RBz;
double LBx, LBy, LBz;

// Offset angles of each servo
double offset1RF = 135;
double offset2RF = 180;
double offset3RF  = 45;

double offset1LF = 45;
double offset2LF = 0;
double offset3LF = 225;

double offset1LB = 135;
double offset2LB = 180;
double offset3LB = 45;

double offset1RB = 45;
double offset2RB = 0;
double offset3RB = 225;

Adafruit_PWMServoDriver pwmDriver = Adafruit_PWMServoDriver(0x40);

// ===================================================================================


// ======================== Setup ====================================================
void setup() {
  //Serial.begin(115200);
  
  // S_RF1.attach(2);
  // S_RF2.attach(3);
  // S_RF3.attach(4);
  // S_LF1.attach(5);
  // S_LF2.attach(6);
  // S_LF3.attach(7);
  // S_RB1.attach(8);
  // S_RB2.attach(9);
  // S_RB3.attach(10);
  // S_LB1.attach(11);
  // S_LB2.attach(12);
  // S_LB3.attach(13);

  pwmDriver.begin();
  pwmDriver.setPWMFreq(60);

  
  // Init coords
  RFx = 8;
  RFy = 8;
  RFz = 8;
  
  LFx = 8;
  LFy = 8;
  LFz = 8;

  RBx = 8;
  RBy = 8;
  RBz = 8;

  LBx = 8;
  LBy = 8;
  LBz = 8;

  movLegs(RFx,RFy,RFz, RBx,RBy,RBz, LFx,LFy,LFz, LBx,LBy,LBz, 10,10);
}
// ===================================================================================



// ======================== Movements methods =======================================

void FullWalk1(int div, int time){
  movLegs(8,7,RFz,      8,2,15,     8,7,LFz,        8,7,LBz,    div,time);
  movLegs(8,7,RFz,      8,2,6,      8,7,LFz,        8,7,LBz,    div,time);
  movLegs(8,7,RFz,      8,7,0,      8,7,LFz,        8,7,LBz,    div,time);
  
  movLegs(8,2,2,        8,7,RBz,    8,7,LFz,        8,7,LBz,    div,time);
  movLegs(8,2,8,        8,7,RBz,    8,7,LFz,        8,7,LBz,    div,time);
  movLegs(8,7,18,       8,7,RBz,    8,7,LFz,        8,7,LBz,    div,time);
  
  movLegs(8,7,RFz,      8,7,RBz,    8,7,LFz,        8,2,15,     div,time);
  movLegs(8,7,RFz,      8,7,RBz,    8,7,LFz,        8,2,6,      div,time);
  movLegs(8,7,RFz,      8,7,RBz,    8,7,LFz,        8,7,0,      div,time);
  
  movLegs(8,7,RFz,      8,7,RBz,    8,2,2,          8,7,LBz,    div,time);
  movLegs(8,7,RFz,      8,7,RBz,    8,2,8,          8,7,LBz,    div,time);
  movLegs(8,7,RFz,      8,7,RBz,    8,7,18,         8,7,LBz,    div,time);
}



void walk2(int div, int time){

  movLegs(9,2,5.5,      9,7,7, 	   	9,7,7,      9,2,8.5, div,time);
  movLegs(9,7,10,       9,7,10,    	9,7,4,     	9,7,4,   div,time);
  movLegs(9,7,7,        9,2,8.5,    9,2,5.5,    9,7,7,   div,time);
  movLegs(9,7,4,        9,7,4,      9,7,10,     9,7,10,  div,time);
  
}
// ===================================================================================



// ======================== Methods to move legs =============================================

// Linearly move all legs to the given coordinates
void movLegs(double t_RFx, double t_RFy, double t_RFz, double t_RBx, double t_RBy, double t_RBz, double t_LFx, double t_LFy, double t_LFz, double t_LBx, double t_LBy, double t_LBz, int div, int time){
  
  // div -> divisions (number of movements to reach the new coordinates)
  // time-> time between each sub-movement

  double subxRF = (t_RFx - RFx)/div;  // Increment to be added to the current position to reach the destination in "div" movements
  double subyRF = (t_RFy - RFy)/div;
  double subzRF = (t_RFz - RFz)/div;
  double subxRB = (t_RBx - RBx)/div;
  double subyRB = (t_RBy - RBy)/div;
  double subzRB = (t_RBz - RBz)/div;
  double subxLF = (t_LFx - LFx)/div;
  double subyLF = (t_LFy - LFy)/div;
  double subzLF = (t_LFz - LFz)/div;
  double subxLB = (t_LBx - LBx)/div;
  double subyLB = (t_LBy - LBy)/div;
  double subzLB = (t_LBz - LBz)/div;

  // Do the "div" movements
  for(int i = 0; i < div; i++){
    RFx += subxRF;
    RFy += subyRF;
    RFz += subzRF;
    RBx += subxRB;
    RBy += subyRB;
    RBz += subzRB;
    LFx += subxLF;
    LFy += subyLF;
    LFz += subzLF;
    LBx += subxLB;
    LBy += subyLB;
    LBz += subzLB;

    updateLServos();
    
    delay(time);
  } 
}



// Linearly move the body
void moveAll(double x, double y, double z, int div, int time){
  
  double subx = x/div;
  double suby = y/div;
  double subz = z/div;

  for(int i = 0; i < div; i++){
    RFx += subx;
    RFy += suby;
    RFz -= subz;

    LFx -= subx;
    LFy += suby;
    LFz -= subz;

    RBx += subx;
    RBy += suby;
    RBz += subz;

    LBx -= subx;
    LBy += suby;
    LBz += subz;

    updateLServos();
    
    delay(time);
  }
}
// ===================================================================================

// ======================== IK functions =============================================
short getSG90PulseFromDegree(byte position180based){
  return map(position180based, 0, 181, SG90_MIN, SG90_MAX);
}

short getMG995PulseFromDegree(byte position180based){
  return map(position180based, 0, 181, MG995_MIN, MG995_MAX);
}

void updateLServos(){
    // Set the servos to their coords with IK functions
    // S_RF1.write(offset1RF - IKang1(RFx,RFy,RFz));
	pwmDriver.setPWM(0, 0, offset1RF - IKang1(RFx,RFy,RFz));
    // S_RF2.write(offset2RF - IKang2(RFx,RFy,RFz));
	pwmDriver.setPWM(1, 0, offset2RF - IKang2(RFx,RFy,RFz));
    // S_RF3.write(IKang3(RFx,RFy,RFz) - offset3RF);
	pwmDriver.setPWM(2, 0, IKang3(RFx,RFy,RFz) - offset3RF);

    // S_LF1.write(IKang1(LFx,LFy,LFz) + offset1LF);
	pwmDriver.setPWM(3, 0, IKang1(LFx,LFy,LFz) + offset1LF);
    // S_LF2.write(IKang2(LFx,LFy,LFz) - offset2LF);
	pwmDriver.setPWM(4, 0, IKang2(LFx,LFy,LFz) - offset2LF);
    // S_LF3.write(offset3LF - IKang3(LFx,LFy,LFz));
	pwmDriver.setPWM(5, 0, offset3LF - IKang3(LFx,LFy,LFz));

    // S_RB1.write(IKang1(RBx,RBy,RBz) + offset1RB );
	pwmDriver.setPWM(6, 0, IKang1(RBx,RBy,RBz) + offset1RB);
    // S_RB2.write(IKang2(RBx,RBy,RBz) - offset2RB);
	pwmDriver.setPWM(7, 0, IKang2(RBx,RBy,RBz) - offset2RB);
    // S_RB3.write(offset3RB - IKang3(RBx,RBy,RBz));
	pwmDriver.setPWM(8, 0, offset3RB - IKang3(RBx,RBy,RBz));
    
    // S_LB1.write(offset1LB - IKang1(LBx,LBy,LBz));
	pwmDriver.setPWM(9, 0, offset1LB - IKang1(LBx,LBy,LBz));
    // S_LB2.write(offset2LB - IKang2(LBx,LBy,LBz));
	pwmDriver.setPWM(10, 0, offset2LB - IKang2(LBx,LBy,LBz));
    // S_LB3.write(IKang3(LBx,LBy,LBz) - offset3LB);
	pwmDriver.setPWM(11, 0, IKang3(LBx,LBy,LBz) - offset3LB);
}

// Angle for servo 1
int IKang1(double x, double y, double z){
  double gamma = atan2(z, x);
  return (gamma * TODEG);
}

// Angle for servo 2
int IKang2(double x, double y, double z){
  double hip = sqrt( pow(y,2) + pow((x - coxa),2));
  double alpha1 = acos(y/hip);
  double alpha2 = acos( (pow(tibia,2) - pow(femur, 2) - pow(hip,2))/(-2*femur*hip));
  double alpha = alpha1 + alpha2;
  return (alpha * TODEG);
}

// Angle for servo 3
int IKang3(double x, double y, double z){
  double hip = sqrt( pow(y,2) + pow((x - coxa),2));
  double beta = acos(( pow(hip,2) - pow(tibia,2) - pow(femur,2))/(-2*tibia*femur));
  return (beta * TODEG);
}
// ===================================================================================




void loop() {

  
  FullWalk1(20, 30);
  delay(10);

}