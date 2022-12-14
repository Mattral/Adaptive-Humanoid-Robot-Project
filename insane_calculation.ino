#include <PS2X_lib.h>
#include <Servo.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
//////////////////////////////////////////////////////////////////////
////////////////////////////// MPU6050 /////////////////////////////// 
//////////////////////////////////////////////////////////////////////
MPU6050 accelgyro;

#define runEvery(t) for (static long _lasttime;\
                         (uint16_t)((uint16_t)millis() - _lasttime) >= (t);\
                         _lasttime += (t))


int16_t ax, ay, az;
float accBiasX, accBiasY, accBiasZ;
float accAngleX, accAngleY;
double accPitch, accRoll;

int16_t gx, gy, gz;
float gyroBiasX, gyroBiasY, gyroBiasZ;
float gyroRateX, gyroRateY, gyroRateZ;
float gyroBias_oldX, gyroBias_oldY, gyroBias_oldZ;
float gyroPitch = 180;
float gyroRoll = -180;
float gyroYaw = 0;

uint32_t timer;

// input
double InputPitch, InputRoll;

// initial values
double InitialRoll;

//////////////////////////////////////////////////////////////////////
//////////////////////////////  Normal ///////////////////////////////
//////////////////////////////////////////////////////////////////////

PS2X ps2x;
int mode=1;
Servo HipL, HipR, KneeL, KneeR;
int X, Y, x, y, a1, b1, a2, b2;
const int in1 = 5;
const int in2 = 6;
const int in3 = 7;
const int in4 = 8;
const int enA = 3;
const int enB = 9;
int fspeed = 255;
int rspeed, lspeed;
byte PS2 = 0;

///////////////////////////////////////////////////////////////////
////////////////////////////  SETUP  //////////////////////////////
///////////////////////////////////////////////////////////////////

void setup() {

  ////////////////////////  NORMAL SETUP  //////////////////////////////
  
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  PS2 = ps2x.config_gamepad(13, 11, 10, 12, false, false);
  HipL.attach(A0);
  HipR.attach(A1);
  KneeL.attach(A2);
  KneeR.attach(A3);
  x = 0;
  y = 0;
  Serial.begin(57600);

  //////////////////////////////// MPU SETUP  /////////////////////////

  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  delay(1500);

 
  
 //TODO: Better calibration 
  accelgyro.setXAccelOffset(-3522);
  accelgyro.setYAccelOffset(2488);
  accelgyro.setZAccelOffset(1022);
  accelgyro.setXGyroOffset(30);
  accelgyro.setYGyroOffset(1682);
  accelgyro.setZGyroOffset(116);

  gyroBiasX = 0;
  gyroBiasY = 0;
  gyroBiasZ = 0;

  accBiasX = 0;//4
  accBiasY = 0;//-4
  accBiasZ = 0;//16378

  //Get Starting Pitch and Roll
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  accPitch = (atan2(-ax, -az) + PI) * RAD_TO_DEG;
  accRoll = (atan2(ay, -az) + PI) * RAD_TO_DEG;

  if (accPitch <= 360 & accPitch >= 180) {
    accPitch = accPitch - 360;
  }

  if (accRoll <= 360 & accRoll >= 180) {
    accRoll = accRoll - 360;
  }

  gyroPitch = accPitch;
  gyroRoll = accRoll;

  timer = micros();
  delay(1000);
  initializeValues ();
   
}  //_____________________ END OF SET UP________________________________
   /////////////////////////////////////////////////////////////////////


double Setpoint;
void initializeValues() {

  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  //////////////////////
  //  Accelerometer   //
  //////////////////////
  accPitch = (atan2(-ax/182.0, -az/182.0) + PI) * RAD_TO_DEG;
  accRoll = (atan2(ay/182.0, -az/182.0) + PI) * RAD_TO_DEG;

  if (accRoll <= 360 & accRoll >= 180) {
    accRoll = accRoll - 360;
  }

  //////////////////////
  //      GYRO        //
  //////////////////////

  gyroRateX = ((int)gx - gyroBiasX) * 131; 

  gyroPitch += gyroRateY * ((double)(micros() - timer) / 1000000);

  
  timer = micros();
  InitialRoll = accRoll;

  Setpoint = InitialRoll;
}
double filtered = 0;


//////////////////////////////////////////////////////////////////
////////////////////////// LOOP //////////////////////////////////
//////////////////////////////////////////////////////////////////

void loop() {
  ps2x.read_gamepad();
  int mode =1;
  if(ps2x.Button(PSB_SELECT)){
    mode+=1;
  }
  if(mode%2==1){
    Normal();
    Serial.println("Normal Drive");
    
  }
  else if (mode%2==0){
    Balance();
    Serial.println("self balanced drive");
  }
  else{
    mode=1;
  }
  
}

/////////////////////////////////////////////////////////////////////
//////////////////////////// Normal mode ////////////////////////////
/////////////////////////////////////////////////////////////////////

void Normal(){
  if (ps2x.Button(PSB_GREEN)) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH );
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enB, fspeed);
    analogWrite(enA, fspeed);
    Serial.println("full speed");
    delay(10);
  }
  if (ps2x.Analog(PSS_RY) > 128) {
    rspeed = map(ps2x.Analog(PSS_RY), 129, 255, 0, 255);
    Serial.println(rspeed);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enA, rspeed);
    delay(10);
  }

  if (ps2x.Analog(PSS_RY) < 128) {
    rspeed = map(ps2x.Analog(PSS_RY), 0, 127, 255, 0);
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    Serial.println(rspeed);
    analogWrite(enA, rspeed);
    delay(10);
  }

  if (ps2x.Analog(PSS_LY) < 128)  {
    lspeed = map(ps2x.Analog(PSS_LY), 0, 127, 255, 0);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enB, lspeed);
    Serial.println(lspeed);
    delay(10);

  }
  if (ps2x.Analog(PSS_LY) > 128)  {
    lspeed = map(ps2x.Analog(PSS_LY), 129, 255, 0, 255);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enB, lspeed);
    Serial.println(lspeed);
    delay(10);
  }

  else {
    analogWrite(enA, 0);
    analogWrite(enB, 0);
  }
  delay(10);
  ///******************  FOOT   *****************///
  if (ps2x.Button(PSB_L2)) {
    a1 = 1;
  }
  else {
    a1 = 0;
  }
  if (ps2x.Button(PSB_L1)) {
    a2 = 1;
  }
  else {
    a2 = 0;
  }
  if (ps2x.Button(PSB_R1)) {
    b1 = 1;
  }
  else {
    b1 = 0;
  }
  if (ps2x.Button(PSB_R2)) {
    b2 = 1;
  }
  else {
    b2 = 0;
  }



  if (a1 == 1) {
    X = ++x;
    delay(5);
  }
  if (a2 == 1) {
    X = --x;
    delay(5);
  }
  if (b1 == 1) {
    Y = ++y;
    delay(5);
  }
  if (b2 == 1) {
    Y = --y;
    delay(5);
  }


  if (X > 180 ) {
    X = 180;
  }
  else if (X < 0) {
    X = 0;
  }
  else {
    X = X;
  }
  if ( Y > 180 ) {

    Y = 180;
  }
  else if (Y < 0 ) {
    Y = 0;
  }
  else {
    Y = Y;
  }
  //Serial.print("  X>>"); Serial.print(X);
  //Serial.print("  Y>>"); Serial.print(Y);
  HipL.write(X);
  KneeL.write(X);
  HipR.write(Y);
  KneeR.write(Y);
  x = X;
  y = Y;
  delay(5);
}  //__________________________ NORMAL MODE END _______________________
   /////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////
////////////////////////////// MPU mode /////////////////////////////
/////////////////////////////////////////////////////////////////////
void Balance() {
   runEvery(10) {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    //////////////////////
    //  Accelerometer   //
    //////////////////////

    accRoll = (atan2(ay/182.0, -az/182.0) + PI) * RAD_TO_DEG;

    if (accRoll <= 360 & accRoll >= 180) {
      accRoll = accRoll - 360;
    }


    //////////////////////
    //      GYRO        //
    //////////////////////

    gyroRateX = -((int)gx - gyroBiasX) / 131; 


    double gyroVal = gyroRateX * ((double)(micros() - timer) / 1000000);

    timer = micros();

    //Complementary filter
    filtered = 0.98 * (filtered + gyroVal) + 0.02 * (accRoll);

    MotorControl(Compute(filtered - InitialRoll));
}
}

/////////////////////////////////////////////////////////////////////
/////////////////////////// MOTOR CONTROL ///////////////////////////
/////////////////////////////////////////////////////////////////////

void MotorControl(double out) {
  if (out < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    
    Serial.print(out);Serial.println("  balancing 1");
    
  } if (out > 0){
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    Serial.print(out);Serial.println("  balancing 2");
  }
  byte vel = abs(out);
  if (vel < 0)
    vel = 0;
  if (vel > 255)
    vel = 255;

  analogWrite( enA, abs(vel));
  analogWrite( enB, abs(vel));
}

/////////////////////////////////////////////////////////////////////////
///////////////////////////////// PID Tuning  ///////////////////////////
/////////////////////////////////////////////////////////////////////////

int outMax = 255;
int outMin = -255;
float lastInput = 0;
double ITerm = 0;
double kp =30;
double ki =1;
double kd =1;

double Compute(double input)
{

  double error = Setpoint - input;
  //Serial.print(error);Serial.println(" == error");
  ITerm += (ki * error);

  if (ITerm > outMax) ITerm = outMax;
  else if (ITerm < outMin) ITerm = outMin;
  double dInput = (input - lastInput);


  /*Compute PID Output*/
  double output = kp * error + ITerm + kd * dInput;

  if (output > outMax) output = outMax;
  else if (output < outMin) output = outMin;

  /*Remember some variables for next time*/
  lastInput = input;
  return output;
}
