/********************************************
 * Self Balancing robot
 * 
 * Robot meant to balance on two wheels using
 * a PID loop.
 * 
 * Chassis built by Nick Tormboukis
 * Code written by Mike Yannuzzi & Nick Tromboukis
 ********************************************/

//Including all the neccessary libraries for the redbot main board and adafruint BNO 055
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <RedBot.h>

//Setting up the IMU
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055();

//Constants and variables
double calibrateNum;
double setPoint;
double currError; 

int motorSpeed;
//Setting up the motors
RedBotMotors motors;

//PID vars
#define GUARD_GAIN 50.0 // prev 25.0
float K = 1.1;
float Kp = 14; //previous value 15
float Ki = 1; //previos value 1
float Kd = 2.4; //previous value 2.4

int last_error = 0;
int integrated_error = 0;
int pTerm = 0, iTerm = 0, dTerm = 0;
void setup() {
  //Initializing sensors and pouring a bowl of serial
  Serial.begin(115200);
//  Serial.println("Intitializing robot...");
  
  /* Initialize the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
//    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  bno.setExtCrystalUse(true);
  delay(1000);
  //Calibration function
  calibrateSensor();
  motors.drive(10);
  delay(500);
  motors.stop();
  delay(500);
  motors.drive(-10);
  delay(500);
  motors.stop();
}

void loop() {
 /**
  * Control loop Hierarchy
  * 
  *  1- read z axis from sensor
  *  2- update motors
  *  3 - delay
  */

  readSensor();
  //updateMotors();
  motors.drive(updatePid(setPoint, readSensor()));
//  Serial.println("Set point:" );
//  Serial.println(setPoint);
//  Serial.println();
  
  delay(BNO055_SAMPLERATE_DELAY_MS);
}

/**
 * Reads the z axis of the sensor
 */
int readSensor(){
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  
//  Serial.println("Z: ");
  Serial.print(euler.z());
  Serial.print('\n');
//  Serial.println();
//  Serial.println("\t\t");
  int zAxis = euler.z();
  //Serial.println(zAxis);
//  Serial.println("");
  return zAxis;
}

void calibrateSensor(){
    for(int i=0; i<50;i++){
      calibrateNum = calibrateNum + readSensor();
    }
    calibrateNum = calibrateNum/50;
    setPoint = calibrateNum;
//    Serial.println(setPoint);
//    Serial.println();
    
}

/*
void updateMotors(){

  float K = 1.7;
  currError = readSensor() - setPoint;
  Serial.println("Error: ");
  Serial.println(currError);
  pTerm = Kp * currError;
  errorSum += currError;
  errorSum = constrain(errorSum, -200,200);
  iTerm = Ki * errorSum;
  dTerm = Kd * (currError - prevError);
  prevError = currError;
  motorSpeed = constrain(K*(pTerm + iTerm + dTerm), -255, 255);
  Serial.println("Motor speed: ");
  Serial.println(motorSpeed);
  motors.drive(motorSpeed);
}
*/

int updatePid(int targetPosition, int currentPosition)   {
  int error = targetPosition - currentPosition; 
  pTerm = Kp * error;
  integrated_error += error;                                       
  iTerm = Ki * constrain(integrated_error, -GUARD_GAIN, GUARD_GAIN);
//  Serial.print("i");
//  Serial.println(iTerm);
//  Serial.println();
//  Serial.println("integrated_error:");
//  Serial.println(integrated_error);
//  Serial.println();
  dTerm = Kd * (error - last_error);                            
  last_error = error;
  return -constrain(K*(pTerm + iTerm + dTerm), -255, 255);
}



















