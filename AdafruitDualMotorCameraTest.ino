
#include <MyAdafruitAccelStepper.h>
#include <MyLidarCamera.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <SoftwareSerial.h>
#include "utility/Adafruit_PWMServoDriver.h"

#include <I2C.h>
#define    LIDARLite_ADDRESS   0x62          // Default I2C Address of LIDAR-Lite.
#define    RegisterMeasure     0x00          // Register to write to initiate ranging.
#define    MeasureValue        0x04          // Value to initiate ranging.
#define    RegisterHighLowB    0x8f          // Register to get both High and Low bytes in 1 call.


#define TOTAL_DISTANCE 1600

#define  stopMotor 0
#define  startMotor 1
#define  pauseMotor 2
#define  restartMotor 3

Adafruit_MotorShield AFMStop(0x60); // Default address, no jumpers

// Connect two steppers with 200 steps per revolution (1.8 degree)
// to the top shield
Adafruit_StepperMotor *myStepper1 = AFMStop.getStepper(200, 1);
Adafruit_StepperMotor *myStepper2 = AFMStop.getStepper(200, 2);

void forwardstep1() {  
  myStepper1->onestep(FORWARD, MICROSTEP);
}
void backwardstep1() {  
  myStepper1->onestep(BACKWARD, MICROSTEP);
}
// wrappers for the second motor!
void forwardstep2() {  
  myStepper2->onestep(FORWARD, MICROSTEP);
}
void backwardstep2() {  
  myStepper2->onestep(BACKWARD, MICROSTEP);
}

MyAdafruitAccelStepper axisStepper(forwardstep1, backwardstep1);
MyAdafruitAccelStepper cameraStepper(forwardstep2, backwardstep2);

MyLidarCamera myLidarCamera(1);

int axisStepperSpeed = 100.0;
int cameraStepperSpeed = 100.0;

int cameraStepperDistance = TOTAL_DISTANCE/2;
int axisStepperBlockDistance = TOTAL_DISTANCE/40;
int axisStepperCurrentDistance = 0;

boolean cameraMotorWasActive = false;
boolean axisMotorWasActive = false;

boolean motorCanRun = false;

boolean startProgram = false;

const int ledPin = 13; 

float phi = 0.0, theta = 0.0;

void setup() {     
    Serial.begin(9600); 
    AFMStop.begin(); // Start the shield
    setUpCameraMotor();
    setUpAxisMotor();
    cameraMotorWasActive = true;
     
     I2c.begin(); // Opens & joins the irc bus as master
     delay(100); // Waits to make sure everything is powered up before sending or receiving data  
     I2c.timeOut(50);
}

String dis;

void loop() {
  
   int message = checkForStart(); 
   if(checkForStart()){
     runMotor();
     runCamera();
   }
}

 void  resetEverything(){
   cameraStepperDistance = TOTAL_DISTANCE/2;
   axisStepperBlockDistance = TOTAL_DISTANCE/40;
   axisStepperCurrentDistance = 0;
   cameraMotorWasActive = false;
   axisMotorWasActive = false;
   motorCanRun = false;
   startProgram = false;
 }

void runMotor(){
   if(!axisStepperFinished()){
       if(axisMotorWasActive && axisStepper.isFinished()){
          resetAxisMotor();
          axisMotorWasActive = false; 
          cameraMotorWasActive = true;
          cameraMotorRun();
       }else if(cameraMotorWasActive && cameraStepper.isFinished()){
         cameraMotorWasActive = false;
         axisMotorWasActive = true;
         resetCameraMotor();
         axisMotorRun();
       }else if(cameraMotorWasActive && !cameraStepper.isFinished()){
         cameraMotorWasActive = true;
         cameraMotorRun();
       }else if(axisMotorWasActive && !axisStepper.isFinished()){
         axisMotorWasActive = true;
         axisMotorRun();
       }
    }
}

void setUpCameraMotor(){
   cameraStepper.setSpeed(cameraStepperSpeed);
}

void setUpAxisMotor(){
   axisStepper.setSpeed(axisStepperSpeed);
}


void cameraMotorRun(){
  cameraStepper.step(cameraStepperDistance);
}

void axisMotorRun(){
  axisStepper.step(axisStepperBlockDistance);
  
}

void resetCameraMotor(){
   cameraStepper.setIsFinished(false);
   cameraStepperSpeed = -cameraStepperSpeed;
    cameraStepper.setSpeed(cameraStepperSpeed);
}

void resetAxisMotor(){
   axisStepperCurrentDistance += axisStepperBlockDistance;
   axisStepper.setIsFinished(false);
}

boolean axisStepperFinished(){
 
  if(axisStepperCurrentDistance == TOTAL_DISTANCE)
    return true;
  else {
      return false;
  }
}

void runCamera(){
  uint8_t nackack = 100;      
  while (nackack != 0){ 
    nackack = I2c.write(LIDARLite_ADDRESS,RegisterMeasure, MeasureValue); // Write 0x04 to 0x00
    delay(1); 
  }
  
  byte distanceArray[2];
  nackack = 100;     
  while (nackack != 0){
    nackack = I2c.read(LIDARLite_ADDRESS,RegisterHighLowB, 2, distanceArray);
    delay(1);
  }
  int distance = (distanceArray[0] << 8) + distanceArray[1];
 
  
  theta = 180 - cameraStepper.currentPosition() * 1.8/8;
  phi = axisStepper.totalDistance() * 1.8/8;
  
  Serial.println(String(phi) + " " + String(theta) + " " + String(distance));
}



int checkForStart(){
  
   if(Serial.available() <5){
       if(startProgram)
         return startMotor;
       else
         return stopMotor;
   }
   
  char receivedString[6];
  
  for(int i = 0; i <5;i++){
    char inByte = Serial.read();
    receivedString[i] = inByte;
  }
  
  receivedString[5] = '\0';
  String start = String("start");
  String rec = String(receivedString);
  
  if(rec == start){
    startProgram = true;
    return true;
  }else{
    return false;
  }
}


