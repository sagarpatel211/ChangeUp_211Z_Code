/*----------------------------------------------------------------------------*/
/*    Module:       main.cpp                                                  */
/*    Author:       sagarpatel and saurinpatel                                */
/*    Description:  Change Up Code 211Z Code                                  */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// LeftBack             motor         1               
// LeftFront            motor         2               
// RightBack            motor         3               
// RightFront           motor         4               
// MotorExample         motor         5               
// ---- END VEXCODE CONFIGURED DEVICES ----



//IN THIS FILE, INCLUDE: AUTONOMOUS SELECTOR, ODOMETRY IN AUTONOMOUS,AND VOLTAGE DRIVER CONTROL



/*-------------------------------Packages------------------------------------*/
#include "vex.h"
#include <math.h>
#include <iostream> 
using namespace vex;
vex::competition    Competition;
/*-------------------------------Variables-----------------------------------*/
/*-------------------------------Variables-----------------------------------*/
//Variables are defined here for use in the Odometry calculations
#define Pi 3.14159265358979323846
#define SL 5 //distance from tracking center to middle of left wheel
#define SR 5 //distance from tracking center to middle of right wheel
#define SS 7.75 //distance from tracking center to middle of the tracking wheel
#define WheelDiam 4.125 //diameter of all the wheels being used for tracking
#define tpr 360  //Degrees per single encoder rotation
double DeltaL,DeltaR,DeltaB,currentL,currentR,currentB,PreviousL,PreviousR,PreviousB,DeltaTheta,fullelapsedTime,
X,Y,Theta,DeltaXSide,DeltaYSide,DeltaXBack,DeltaYBack,SideChord,BackChord,OdomHeading,elapsedTimeAngle,XError,
AngleError,AngleDerivative,AnglePreviousError,AngleIntegral,AngleCount,AnglemVrequest,XmVrequest,
XDerivative,XPreviousError,XIntegral,XCount,YmVrequest,YError,YDerivative,YPreviousError,YIntegral,YCount,
LeftFrontValue,LeftBackValue,RightFrontValue,RightBackValue,MaxValue,MaxVoltage,elapsedTimeX,elapsedTimeY;
double startingTime,starttimeAngle,starttimeX,starttimeY = 0;
/*---------------------------------------------------------------------------*/
/*                              Motion Functions                             */
/*---------------------------------------------------------------------------*/
//Odometry Function/////////////////////////////////////////////////////////////////////////////////////////
void TrackPOS() {
// 2 cases could be occuring in odometry
// 1: Going in a straight line
// 2: Going in an arc motion
// If the bot is on an angle and going straight the displacement would be linear at angle Theta, meaning a right triangle is formed 
// Since it is a linear motion, the Left and right will move the same amount so we can just pick a side and do our movement calculation
// Since this calculation is working based of very infinitely small arcs, the displacement of the robot will be a chord
// Below it Averages the Left and Right integrated motor encoders since we don't have encoders yet
  currentR = (RightFront.position(degrees) + RightBack.position(degrees)) / 2;
  currentL = (LeftFront.position(degrees) + LeftBack.position(degrees)) / 2;

  //Creates variables for change in each side info in inches (12.9590697 is circumference of wheel)
  DeltaL = ((currentL - PreviousL) * 12.9590697) / tpr;
  DeltaR = ((currentR - PreviousR) * 12.9590697) / tpr;
  //DeltaB = ((currentB - PreviousB) * 12.9590697) / tpr;

  //Determines the change in angle of the robot using the rotational change in each side
  DeltaTheta = (DeltaR - DeltaL) / (SL + SR);

  //Creates an if/else statement to prevent NaN values from appearing and causing issues with calculation
  if(DeltaTheta == 0) {  //If there is no change in angle
    X += DeltaL * sin (Theta);
    Y += DeltaL * cos (Theta);
    //X += DeltaB * cos (Theta + 1.57079633);
    //Y += DeltaB * sin (Theta + 1.57079633);

  //If there is a change in angle, it will calculate the changes in X,Y from chords of an arc/circle.
  } else {  //If the angle changes
      SideChord = 2 * ((DeltaL / DeltaTheta) + SL) * sin (DeltaTheta / 2);
      //BackChord = 2 * ((DeltaB / DeltaTheta) + SS) * sin (DeltaTheta / 2);
      DeltaYSide = SideChord * cos (Theta + (DeltaTheta / 2));
      DeltaXSide = SideChord * sin (Theta + (DeltaTheta / 2));
      //DeltaXBack = BackChord * sin (Theta + (DeltaTheta / 2));
      //DeltaYBack = -BackChord * cos (Theta + (DeltaTheta / 2));
      Theta += DeltaTheta;
      X += DeltaXSide;
      Y += DeltaYSide;
    }

    //Odom heading is converting the radian value of Theta into degrees
    OdomHeading = Theta * 57.295779513;

    //Converts values into newer values to allow for code to effectively work in next cycle
    PreviousL = currentL;
    PreviousR = currentR;
    DeltaTheta = 0;
}

//PID #1 - X Displacement PID//////////////////////////////////////////////////////////////////////////////////
void XdisplacementPID(double desiredX, double XkP, double XkI, double XkD){
  XError = desiredX - X; //X error is calculated
  //Currently we are stuck with 2 conflicting choices: accounting for timescale and not accounting for it...
/*------------------------------------------------------------------------------------------------------------*/
//Option 1: Does not account for timescale (currently commented out)
  //XDerivative = (XError - XPreviousError);  //Derivative is calculated from (error - previous error)

//Option 2: Does account for timescale (has 3 lines of code)
  //calculate the time from the last step and this step (first run will be wrong but it's okay since 10ms is insignificant)
  elapsedTimeX = Brain.timer(timeUnits::msec) - starttimeX;
  XDerivative = (XError - XPreviousError)/elapsedTimeX;  //Derivative is calculated from (error - previous error)/dt
  starttimeX = Brain.timer(timeUnits::msec); //start the timer for this step to be used when this runs over again
/*------------------------------------------------------------------------------------------------------------*/
if ((XError < 0.07) && (XError > -0.07)){ //if the error is really small (0.07 inches)...
    XIntegral = 0; //Keep integral at 0
     XCount += 1; //add 1 to count
  } else { 
    XIntegral += XError; //error is added to integral
    XCount = 0; //the count remains 0 if error is significant
  }
  if (XCount >= 10) { //if count reaches 10...
    XmVrequest = 0; //set voltage of x displacement control to 0
    return; //break from function
  }
  XmVrequest = XError * XkP + XDerivative * XkD + XIntegral * XkI; //X voltage is calculated
  XPreviousError = XError; //previous error is calculated from error
}

//PID #2 - Y Displacement PID//////////////////////////////////////////////////////////////////////////////////
void YdisplacementPID(double desiredY, double YkP, double YkI, double YkD){
  YError = desiredY - Y; //X error is calculated
  //Currently we are stuck with 2 conflicting choices: accounting for timescale and not accounting for it...
/*------------------------------------------------------------------------------------------------------------*/
//Option 1: Does not account for timescale (currently commented out)
  //YDerivative = (YError - YPreviousError);  //Derivative is calculated from (error - previous error)

//Option 2: Does account for timescale (has 3 lines of code)
  //calculate the time from the last step and this step (first run will be wrong but it's okay since 10ms is insignificant)
  elapsedTimeY = Brain.timer(timeUnits::msec) - starttimeY;
  YDerivative = (YError - YPreviousError)/elapsedTimeY;  //Derivative is calculated from (error - previous error)/dt
  starttimeY = Brain.timer(timeUnits::msec); //start the timer for this step to be used when this runs over again
/*------------------------------------------------------------------------------------------------------------*/
  if ((YError < 0.07) && (YError > -0.07)){ //if the error is really small (0.07 inches)...
    YIntegral = 0; //Keep integral at 0
     YCount += 1; //add 1 to count
  } else { 
    YIntegral += YError; //error is added to integral
    YCount = 0; //the count remains 0 if error is significant
  }
  if (YCount >= 10) { //if count reaches 10...
    YmVrequest = 0; //set voltage of y displacement control to 0
    return; //break from function
  }
  YmVrequest = YError * YkP + YDerivative * YkD + YIntegral * YkI;  //Y voltage is calculated
  YPreviousError = YError; //previous error is calculated from error
}

//PID #3 - Angle PID///////////////////////////////////////////////////////////////////////////////////////////
void AngledisplacementPID(double desiredangle, double anglekP, double anglekI, double anglekD){
  AngleError = desiredangle - OdomHeading; //angle error is calculated
  //Currently we are stuck with 2 conflicting choices: accounting for timescale and not accounting for it...
/*------------------------------------------------------------------------------------------------------------*/
//Option 1: Does not account for timescale (currently commented out)
  //AngleDerivative = (AngleError - AnglePreviousError);  //Derivative is calculated from (error - previous error)

//Option 2: Does account for timescale (has 3 lines of code)
  //calculate the time from the last step and this step (first run will be wrong but it's okay since 10ms is insignificant)
  elapsedTimeAngle = Brain.timer(timeUnits::msec) - starttimeAngle;
  AngleDerivative = (AngleError - AnglePreviousError)/elapsedTimeAngle;  //Derivative is calculated from (error - previous error)/dt
  starttimeAngle = Brain.timer(timeUnits::msec); //start the timer for this step to be used when this runs over again
/*------------------------------------------------------------------------------------------------------------*/
  if ((AngleError < 0.1) && (AngleError > -0.1)){  //if the error is really small (0.1 degrees)...
      AngleIntegral = 0;  //Keep integral at 0
      AngleCount += 1; //add 1 to count
    }
    else { 
    AngleIntegral += AngleError; //error is added to integral
    AngleCount = 0;  //the count remains 0 if error is significant
  }
  if (AngleCount >= 10) { //if count reaches 10...
    AnglemVrequest = 0; //set voltage of angle displacement control to 0
    return; //break from function
  }
  AnglemVrequest = AngleError * anglekP + AngleDerivative * anglekD + AngleIntegral * anglekI;//angle voltage is calculated
  AnglePreviousError = AngleError; //previous error is calculated from error
}

//Combined PID/////////////////////////////////////////////////////////////////////////////////////////////////
void MoveToPos(double Xvalue, double Yvalue, double Angle, double kPx, double kIx, double kDx,
  double kPy, double kIy,double kDy,double kPa, double kIa,double kDa ){
  while (Xvalue != X && Yvalue != Y && Angle != OdomHeading) //if the robot reaches position, break loop. else...
    startingTime = Brain.timer(timeUnits::msec);
    TrackPOS(); //run odometry function
    XdisplacementPID (Xvalue, kPx, kIx, kDx); //call the x displacement function
    YdisplacementPID (Yvalue, kPy, kIy, kDy); //call the y displacement function
    AngledisplacementPID (Angle, kPa, kIa, kDa); //call the angle displacement function

    if (AngleCount >= 10 && YCount >= 10 && XCount >= 10){ //if all count variables are 10 more more...
      return; //break from function
    }
    LeftFrontValue = XmVrequest + YmVrequest - AnglemVrequest; //voltage for each motor is calculated
    LeftBackValue = XmVrequest + YmVrequest + AnglemVrequest;  //voltage for each motor is calculated
    RightFrontValue = XmVrequest - YmVrequest + AnglemVrequest;//voltage for each motor is calculated
    RightBackValue = XmVrequest - YmVrequest - AnglemVrequest; //voltage for each motor is calculated
    MaxVoltage = 12000; //this is the highest valid value for the spin function in millivolts
    //If the voltage of any of the motors are greater than the Max voltage, they will be normalized
    if (fabs(LeftFrontValue) > MaxVoltage || fabs(LeftBackValue) > MaxVoltage || fabs(RightFrontValue) > MaxVoltage || fabs(RightBackValue) > MaxVoltage){
      //if the leftfront motor has the highest voltage value...
      if (LeftFrontValue>=LeftBackValue && LeftFrontValue>=RightFrontValue && LeftFrontValue>=RightBackValue) {
        MaxValue = LeftFrontValue / MaxVoltage; //the constant to divide all the motors is assigned a value so they are all under 12,000 V
      }
      //if the leftback motor has the highest voltage value...
      if (LeftBackValue>=LeftFrontValue && LeftBackValue>=RightFrontValue && LeftBackValue>=RightBackValue) {
        MaxValue = LeftBackValue / MaxVoltage; //the constant to divide all the motors is assigned a value so they are all under 12,000 V
      }
      //if the rightfront motor has the highest voltage value...     
      if (RightFrontValue>=LeftFrontValue && RightFrontValue>=LeftBackValue && RightFrontValue>=RightBackValue) {
        MaxValue = RightFrontValue / MaxVoltage; //the constant to divide all the motors is assigned a value so they are all under 12,000 V
      }
      //if the rightback motor has the highest voltage value...
      if (RightBackValue>=LeftFrontValue && RightBackValue>=LeftBackValue && RightBackValue>=RightFrontValue) {
        MaxValue = RightBackValue / MaxVoltage; //the constant to divide all the motors is assigned a value so they are all under 12,000 V
      }
      LeftFrontValue /= MaxValue; //divide the motor value by the normalization factor
      LeftBackValue /= MaxValue; //divide the motor value by the normalization factor
      RightFrontValue /= MaxValue; //divide the motor value by the normalization factor
      RightBackValue /= MaxValue; //divide the motor value by the normalization factor
    }
    LeftFront.spin(forward, LeftFrontValue, voltageUnits::mV); //Motion
    LeftBack.spin(forward, LeftBackValue, voltageUnits::mV);  //Motion
    RightFront.spin(forward, RightFrontValue, voltageUnits::mV);//Motion
    RightBack.spin(forward, RightBackValue, voltageUnits::mV); //Motion
    fullelapsedTime = Brain.timer(timeUnits::msec) - startingTime;
    vex::task::sleep(15 - fullelapsedTime); //Slight delay so the Brain doesn't overprocess
}

void VariableReset ( void ){  //This resets all the variables for the PID functions
  AngleCount = 0;
  YCount = 0;
  XCount = 0;
  XmVrequest = 0;
  YmVrequest = 0;
  AnglemVrequest = 0;
  AnglePreviousError = 0;
  YPreviousError = 0;
  XPreviousError = 0;
  AngleIntegral = 0;
  AngleError = 0;
  AngleDerivative = 0;
  XIntegral = 0;
  XError = 0;
  XDerivative = 0;
  YIntegral = 0;
  YError = 0;
  YDerivative = 0;
  Brain.resetTimer();
}
/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*---------------------------------------------------------------------------*/
void pre_auton( void ) {
  vexcodeInit(); //Initializing Robot Configuration - Required!!!
  Brain.resetTimer(); //Resets The Brain Timer
  RightFront.resetRotation(); //Resets Motor Values For Accuracy In Autonomous
  RightBack.resetRotation(); //Resets Motor Values For Accuracy In Autonomous
  LeftFront.resetRotation(); //Resets Motor Values For Accuracy In Autonomous
  LeftBack.resetRotation(); //Resets Motor Values For Accuracy In Autonomous
}
/*---------------------------------------------------------------------------*/
/*                              Autonomous Task                              */
/*---------------------------------------------------------------------------*/
void autonomous( void ) {
  VariableReset(); //Always reset first since the brain timer must reset for the PID loop to work
  //MoveToPos (X, Y, angle, PID values for x, PID values for y, PID values for angle)
  MotorExample.spin(forward,100,velocityUnits::pct);//sets an example motor to 100 to see if it will run while moving
  MoveToPos(10, 10, 90, 0, 0, 0, 0, 0, 0, 0, 0, 0); //Example: X, Y = 10 and angle = 90
  VariableReset(); //Resets all values so the next function can run (just in case!!)
  MotorExample.stop(brakeType::brake);  //sets to motor to 0 to stop it
  MoveToPos(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0); //Example: X, Y, and angle = 0
  MoveToPos(20, 20, 40, 0, 0, 0, 0, 0, 0, 0, 0, 0); //Example: X, Y, and angle = 0

}
/*----------------------------------------------------------------------------*/
/*                              User Control Task                             */
/*----------------------------------------------------------------------------*/
void usercontrol( void ) {
  while (1){
    //provides power to the motors to allow for movement of robot for testing using controller
    LeftBack.spin(vex::directionType::fwd, ((Controller1.Axis3.value()) + (Controller1.Axis4.value()) - (Controller1.Axis1.value())), vex::velocityUnits::pct);
    LeftFront.spin(vex::directionType::fwd, ((Controller1.Axis3.value()) + (Controller1.Axis4.value()) + (Controller1.Axis1.value())), vex::velocityUnits::pct);
    RightBack.spin(vex::directionType::fwd, ((Controller1.Axis3.value()) - (Controller1.Axis4.value()) + (Controller1.Axis1.value())), vex::velocityUnits::pct);
    RightFront.spin(vex::directionType::fwd, ((Controller1.Axis3.value()) - (Controller1.Axis4.value()) - (Controller1.Axis1.value())), vex::velocityUnits::pct);
    TrackPOS(); //Calls the TrackPosition function
    vex::task::sleep(10); //Slight delay so the Brain doesn't overprocess
  }
}
int main() {
    pre_auton(); //Calls the pre-autonomous function
    Competition.autonomous( autonomous ); //Calls the autonomous function
    Competition.drivercontrol( usercontrol ); //Calls the driver control function
    while(1) {
      vex::task::sleep(15); //Slight delay so the Brain doesn't overprocess
    }
}