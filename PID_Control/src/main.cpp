/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// backLeft             motor         20              
// backRight            motor         10              
// topLeft              motor         18              
// topRight             motor         8               
// middleRight          motor         9               
// middleLeft           motor         19              
// Controller1          controller                    
// flywheel             motor         1               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void rotateMotors(double rotateQuant)
{
 topLeft.spinToPosition(rotateQuant, degrees, false);
 topRight.spinToPosition(rotateQuant, degrees, false);
 middleLeft.spinToPosition(rotateQuant, degrees, false);
 middleRight.spinToPosition(rotateQuant, degrees, false);
 backLeft.spinToPosition(rotateQuant, degrees, false);
 backRight.spinToPosition(rotateQuant, degrees, false);
}
void flywheelSpin(int targetRPM)
{ 
  double currentRPM;
  double Kp = 0.01;
  double Kd = 0;
  double error;
  double porportional;
  double derivative;
  double previousError = 0;
  double output;
  struct timer curTime = timer();
  double dt;

  for (int x = 0; x < 1; x++)
  {
    curTime.clear();
    dt = curTime.time();
  }
  //flywheel.spin(forward, 177, rpm);
  while (true)
  {
  currentRPM = flywheel.velocity(rpm);
  error = targetRPM - currentRPM;
  porportional = Kp * error;
  derivative = Kd * ((error - previousError) / dt);
  output = porportional; //+ derivative;
  flywheel.spin(forward, output, rpm);

  Controller1.Screen.setCursor(1,1);
  Controller1.Screen.print(flywheel.velocity(rpm));
  Controller1.Screen.setCursor(2,1);
  Controller1.Screen.print(porportional);
  Controller1.Screen.clearScreen();

  //vex::task::sleep(25);
  }
}

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {

  Controller1.Screen.setCursor(1,1);
  /*
  topLeft.setRotation(0, degrees);
  topRight.setRotation(0, degrees);
  middleLeft.setRotation(0, degrees);
  middleRight.setRotation(0, degrees);
  backLeft.setRotation(0, degrees);
  backRight.setRotation(0, degrees);
  */
  //double error_prior = 0;
  //double KP = 1;
  //rotateMotors(270);
  // User control code here, inside the loop
  struct timer timeThing = timer();
  while (1) {
    //flywheel.setVelocity(177, rpm);
    flywheelSpin(177);
    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
