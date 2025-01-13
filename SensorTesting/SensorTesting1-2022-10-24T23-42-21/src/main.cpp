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
// GyroH                gyro          H               
// Controller1          controller                    
// GyroA                gyro          A               
// Motor19              motor         19              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
void flywheel(double power)
{
  Motor19.spin(reverse, power, velocityUnits::pct);
}
/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ..

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
double previousError = Motor19.velocity(percent);
double targetPercent = 60.0;
double KP = 1;
double KI = 0;
double KD = 0;
void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
  

  double currentPercent = Motor19.velocity(percent);
  double errorValue = currentPercent - targetPercent;
  double previousError = errorValue - 0.1;
  double dt = errorValue - previousError;

  double porportional = KP * errorValue;
  double integral = KI * (errorValue * dt);
  double derivative = KD * (errorValue / dt);

  double inputValue = porportional + integral + derivative;

  if (Controller1.ButtonX.pressing())
  {
    flywheel(inputValue);
  }
  else if (Controller1.ButtonY.pressing())
  {
    flywheel(1);
  }
  else if (Controller1.ButtonA.pressing())
  {
    Motor19.stop(brakeType::coast);
  }
  Brain.Screen.printAt(1, 20, "Percent: %f percent", Motor19.velocity(percent));
  Brain.Screen.printAt(1, 40, "inputValue: %f", inputValue);
  Brain.Screen.printAt(1, 60,"Current RPM: %f", Motor19.velocity(rpm));
  Brain.Screen.printAt(1, 80, "Porportional: %f", porportional);
  Brain.Screen.printAt(1, 100, "Integral: %f", integral);
  Brain.Screen.printAt(1, 120, "Derivative: %f", derivative);
  Brain.Screen.printAt(1, 140, "dt: %f", dt);
  Brain.Screen.printAt(1, 160, "errorValue: %f", errorValue);
  Brain.Screen.printAt(1, 180, "inputValue: %f", inputValue);
  previousError = errorValue;
    /*double rot = 0;
    rot = (GyroH.rotation(degrees) + GyroA.rotation(degrees)) / 2;
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(1, 20, "rotation: %f degrees", rot);
    // Brain.Screen.printAt(1, 40, "percent: %f %%", GyroH.value(percentUnits::pct));
    // Brain.Screen.printAt(1, 60, "analog: %f mV", GyroH.value(analogUnits::mV));
    */
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
