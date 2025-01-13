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
// FL                   motor         18              
// FR                   motor         12              
// BL                   motor         13              
// BR                   motor         14              
// Controller1          controller                    
// GripB                motor         3               
// Arm                  motor         19              
// Grip                 motor         21              
// Rotation             motor         2               
// Vision               vision        10              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
int rightside = 275;
int center = 158;// The x coordinate for the center of the vision sensor
int offsetx = 50; //Used to set a range of values to count is being just in front.
int desiredX = 142;
//Functions
void moveF (int time)
{
FL.spin(directionType::fwd, 80, velocityUnits::pct);
BL.spin(directionType::fwd, 80, velocityUnits::pct);
FR.spin(directionType::rev, 80, velocityUnits::pct);
BR.spin(directionType::rev, 80, velocityUnits::pct);
  vex::task::sleep(time);
FL.stop(brakeType::brake);
BL.stop(brakeType::brake);
FR.stop(brakeType::brake);
BR.stop(brakeType::brake);
}
void moveB (int time)
{
FL.spin(directionType::rev, 50, velocityUnits::pct);
BL.spin(directionType::rev, 50, velocityUnits::pct);
FR.spin(directionType::fwd, 50, velocityUnits::pct);
BR.spin(directionType::fwd, 50, velocityUnits::pct);
  vex::task::sleep(time);
FL.stop(brakeType::brake);
BL.stop(brakeType::brake);
FR.stop(brakeType::brake);
BR.stop(brakeType::brake);
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
moveF(1600);
vex::task::sleep(200);
Grip.spin(directionType::rev, 80 , velocityUnits::pct);
 vex::task::sleep(1200);
Grip.stop(brakeType::hold);
 vex::task::sleep(200);
Arm.spin(directionType::fwd, 80 , velocityUnits::pct);
 vex::task::sleep(200);
Arm.stop(brakeType::hold);
moveB(1000);
 vex::task::sleep(200);
bool detect = true;
while(detect){
  Vision.takeSnapshot(Vision__SIG_1);
    FR.spin(directionType::rev, 15, velocityUnits::pct);
    BR.spin(directionType::rev, 15, velocityUnits::pct);
    FL.spin(directionType::rev, 15, velocityUnits::pct);
    BL.spin(directionType::rev, 15, velocityUnits::pct);
  if (Vision.largestObject.exists)
  {
  if(Vision.largestObject.centerX > center + offsetx) //If the object is to the left of center
  {
    FR.spin(directionType::fwd, 15, velocityUnits::pct);
    BR.spin(directionType::fwd, 15, velocityUnits::pct);
    FL.spin(directionType::fwd, 15, velocityUnits::pct);
    BL.spin(directionType::fwd, 15, velocityUnits::pct);
  } else if (Vision.largestObject.centerX < center - offsetx) //If the object is to the right of center
    {
    FR.spin(directionType::rev, 15, velocityUnits::pct);
    BR.spin(directionType::rev, 15, velocityUnits::pct);
    FL.spin(directionType::rev, 15, velocityUnits::pct);
    BL.spin(directionType::rev, 15, velocityUnits::pct);
  } else //The object is not to the right of center and not to the left of center
    {
    FL.stop(brakeType::brake);
    BL.stop(brakeType::brake);
    FR.stop(brakeType::brake);
    BR.stop(brakeType::brake);
    task::sleep(400);
    detect = false;
}
}
}
    task::sleep(400);
    Rotation.rotateTo(200, rotationUnits::deg, 100, velocityUnits::rpm);
moveB(500);
GripB.spinFor(vex::directionType::fwd, 1.2, vex::timeUnits::sec, 100, velocityUnits::pct);
    GripB.stop(brakeType::hold); 
    Rotation.rotateTo(0, rotationUnits::deg, 100, velocityUnits::rpm);
    FR.spin(directionType::fwd, 50, velocityUnits::pct);
    BR.spin(directionType::fwd, 50, velocityUnits::pct);
    FL.spin(directionType::fwd, 50, velocityUnits::pct);
    BL.spin(directionType::fwd, 50, velocityUnits::pct);
    vex::task::sleep(800);
    FL.stop(brakeType::brake);
    BL.stop(brakeType::brake);
    FR.stop(brakeType::brake);
    BR.stop(brakeType::brake);
    moveB(1000);

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
  // User control code here, inside the loop
  
  while (1) {
    FL.spin(directionType::fwd, (Controller1.Axis1.value() + Controller1.Axis3.value())/2, velocityUnits::pct);
    FR.spin(directionType::fwd, (Controller1.Axis1.value() - Controller1.Axis3.value())/2, velocityUnits::pct);
    BL.spin(directionType::fwd, (Controller1.Axis1.value() + Controller1.Axis3.value())/2, velocityUnits::pct);
    BR.spin(directionType::fwd, (Controller1.Axis1.value() - Controller1.Axis3.value())/2, velocityUnits::pct); 
    wait(20, msec); 
    if(Controller1.ButtonL1.pressing()){

      Rotation.spin(directionType::rev, 80 , velocityUnits::pct);
    }
    else if(Controller1.ButtonL2.pressing()){
      Rotation.spin(directionType::fwd, 80 , velocityUnits::pct);
    }
    else{
      Rotation.stop(brakeType::hold); 
    }
    if(Controller1.ButtonUp.pressing())
    {
    GripB.spin(directionType::rev, 80 , velocityUnits::pct);
    }
    else if(Controller1.ButtonDown.pressing()){
      GripB.spin(directionType::fwd, 80 , velocityUnits::pct);
    }
    else{
      GripB.stop(brakeType::hold);  
    }
    if(Controller1.ButtonR1.pressing())
    {
      Arm.spin(directionType::fwd, 80 , velocityUnits::pct);
    }
    else if(Controller1.ButtonR2.pressing()){
      Arm.spin(directionType::rev, 80 , velocityUnits::pct);
    }
    else{
      Arm.stop(brakeType::hold);
    }
    if(Controller1.ButtonA.pressing())
    {
      Grip.spin(directionType::fwd, 80 , velocityUnits::pct);
    }
    else if(Controller1.ButtonB.pressing()){
      Grip.spin(directionType::rev, 80 , velocityUnits::pct);
    }
    else{
      Grip.stop(brakeType::hold);
    }
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
