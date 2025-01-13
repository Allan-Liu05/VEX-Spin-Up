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
// leftForward          motor         10              
// leftMiddle           motor         8               
// leftBack             motor         9               
// rightForward         motor         14              
// rightMiddle          motor         6               
// rightBack            motor         5               
// intakeMotor          motor         7               
// catapult             motor         16              
// Controller1          controller                    
// pneumaticA           digital_out   A               
// pneumaticE           digital_out   E               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
void leftSide(int turn)
{
  leftForward.spin(forward, turn, percent);
  leftMiddle.spin(forward, turn, percent);
  leftBack.spin(forward, turn, percent);
}
void rightSide(int turn)
{
  rightForward.spin(forward, turn, percent);
 rightMiddle.spin(forward, turn, percent);
  rightBack.spin(forward, turn, percent);
}

void intake(bool turnForward, bool turnBackwards)
{
  if (turnForward)
  {
    intakeMotor.spin(forward, 100, percent);
  }
  else if (turnBackwards)
  {
    intakeMotor.spin(reverse, 100, percent);
  }
  else
  {
    intakeMotor.stop(coast);
  }
}

void setCatapult(bool playerInputLoad, bool playerInputFire, int errorCorrection)
{
  if (playerInputFire)
    {
      catapult.spinFor(reverse, 230, degrees, false);
      
    }
    else if (playerInputLoad)
    { 
      catapult.spinFor(reverse, 1600 - errorCorrection * 20, degrees, false);
      
    }
}

void moveForward(int power, int time)
{
leftForward.spin(forward, power, percent);
  leftMiddle.spin(forward, power, percent);
  leftBack.spin(forward, power, percent);
  rightForward.spin(forward, power, percent);
  rightMiddle.spin(forward, power, percent);
  rightBack.spin(forward, power, percent);
  vex::task::sleep(time);
  leftForward.stop(brake);
  leftMiddle.stop(brake);
  leftBack.stop(brake);
  rightForward.stop(brake);
  rightMiddle.stop(brake);
  rightBack.stop(brake);
}

void moveBackwards(int power, int time)
{
  leftForward.spin(reverse, power, percent);
  leftMiddle.spin(reverse, power, percent);
  leftBack.spin(reverse, power, percent);
  rightForward.spin(reverse, power, percent);
  rightMiddle.spin(reverse, power, percent);
  rightBack.spin(reverse, power, percent);
  vex::task::sleep(time);
  leftForward.stop(brake);
  leftMiddle.stop(brake);
  leftBack.stop(brake);
  rightForward.stop(brake);
  rightMiddle.stop(brake);
  rightBack.stop(brake);
}

void turnRoller(int time)
{
  intakeMotor.spin(reverse, 100, velocityUnits::pct);
  vex::task::sleep(time);
  intakeMotor.stop(brakeType::brake);
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
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
  catapult.spinFor(reverse, 1600, degrees);
  vex::task::sleep(1000);
  catapult.spinFor(reverse, 230, degrees);
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
bool catapultPosition = false;
bool pneumaticSet = false;
bool inverseControl = false;

int count = 0;
void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    /*
    if (count == 0)
    {
      catapult.spinFor(reverse, 1600, degrees, false);
      count++;
    }
    */
    int rightSideTurn = Controller1.Axis2.position(percent);
    int leftSideTurn = Controller1.Axis3.position(percent);
    bool turnForward = Controller1.ButtonL1.pressing();
    bool turnBackward = Controller1.ButtonL2.pressing();
    bool playerInputLoad = Controller1.ButtonR1.pressing();
    bool playerInputFire = Controller1.ButtonR2.pressing();
    
    leftSide(leftSideTurn);
    rightSide(rightSideTurn);
    
    intake(turnForward, turnBackward);
    setCatapult(playerInputLoad, playerInputFire, count);
    pneumaticA.set(pneumaticSet);
    pneumaticE.set(pneumaticSet);
    //pneumaticB.set(pneumaticSet);
    
    if (Controller1.ButtonLeft.pressing() && Controller1.ButtonRight.pressing())
    {
      pneumaticSet = true;
    }
  
    // load position is 1380 degrees AFTER firing
    // to fire: turn 350
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
