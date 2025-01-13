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
// frontLeft            motor         2               
// backRight            motor         10              
// backLeft             motor         1               
// Controller1          controller                    
// flywheelMotor1       motor         4               
// intake               motor         8               
// pneumaticA           digital_out   A               
// flywheelMotor2       motor         7               
// rollerMotor          motor         3               
// frontRight           motor         9               
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

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

// Functions for autonomous
void moveForward(int time, int power)
// works
{
frontRight.spin(directionType::fwd, power, velocityUnits::pct);
frontLeft.spin(directionType::fwd, power, velocityUnits::pct);
backRight.spin(directionType::fwd, power, velocityUnits::pct);
backLeft.spin(directionType::fwd, power, velocityUnits::pct);
  vex::task::sleep(time);
frontRight.stop(brakeType::brake);
frontLeft.stop(brakeType::brake);
backRight.stop(brakeType::brake);
backLeft.stop(brakeType::brake);
}

void moveForwardNoStop(int power)
{
frontRight.spin(directionType::fwd, power, velocityUnits::pct);
frontLeft.spin(directionType::fwd, power, velocityUnits::pct);
backRight.spin(directionType::fwd, power, velocityUnits::pct);
backLeft.spin(directionType::fwd, power, velocityUnits::pct);
}
void stopMoveForward(void)
{
frontRight.stop(brakeType::brake);
frontLeft.stop(brakeType::brake);
backRight.stop(brakeType::brake);
backLeft.stop(brakeType::brake);
}
void moveBackwards(int time, int power)
// works
{
frontRight.spin(directionType::rev, power, velocityUnits::pct);
frontLeft.spin(directionType::rev, power, velocityUnits::pct);
backRight.spin(directionType::rev, power, velocityUnits::pct);
backLeft.spin(directionType::rev, power, velocityUnits::pct);
  vex::task::sleep(time);
frontRight.stop(brakeType::brake);
frontLeft.stop(brakeType::brake);
backRight.stop(brakeType::brake);
backLeft.stop(brakeType::brake);
}

void moveLateralRight(int time, int power)
// moveLateralRight moves the robot Right without changing the direction it's pointing towards
// works
{
frontRight.spin(directionType::rev, power, velocityUnits::pct);
frontLeft.spin(directionType::fwd, power, velocityUnits::pct);
backRight.spin(directionType::fwd, power, velocityUnits::pct);
backLeft.spin(directionType::rev, power, velocityUnits::pct);
  vex::task::sleep(time);
frontRight.stop(brakeType::brake);
frontLeft.stop(brakeType::brake);
backRight.stop(brakeType::brake);
backLeft.stop(brakeType::brake);
}

void moveLateralLeft(int time, int power)
// moveLateralLeft moves the robot left without changing the direction it's pointing towards
//this might have to be swapped depending on moveForward function
{
frontRight.spin(directionType::fwd, power, velocityUnits::pct);
frontLeft.spin(directionType::rev, power, velocityUnits::pct);
backRight.spin(directionType::rev, power, velocityUnits::pct);
backLeft.spin(directionType::fwd, power, velocityUnits::pct);
  vex::task::sleep(time);
frontRight.stop(brakeType::brake);
frontLeft.stop(brakeType::brake);
backRight.stop(brakeType::brake);
backLeft.stop(brakeType::brake);
}

void turnRoller(int time)
{
  rollerMotor.spin(directionType::fwd, 100, velocityUnits::pct);
  vex::task::sleep(time);
  rollerMotor.stop(brakeType::brake);
}

void rotateLeft(int time, int power)
{
frontRight.spin(directionType::fwd, power, velocityUnits::pct);
backRight.spin(directionType::fwd, power, velocityUnits::pct);
frontLeft.spin(directionType::rev, power, velocityUnits::pct);
backLeft.spin(directionType::rev, power, velocityUnits::pct);
  vex::task::sleep(time);
frontRight.stop(brakeType::brake);
frontLeft.stop(brakeType::brake);
backRight.stop(brakeType::brake);
backLeft.stop(brakeType::brake);
}

void rotateRight(int time, int power)
{
frontRight.spin(directionType::rev, power, velocityUnits::pct);
backRight.spin(directionType::rev, power, velocityUnits::pct);
frontLeft.spin(directionType::fwd, power, velocityUnits::pct);
backLeft.spin(directionType::fwd, power, velocityUnits::pct);
  vex::task::sleep(time);
frontRight.stop(brakeType::brake);
frontLeft.stop(brakeType::brake);
backRight.stop(brakeType::brake);
backLeft.stop(brakeType::brake);
}

void discIntake(int time, int power)
{
intake.spin(directionType::rev, power, velocityUnits::pct);
  vex::task::sleep(time);
intake.stop(brakeType::brake);
}

void moveForwardAndIntake(int time, int power)
{
  moveForward(time, power);
  discIntake(time, power);
}

void flywheelSpin(int time, int power)
{
flywheelMotor1.spin(directionType::rev, power, velocityUnits::pct);
flywheelMotor2.spin(directionType::rev, power, velocityUnits::pct);
  vex::task::sleep(time);
flywheelMotor1.stop(brakeType::brake);
flywheelMotor2.stop(brakeType::brake);
}

void flywheelForAuton(void)
{
  flywheelMotor1.spin(directionType::fwd, 100, velocityUnits::pct);
  flywheelMotor2.spin(directionType::fwd, 100, velocityUnits::pct);
}

int shootDisc(void)
{

    flywheelForAuton();
    double rpmRead1 = flywheelMotor1.velocity(rpm);
    double rpmRead2 = flywheelMotor2.velocity(rpm);
    if (rpmRead1 > 50.00 || rpmRead2 > 50.00)
    {
      pneumaticA.set(true);    
      vex::task::sleep(50);
      pneumaticA.set(false);
      return 1;
    }
    return 0;
  
}
void turn180(void)
{
  frontRight.spin(forward, 100, percent);
  frontLeft.spin(reverse, 100, percent);
  backRight.spin(forward, 100, percent);
  backLeft.spin(forward, 100, percent);
  vex::task::sleep(3000);
  frontRight.stop(brakeType::brake);
  frontLeft.stop(brakeType::brake);
  backRight.stop(brakeType::brake);
  backLeft.stop(brakeType::brake);
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

void autonomous(void) 
{

//moveLateralLeft(1250, 80);
moveForward(300, 100);
turnRoller(300);
vex::task::sleep(350);
moveBackwards(200,80);

/*
int count = 0;
while(count < 2)
{
  count = count + shootDisc(); 
  if (count == 1)
  {
    intake.spin(forward, 100, percent);
    vex::task::sleep(3000);
    intake.stop(brake);
  }
  vex::task::sleep(750);

}
*/
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
    // This is the main execution loop for the user control program.


    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.
    
int left = Controller1.Axis3.position(vex::percent);
int right = Controller1.Axis2.position(vex::percent);
int sideways = Controller1.Axis4.position(vex::percent);



frontRight.spin(vex::forward, right + sideways, vex::percent);
frontLeft.spin(vex::forward,  left  - sideways, vex::percent);
backRight.spin(vex::forward,  right - sideways, vex::percent);
backLeft.spin(vex::forward,   left  + sideways, vex::percent);
    // Displays temperature data and rpm date
    
    // Pneumatic control
    if (Controller1.ButtonY.pressing())
    {
      pneumaticA.set(true);    
      vex::task::sleep(100);
      pneumaticA.set(false);
    }

    // Intake Control
    if (Controller1.ButtonR1.pressing())
    {
      intake.spin(directionType::fwd, 100, velocityUnits::pct);
    }
    else if(Controller1.ButtonL1.pressing())
    {
      intake.spin(reverse, 100, percent);
    }
    else if (Controller1.ButtonL2.pressing())
    {
      intake.stop(brakeType::coast);
    }

    // Flywheel control
    if (Controller1.ButtonX.pressing())
    { 
      
        flywheelMotor1.spin(directionType::fwd, 100, velocityUnits::pct);
        flywheelMotor2.spin(directionType::fwd, 100, velocityUnits::pct);
    }
    // 180 turn
    if (Controller1.ButtonR2.pressing())
    {
      rotateRight(1000, 100);
    }
    else if (Controller1.ButtonA.pressing()) 
    {
        flywheelMotor1.stop(brakeType::coast);
        flywheelMotor2.stop(brakeType::coast);
    }
    
    // Roller Control
    if (Controller1.ButtonUp.pressing() == true)
    {
      rollerMotor.spin(forward, 100, velocityUnits::pct);
    }
    

    else if (Controller1.ButtonDown.pressing() == true)
    {
      rollerMotor.spin(reverse, 100, percent);
    }
    /*else if (Controller1.ButtonDown.pressing() == false)
    {
      rollerMotor.stop(coast);
    }*/
    else
    {
      rollerMotor.stop(coast);
    }
    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

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
