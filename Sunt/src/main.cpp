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
// intakeMotor          motor         2               
// Controller1          controller                    
// indexer              digital_out   B               
// expansion1           digital_out   E               
// flywheel             motor         1               
// topLeft              motor         18              
// topRight             motor         8               
// middleRight          motor         9               
// middleLeft           motor         19              
// testingMotor         motor         5               
// expansion2           digital_out   C               
// inertialSens         inertial      17              
// dragonWings          digital_out   A               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <cmath>
using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
bool ifCoast = true;

// AUTON FUNCTIONS
// AUTON FUNCTIONS
// AUTON FUNCTIONS
void leftTurn(int time, int power)
{
  backLeft.spin(forward, power, percent);
  topLeft.spin(forward, power, percent);
  middleLeft.spin(forward, power, percent);

  backRight.spin(forward, power, percent);
  topRight.spin(forward, power, percent);
  middleRight.spin(forward, power, percent);

  vex::task::sleep(time);

  backLeft.stop(brake);
  backRight.stop(brake);
  middleLeft.stop(brake);
  middleRight.stop(brake);
  topLeft.stop(brake);
  topRight.stop(brake);
}
void rightTurn(int time, int power)
{
  backLeft.spin(reverse, power, percent);
  topLeft.spin(reverse, power, percent);
  middleLeft.spin(reverse, power, percent);

  topRight.spin(reverse, power, percent);
  middleRight.spin(reverse, power, percent);
  backRight.spin(reverse, power, percent);

  vex::task::sleep(time);

  backLeft.stop(brake);
  backRight.stop(brake);
  middleLeft.stop(brake);
  middleRight.stop(brake);
  topLeft.stop(brake);
  topRight.stop(brake);
}

void moveForward(int time, int power)
{
  backLeft.spin(forward, power, percent);
  topLeft.spin(forward, power, percent);
  middleLeft.spin(forward, power, percent);

  topRight.spin(reverse, power, percent);
  middleRight.spin(reverse, power, percent);
  backRight.spin(reverse, power, percent);

  vex::task::sleep(time);

  backLeft.stop(brake);
  backRight.stop(brake);
  middleLeft.stop(brake);
  middleRight.stop(brake);
  topLeft.stop(brake);
  topRight.stop(brake);
}
void moveBackward(int time, int power)
{
  backLeft.spin(reverse, power, percent);
  topLeft.spin(reverse, power, percent);
  middleLeft.spin(reverse, power, percent);

  topRight.spin(forward, power, percent);
  middleRight.spin(forward, power, percent);
  backRight.spin(forward, power, percent);

  vex::task::sleep(time);

  backLeft.stop(brake);
  backRight.stop(brake);
  middleLeft.stop(brake);
  middleRight.stop(brake);
  topLeft.stop(brake);
  topRight.stop(brake);
}

void shootDisc(int targetRPM)
{
  int i = 0;
  flywheel.spin(forward, 100, percent);
  vex::task::sleep(1000);
  while (i < 2)
  {
    {
      if (flywheel.velocity(rpm) > targetRPM)
      {
        vex::task::sleep(100);
        indexer.set(true);
        vex::task::sleep(150);
        indexer.set(false);
        i++;
        vex::task::sleep(750);
      }
    }
  }
  flywheel.stop(coast);
}

void turnRoller(int time)
{
  intakeMotor.spin(forward, 100, percent);
  vex::task::sleep(time);
  intakeMotor.stop(brake);
}

void intakeOn(void)
{
  intakeMotor.spin(forward, 100, percent);
}
void intakeOff(void)
{
  intakeMotor.stop(coast);
}
// AUTON FUNCTIONS END
// AUTON FUNCTIONS END
// AUTON FUNCTIONS END
void leftSide(int power)
{
  backLeft.spin(forward, power, percent);
  topLeft.spin(forward, power, percent);
  middleLeft.spin(forward, power, percent);
}

void rightSide(int power)
{
  backRight.spin(forward, power, percent);
  topRight.spin(forward, power, percent);
  middleRight.spin(forward, power, percent);
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
    if (ifCoast == true)
    {
    intakeMotor.stop(coast);
    }
  }
}

void indexerActivate(void)
{
    indexer.set(true);
    vex::task::sleep(100);
    indexer.set(false);
    //vex::task::sleep(200);
}

void flywheelActivate(bool turnOn, bool turnOff, int targetValue)
{
  // PID Variables
  static int desiredValue = targetValue;
  static int error;
  static int prevError = 0;

  static int porportional;
  static int derivative;
  static int integral = 0;
  static int integralCap = 400; // So past error doesn't get kept for too long

  static double kp = 0.0034;
  static double kd = 0.002;
  static double ki = 0.0001;

static double output;
  if (turnOn)
  {
    error = desiredValue - flywheel.velocity(rpm);
    
    porportional = error;
    derivative = error - prevError; 

    if (abs(error) < integralCap)
    {
      integral += error;
    }  
    else
    {
      integral = 0;
    }
    output = kp * porportional + kd * derivative + ki * integral + desiredValue;
    flywheel.spin(forward, output, rpm);
    Controller1.Screen.setCursor(1,1);
    Controller1.Screen.print(flywheel.velocity(rpm));
    Controller1.Screen.clearScreen();
    prevError = error;
  }
  else if (turnOff)
  {
    flywheel.stop(coast);
  }
}

// NEW DRIVE METHODS -- BASED OFF WHEEL CIRCUMFERENCE AND NOT POWER/TIME
// THE 'D' AFTER THE METHOD NAME DENOTES THAT THE METHOD IS BASED OFF DISTANCE
// THIS IS DONE TO AVOID OVERLOADING
// the f here denotes that it is lateral motion -- no rotation

void moveForwardD(double travelDistance, int power)
{
  inertialSens.setHeading(0,degrees);
  // convert distance into wheel rotations -- 1 rotation = 27.13cm
  double axelRotate = (travelDistance / 27.13);

  topLeft.rotateFor(fwd, axelRotate, turns, power, velocityUnits::pct, false);
  middleLeft.rotateFor(fwd, axelRotate, turns, power, velocityUnits::pct, false);
  backLeft.rotateFor(fwd, axelRotate, turns, power, velocityUnits::pct, false);

  topRight.rotateFor(reverse, axelRotate, turns, power, velocityUnits::pct, false);
  middleRight.rotateFor(reverse, axelRotate, turns, power, velocityUnits::pct, false);
  backRight.rotateFor(reverse, axelRotate, turns, power, velocityUnits::pct, false);
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
int limitExpansion = 0;
// autonTest is so that I can easily change the boolean to test other auton ideas
// this is so I dont have to comment every time
// DEFAULT SHOULD AWLWAYS BE: bool autonTest = false;
// DEFAULT SHOULD AWLWAYS BE: bool autonTest = false;
// DEFAULT SHOULD AWLWAYS BE: bool autonTest = false;
bool autonTest = false;
void autonomous(void) 
{
  if (!autonTest)
  {
  moveBackward(200, 75);
  turnRoller(300);
  moveForward(100, 100);
  vex::task::sleep(30);
  leftTurn(300, 30);
  vex::task::sleep(30);
  moveForward(600, 60);
  vex::task::sleep(30);
  rightTurn(480, 43);
  vex::task::sleep(30);
  moveForward(300, 30);

  shootDisc(173);
  
  
  leftTurn(950, 35);
  vex::task::sleep(100);
  moveForward(500, 90);
  vex::task::sleep(500);
  //vex::task::sleep(500);
  intakeOn();
  for (int x = 0; x < 3; x++)
  {
    moveForward(550, 20);
    vex::task::sleep(750);
    if (x == 2)
    {
      moveForward(550, 35);
    vex::task::sleep(750);
    }
  }
  vex::task::sleep(750);
  intakeOff();
  }
  /*
  topLeft.setPosition(0, degrees);
  Controller1.Screen.setCursor(1,1);
  Controller1.Screen.print(topLeft.position(degrees));
  moveForwardD(100, 100);
  Controller1.Screen.clearScreen();
  Controller1.Screen.print(topLeft.position(degrees));
  */
}


bool pneumaticSet = false;
bool dragonWingsDeploy = false;
bool activate = false;
int resetInertial = 0;

void usercontrol(void) {
  inertialSens.setRotation(0,degrees);
  inertialSens.calibrate();
  // User control code here, inside the loop
  while (1) {
  int rightSideTurn = Controller1.Axis2.position(percent);
  int leftSideTurn = Controller1.Axis3.position(percent);

  bool turnForward = Controller1.ButtonL1.pressing();
  bool turnBackward = Controller1.ButtonL2.pressing();
  
  Controller1.ButtonR1.pressed(indexerActivate);
  bool turnOn = Controller1.ButtonX.pressing();
  bool turnOff = Controller1.ButtonB.pressing();

  rightSide(-1 * rightSideTurn);
  leftSide(leftSideTurn); 
  intake(turnForward, turnBackward);
  flywheelActivate(turnOn, turnOff, 130);
  Controller1.Screen.setCursor(1,1);
  Controller1.Screen.print(flywheel.velocity(rpm));
  Controller1.Screen.clearScreen();
  expansion1.set(pneumaticSet);
  expansion2.set(pneumaticSet);
  dragonWings.set(dragonWingsDeploy);
  if (Controller1.ButtonLeft.pressing() && Controller1.ButtonUp.pressing() && limitExpansion == 0)
    {
      pneumaticSet = true;
      expansion1.set(pneumaticSet);
      vex::task::sleep(150);
      expansion1.set(false);
      vex::task::sleep(150);
      expansion2.set(pneumaticSet);
      vex::task::sleep(150);
      expansion2.set(false);
      limitExpansion++;
      pneumaticSet = false;
    }
    if (Controller1.ButtonRight.pressing() && Controller1.ButtonRight.pressing())
    {
      dragonWingsDeploy = true;
      dragonWings.set(dragonWingsDeploy);
    }
    }
    resetInertial++;
    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
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
