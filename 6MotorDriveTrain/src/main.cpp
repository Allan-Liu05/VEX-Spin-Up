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
// leftForward          motor         3               
// leftMiddle           motor         7               
// leftBack             motor         4               
// rightForward         motor         11              
// rightMiddle          motor         20              
// rightBack            motor         2               
// Controller1          controller                    
// intakeMotor          motor         12              
// pneumaticA           digital_out   A               
// catapult             motor         6               
// pneumaticH           digital_out   H               
// pneumaticB           digital_out   B               
// flywheelMotor        motor         1               
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
void intake(bool turnForward, bool turnBack)
{
  if (turnForward == true)
  {
    intakeMotor.spin(forward, 100, percent); 
  }
  else if (turnBack == true)
  {
    intakeMotor.spin(reverse, 100, percent);
  }
  else
  {
    intakeMotor.stop(brakeType::coast);
  }
}
void flywheel(bool turnForward, bool turnBack)
{
  if (turnForward == true)
  {
    flywheelMotor.spin(forward, 100, percent); 
    //Controller1.Screen.print("Started");
    
  }
  else if (turnBack == true)
  {
    flywheelMotor.stop(coast);
    //Controller1.Screen.print("Off");
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
void turnRoller(int time)
{
  intakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);
  vex::task::sleep(time);
  intakeMotor.stop(brakeType::brake);
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
int shootDisc(void)
{

    flywheelMotor.spin(forward,100,percent);
    double rpmRead1 = flywheelMotor.velocity(rpm);
    
    if (rpmRead1 > 50.00)
    {
      pneumaticB.set(true);    
      vex::task::sleep(50);
      pneumaticB.set(false);
      return 1;
    }
    return 0;
  
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
  //moveLateralLeft(1250, 80);
  
moveForward(100, 750);
turnRoller(500);
vex::task::sleep(350);
//moveBackwards(80,200);

/*
int count = 0;
while(count < 2)
{
  count = count + shootDisc(); 
  if (count == 1)
  {
    intakeMotor.spin(forward, 100, percent);
    vex::task::sleep(3000);
    intakeMotor.stop(brake);
  }
  vex::task::sleep(750);

}
*/
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
bool pneumaticSet = false;
void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    
    int rightSideTurn = Controller1.Axis2.position(percent);
    int leftSideTurn = Controller1.Axis3.position(percent);
    leftSide(leftSideTurn);
    rightSide(rightSideTurn);

    bool turnUp = Controller1.ButtonUp.pressing();
    bool turnDown = Controller1.ButtonDown.pressing();
    intake(turnUp, turnDown);

    bool commence = Controller1.ButtonX.pressing();
    bool halt = Controller1.ButtonB.pressing();
    flywheel(commence, halt);
    pneumaticA.set(pneumaticSet);
    pneumaticH.set(pneumaticSet);
    
    if (Controller1.ButtonA.pressing())
    {
      pneumaticSet = true;
    }
    
    if (Controller1.ButtonR1.pressing())
    {
      pneumaticB.set(true);
      vex::task::sleep(50);
      pneumaticB.set(false);
    }
    
    Controller1.Screen.setCursor(1,1);
    Controller1.Screen.print(flywheelMotor.velocity(percent));
    Controller1.Screen.clearScreen();
    
/*
    Controller1.Screen.setCursor(1,1);
    Controller1.Screen.print(leftForward.velocity(percent));
    
    Controller1.Screen.setCursor(1,10);
    Controller1.Screen.print(leftMiddle.velocity(percent));
    
    Controller1.Screen.setCursor(1,20);
    Controller1.Screen.print(leftBack.velocity(percent));
    

    Controller1.Screen.setCursor(2,1);
    Controller1.Screen.print(rightForward.velocity(percent));
    
    Controller1.Screen.setCursor(2,10);
    Controller1.Screen.print(rightMiddle.velocity(percent));
    
    Controller1.Screen.setCursor(2,20);
    Controller1.Screen.print(rightBack.velocity(percent));

    Controller1.Screen.clearScreen();
*/
    
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
