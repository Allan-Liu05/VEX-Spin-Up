/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       allanliu                                                  */
/*    Created:      Mon Jan 09 2023                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// backLeft             motor         9               
// backRight            motor         10              
// intakeMotor          motor         8               
// Controller1          controller                    
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;
void leftSide(int power)
{
  backLeft.spin(forward, power, percent);
}
void rightSide(int power)
{
  backRight.spin(forward, power, percent);
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
int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  
}
void usercontrol(void)
{
 int rightSideTurn = Controller1.Axis2.position(percent);
  int leftSideTurn = Controller1.Axis3.position(percent);

  bool turnForward = Controller1.ButtonL1.pressing();
    bool turnBackward = Controller1.ButtonL2.pressing();
 rightSide(rightSideTurn);
 leftSide(leftSideTurn); 
 intake(turnForward, turnBackward);
}