#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor leftForward = motor(PORT10, ratio18_1, false);
motor leftMiddle = motor(PORT8, ratio18_1, true);
motor leftBack = motor(PORT9, ratio18_1, true);
motor rightForward = motor(PORT14, ratio18_1, false);
motor rightMiddle = motor(PORT6, ratio18_1, false);
motor rightBack = motor(PORT5, ratio18_1, false);
motor intakeMotor = motor(PORT7, ratio18_1, false);
motor catapult = motor(PORT16, ratio36_1, false);
controller Controller1 = controller(primary);
digital_out pneumaticA = digital_out(Brain.ThreeWirePort.A);
digital_out pneumaticE = digital_out(Brain.ThreeWirePort.E);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}