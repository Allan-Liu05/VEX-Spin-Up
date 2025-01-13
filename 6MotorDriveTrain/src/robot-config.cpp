#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor leftForward = motor(PORT3, ratio18_1, true);
motor leftMiddle = motor(PORT7, ratio18_1, false);
motor leftBack = motor(PORT4, ratio18_1, false);
motor rightForward = motor(PORT11, ratio18_1, false);
motor rightMiddle = motor(PORT20, ratio18_1, true);
motor rightBack = motor(PORT2, ratio18_1, true);
controller Controller1 = controller(primary);
motor intakeMotor = motor(PORT12, ratio36_1, false);
digital_out pneumaticA = digital_out(Brain.ThreeWirePort.A);
motor catapult = motor(PORT6, ratio6_1, false);
digital_out pneumaticH = digital_out(Brain.ThreeWirePort.H);
digital_out pneumaticB = digital_out(Brain.ThreeWirePort.B);
motor flywheelMotor = motor(PORT1, ratio6_1, false);

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