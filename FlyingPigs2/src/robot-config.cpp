#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor frontLeft = motor(PORT2, ratio18_1, false);
motor backRight = motor(PORT10, ratio18_1, true);
motor backLeft = motor(PORT1, ratio18_1, false);
controller Controller1 = controller(primary);
motor flywheelMotor1 = motor(PORT4, ratio18_1, false);
motor intake = motor(PORT8, ratio18_1, false);
digital_out pneumaticA = digital_out(Brain.ThreeWirePort.A);
motor flywheelMotor2 = motor(PORT7, ratio18_1, true);
motor rollerMotor = motor(PORT3, ratio18_1, false);
motor frontRight = motor(PORT9, ratio18_1, true);

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