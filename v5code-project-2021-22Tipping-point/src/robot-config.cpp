#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor FL = motor(PORT18, ratio18_1, false);
motor FR = motor(PORT12, ratio18_1, false);
motor BL = motor(PORT13, ratio18_1, false);
motor BR = motor(PORT14, ratio18_1, false);
controller Controller1 = controller(primary);
motor GripB = motor(PORT3, ratio36_1, true);
motor Arm = motor(PORT19, ratio36_1, true);
motor Grip = motor(PORT21, ratio36_1, false);
motor Rotation = motor(PORT2, ratio18_1, false);
/*vex-vision-config:begin*/
signature Vision__SIG_1 = signature (1, 7697, 8711, 8204, -1057, -421, -739, 3, 0);
signature Vision__SIG_2 = signature (2, -2683, -2211, -2447, 10153, 11883, 11018, 2.5, 0);
vision Vision = vision (PORT10, 50, Vision__SIG_1, Vision__SIG_2);
/*vex-vision-config:end*/

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