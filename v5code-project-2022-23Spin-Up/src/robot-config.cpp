#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor FL = motor(PORT2, ratio18_1, false);
motor BL = motor(PORT17, ratio18_1, false);
motor FR = motor(PORT9, ratio18_1, true);
motor BR = motor(PORT12, ratio18_1, true);
controller Controller1 = controller(primary);
motor Flywheel1 = motor(PORT14, ratio18_1, true);
motor Intake1 = motor(PORT13, ratio18_1, true);
digital_out Puncher = digital_out(Brain.ThreeWirePort.A);
motor Flywheel2 = motor(PORT15, ratio18_1, true);
motor Roller = motor(PORT4, ratio18_1, false);
digital_out Expansion = digital_out(Brain.ThreeWirePort.B);
inertial Inertial = inertial(PORT18);
digital_out Expansion2 = digital_out(Brain.ThreeWirePort.C);
gps GPS = gps(PORT8, 146.05, -196.85, mm, 180);
digital_out Wing = digital_out(Brain.ThreeWirePort.H);

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