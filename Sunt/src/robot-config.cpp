#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor backLeft = motor(PORT20, ratio18_1, false);
motor backRight = motor(PORT10, ratio18_1, false);
motor intakeMotor = motor(PORT2, ratio6_1, false);
controller Controller1 = controller(primary);
digital_out indexer = digital_out(Brain.ThreeWirePort.B);
digital_out expansion1 = digital_out(Brain.ThreeWirePort.E);
motor flywheel = motor(PORT1, ratio18_1, false);
motor topLeft = motor(PORT18, ratio18_1, false);
motor topRight = motor(PORT8, ratio18_1, false);
motor middleRight = motor(PORT9, ratio18_1, false);
motor middleLeft = motor(PORT19, ratio18_1, false);
motor testingMotor = motor(PORT5, ratio18_1, false);
digital_out expansion2 = digital_out(Brain.ThreeWirePort.C);
inertial inertialSens = inertial(PORT17);
digital_out dragonWings = digital_out(Brain.ThreeWirePort.A);

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