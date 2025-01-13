using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor backLeft;
extern motor backRight;
extern motor intakeMotor;
extern controller Controller1;
extern digital_out indexer;
extern digital_out expansion1;
extern motor flywheel;
extern motor topLeft;
extern motor topRight;
extern motor middleRight;
extern motor middleLeft;
extern motor testingMotor;
extern digital_out expansion2;
extern inertial inertialSens;
extern digital_out dragonWings;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );