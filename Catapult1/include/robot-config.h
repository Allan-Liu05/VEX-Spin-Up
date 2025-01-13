using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor leftForward;
extern motor leftMiddle;
extern motor leftBack;
extern motor rightForward;
extern motor rightMiddle;
extern motor rightBack;
extern motor intakeMotor;
extern motor catapult;
extern controller Controller1;
extern digital_out pneumaticA;
extern digital_out pneumaticE;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );