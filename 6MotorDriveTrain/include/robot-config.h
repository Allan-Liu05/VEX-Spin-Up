using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor leftForward;
extern motor leftMiddle;
extern motor leftBack;
extern motor rightForward;
extern motor rightMiddle;
extern motor rightBack;
extern controller Controller1;
extern motor intakeMotor;
extern digital_out pneumaticA;
extern motor catapult;
extern digital_out pneumaticH;
extern digital_out pneumaticB;
extern motor flywheelMotor;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );