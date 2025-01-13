using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor frontLeft;
extern motor backRight;
extern motor backLeft;
extern controller Controller1;
extern motor flywheelMotor1;
extern motor intake;
extern digital_out pneumaticA;
extern motor flywheelMotor2;
extern motor rollerMotor;
extern motor frontRight;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );