using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor FL;
extern motor BL;
extern motor FR;
extern motor BR;
extern controller Controller1;
extern motor Flywheel1;
extern motor Intake1;
extern digital_out Puncher;
extern motor Flywheel2;
extern motor Roller;
extern digital_out Expansion;
extern inertial Inertial;
extern digital_out Expansion2;
extern gps GPS;
extern digital_out Wing;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );