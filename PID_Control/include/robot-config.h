using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor backLeft;
extern motor backRight;
extern motor topLeft;
extern motor topRight;
extern motor middleRight;
extern motor middleLeft;
extern controller Controller1;
extern motor flywheel;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );