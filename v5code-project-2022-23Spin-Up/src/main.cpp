/*----------------------------------------------------------------------------*/
/* */
/* Module: main.cpp */
/* Author: VEX */
/* Created: Thu Sep 26 2019 */
/* Description: Competition Template */
/* */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// FL                   motor         2               
// BL                   motor         17              
// FR                   motor         9               
// BR                   motor         12              
// Controller1          controller                    
// Flywheel1            motor         14              
// Intake1              motor         13              
// Puncher              digital_out   A               
// Flywheel2            motor         15              
// Roller               motor         4               
// Expansion            digital_out   B               
// Inertial             inertial      18              
// Expansion2           digital_out   C               
// GPS                  gps           8               
// Wing                 digital_out   H               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <cmath>
using namespace vex;

// A global instance of competition
competition Competition;


 double desiredFace;
 double facing;
 double xvalue;
 double xdes = 144;
 double yvalue;
 double ydes = 144;
 double deltax;
 double deltay;
 double prevD = 0;
 double gpsDerrivative;


double rpmRead1 = Flywheel1.velocity(rpm);
double rpmRead2 = Flywheel1.velocity(rpm);
int rightside = 275;
int center = 158;// The x coordinate for the center of the vision sensor
int offsetx = 50; //Used to set a range of values to count is being just in front.
int desiredX = 142;
//not using vison sensor this year but might in the future

int ticks = 1627;
void forwardVoltage(double left, double right) {
 //12000 max mv
 int voltage = 120;
 FR.spin(fwd, right * voltage, voltageUnits::mV);
 BR.spin(fwd, right * voltage, voltageUnits::mV);

 FL.spin(fwd, left * voltage, voltageUnits::mV);
 BL.spin(fwd, left * voltage, voltageUnits::mV);
}
///Settings
double kP = 0.0226;
double kI = 0.0001;
double kD = 0.0015;
double degkP = 0.02;//p controller with inertial sensor
double turnkP = 0.028;
double turnkI = 0.0002;
double turnkD = 0.00185;
double flykP = 0.008;
double flykI = 0.001;
double flykD = 0.001;
int maxTurnIntegral = 300; // These cap the integrals
int maxIntegral = 300;
int integralBound = 3; //If error is outside the bounds, then apply the integral. This is a buffer with +-integralBound degrees

//Autonomous Settings
int desiredValue = 200;
int desiredTurnValue = 0;
int desiredFlyValue = 0;

int flyError;
int prevFlyError;
int flyTotalError;

int error; //SensorValue - DesiredValue : Position
int error2;
int prevError = 0; //Position 20 miliseconds ago
int derivative; // error - prevError : Speed
int totalError = 0; //totalError = totalError + error

int turnError; //SensorValue - DesiredValue : Position
int turnPrevError = 0; //Position 20 miliseconds ago
int turnDerivative; // error - prevError : Speed
int turnTotalError = 0; //totalError = totalError + error

bool resetDriveSensors = false;

//Variables modified for use
bool enableDrivePID = true;
bool enableFlyPID = true;
//Pasted from a C++ resource
double signnum_c(double x) {
 if (x > 0.0) return 1.0;
 if (x < 0.0) return -1.0;
 return x;
}
int flyPID()
{
 while (enableFlyPID)
 {
 Controller1.Screen.clearLine();
 Controller1.Screen.setCursor(1,1);
 Controller1.Screen.print(Flywheel1.velocity(rpm));
 //get the velocity of both motors
 int topMotorVelocity = Flywheel1.velocity(rpm);
 int bottomMotorVelocity = Flywheel2.velocity(rpm);

 //get average velocity
 int averageVelocity = (topMotorVelocity + bottomMotorVelocity)/2;

 //Potential
 flyError = desiredFlyValue - averageVelocity;

 //Derivative
 derivative = flyError - prevFlyError;

 //Integral
 if(abs(flyError) < integralBound){
 flyTotalError+=flyError; 
 } 
 else {
 flyTotalError = 0; 
 }
 double flyMotorPower = error * flykP + derivative * flykD + totalError * flykI + desiredFlyValue;
 Flywheel1.spin(forward, flyMotorPower, rpm);
 Flywheel2.spin(forward, -flyMotorPower, rpm);
 }
 return 1;
}
int drivePID()
{
 
 double currentDeg;
 double inertialError;
 double desiredDegree;
 while(enableDrivePID)
 {

 if (resetDriveSensors) {
 resetDriveSensors = false;
 BL.setPosition(0,degrees);
 BR.setPosition(0,degrees);
 desiredDegree = Inertial.rotation();
 degkP = 0;
 }
 

 //Get the position of both motors
 int leftMotorPosition = BL.position(degrees);
 int rightMotorPosition = BR.position(degrees);

 ///////////////////////////////////////////
 // movement PID
 /////////////////////////////////////////////////////////////////////
 //Get average of the two motors
 int averagePosition = (leftMotorPosition + rightMotorPosition)/2;

 //Potential
 error = desiredValue - leftMotorPosition;
 error2 = desiredValue - rightMotorPosition;
 /*
 if (averagePosition >= desiredValue/2)
 {
 degkP = 0;//cuts off inertial to prevent it hurting the stop.
 }
 */
 //Derivative
 derivative = error - prevError;

 //Integral
 if(abs(error) < integralBound){
 totalError+=error; 
 } 
 else {
 totalError = 0; 
 }
 //totalError += error;

 //This would cap the integral
 totalError = abs(totalError) > maxIntegral ? signnum_c(totalError) * maxIntegral : totalError;

 //Potential for inertial
 currentDeg = Inertial.rotation();
 inertialError = desiredDegree + currentDeg;

 double lateralMotorPowerRight = error2 * kP + derivative * kD + totalError * kI + inertialError * degkP;
 double lateralMotorPowerLeft = error * kP + derivative * kD + totalError * kI - inertialError * degkP;
 /////////////////////////////////////////////////////////////////////


 ///////////////////////////////////////////
 //Turning movement PID
 /////////////////////////////////////////////////////////////////////
 //Get average of the two motors
 int turnDifference = leftMotorPosition - rightMotorPosition;

 //Potential
 turnError = desiredTurnValue - turnDifference;

 //Derivative
 turnDerivative = turnError - turnPrevError;

 //Integral
 if(abs(error) < integralBound){
 turnTotalError+=turnError; 
 } else {
 turnTotalError = 0; 
 }
 //turnTotalError += turnError;



 //This would cap the integral
 turnTotalError = abs(turnTotalError) > maxIntegral ? signnum_c(turnTotalError) * maxIntegral : turnTotalError;

 double turnMotorPower = turnError * turnkP + turnDerivative * turnkD + turnTotalError * turnkI;
 /////////////////////////////////////////////////////////////////////

 FL.spin(forward, lateralMotorPowerLeft + turnMotorPower, voltageUnits::volt);
 FR.spin(forward, lateralMotorPowerRight - turnMotorPower, voltageUnits::volt);
 BL.spin(forward, lateralMotorPowerLeft + turnMotorPower, voltageUnits::volt);
 BR.spin(forward, lateralMotorPowerRight - turnMotorPower, voltageUnits::volt);
 
 //Print value -Test
 /*
 Brain.Screen.setCursor(1,1);
 Brain.Screen.print(LeftMotorB.position(degrees));
 Brain.Screen.setCursor(2,1);
 Brain.Screen.print(RightMotorB.position(degrees));
 Brain.Screen.setCursor(3,1);
 Brain.Screen.print(averagePosition);
 Brain.Screen.setCursor(4,1);
 Brain.Screen.print(turnDifference);
 Brain.Screen.setCursor(5,1);
 */
 Controller1.Screen.setCursor(1,1);
 Controller1.Screen.print(BL.position(degrees));
 Controller1.Screen.setCursor(2,1);
 Controller1.Screen.print(BR.position(degrees));
 Controller1.Screen.setCursor(3,1);
 Controller1.Screen.print(averagePosition);
 Controller1.Screen.setCursor(4,1);
 Controller1.Screen.print(turnDifference);
 /*
 Controller1.Screen.setCursor(3,1);
 Controller1.Screen.print(averagePosition);
 Brain.Screen.print(shiftDifferenceL);
 Brain.Screen.setCursor(5,1);
 Brain.Screen.print(shiftDifferenceR);
 */

 vex::task::sleep(20);

 }

 return 1;
}
/* 7842x PID used for reference.
void turnPID(double targetDegrees, int turnTimeOut){

 // tuning constants
 double kP = 0.0095;
 double kI = 0.001;
 double kD = 0.0001;

 double startTurnIntegral = 15;


 double currentDeg = Inertial.rotation(); // get current rotation of the robot
 double error = targetDegrees - currentDeg; // calculate the difference between wanted and current position
 double lastError = error; // store error for next loop
 double integral = error; // integral is the sum of error
 double errorRange = 0.05; // set errorRange

 // get currentTime for startTime
 int startTime = vex::timer::system();

 while ((abs(error) > errorRange)){

 // if startTime > timeOut break the loop
 if ((vex::timer::system() - startTime) > turnTimeOut) break;


 // only start integral when robot is 25 degrees away from target
 if(abs(error) > startTurnIntegral){
 integral = 0;
 }

 // limit integral both left and right
 if (integral >= 250 && integral > 1) integral = 250;
 if (integral <= -250 && integral < 1) integral = -250;

 Brain.Screen.setCursor(1, 1);
 Brain.Screen.print(Inertial.rotation());
 
 currentDeg = Inertial.rotation(); // get the current rotation of the robot

 // find the shortest way to get to the desired degree
 error = targetDegrees - currentDeg;
 error = error - floor((error + 180)/360) * 360;
 printf("turnPID error %f\n", error);

 double change = error - lastError; // calculate change in error
 lastError = error; // store error for next loop
 integral = integral + error; // integral is the sum of error

 double tPower = error * kP + integral * kI + change * kD;

 forwardVoltage(tPower, -tPower);
 wait(10, msec);
 }
 forwardVoltage(0, 0);
}
*/
bool enableGPS = true;
double totalD = 0;
int integralBoundD = 5;
int facingError;
int prevFaceError = 0;
int faceDerrivative;
double kGP = 0.1;
double kGD = 0.0;
double kGI = 0;
double kTP = 0;
double kTD = 0;
int Switch = 1;
int GPSDrive ()
{
 Controller1.Screen.clearScreen();
 Controller1.Screen.setCursor(0,0);
 while(enableGPS)
 {
 Controller1.Screen.print(facingError);
 Controller1.Screen.newLine();
 xvalue = GPS.xPosition(inches);
 yvalue = GPS.yPosition(inches);
 GPS.setOrigin(-72, -72,inches);
 deltax = xdes - xvalue;
 deltay = ydes - yvalue;
 //proportional
 double deltaD = sqrt(deltax * deltax + deltay * deltay);

 //integral
 if(fabs(deltaD) < integralBoundD){
 totalD+=deltaD; 
 } 
 else {
 totalD = 0; 
 }
 totalD = fabs(totalD) > maxIntegral ? signnum_c(totalD) * maxIntegral : totalD;
 //derrivataive
 gpsDerrivative = deltaD - prevD;


 //Heading porportion
 facing = GPS.heading();

 desiredFace = atan(deltax/deltay);

 if (deltay < 0)
 {
 desiredFace += 180;
 }

 facingError = desiredFace - facing;
 
 if(facingError > 180)
 {
 facingError = facingError - 360 ;
 }

 //Heading derrivative
 faceDerrivative = facingError - prevFaceError;
 
 

 //volts
 double leftMotorVolts = deltaD * kGP + gpsDerrivative * kGD + totalD * kGI;
 double rightMotorVolts = deltaD * kGP + gpsDerrivative * kGD + totalD * kGI;
 if (leftMotorVolts >= 10)
 {
 leftMotorVolts = 10;
 rightMotorVolts = 10;
 }
 leftMotorVolts += facingError * kTP + faceDerrivative * kTD;
 rightMotorVolts += -(facingError * kTP + faceDerrivative * kTD) ;

 //motor spinning
 FL.spin(forward, Switch * leftMotorVolts, voltageUnits::volt);
 FR.spin(forward, Switch * rightMotorVolts, voltageUnits::volt);
 BL.spin(forward, Switch * leftMotorVolts, voltageUnits::volt);
 BR.spin(forward, Switch * rightMotorVolts, voltageUnits::volt);
 
 prevD = deltaD;
 prevFaceError = facingError;
 }
 return 1;
}
//Functions
void dragon()
{
  Wing.set(true);
}
void heckenExplode ()// expand... has covered 18 tiles at most
{
 Expansion.set(true);
 vex::task::sleep(500);
 Expansion.set(false);
 vex::task::sleep(100);
 Expansion2.set(true);
 vex::task::sleep(500);
 Expansion2.set(false);
 
}
void intakeStart ()
{
 Intake1.spin(directionType::rev, 100, velocityUnits::pct);
 Roller.spin(directionType::rev, 100, velocityUnits::pct);
}
void intakeStop ()
{
 Intake1.stop(brakeType::hold);
 Roller.stop(brakeType::brake);

}
void punch () //shoot disc / activate the punch pneumatics
{
 Puncher.set(true);
 vex::task::sleep(100); //changed from 50
 Puncher.set(false);
}
void shoot () //activate the flywheel
{
 desiredFlyValue = 470; //X
}
void shoot2 () //activate the flywheel
{
 desiredFlyValue = 150; //R2
}
void shootnt () //stop flywheel
{
 desiredFlyValue = 0;
 Flywheel1.stop(brakeType::coast);
 Flywheel2.stop(brakeType::coast);
}
void moveF (int time, int speed)
{
FL.spin(directionType::fwd, speed, velocityUnits::pct);
BL.spin(directionType::fwd, speed, velocityUnits::pct);
FR.spin(directionType::fwd, speed, velocityUnits::pct);
BR.spin(directionType::fwd, speed, velocityUnits::pct);
 vex::task::sleep(time);
FL.stop(brakeType::brake);
BL.stop(brakeType::brake);
FR.stop(brakeType::brake);
BR.stop(brakeType::brake);
}
void moveB (int time, int speed)
{
FL.spin(directionType::rev, speed, velocityUnits::pct);
BL.spin(directionType::rev, speed, velocityUnits::pct);
FR.spin(directionType::rev, speed, velocityUnits::pct);
BR.spin(directionType::rev, speed, velocityUnits::pct);
 wait (time, msec);
FL.stop(brakeType::brake);
BL.stop(brakeType::brake);
FR.stop(brakeType::brake);
BR.stop(brakeType::brake);
}
void moveR (int time, int speed)//mechanum drive right aka strafe
{
FL.spin(directionType::fwd, speed, velocityUnits::pct);
BL.spin(directionType::rev, speed, velocityUnits::pct);
FR.spin(directionType::rev, speed, velocityUnits::pct);
BR.spin(directionType::fwd, speed, velocityUnits::pct);
 vex::task::sleep(time);
FL.stop(brakeType::brake);
BL.stop(brakeType::brake);
FR.stop(brakeType::brake);
BR.stop(brakeType::brake);
}

void moveL (int time, int speed) //mechanum drive left
{
FL.spin(directionType::rev, speed, velocityUnits::pct);
BL.spin(directionType::fwd, speed, velocityUnits::pct);
FR.spin(directionType::fwd, speed, velocityUnits::pct);
BR.spin(directionType::rev, speed, velocityUnits::pct);
 vex::task::sleep(time);
FL.stop(brakeType::brake);
BL.stop(brakeType::brake);
FR.stop(brakeType::brake);
BR.stop(brakeType::brake);
}
void turnR (int speed)
{
FL.spin(directionType::fwd, speed, velocityUnits::pct);
BL.spin(directionType::fwd, speed, velocityUnits::pct);
FR.spin(directionType::rev, speed, velocityUnits::pct);
BR.spin(directionType::rev, speed, velocityUnits::pct);
}
void turnL (int speed)
{
FL.spin(directionType::rev, speed, velocityUnits::pct);
BL.spin(directionType::rev, speed, velocityUnits::pct);
FR.spin(directionType::fwd, speed, velocityUnits::pct);
BR.spin(directionType::fwd, speed, velocityUnits::pct);
}

void shootMax()
{
 Flywheel1.spin(directionType::fwd, 100, velocityUnits::pct);
 Flywheel2.spin(directionType::rev, 100, velocityUnits::pct);
 vex::task::sleep(2000);
}

void stopShooting()
{
 Flywheel1.stop(brakeType::brake);
 Flywheel2.stop(brakeType::brake);
}



void halt ()// stop drive
{
FL.stop(brakeType::brake);
BL.stop(brakeType::brake);
FR.stop(brakeType::brake);
BR.stop(brakeType::brake);
}
void turnLeft (int speed, double degreesturn)
{
double ticksToTurn = (degreesturn * ticks) / 360;
FL.rotateFor(fwd, -ticksToTurn, deg, speed, velocityUnits::pct, false);
FR.rotateFor(fwd, ticksToTurn, deg, speed, velocityUnits::pct, false);
BL.rotateFor(fwd, -ticksToTurn, deg, speed, velocityUnits::pct, false);
BR.rotateFor(fwd, ticksToTurn, deg, speed, velocityUnits::pct, true);
}
void turnRight (int speed, double degreesturn)
{
double ticksToTurn = (degreesturn * ticks) / 360;
FL.rotateFor(fwd, ticksToTurn, deg, speed, velocityUnits::pct, false);
FR.rotateFor(fwd, -ticksToTurn, deg, speed, velocityUnits::pct, false);
BL.rotateFor(fwd, ticksToTurn, deg, speed, velocityUnits::pct, false);
BR.rotateFor(fwd, -ticksToTurn, deg, speed, velocityUnits::pct, true);
}


/*---------------------------------------------------------------------------*/
/* Pre-Autonomous Functions */
/* */
/* You may want to perform some actions before the competition starts. */
/* Do them in the following function. You must return from this function */
/* or the autonomous and usercontrol tasks will not be started. This */
/* function is only called once after the V5 has been powered on and */
/* not every time that the robot is disabled. */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
 // Initializing Robot Configuration. DO NOT REMOVE!
 vexcodeInit();

 // All activities that occur before the competition starts
 // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/* */
/* Autonomous Task */
/* */
/* This task is used to control your robot during the autonomous phase of */
/* a VEX Competition. */
/* */
/* You must modify the code to add your own robot specific commands here. */
/*---------------------------------------------------------------------------*/

void autonomous(void)
{
 //New auto updates 01-30-2023. We have a new speed for drive train. edits to speed and/or timings of the autos will need to be done
 //new PID created using a resource, other teams and a previous SLS team. it is my first PID.

 //switch which side the auto does
int autoside = 1; //1 right side 0 left side 2 skills. 4 PID practice. doesn't work
// (time in ms, speed in pct)




// FOR INT AUTOSIDE

// LEFT IS 0

// RIGHT IS 1

// SKILLS IS 2





//right side
if (autoside == 1)
{
 //Move Forward and shoot from near middle
 //moveF(850, 60);
 //1000 is ~4 ft 6 in, or 4.5 feet for desiredvalue
 //1000 is ~ 160 degrees
 //right turn is negative
vex::task funny(drivePID);// activate PID for turn
 resetDriveSensors = true;
 desiredValue = 520; //IMPORTANT LINE
 desiredTurnValue = 0; // the turn required
 vex::task::sleep(1000);
 task::yield();
 vex::task::sleep(100);

 vex::task funny1(drivePID);// activate PID for turn
 resetDriveSensors = true;
 desiredValue = 0;
 desiredTurnValue = 245; // the turn required 405
 vex::task::sleep(600);
 task::yield();
 //enableDrivePID = false; 

 //FlYWHEEL
 vex::task memes(flyPID); // activate flywheel PID
 desiredFlyValue = 400; // flywheel rpm 
 vex::task::sleep(2500);
 punch();
 vex::task::sleep(200);
 desiredFlyValue = 400; //down from 532/540
 vex::task::sleep(1000);
 punch();
 vex::task::sleep(500);
 shootnt(); //flywheel to 0
 vex::task::sleep(100);
 //discs shot. moving to roller
 //turnLeft(50, 68); 

 //2nd turn
 vex::task funny2(drivePID);
 resetDriveSensors = true;
 desiredValue = 0;
 desiredTurnValue = -490; // the turn required
 vex::task::sleep(1000);
 task::yield();
 
 //long move back
 vex::task::sleep(100);
 //moveB(1050,50);

 vex::task funny3(drivePID);
 resetDriveSensors = true;
 desiredValue = -600;
 desiredTurnValue = 0; // the turn required
 vex::task::sleep(1000);
 task::yield();
 vex::task::sleep(300);

 //turn right so roller is aligned with our roller
 //turnRight(50, 43);
 vex::task funny4(drivePID);
 resetDriveSensors = true;
 desiredValue = 0;
 desiredTurnValue = 340; // the turn required
 vex::task::sleep(1000);
 task::yield();
 vex::task::sleep(300);

 //moveB(700,35);
 //vex::task::sleep(300);
 vex::task funny5(drivePID);
 resetDriveSensors = true;
 desiredValue = -300;
 desiredTurnValue = 0; // the turn required
 vex::task::sleep(1000);
 task::yield();
 vex::task::sleep(300);
 
 //at roller, spinning it
 intakeStart();
 moveB(1100,25);
 vex::task::sleep(40);
 intakeStop();
 task::yield();
 task::yield();
 vex::task::stop(funny5);
}





//leftside
if (autoside == 0)
{
 
 
 //adjust the roller
 intakeStart();
 moveB(400, 50);
 vex::task::sleep(100);
 intakeStop();
 
 //go forward
 vex::task funny(drivePID);
 resetDriveSensors = true;
 desiredValue = 100;
 desiredTurnValue = 0; // the turn required
 vex::task::sleep(500);
 task::yield();
 vex::task::sleep(300);

 //turn right like 45 degrees
 vex::task funny1(drivePID);
 resetDriveSensors = true;
 desiredValue = 0;
 desiredTurnValue = 350; // the turn required
 vex::task::sleep(1000);
 task::yield();
 vex::task::sleep(300);

 //forward to middleish
 vex::task funny2(drivePID);
 resetDriveSensors = true;
 desiredValue = 890;
 desiredTurnValue = 0; // the turn required
 vex::task::sleep(1000);
 task::yield();
 vex::task::sleep(300);

 //turn left to face high goal
 vex::task funny3(drivePID);
 resetDriveSensors = true;
 desiredValue = 0;
 desiredTurnValue = -555; // the turn required
 vex::task::sleep(1000);
 task::yield();
 vex::task::sleep(300);


 vex::task megolovania(drivePID);
 resetDriveSensors = true;
 desiredValue = 180;
 desiredTurnValue = 0; // the turn required
 vex::task::sleep(500);
 task::yield();
 vex::task::sleep(300);

 //shoot twice
 vex::task bananas(flyPID); // activate flywheel PID
 desiredFlyValue = 600; // flywheel rpm 
 vex::task::sleep(4000);
 punch();
 vex::task::sleep(200);
 desiredFlyValue = 600; //down from 532/540
 vex::task::sleep(2500);
 punch();
 vex::task::sleep(200);
 desiredFlyValue = 600; //down from 532/540
 vex::task::sleep(2500);
 punch();
 vex::task::sleep(500);
 shootnt(); //flywheel to 0
 vex::task::sleep(100);
 
 
 // shootMax();
 // vex::task::sleep(2000);
 // punch();
 // vex::task::sleep(2000);
 // punch();
 // vex::task::sleep(500);

task::yield();
task::yield();
vex::task::stop(megolovania);
}
//skills
if (autoside == 2)
{
 //roller
 //adjust the roller
 intakeStart();
 moveB(400, 50);
 vex::task::sleep(400);
 intakeStop();
 vex::task::sleep(100);
 // move to shoot on same side highgoal
 moveF(200, 80);
 vex::task::sleep(100);

 vex::task funny(drivePID);
 resetDriveSensors = true;
 desiredValue = 0;
 desiredTurnValue = 563; // the turn required
 vex::task::sleep(700);
 task::yield();
 vex::task::sleep(300);

 vex::task funny2(drivePID);
 resetDriveSensors = true;
 desiredValue = 1057;
 desiredTurnValue = 0; // the turn required
 vex::task::sleep(1000);

 task::yield();
vex::task::sleep(100);
vex::task funny3(drivePID);
 resetDriveSensors = true;
 desiredValue = 0;
 desiredTurnValue = 20; // the turn required
 vex::task::sleep(500);
 task::yield();

 vex::task memes(flyPID);
 shoot2();
 vex::task::sleep(2300);
punch();
vex::task::sleep(1000);
punch();
vex::task::sleep(200);
shootnt();
vex::task::sleep(100);
task::yield();

vex::task funny4(drivePID);
 resetDriveSensors = true;
 desiredValue = 0;
 desiredTurnValue = 325; // the turn required
 vex::task::sleep(500);
 task::yield();
vex::task::sleep(100);

vex::task funny5(drivePID);
 resetDriveSensors = true;
 desiredValue = -520;
 desiredTurnValue = 0; // the turn required
 vex::task::sleep(500);
 task::yield();
vex::task::sleep(800);

intakeStart();

moveB(1000, 50);


intakeStop();

vex::task funny7(drivePID);
 resetDriveSensors = true;
 desiredValue = 0;
 desiredTurnValue = -200; // the turn required
 vex::task::sleep(500);
 task::yield();
vex::task::sleep(100);

vex::task funny6(drivePID);
 resetDriveSensors = true;
 desiredValue = 0;
 desiredTurnValue = -150; // the turn required
 vex::task::sleep(500);
 task::yield();
vex::task::sleep(100);

vex::task memes2(flyPID);
 shoot2();
 vex::task::sleep(2300);
punch();
vex::task::sleep(1000);
punch();
vex::task::sleep(200);
shootnt();
vex::task::sleep(100);
task::yield();
intakeStop();
vex::task funny8(drivePID);
 resetDriveSensors = true;
 desiredValue = 0;
 desiredTurnValue = -500; // the turn required
 vex::task::sleep(500);
 task::yield();
vex::task::sleep(100);

vex::task funny123(drivePID);
 resetDriveSensors = true;
 desiredValue = -500;
 desiredTurnValue = 0; // the turn required
 vex::task::sleep(500);
 task::yield();
vex::task::sleep(800);

vex::task funny0(drivePID);
 resetDriveSensors = true;
 desiredValue = 0;
 desiredTurnValue = 530; // the turn required
 vex::task::sleep(500);
 task::yield();
vex::task::sleep(100);

vex::task funny10(drivePID);
 resetDriveSensors = true;
 desiredValue = -820;
 desiredTurnValue = 0; // the turn required
 vex::task::sleep(500);
 task::yield();
vex::task::sleep(800);
  task::stop(drivePID);
intakeStart();
 moveB(400, 50);
 vex::task::sleep(400);
 intakeStop();
 vex::task::sleep(100);
 // move to shoot on same side highgoal
 moveF(200, 80);
 vex::task::sleep(100);

vex::task funny11(drivePID);
 resetDriveSensors = true;
 desiredValue = 0;
 desiredTurnValue = -563; // the turn required
 vex::task::sleep(500);
 task::yield();
vex::task::sleep(800);

vex::task funny12(drivePID);
 resetDriveSensors = true;
 desiredValue = -520;
 desiredTurnValue = 0; // the turn required
 vex::task::sleep(500);
 task::yield();
vex::task::sleep(800);

vex::task funny13(drivePID);
 resetDriveSensors = true;
 desiredValue = 0;
 desiredTurnValue = 223; // the turn required
 vex::task::sleep(500);
 task::yield();
vex::task::sleep(800);
heckenExplode();
}
if (autoside == 4)
{
 vex::task memes(GPSDrive);
 xdes = 72;
 ydes = 72;
}
}
/*---------------------------------------------------------------------------*/
/* */
/* User Control Task */
/* */
/* This task is used to control your robot during the user control phase of */
/* a VEX Competition. */
/* */
/* You must modify the code to add your own robot specific commands here. */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
 // User control code here, inside the loop
 vex::task::stop(drivePID);
 
 while (1) {

 //get the velocity of both motors
 int topMotorVelocity = Flywheel1.velocity(rpm);
 int bottomMotorVelocity = Flywheel2.velocity(rpm);

 //get average velocity
 int averageVelocity = (topMotorVelocity + bottomMotorVelocity)/2;

 //Potential
 error = desiredFlyValue - averageVelocity;

 //Derivative
 derivative = error - prevError;

 //Integral
 if(abs(error) < integralBound){
 totalError+=error; 
 } 
 else {
 totalError = 0; 
 }
 double flyMotorPower = error * flykP + derivative * flykD + totalError * flykI + desiredFlyValue;
 Flywheel1.spin(forward, flyMotorPower, rpm);
 Flywheel2.spin(forward, -flyMotorPower, rpm);
 
 //print the fly wheel temps as a percentage... 50 percent is the begining of the motors slowing
 Controller1.Screen.clearLine();
 Controller1.Screen.setCursor(1,1);
 Controller1.Screen.print(Inertial.rotation());
 Controller1.Screen.setCursor(2,1);
 Controller1.Screen.print(FL.temperature(percent));
 // print fly wheel velocity for knowing when to shoot
 Controller1.Screen.setCursor(3,1);
 Controller1.Screen.print(Flywheel2.velocity(rpm));
 Controller1.Screen.setCursor(4,1);
 Controller1.Screen.print(Flywheel1.velocity(rpm));
 //drive: split arcade with mechanum
 FL.spin(directionType::fwd, (Controller1.Axis1.value() + Controller1.Axis3.value() + (Controller1.Axis4.value()/4))/2, velocityUnits::pct);
 FR.spin(directionType::fwd, (Controller1.Axis3.value() - Controller1.Axis1.value() - (Controller1.Axis4.value()/4))/2, velocityUnits::pct);
 BL.spin(directionType::fwd, (Controller1.Axis1.value() + Controller1.Axis3.value() + (Controller1.Axis4.value()/4))/2 , velocityUnits::pct);
 BR.spin(directionType::fwd, (Controller1.Axis3.value() - Controller1.Axis1.value() - (Controller1.Axis4.value()/4))/2, velocityUnits::pct); 
 //intake and roller 
 if(Controller1.ButtonL1.pressing())
 {
 Intake1.spin(directionType::fwd, 100, velocityUnits::pct);
 Roller.spin(directionType::rev, 100, velocityUnits::pct);
 }
 else if(Controller1.ButtonL2.pressing())
 {
 Intake1.spin(directionType::rev, 100, velocityUnits::pct);
 Roller.spin(directionType::fwd, 100, velocityUnits::pct);
 }
 else
 {
 Intake1.stop(brakeType::hold);
 Roller.stop(brakeType::brake);
 }
 //start the flywheel
 Controller1.ButtonR2.pressed(shoot2); //small shoot
 Controller1.ButtonX.pressed(shoot); //big shoot
 Controller1.ButtonR1.pressed(shootnt);
 // punch discs
 Controller1.ButtonA.pressed(punch);
 //expand... not to be pressed prematurely!!!
 Controller1.ButtonDown.pressed(heckenExplode);
 Controller1.ButtonY.pressed(dragon);
 wait(20, msec); 
}
}
//
// Main will set up the competition functions and callbacks.
//
int main() {
 // Set up callbacks for autonomous and driver control periods.
 Competition.autonomous(autonomous);
 Competition.drivercontrol(usercontrol);

 // Run the pre-autonomous function.
 pre_auton();

 // Prevent main from exiting with an infinite loop.
 while (true) {
 wait(100, msec);
 }
}