#include "pid.h"
//define global variables here

//motors, pneumatic objects
extern pros::Motor backLift;
extern pros::Motor frontLift;
extern pros::Motor backLeft;
extern pros::Motor midLeft;
extern pros::Motor frontLeft;
extern pros::Motor backRight;
extern pros::Motor midRight;
extern pros::Motor frontRight;
extern Piston frontPneu;
extern Piston backPneu;
extern Piston frontSpecialPneu;
extern pros::Controller con;
extern pros::IMU inert;


//global variables
extern double frontGoals;
extern double backGoals;
extern double globalRotation;

extern PID turnPID;
extern PID drivePID;
extern PID autoStraightPID;

//auton global def
extern void redElevatedLong();
extern void redElevatedShort();
extern void redDeElevatedLong();
extern void redDeElevatedShort();
extern void blueElevatedLong();
extern void blueElevatedShort();
extern void blueDeElevatedLong();
extern void blueDeElevatedShort();
extern void redBoth();
extern void blueBoth();
extern void neutralRush();
extern void skills();
extern pros::Motor backLift;
extern pros::Motor frontLift;
extern pros::Motor backLeft;
extern pros::Motor midLeft;
extern pros::Motor frontLeft;
extern pros::Motor backRight;
extern pros::Motor midRight;
extern pros::Motor frontRight;
extern void delay();


//motionProfile
extern void generatePath();
extern void move();