#include "pid.h"
using namespace pros;

//define global variables here

//motors, pneumatic objects
extern Motor backLift;
extern Motor frontLift;
extern Motor backLeft;
extern Motor midLeft;
extern Motor frontLeft;
extern Motor backRight;
extern Motor midRight;
extern Motor frontRight;
extern Piston frontPneu;
extern Piston backPneu;
extern Piston frontSpecialPneu;
extern Controller con;
extern IMU inert;


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