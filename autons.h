#include "main.h"
#include "chassis.h"

Chassis ch;

double backGoals = 0;
double frontGoals = 0;
void drive(double targetEnc, int timeout = 4000, double maxspeed = 1, double errorRange = 3)
{
	if(timeout == 4000) {timeout = abs(targetEnc)*4;}
	ch.drive(targetEnc, timeout, maxspeed, errorRange);
}
void curve(double degrees, bool clockwise=true, double amplifier=1, int timeout=5000) {ch.curve(degrees, clockwise, amplifier, timeout);}
void rotate(double degrees, int timeout = 5000, double maxspeed = .9)  { ch.rotate(degrees, timeout, maxspeed); }
void rotateTo(double degrees, int timeout = 5000, double speedM  = .9) { ch.rotate(degrees - globalRotation, timeout, speedM); }

void redElevatedLong()
{
	ch.spinTo(700,127);
	frontPneu.toggle();
	frontGoals = 1;
	frontLift.move_absolute(-800, -127);
	ch.spinTo(-700, -127);
	ch.spinTo(-500, -50, 1, 1500);
	drive(450, 2000);
	rotateTo(-90);
	backLift.move_absolute(-2400, -100);
	drive(-100, 2000);
	delay(500);
	drive(300);
	backLift.move_absolute(0, 127);
	rotateTo(-80, 900);
	drive(-500, 1200);
	backPneu.toggle();
	delay(200);
	drive(400);
}

void redElevatedShort()
{
	ch.spinTo(700,127);
	frontPneu.toggle();
	frontGoals = 1;
	frontLift.move_absolute(-800, -127);
	drive(-655, 2500, 1, 80);
	frontPneu.toggle();
	drive(-250);
	rotateTo(-39, 1200);
	frontLift.move_absolute(0, -127);
	drive(1000, 2000, 1, 50);
	frontPneu.toggle();
	frontGoals = 1;
	frontLift.move_absolute(-800, -127);
	drive(-1000);
	backLift.move_absolute(-3000, -127);
	rotateTo(-135, 1400);
	drive(-400, 1500);
	backLift.move_absolute(-1000, 127);
	delay(200);
	drive(200);



}

void redDeElevatedLong()
{
	ch.spinTo(760,127);
	frontPneu.toggle();
	frontGoals = 1;
	frontLift.move_absolute(-800, -127);
	ch.spinTo(-800, -127);
	ch.stop();
	rotateTo(-20);
	ch.spinTo(-400, -50, 1, 1500);
	drive(165, 2000);
	/*
	rotate(-80, 1500);
	backLift.move_absolute(-2400, -127);
	drive(-100, 1500);
	delay(500);
	drive(300);
	*/
	rotateTo(-81, 2000);
	backLift.move_absolute(-3000, -127);
	drive(150, 1500);
	drive(-500, 1500);
	backLift.move_absolute(-1000, 127);
	delay(500);

}

void redDeElevatedShort()
{
	ch.curveNew(250, 250, 90);
}

void redBoth()
{
	ch.park();
}

void blueElevatedLong()
{
	ch.spinTo(700,127);
	frontPneu.toggle();
	frontGoals = 1;
	frontLift.move_absolute(-800, -127);
	ch.spinTo(-700, -127);
	ch.spinTo(-500, -50, 1, 1500);
	drive(450, 2000);
	rotateTo(-90);
	backLift.move_absolute(-2400, -100);
	drive(-100, 2000);
	delay(500);
	drive(300);
	backLift.move_absolute(0, 127);
	rotateTo(-80, 900);
	drive(-500, 1200);
	backPneu.toggle();
	delay(200);
	drive(400);
}

void blueElevatedShort()
{
	ch.spinTo(700,127);
	frontPneu.toggle();
	frontGoals = 1;
	frontLift.move_absolute(-800, -127);
	drive(-725);
	rotateTo(50, 1500);
	frontPneu.toggle();
	drive(-125);
	rotateTo(-44, 1500);
	frontLift.move_absolute(0, -127);
	drive(700, 2000);
	frontPneu.toggle();
	frontGoals = 1;
	frontLift.move_absolute(-800, -127);
	drive(-800);
}

void blueDeElevatedLong()
{
	ch.spinTo(760,127);
	frontPneu.toggle();
	frontGoals = 1;
	frontLift.move_absolute(-800, -127);
	ch.spinTo(-800, -127);
	ch.stop();
	rotate(-20);
	ch.spinTo(-400, -50, 1, 1500);
	drive(165, 2000);
	rotate(-82, 1500);
	backLift.move_absolute(-2400, -127);
	drive(-85, 1500);
	delay(500);
	drive(300);
}

void blueDeElevatedShort()
{
	drive(200);
	rotate(45);
	drive(400);
	rotate(90);
	drive(100);
	rotate(45);
	drive(600);
}

void blueBoth()
{
	drive(500);
}

void neutralRush()
{
	ch.spinTo(760,127);
	frontPneu.toggle();
	frontLift.move_absolute(-600, -127);
	drive(-700);
}


void skills()
{
	//actually skills lol
	backLift.move_absolute(-3000, -127);
	drive(410);
	rotateTo(-90);
	drive(-300, 1500);
	backLift.move_absolute(-1200, 127);

	backGoals = 1;

	delay(500);
	drive(300);
	rotateTo(0);
	drive(400);
	frontPneu.toggle();
	delay(100);
	frontLift.move_absolute(-750, -127);

	frontGoals = 1;

	drive(150);
	rotateTo(30);
	drive(710);
	rotateTo(0);
	drive(300);
	frontLift.move_absolute(10, 127);
	delay(200);
	frontPneu.toggle();

	frontGoals = 0;

	delay(1000);
	drive(-250, 2000);
	rotateTo(-46);
	drive(475);
	frontPneu.toggle();
	delay(200);
	frontLift.move_absolute(-750, -127);

	frontGoals = 1;

	delay(650);
	drive(-435);
	rotateTo(-90);
	backLift.move_absolute(-3050, 127);
	drive(200);
	frontLift.move_absolute(-1200, -127);
	delay(300);

	backGoals = 0;

	drive(170);
	backLift.move_absolute(30, 127);
	//delay(500);
	rotateTo(90);
	backLift.move_absolute(-3000, -127);
	drive(-1000, 2000, 0.7);
	rotateTo(90, 500, 1.2);
	drive(-550, 1000);

	backGoals = 1;

	backLift.move_absolute(100, 127);
	delay(100);
	rotateTo(90, 800, 1.2);
	drive(250);
	rotateTo(0, 2000);
	backLift.move_absolute(-3000, -127);
	delay(650);
	drive(-525, 2000, 0.4);
	backLift.move_absolute(-1200, 127);

	frontGoals = 1;
	backGoals = 2;


	delay(200);
	rotateTo(35, 1000);
	drive(-300, 1500);
	rotateTo(180, 2000);
	frontLift.move_absolute(-4200, -127);
	drive(1130, 3100, 0.8);
	drive(-100, 800);
	rotateTo(90,2000, 1.05);
	drive(300, 1000);
	frontLift.move_absolute(-2000, -127);
	frontGoals = 2;
	backGoals = 2;

	rotateTo(88.5, 500, 2);
	ch.park();
}





/*
void drive(double targetEnc, int timeout = 4000, double maxspeed = .6, double errorRange = 3)
{
	/*drive function called with targetEnc required, timeout defaulted to 4 seconds,
	maxspeed defaulted to 60%, and max error range to exit at 3 encoders

	// Timeout counter
	int time = 0;
	float slewMult = 0.05;	//slew to prevent jerk
	// Drive distance variables: uses motor encodings with distance error
	double leftStartPos = chas.getLeftPos();
	double rightStartPos = chas.getRightPos();
	double distError = 0.1;
	double currentPos = 0.0;
	double baseSpeed = 0.0;
	float distKp = 1.5;
	bool withinRange = false;
	int withinRangeTime = 0;
	// Drive straight variables: uses the intertial with PID
	inert.set_heading(180);
	double initialRotation = inert.get_heading();
	double error = 0.0;
	float kP = (targetEnc >= 0) ? (1) : (-1);
	kP *= (1 + goalsPossessed/3);		//change kP based on how many goals(will change weight of robot)
	// Drive loop, might add a timeout thing if it's needed
	while(timeout > time && !killAuton)		//killAuton exits auton in case of crucial error to prevent damage
	{
		// Drive code: Distance error set to target encoding - average of left + right encodings
		distError = targetEnc - ((leftStartPos - chas.getLeftPos()) + (rightStartPos - chas.getRightPos()) / 2);
		baseSpeed = slewMult * ((abs(distError * distKp) > 127 * maxspeed) ? (targetEnc > 0 ? 127 * maxspeed : -127 * maxspeed) : (distError * distKp));
		if(distError < -3)
		{
			baseSpeed = (baseSpeed > -15) ? (-15) : (baseSpeed);
		}
		else if(distError > 3)
		{
			baseSpeed = (baseSpeed < 15) ? (15) : (baseSpeed);
		}
		// Drive straight code: Changes left side of the chassis' speed according to the intertial sensor's readings
		error = initialRotation - inert.get_heading();
		if(abs(error) > 25)	//kill auton if the robot jerks off course severely
		{
			killAuton = true;
			break;
		}
		// Apply speeds
		chas.spinLeft(baseSpeed + (error * kP)*(baseSpeed/80));
		chas.spinRight(baseSpeed - (error * kP)*(baseSpeed/80));
		//break if within error range for more than 500 ms
		if(abs(distError) < errorRange)
		{
			if(!withinRange)
		{
				withinRangeTime = time;
				withinRange = true;
			}
			else if(time >= withinRangeTime + 500) { break; }
		}
		else
		{
			withinRange = false;
		}
		// delay while loop
		delay(10);
		time += 10;
		currentPos = chas.getLeftPos();
		if(slewMult < 1) {slewMult += 0.05;}
	}
	// Stop robot after loop
	chas.changeBrake(chas.HOLD);
	chas.stop();
	globalRotation += inert.get_heading() - initialRotation;	//used for absolute rotate function
}
void rotate(double degrees, int timeout = 60000, double speedM = 1)
{
	// Timeout counter
	if(speedM > 1) {speedM = speedM/100;}
	int time = 0;
	// ensure the inertial sensor does not go from 359 degrees to 0
	if(degrees < 0)
		inert.set_heading(350);
	else if(degrees > 0)
		inert.set_heading(10);
    else
		return;
	double targetHeading = inert.get_heading() + degrees;
	double currentRotation = inert.get_heading();
	double initialRotation = inert.get_heading();
	double error = targetHeading - currentRotation;
	double lastError = error;
	double integral = 0.0;
	double derivative = 0.0;
	double speed = 0.0;
	double rotateStartI = goalsPossessed + (1 + goalsPossessed/4);
	float kP = (1.5*(90/degrees) > 2 ? (2) : (1.5*(90/degrees) < 1.5 ? (1.5) : 1.5*(90/degrees))) * (1 + goalsPossessed/6);
	float kI = 0.1;
	float kD = 2;
	bool integ = false;
	bool withinRange = false;
	int withinRangeTime = 0;
	int sameErrorCount = 0;
	double errorCheck;
	while(!killAuton)
	{
		currentRotation = inert.get_heading();
		if(time % 200 == 0)
		{
			errorCheck = error;
		}
		lastError = error;
		error = targetHeading - currentRotation;
		if(abs(error) <= rotateStartI)
		{
			integ = true;
		}
		else if(abs(errorCheck - error) < 0.0001 && error < 5)
		{
			sameErrorCount++;
		}
		else
		{
			sameErrorCount = 0;
		}
		if(integ) { integral += error; }
		derivative = lastError - error;
		speed = (error * kP) + (integral * kI) + (derivative * kD);
		chas.spinLeft(speed * speedM);
		chas.spinRight(-speed * speedM);
		if(abs(error) <= 0.5)
		{
			if(!withinRange)
		 	{
				withinRangeTime = time;
				withinRange = true;
			}
			else if(time >= withinRangeTime + 300) { break; }
		}
		else { withinRange = false; }
		if(time > timeout) {break;}
		// delay while loop
		delay(5);
		time += 5;
	}
	// Stop robot after loop
	chas.changeBrake(chas.HOLD);
	chas.stop();
	globalRotation += inert.get_heading() - initialRotation; 	//is used for absolute rotate
}
//absolute rotate
void rotateTo(double degrees, int timeout=100000, double speedM  = 1)
{
	rotate(degrees - globalRotation, timeout, speedM);
}
// curve
void curve(double degrees, double strength, double timeout)
{
	int time = 0;
	if(degrees > 0)
		inert.set_heading(10);
	else if(degrees < 0)
		inert.set_heading(350);
	else
		return;

	double startDeg = inert.get_heading();
	double error;
	float kP = 1.1;
	int withinRangeTime = 0;
	bool withinRange = false;
	while(time <= timeout)
	{
		error = degrees - (inert.get_heading() - startDeg);
		double baseSpeed = (error * kP)
		if(degrees > 0)
		{
			chas.spinLeft(baseSpeed + (strength * (error/2) / 100));
			chas.spinright(baseSpeed);
		}
		else
		{
			chas.spinLeft(baseSpeed);
			chas.spinright(baseSpeed + (strength * (error/2) / 100));
		}
		if(abs(error < 1.0))
		{
			if(!withinRange)
				withinRange = true;
				withinRangeTime = time
			else if(withinRangeTime + 400 <= time)
			{
				chas.stop()
				return;
			}
		}
		else
		{
			withinRange = false;
		}
		delay(5);
		time += 5;
	}
}
void redElevatedLong() // final b4 comp for all reds
{
	goalsPossessed = 0;
	drive(400, 1800, 1, 10);
	drive(111,230, 1);
	frontPneu.toggle();
	frontLift.move_absolute(-200,-127);
	goalsPossessed = -0.3;
	delay(300);
	drive(-215,1300);
	backLift.move_absolute(-2230,-127);
	rotateTo(-95,2000);
	drive(50, 500);
	drive(-88,1000, 0.4);
	delay(250);
	drive(150,1000);
	delay(250);
	rotate(18.5, 1000);
	backLift.move_absolute(-3000,-127);
	delay(250);
	drive(-250,1500);
	backLift.move_absolute(-2000,127);
	delay(250);
	rotate(-55, 1000);
	drive(200, 1000);
}
void redElevatedShort()
{

	drive(150);
	drive(20,500);
	frontPneu.toggle();
	frontLift.move_absolute(-200, -127);
	drive(-150);
}
void redDeElevatedLong()
{
	drive(415, 1800, 1, 10);
	drive(200,200);
	frontPneu.toggle();
	goalsPossessed = -0.3;
	drive(-100, 500, 1, 10);
	frontLift.move_absolute(-200,-127);
	drive(-305,1500);
	backLift.move_absolute(-1800, -127);
	rotateTo(-86,2000);
	drive(-30,800);
	backLift.move_absolute(-2442, -127);
	delay(1000);
	drive(120, 1200);
}
void redDeElevatedShort()
{
	drive(130);
	drive(25, 350);
	frontPneu.toggle();
	delay(200);
	frontPneu.toggle();
	drive(-150);
}
void redBoth() // 10/11
{
	drive(150, 1500);
	drive(-150, 800);
	rotateTo(-90, 1600);
	drive(175, 2000);
	rotateTo(-179, 1400);
	drive(-755, 4000, 0.73);
	backLift.move_absolute(-2160, -127);
	rotateTo(-160, 900);
	delay(400);
	drive(-90, 1300, 0.4);
	drive(100, 1000);
	backLift.move_absolute(-2750, -127);
	drive(-200, 1500);
	backLift.move_absolute(-1800, 127);
	delay(300);
	drive(210);
}
void blueElevatedLong()
{
	goalsPossessed = 0;
	drive(400, 1800, 1, 10);
	drive(111,230, 1);
	frontPneu.toggle();
	frontLift.move_absolute(-200,-127);
	goalsPossessed = -0.3;
	delay(300);
	drive(-215,1300);
	backLift.move_absolute(-2230,-127);
	rotateTo(-95,2000);
	drive(50, 500);
	drive(-88,1000, 0.4);
	delay(250);
	drive(150,1000);
	delay(250);
	rotate(18.5, 1000);
	backLift.move_absolute(-3000,-127);
	delay(250);
	drive(-250,1500);
	backLift.move_absolute(-2000,127);
	delay(250);
	rotate(-55, 1000);
	drive(200, 1000);
}
void blueElevatedShort()
{

	drive(150);
	drive(20,500);
	frontPneu.toggle();
	frontLift.move_absolute(-200, -127);
	drive(-150);
}
void blueDeElevatedLong()
{
	drive(415, 1800, 1, 10);
	drive(200,200);
	frontPneu.toggle();
	goalsPossessed = -0.3;
	drive(-100, 500, 1, 10);
	frontLift.move_absolute(-200,-127);
	drive(-305,1500);
	backLift.move_absolute(-1800, -127);
	rotateTo(-86,2000);
	drive(-30,800);
	backLift.move_absolute(-2442, -127);
	delay(1000);
	drive(120, 1200);
}
void blueDeElevatedShort()
{
	drive(130);
	drive(25, 350);
	frontPneu.toggle();
	delay(200);
	frontPneu.toggle();
	drive(-150);
}
void blueBoth()
{
	drive(150, 1500);
	drive(-150, 800);
	rotateTo(-90, 1600);
	drive(175, 2000);
	rotateTo(-179, 1400);
	drive(-755, 4000, 0.73);
	backLift.move_absolute(-2160, -127);
	rotateTo(-160, 900);
	delay(400);
	drive(-90, 1300, 0.4);
	drive(100, 1000);
	backLift.move_absolute(-2750, -127);
	drive(-200, 1500);
	backLift.move_absolute(-1800, 127);
	delay(300);
	drive(210);
}
void skillsFuture()
{
	backLift.move_absolute(-2450, -127);
	delay(1500);
	drive(-130, 2000);
	backLift.move_absolute(0, 127);
	goalsPossessed++;
	delay(1500); // first alliance goal should be obtained by here
	drive(130, 2000);
	rotateTo(-90);
	drive(140, 2000);
	rotateTo(-179);
	drive(-700, 4000, 0.73); // drive far towards 2nd alliance goal
	backLift.move_absolute(-2450, -127);
	drive(-150, 2000, 0.5);
	backLift.move_absolute(-1000, 126);
	delay(750); // 2nd alliance goal should be obtained by here
	goalsPossessed++;
	drive(800), 1500;
	rotateTo(-270, 1500);
	drive(400, 1500);
	drive(150, 1500);
	frontPneu.toggle(); // 3rd alliance goal should be obtained here
	goalsPossessed++;
	frontLift.move_absolute(-2000, -127);
	drive(400, 2000);
	rotate(90);

}
void skills()
{

	drive(-220);
	rotateTo(90);
	backLift.move_absolute(-3000, -127);
	delay(1500);
	drive(-120);
	backLift.move_absolute(-1000, 127);
	goalsPossessed++;
	delay(500);
	drive(140);
	rotateTo(0, 3000);
	backLift.move_absolute(-800, 127);
	delay(800);
	backLift.move_absolute(300, 127);
	delay(1500);
	backLift.move_absolute(-3000, -127);
	delay(1500);
	drive(-250, 3000, 0.35);
	backLift.move_absolute(-1500, 127);
	//goalsPossessed++;
	delay(1000);
	backLift.move(20);
	drive(-150);
	rotateTo(26, 3000);
	drive(-300);
	rotateTo(180, 4000, 0.75);
	drive(120);
	rotateTo(105);
	drive(120, 1200);
	//frontPneu.toggle();
	rotateTo(91, 1200);
	goalsPossessed = 3;
	park();
}
void neutralRush()
{
	chas.spinTo(2100,127);
	frontPneu.toggle();
	frontLift.move_absolute(-600, -127);
	delay(200);
	drive(-400);
	rotate(-140,4000,0.4);
}
*/
