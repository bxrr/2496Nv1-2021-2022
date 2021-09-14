#include "main.h"
#include "chassis.h"
#include "ports.h"
#include "piston.h"
#include <iostream>

#include <string.h>

// Globals
Chassis chas;
Piston frontPneu(FRONT_PNEUMATIC_PORT);
Piston backPneu(BACK_PNEUMATIC_PORT);
pros::Motor backLift(BACK_LIFT_PORT, pros::E_MOTOR_GEARSET_18, false);
pros::Motor frontLift(FRONT_LIFT_PORT, pros::E_MOTOR_GEARSET_36, false);

pros::Imu inert(INERT_PORT);
pros::Controller con(pros::E_CONTROLLER_MASTER);

int globalTime;		//time since code has initialized, used as a timer
bool disableAll = false;
double globalRotation = 0;

// misc functions ==============================================================
void checkInertial(int lineNum=1)
{
	while(inert.is_calibrating())
	{
		pros::lcd::set_text(lineNum, "inertial is calibrating...");
	}
	if(pros::lcd::is_initialized())
	{
		pros::lcd::clear_line(lineNum);
	}
}

bool autonCurrentlySelecting = true;
int autonNum = 1;
void autonSelector()
{
	static bool firstTime = true;
	static int localTime = 0;
	if(localTime > 100) {localTime = 0;}
	if(localTime == 50) {con.set_text(0,0, "Select Auton:");}

	if (con.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT))
	{
		con.clear();
		if(autonNum == 6){autonNum = 1;}
		else{autonNum++;}
		while (con.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)){}
	}
	else if (con.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT))
	{
		con.clear();
		if(autonNum == 0){autonNum = 6;}
		else{autonNum--;}
		while (con.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)){}
	}
	
	if(localTime ==100) 
	{
		if (autonNum == 1){con.set_text(1,0, "Red  1");}
		else if (autonNum == 2){con.set_text(1,0, "Red  2");}
		else if (autonNum == 3){con.set_text(1,0, "Blue 1");}
		else if (autonNum == 4){con.set_text(1,0, "Blue 2");}
		else if (autonNum == 5){con.set_text(1,0, "Skills");}
		else if(autonNum == 6) {con.set_text(1,0, "None");}
 	}

	if (con.get_digital(pros::E_CONTROLLER_DIGITAL_A))
	{
		autonCurrentlySelecting = false;
	}

	localTime += 5;
	pros::delay(5);
	
}



// auton functions =============================================================
double goalsPossessed = 0;

void drive(double targetEnc, int timeout = 4000, double maxspeed = .6) // timeout in milliseconds
{
	// Timeout counter
	int time = 0;
	
	float slewMult = 0.05; 

	// Drive distance variables: uses motor encodings with distance error
	double leftStartPos = chas.getLeftPos();
	double rightStartPos = chas.getRightPos();

	double distError = 0.1;
	double currentPos = 0.0;
	double baseSpeed = 0.0;

	float distKp = 0.1;

	bool withinRange = false;
	int withinRangeTime = 0;

	// Drive straight variables: uses the intertial with PID
	inert.set_heading(180);
	double initialRotation = inert.get_heading();

	double lastError, derivative;
	double error = 0.0;
	double integral = 0.0;

	float kP = (targetEnc >= 0) ? (5) : (-5);
	kP *= (1 + goalsPossessed/3);

	con.clear();

	// Drive loop, might add a timeout thing if it's needed
	while(timeout > time)
	{
		if(time % 50 == 0) {con.print(1,0,"GH: %.1f", (distError));}
		// Drive code: Distance error set to target encoding - average of left + right encodings
		distError = targetEnc + (rightStartPos - chas.getRightPos());
		baseSpeed = slewMult * ((abs(distError * distKp) > 127 * maxspeed) ? (targetEnc > 0 ? 127 * maxspeed : -127 * maxspeed) : (distError * distKp)); //? (distError * distKp) : (5); // If the base speed is below 3.5, set the base speed to 3.5
	
		//200 - (negative + )

		// Drive straight code: Changes left side of the chassis' speed according to the intertial sensor's readings
		

		lastError = error;
		error = initialRotation - inert.get_heading();
		integral += error;
		derivative = error - lastError;

		// Apply speeds
		chas.spinLeft(baseSpeed + (error * kP)*(baseSpeed/80));
		chas.spinRight(baseSpeed - (error * kP)*(baseSpeed/80));
		
		if(abs(distError) < 3)
		{
			if(!withinRange)
		{
				withinRangeTime = time;
				withinRange = true;
			}
			else if(time >= withinRangeTime + 500)
			{
				break;
			}
		}
		else
		{
			withinRange = false;
		}
		
		// delay while loop
		pros::delay(10);
		time += 10;
		// check timeout

		currentPos = chas.getLeftPos();
		if(slewMult < 1) {slewMult += 0.05;}
	}
	// Stop robot after loop
	chas.changeBrake(chas.HOLD);
	chas.stop();
	globalRotation += inert.get_heading() - initialRotation;
}

enum rotateDirection {CW, CCW}; // clockwise / counter clockwise

void rotate(double degrees, int timeout = 100000, rotateDirection dir=CW)
{
	if(dir == CCW)
		degrees = -degrees;
	// Timeout counter
	int time = 0;

	// Rotate variables: Uses inertial sensor and slows down as it gets closer to the target by using an error
	if(degrees < 0)
		inert.set_heading(350);
	else if(degrees > 0)
		inert.set_heading(10);
    else
		return;

	//double globalHeading = inert.get_heading();
	double targetHeading = inert.get_heading() + degrees;
	double currentRotation = inert.get_heading();
	double initialRotation = inert.get_heading();

	double error = targetHeading - currentRotation;
	double lastError = error;
	double integral = 0.0;
	double derivative = 0.0;

	double speed = 0.0;

	double rotateStartI = goalsPossessed + (1 + goalsPossessed/4);
	float kP = (1.5*(90/degrees) > 2 ? (2) : (1.5*(90/degrees) < 1.5 ? (1.5) : 1.5*(90/degrees))) * (1 + goalsPossessed/2);
	float kI = 0.1;
	float kD = 2;

	bool integ = false;
	bool withinRange = false;
	int withinRangeTime = 0;

	int sameErrorCount = 0;
	double errorCheck;

	while(true)
	{
		if(time % 50 == 0) {con.print(1,0,"GH: %d", time);}
		currentRotation = inert.get_heading();

		if(time % 200 == 0)
		{
			errorCheck = error;
		}

		/*
		if(dir == CW)
			globalHeading += (currentRotation >= lastRotation) ? (currentRotation - lastRotation) : ((360 - lastRotation) + currentRotation);
		else
			globalHeading += (currentRotation <= lastRotation) ? (currentRotation - lastRotation) : ((currentRotation - 360) - lastRotation);
		*/

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

		chas.spinLeft(speed);
		chas.spinRight(-speed);
		
		if(abs(error) <= 0.5)
		{
			if(!withinRange)
		 	{
				withinRangeTime = time;
				withinRange = true;
			}
			else if(time >= withinRangeTime + 300)
			{
				break;
			}
		}
		else
		{
			withinRange = false;
		}

		if(time > timeout) {break;}

		// delay while loop
		pros::delay(5);
		time += 5;
	}
	// Stop robot after loop 
	chas.changeBrake(chas.HOLD);
	chas.stop();
	globalRotation += inert.get_heading() - initialRotation;
}


void rotateTo(double degrees, int timeout=100000, rotateDirection dir = CW) { rotate(degrees - globalRotation, timeout, dir); }

// driver control functions ====================================================

void arcadeDrive()
{
	if(abs(con.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) > 10 || abs(con.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) > 10)
	{
		double turnStick = (chas.reverseStatus()) ? (-con.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) : (con.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
		chas.spinLeft(con.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) + turnStick);
		chas.spinRight(con.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) - turnStick);
	}
	else
	{
	chas.stop();
	}
}



double autoStraightVal = 0;
void arcadeAuto()
{
	static double inertialStart;
	static bool autoStraight = false;
	double kpAuto = con.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) > 0 ? 0.02 : -0.02;
	if(abs(con.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) > 10 || abs(con.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) > 10)
	{
			double turnStick = (chas.reverseStatus()) ? (-con.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) : (con.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
			if(abs(turnStick) < 10 && !autoStraight) 
			{
				autoStraight = true;
				inert.set_heading(180);
				inertialStart = inert.get_heading();
			}
			else if(abs(turnStick) > 10) {autoStraight = false;}

			if(autoStraight) 
			{
				autoStraightVal = inert.get_heading() - inertialStart;
			}
			else {autoStraightVal = 0;}

			chas.spinLeft(con.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) + turnStick - (autoStraightVal*kPauto)*con.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
			chas.spinRight(con.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) - turnStick + (autoStraightVal*kPauto)*con.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
	}
	else
	{
		chas.stop();
	}
}


void tankDrive()
{
	if(abs(con.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) > 10 || abs(con.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)) > 10)
	{
		chas.spinLeft(con.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
		chas.spinRight(con.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
	}
	else
	{
		chas.stop();
	}
}

void reverseToggle()
{
	if(con.get_digital(pros::E_CONTROLLER_DIGITAL_Y))
		chas.reverseControls();
	else
		chas.reverseReleased();
}

//brake type control
bool disableAuto = false;
void brakeType()
{
	bool brakeinitiate = true;
	static int startTime = 0;
	if(con.get_digital(pros::E_CONTROLLER_DIGITAL_UP))
	{
		if(chas.getBrakeMode() == 0) {chas.changeBrake(chas.HOLD);}
		else {chas.changeBrake(chas.COAST);}
		disableAuto = true;
		startTime = globalTime;
		while(con.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {}
	}
	//disable automatic brake mode selection if manual selection has been made within 10 sec
	if(!(startTime == 0) && !disableAll) {globalTime - startTime < 8000 ? disableAuto = true : disableAuto = false;}
}

void autoBrakeMode()	//automatically sets brake mode
{
	static bool getStartTime = true;
	static int startTime;
	//disable automatic brake mode selection if manual selection has been made within 10 sec
	if(!disableAuto)
	{
		//set brake type to hold if robot is on platform and is at risk of sliding off
		if(abs(int(inert.get_pitch())) > 10 && globalTime > 3000)
		{
			if (abs(con.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) > 10|| abs(con.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) > 10)
			{
				chas.changeBrake(chas.HOLD);
			}
			else
			{
				chas.changeBrake(chas.S_HOLD, inert.get_pitch());
			}
			getStartTime = true;		//reset the 2 second timer
		}
		//set brake type to coast w/ 2 second delay
		else
		{
			if(chas.getBrakeMode() == 1 && globalTime > 3000)
			{
				if(getStartTime)
				{
					startTime = globalTime;
					getStartTime = false;
				}
				if (globalTime - startTime > 2000)
				{
					chas.changeBrake(chas.COAST);
					chas.stop();
					getStartTime = true;
				}
			}
		}
	}
}


void autoPark()
{
	static bool aPark = false;
	static int goalsPossessed = 0;
	if(con.get_digital(pros::E_CONTROLLER_DIGITAL_B)) 
	{
		disableAuto = true;
		aPark = true;
	}

	if (aPark) {chas.changeBrake(chas.S_HOLD, inert.get_pitch(), 5 + goalsPossessed);}
}


// lifts
bool front = true;
void liftControl()
{
	// Check if 'X' is being pressed and set back lift to coast if it is
	if(con.get_digital(pros::E_CONTROLLER_DIGITAL_X))
		backLift.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	else
		backLift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	// Check if driver is currently controlling pneumatics or lifts
	if(con.get_digital(pros::E_CONTROLLER_DIGITAL_L2) == false)
	{

		// Check L1 to see if driver wants to control front lift or back lift
		
		if(con.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {front = false;}
		else {front = true;}

		// Move lift up
		if(con.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
		{
			if(front)
			{
				frontLift.move(-127);
				backLift.move(0);
			}
			else
			{
				backLift.move(127);
				frontLift.move(0);
			}
		}
		// Move lift down
		else if(con.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
		{
			if(front)
			{
				frontLift.move(127);
				backLift.move(0);
			}
			else
			{
				backLift.move(-127);
				frontLift.move(0);
			}
		}
		// No buttons are being pressed so stop both lifts
		else
		{
			frontLift.move(0);
			backLift.move(0);
		}
	}
}

// Pneumatics
void pneumaticControl()
{
	// Check if l2 is pressed down, meaning driver wants to control the pneumatics
	if(con.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
	{
		// firstPress1 boolean is used to make sure the front pneumatic
		// doesn't repeatedly toggle on and off when R1 is held
		static bool firstPress1 = true;
		if(con.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
		{
			// Check if this is R1's initial press, and toggle the pneumatic if it is.
			if(firstPress1)
			{
				backPneu.toggle();
				firstPress1 = false;
			}
		}
		// Allow the pneumatic to be toggled again after the button has been released
		else
			firstPress1 = true;

		// firstPress2 boolean is used to make sure the back pneumatic
		// doesn't repeatedly toggle on and off when the R2 is held
		static bool firstPress2 = true;
		if(con.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
		{
			// Check if this is R2's initial press, and toggle the pneumatic if it is.
			if(firstPress2)
			{
				frontPneu.toggle();
				firstPress2 = false;
			}
		}
		// Allow the pneumatic to be toggled again after the button has been released
		else
			firstPress2 = true;
	}
}

void killAllAuto()
{
	if(disableAll) {disableAuto = true;}
	if(con.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT) && con.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT))
	{
		con.clear();
		disableAll = !disableAll;
		while(con.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT) && con.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {}
	}
}

// opControl/auton functions
void printInfo()
{
	chas.getLeftPos();
	chas.getRightPos();
	if(globalTime < 2000)
	{
		if(globalTime % 50 == 0)
		{ con.set_text(0,0, "Selected:       "); }
	}

	else
	{

		static int counter = 0;
		if(counter == 10)
		{
			//print whether the chassis controls are reversed or not
			/*
			if(chas.reverseStatus() == false) {con.set_text(0, 0, "Chas: FORWARD");}
			else {con.set_text(0,0, "Chas: REVERSE");}
			*/
			con.print(0,0,"Autoval: %.1f", autoStraightVal);
		}
		if(counter == 20)
		{
			/*
			//print the brake type for the chassis
			if(disableAuto)
			{
				if(chas.getBrakeMode() == 0) {con.set_text(1,0, "Brake Mode: COAST");}
				else if(chas.getBrakeMode() == 1) {con.set_text(1,0, "Brake Mode : HOLD");}
			}
			else
			{
				if(chas.getBrakeMode() == 0) {con.set_text(1,0, "Brake M(A): COAST");}
				else if(chas.getBrakeMode() == 1) {con.set_text(1,0, "Brake M(A) : HOLD");}
			}
			*/
			con.print(1,0, "Inert: %.1f", inert.get_heading());
			
		}
		if(counter == 30)
		{
			//prints the temperature of the chassis

			if(disableAll) {con.print(2, 0, "ALL AUTO DISABLED");}
			else 
			{
				if((chas.leftTemp() + chas.rightTemp()) /2 > 53)
				{
					con.print(2, 0, "Chassis(HOT): %.0f°C", ((chas.leftTemp() + chas.rightTemp()) / 2));
				}
				else
				{
					con.print(2, 0, "Chassis: %.0f°C", ((chas.leftTemp() + chas.rightTemp()) / 2));
				}
			}

			//con.print(2, 0, "Inert: %.2f", inert.get_pitch());
			counter = 0;
		}
		counter++;
	}
}

//autonomous functions
//drive(enc, timeout, pct power)
//rotate(degrees, direction)



void red1()
{
	backLift.move_relative(-3000, -127);
	pros::delay(300);
	drive(-470, 2500);
	backLift.move_absolute(0, 127);	
	goalsPossessed++;
	pros::delay(300);			
	drive(250);
	rotateTo(-45);
	drive(-340, 2000, 0.8);
	drive(-25, 1000, 1);
	backPneu.toggle();
	goalsPossessed++;
	rotateTo(-60, 1500);
	drive(445, 2500, 1);
	frontPneu.toggle();
	goalsPossessed++;
	drive(-100, 2000, 1);
	rotate(-90);
	drive(-200, 1000, 1);
}
void red2()
{
	drive(500);
	frontPneu.toggle();
}
void blue1()
{
	rotate(90);
}
void blue2()
{

}
void skills()
{

}

//autonomous(will be called by competition)
void autonomous()
{
	
	if (autonNum == 1){red1();}
	if (autonNum == 2){red2();}
	if (autonNum == 3){blue1();}
	if (autonNum == 4){blue2();}
	if (autonNum == 5){skills();}
	
	
	
	/*
	backLift.move_absolute(-1500, -100);
	pros::delay(1000);
	drive(-150);
	backLift.move_absolute(-1200, 100);
	pros::delay(1000);
	drive(-350);
	*/
}

// main control functions ======================================================
void initialize()
{
	pros::lcd::initialize();
	frontLift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	backLift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	chas.changeBrake(chas.COAST);
	disableAuto = false;
}



void disabled() {}

void competition_initialize() {}

void opcontrol()
{
	pros::lcd::set_text(0, "the impostor from among us is in ur code");
	con.clear();
	while(autonCurrentlySelecting){autonSelector();}

	chas.changeBrake(chas.COAST);
	backLift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	while (true)
	{
		
		// Drive loop (there's an arcadeDrive() function and tankDrive() function.
		arcadeDrive();
		liftControl();
		pneumaticControl();
		reverseToggle();
		brakeType();
		autoBrakeMode();
		killAllAuto();
		autoPark();
		if(con.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN))
			autonomous();
	
		// print information to controller
		printInfo();
	
		pros::delay(5);
		globalTime += 5;
	}
}
