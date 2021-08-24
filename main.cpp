#include "main.h"
#include "chassis.h"
#include "ports.h"
#include "piston.h"

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





// auton functions =============================================================
void drive(double targetEnc, int timeout = 4000) // timeout in milliseconds
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

	float distKp = 1.5;

	bool withinRange = false;
	int withinRangeTime = 0;

	// Drive straight variables: uses the intertial with PID
	inert.set_heading(180);
	double initialRotation = inert.get_heading();

	double lastError, derivative;
	double error = 0.0;
	double integral = 0.0;

	float kP = (targetEnc >= 0 ? 5 : -5);

	con.clear();

	// Drive loop, might add a timeout thing if it's needed
	while(timeout > time)
	{
		if(time % 50 == 0) {con.print(1,0,"GH: %.1f", distError);}
		// Drive code: Distance error set to target encoding - average of left + right encodings
		distError = targetEnc - ((leftStartPos - chas.getLeftPos()) + (rightStartPos - chas.getRightPos()) / 2);
		baseSpeed = slewMult * ((abs(distError * distKp) > 127 * 0.6) ? (targetEnc > 0 ? 127 * 0.6 : -127 * 0.6) : (distError * distKp)); //? (distError * distKp) : (5); // If the base speed is below 3.5, set the base speed to 3.5
		if(distError < -3)
		{
			baseSpeed = (baseSpeed > -15) ? (-15) : (baseSpeed);
		}
		else if(distError > 3)
		{
			baseSpeed = (baseSpeed < 15) ? (15) : (baseSpeed);
		}
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
	chas.stop();
}







enum rotateDirection {CW, CCW}; // clockwise / counter clockwise
void rotate(double degrees, int timeout=3000, rotateDirection dir=CW)
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

	double error = targetHeading - currentRotation;
	double lastError = error;
	double intergral = 0.0;
	double derivative = 0.0;

	double speed = 0.0;

	float kP = 1.5;
	float kI = 0.1;
	float kD = 2;

	bool integ = false;

	while(true)
	{
		if(time % 50 == 0) {con.print(1,0,"GH: %.1f", error);}
		currentRotation = inert.get_heading();

		/*
		if(dir == CW)
			globalHeading += (currentRotation >= lastRotation) ? (currentRotation - lastRotation) : ((360 - lastRotation) + currentRotation);
		else
			globalHeading += (currentRotation <= lastRotation) ? (currentRotation - lastRotation) : ((currentRotation - 360) - lastRotation);
		*/

		lastError = error;
		error = targetHeading - currentRotation;
		if(error <= 1)
		{
			integ = true;
		}
		if(integ) { intergral += error; }
		derivative = lastError - error;
		speed = error * kP + intergral * kI + derivative * kD;

		chas.spinLeft(speed);
		chas.spinRight(-speed);

		// delay while loop
		pros::delay(5);
		time += 5;
		// check timeout
		if(timeout <= time)
			break;
	}
	chas.changeBrake(chas.HOLD);
	chas.stop();
}



void blah()
{
	rotate(90, 10000);
}

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
	if(!(startTime == 0)) {globalTime - startTime < 8000 ? disableAuto = true : disableAuto = false;}
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

// lifts

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
		bool front = true;
		if(con.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
			front = false;

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
				if(backLift.get_position() < -100 || disableAll) {backLift.move(100);}
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
				if(backLift.get_position() > -1500 || disableAll) {backLift.move(-100);}
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
				frontPneu.toggle();
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
				backPneu.toggle();
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
	static int counter = 0;
	if(counter == 5)
	{
		//print whether the chassis controls are reversed or not
		if(chas.reverseStatus() == false) {con.set_text(0, 0, "Chas: FORWARD");}
		else {con.set_text(0,0, "Chas: REVERSE");}
	}
	if(counter == 10)
	{
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
	}
	if(counter == 15)
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

// main auton function
void autonomous()
{
	blah();
	// frontLift.move(127);
	// frontLift.move(-127);
	// backPneu.toggle();
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
	pros::lcd::set_text(0, "running");

	con.clear();
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
		if(con.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN))
			autonomous();
	
		// print information to controller
		printInfo();
	
		pros::delay(10);
		globalTime += 10;
	}
}
