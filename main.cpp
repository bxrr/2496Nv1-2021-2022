#include "main.h"
#include "chassis.h"
#include "piston.h"
#include "ports.h"

#include <string.h>

Chassis chas;
Piston frontLeftPneu(FRONT_L_PNEUMATIC_PORT);
Piston frontRightPneu(FRONT_R_PNEUMATIC_PORT);
Piston backPneu(BACK_PNEUMATIC_PORT);
pros::Motor backLift(BACK_LIFT_PORT, pros::E_MOTOR_GEARSET_18, false);
pros::Motor frontLift(FRONT_LIFT_PORT, pros::E_MOTOR_GEARSET_36, false);

pros::Imu inert(INERT_PORT);
pros::Controller con(pros::E_CONTROLLER_MASTER);

// misc functions ==============================================================
void checkInertial(int lineNum=1)
{
	while(inert.is_calibrating())
	{
		if(pros::lcd::is_initialized())
		{
			pros::lcd::set_text(lineNum, "inertial is calibrating...");
		}
	}
	if(pros::lcd::is_initialized())
	{
		pros::lcd::clear_line(lineNum);
	}
}

// auton functions =============================================================
void drive(double targetEnc, int timeout=4000) // timeout in milliseconds
{
	// Timeout counter
	int time = 0;

	// Drive distance variables: uses motor encodings with distance error
	double leftStartPos = chas.getLeftPos();
	double rightStartPos = chas.getRightPos();

	double distError = 0.0;
	double currentPos = 0.0;
	double baseSpeed = 0.0;

	float distKp = 0.1;

	// Drive straight variables: uses the intertial with PID
	double initialRotation = inert.get_rotation();
	double lastError, derivative;
	double error = 0.0;
	double integral = 0.0;

	float kP = 0.2;
	float kI = 0.07;
	float kD = 0.03;

	// Drive loop, might add a timeout thing if it's needed
	while(currentPos < targetEnc)
	{
		// Drive code: Distance error set to target encoding - average of left + right encodings
		distError = targetEnc - ((leftStartPos - chas.getLeftPos()) + (rightStartPos - chas.getRightPos()) / 2);
		baseSpeed = (distError * distKp > 3.5) ? (distError * distKp) : 3.5; // If the base speed is below 3.5, set the base speed to 3.5

		// Drive straight code: Changes left side of the chassis' speed according to the intertial sensor's readings
		lastError = error;
		error = initialRotation - inert.get_rotation();
		integral += error;
		derivative = error - lastError;

		// Apply speeds
		chas.spinLeft((baseSpeed) + (error * kP) + (integral * kI) + (derivative * kD));
		chas.spinRight(baseSpeed);

		// delay while loop
		pros::delay(10);
		time += 10;
		// check timeout
		if(timeout <= time)
			break;
	}
	// Stop robot after loop
	chas.stop();
}

enum rotateDirection {CW, CCW}; // clockwise / counter clockwise
void rotate(double degrees, int timeout=3000, rotateDirection dir=CW)
{
	// Timeout counter
	int time = 0;

	// Rotate variables: Uses inertial sensor and slows down as it gets closer to the target by using an error
	double targetRotation = (dir == CW) ? (inert.get_rotation() + degrees) : (inert.get_rotation() - degrees);
	double currentRotation = inert.get_rotation();
	double error = targetRotation - currentRotation;
	double speed = 0.0;

	float kP = 0.7;

	while(currentRotation < targetRotation)
	{
		currentRotation = inert.get_rotation();
		error = targetRotation - currentRotation;
		speed = (error * kP > 3.5) ? (error * kP) : (3.5);

		chas.spinLeft(speed);
		chas.spinRight(speed);

		// delay while loop
		pros::delay(10);
		time += 10;
		// check timeout
		if(timeout <= time)
			break;
	}
	chas.stop();
}

// main auton function
void autonomous()
{
	// drive(180.0); // go forward 1 wheel revolution (no 2nd parameter defaults to FORWARD)
	// drive(180.0, BW);
	// rotate(90.0); // 2nd parameter defaults to clockwise
	// rotate(90.0, CCW);
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


void brakeType()
{
	bool brakeinitiate = true;

	if(con.get_digital(pros::E_CONTROLLER_DIGITAL_X))
	{
		if(chas.getBrakeMode() == 0) {chas.changeBrake(chas.HOLD);}
		else {chas.changeBrake(chas.COAST);}
		while(con.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {}
	}
}

// lifts
void stopFrontLift()
{
	frontLift.move_velocity(0);
}

void stopBackLift()
{
	backLift.move_velocity(0);
}

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
		bool front = false;
		if(con.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
			front = true;

		// Move lift up
		if(con.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
		{
			if(front)
			{
				frontLift.move(-127);
				stopBackLift();
			}
			else
			{
				backLift.move(100);
				stopFrontLift();
			}
		}
		// Move lift down
		else if(con.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
		{
			if(front)
			{
				frontLift.move(127);
				stopBackLift();
			}
			else
			{
				backLift.move(-100);
				stopFrontLift();
			}
		}
		// No buttons are being pressed so stop both lifts
		else
		{
			stopFrontLift();
			stopBackLift();
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
				frontLeftPneu.toggle();
				frontRightPneu.toggle();
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

// main control functions ======================================================
void initialize()
{
	pros::lcd::initialize();
	frontLift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	backLift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	chas.changeBrake(chas.COAST);
}

void disabled() {}

void competition_initialize() {}

void opcontrol()
{
	pros::lcd::set_text(0, "running");

	short int counter = 0;
	con.clear();
	while (true)
	{
		// Drive loop (there's an arcadeDrive() function and tankDrive() function.
		arcadeDrive();
		liftControl();
		pneumaticControl();
		reverseToggle();
		brakeType();

		// print information to controller
		if(counter == 5)
		{
			if(chas.reverseStatus() == false) {con.set_text(0, 0, "Chas: FORWARD");}
			else {con.set_text(0,0, "Chas: REVERSE");}
		}
		if(counter == 10)
		{
			if(chas.getBrakeMode() == 0) {con.set_text(1,0, "Brake Type: COAST");}
			else if(chas.getBrakeMode() == 1) {con.set_text(1,0, "Brake Type : HOLD");}
		}
		if(counter == 15)
		{
			con.print(2, 0, "Chassis: %.2f°C", ((chas.leftTemp() + chas.rightTemp()) / 2));
			counter = 0;
		}
		counter++;

		pros::delay(10);
	}
}
