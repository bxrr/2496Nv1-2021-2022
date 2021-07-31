#include "main.h"
#include "chassis.h"
#include "piston.h"
#include "ports.h"

Chassis chas;
Piston pneu;
pros::Motor backLift(17, pros::E_MOTOR_GEARSET_18, false);

pros::Imu inert(INERT_PORT);
pros::Controller con(pros::E_CONTROLLER_MASTER);

// misc functions ==============================================================
void checkInertial(int lineNum=1)
{
	while(inert.is_calibrating())
	{
		if(pros::lcd::is_initialized())
		{
			pros::lcd::print(lineNum, "inertial is calibrating...");
		}
	}
	if(pros::lcd::is_initialized())
	{
		pros::lcd::clear_line(lineNum);
	}
}

// auton functions =============================================================
enum driveDirection {FORWARD, BACKWARD};
void drive(int targetEnc, driveDirection dir=FORWARD)
{
	// Drive distance variables: uses motor encodings with distance error
	int leftStartPos = chas.getLeftPos();
	int rightStartPos = chas.getRightPos();

	double distError;
	double currentPos = 0.0;
	float distKp = 0.2;

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

		// Drive straight code: Changes left side of the chassis' speed according to the motor encodings of the left and right sides of the chassis
		lastError = error;
		error = initialRotation - inert.get_rotation();
		integral += error;
		derivative = error - lastError;

		// Apply speeds
		chas.spinLeft((distError * distKp) + (error * kP) + (integral * kI) + (derivative * kD));
		chas.spinRight(distError * distKp);
	}
	// Stop robot after loop
	chas.spinLeft(0);
	chas.spinRight(0);
}

enum rotateDirection {CW, CCW};
void rotate(int degrees, rotateDirection dir=CW)
{   // Rotate variables: Uses inertial sensor and slows down as it gets closer to the target by using an error
	double targetRotation = (dir == CW) ? (inert.get_rotation() + degrees) : (inert.get_rotation() - degrees);
	double currentRotation = inert.get_rotation();
	double error = targetRotation - currentRotation;

	float kP = 0.5;

	while(currentRotation < targetRotation)
	{
		currentRotation = inert.get_rotation();
		error = targetRotation - currentRotation;

		chas.spinLeft((dir == CW) ? (1) : (-1) * error * kP);
		chas.spinRight((dir == CW) ? (-1) : (1) * error * kP);
	}
}

// main auton function
void autonomous()
{

}

// driver control functions ====================================================
void arcadeDrive()
{
	if(abs(con.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) > 10 || abs(con.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) > 10)
	{
		chas.spinLeft(con.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) + con.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
		chas.spinRight(con.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) - con.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
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

// main control functions ======================================================
void initialize()
{
	pros::lcd::initialize();
}

void disabled() {}

void competition_initialize() {}

void opcontrol()
{
	pros::lcd::set_text(1, "opcontrol is running");
	while (true)
	{ 	// Drive loop (there's an arcadeDrive() function and tankDrive() function.
		arcadeDrive();
		if(con.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
		{
			backLift.move(127);
		}
		else if(con.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
		{
			backLift.move(-127);
		}
		else
		{
			backLift.move_velocity(0);
			backLift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		}

		if(con.get_digital(pros::E_CONTROLLER_DIGITAL_A))
		{
			pneu.toggle();
		}
		pros::delay(10);
	}
}
