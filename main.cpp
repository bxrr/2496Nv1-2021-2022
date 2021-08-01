#include "main.h"
#include "chassis.h"
#include "piston.h"
#include "ports.h"

Chassis chas;
Piston pneu(PNEUMATIC_PORT);
pros::Motor backLift(BACK_LIFT_PORT, pros::E_MOTOR_GEARSET_18, false);

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
	// Drive distance variables: uses motor encodings with distance error and integral
	int leftStartPos = chas.getLeftPos();
	int rightStartPos = chas.getRightPos();

	double distError = 0;
	double distIntegral = 0;
	double currentPos = 0.0;
	float distKp = 0.1;
	float distKi = 0.0005;

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
		distError = (distError > 100) ? (100) : (distError);
		distIntegral += distError;

		// Drive straight code: Changes left side of the chassis' speed according to the motor encodings of the left and right sides of the chassis
		lastError = error;
		error = initialRotation - inert.get_rotation();
		integral += error;
		derivative = error - lastError;

		// Apply speeds
		chas.spinLeft((distError * distKp + distIntegral * distKi) + (error * kP) + (integral * kI) + (derivative * kD));
		chas.spinRight(distError * distKp + distIntegral * distKi);
	}
	// Stop robot after loop
	chas.spinLeft(0);
	chas.spinRight(0);
}

enum rotateDirection {CW, CCW};
void rotate(int degrees, rotateDirection dir=CW)
{   // Rotate variables: Uses inertial sensor and slows down as it gets closer to the target by using an error and integral
	double targetRotation = (dir == CW) ? (inert.get_rotation() + degrees) : (inert.get_rotation() - degrees);
	double currentRotation = inert.get_rotation();
	double error = targetRotation - currentRotation;
	double integral = 0;

	float kP = 0.5;
	float kI = 0.002;

	while(currentRotation < targetRotation)
	{
		currentRotation = inert.get_rotation();
		error = targetRotation - currentRotation;
		integral += error;

		chas.spinLeft( ((dir == CW) ? (1) : (-1)) * (error * kP) * (integral * kI) );
		chas.spinRight( ((dir == CW) ? (-1) : (1)) * (error * kP) * (integral * kI) );
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

// Lifts
void backLiftControl()
{
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
}

// pneumatic
void controlPneumatic()
{
	if(con.get_digital(pros::E_CONTROLLER_DIGITAL_A))
	{
		pneu.toggle();
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
	pros::lcd::set_text(0, "sussy fortnight :D");
	while (true)
	{ 	// Drive loop (there's an arcadeDrive() function and tankDrive() function.
		arcadeDrive();
		backLiftControl();
		controlPneumatic();

		pros::delay(10);
	}
}
