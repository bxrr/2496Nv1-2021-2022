#include "main.h"
#include "chassis.h"
#include "piston.h"
#include "ports.h"

#include <string.h>

Chassis chas;
Piston pneu(PNEUMATIC_PORT);
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
enum driveDirection {FORWARD, BACKWARD};
void drive(double targetEnc, driveDirection dir=FORWARD)
{
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
	}
	// Stop robot after loop
	chas.stop();
}

enum rotateDirection {CW, CCW}; // clockwise / counter clockwise
void rotate(double degrees, rotateDirection dir=CW)
{   // Rotate variables: Uses inertial sensor and slows down as it gets closer to the target by using an error
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
	}
	chas.stop();
}

// main auton function
void autonomous()
{
	// drive(180.0); // go forward 1 wheel revolution (no 2nd parameter defaults to FORWARD)
	// drive(180.0, BACKWARD);
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

// lifts
void stopFrontLift()
{
	frontLift.move_velocity(0);
	frontLift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

void stopBackLift()
{
	backLift.move_velocity(0);
	backLift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

void liftControl()
{
	bool front = false;
	if(con.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
		front = true;

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
	else
	{
		stopFrontLift();
		stopBackLift();
	}
}

// pneumatic
void pneumaticControl()
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
	pros::lcd::set_text(0, "running");

	int counter = 0;
	con.clear();
	while (true)
	{ 	// Drive loop (there's an arcadeDrive() function and tankDrive() function.
		arcadeDrive();
		liftControl();
		pneumaticControl();
		reverseToggle();

		if(counter == 5) // print all temperatures every 50 milliseconds
		{
			con.clear();
			con.set_text(0, 0, "Reverse: " + std::to_string(chas.reverseStatus()));
			con.set_text(1, 0, "Left   : " + std::to_string(chas.leftTemp()) + "°C");
			con.set_text(2, 0, "Right  : " + std::to_string(chas.rightTemp()) + "°C");
			counter = 0;
		}

		counter++;
		pros::delay(10);
	}
}
