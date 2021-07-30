#include "main.h"
#include "chassis.h"
#include "piston.h"
#include "ports.h"

Chassis chas;
Piston pneu;
pros::Imu inert(INERT_PORT);
pros::Controller con(pros::E_CONTROLLER_MASTER);

void initialize()
{
	inert.reset();
	pros::lcd::initialize();
}

 // auton functions =============================================================
 enum driveDirection {FORWARD, BACKWARD};
 void drive(int targetPos, driveDirection dir=FORWARD) // Need to implement inertial for straight driving
 {
     int leftStartPos = chas.getLeftPos();
     int rightStartPos = chas.getRightPos();

		 // Distance error: motor encoding + P
		 double distError;
		 double currentPos = 0.0;
		 float distKp = 0.2;

		 // Drive straight stuff: intertial + PID
		 double initialRotation = inert.get_rotation();
     double lastError, derivative;
     double error = 0.0;
     double integral = 0.0;

		 float kP = 0.2;
		 float kI = 0.07;
		 float kD = 0.03;

		 // Drive loop, might add a timeout thing if it's needed
     while(currentPos < targetPos)
     {
			 distError = targetPos - ((leftStartPos - chas.getLeftPos()) + (rightStartPos - chas.getRightPos()) / 2);

			 lastError = error;
			 error = initialRotation - inert.get_rotation();
			 integral += error;
			 derivative = error - lastError;

			 chas.spinLeft((distError * distKp) + (error * kP) + (integral * kI) + (derivative * kD));
			 chas.spinRight(distError * distKp);
     }
		 chas.spinLeft(0);
		 chas.spinRight(0);
 }

 enum rotateDirection {CW, CCW};
 void rotate(int degrees, rotateDirection dir=CW)
 {
	 double targetRotation = (dir == CW) ? (inert.get_rotation() + degrees) : (inert.get_rotation() - degrees);
	 double currentRotation = inert.get_rotation();
	 double error = targetRotation - currentRotation;

	 float kP = 0.1;

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
	while(inert.is_calibrating()) // Check if the intertial sensor is calibrating before doing anything
	{
		pros::delay(5);
	}
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
				chas.spinLeft(0);
				chas.spinRight(0);
        chas.stop();
    }
}

// main function
void opcontrol()
{
	pros::lcd::set_text(1, "opcontrol() is running.");

	while (true)
	{ // Drive loop (there's an arcadeDrive() function and tankDrive() function.
		arcadeDrive();
		if(con.get_digital(pros::E_CONTROLLER_DIGITAL_A))
		{
			pneu.toggle();
		}

		if(con.get_digital(pros::E_CONTROLLER_DIGITAL_B))
		{
			break; // Temporary exit code emergency button
		}
		pros::delay(10);
	}
}
