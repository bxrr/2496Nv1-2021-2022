#include "main.h"
#include "chassis.h"
#include <iostream>
#include "ports.h"

Chassis chas;
pros::ADIAnalogOut pneu(PNEUMATIC_PORT); // pneumatic code: pneu.set_value(true/false);
pros::Imu inert(INERT_PORT);
pros::Controller con(pros::E_CONTROLLER_MASTER);
pros::ADIDigitalOut pneu(PNEUMATIC_PORT);

void initialize()
{
	pros::lcd::initialize();
	inert.reset();
	pros::lcd::print(1, "Initialized");
}

 // auton functions =============================================================
 enum driveDirection {FORWARD, BACKWARD};
 void drive(int targetPos, driveDirection dir=FORWARD) // Need to implement inertial for straight driving
 {
	   chas.brakeCoast();

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
     while(currentPos < targetPos - 5)
     {
			 distError = targetPos - ((leftStartPos - chas.getLeftPos()) + (rightStartPos - chas.getRightPos()) / 2);

			 lastError = error;
			 error = initialRotation - inert.get_rotation();
			 integral += error;
			 derivative = error - lastError;

			 double rightSpeed = distError * distKp;
			 double leftSpeed = rightSpeed + (error * kP) + (integral * kI) + (derivative * kD);

			 chas.spinLeft(leftSpeed);
			 chas.spinRight(rightSpeed);
     }
 }

 enum rotateDirection {CW, CCW};
 void rotate(int degrees, rotateDirection dir=CW)
 {

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
    if(abs(con.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) > 3 || abs(con.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) > 3)
    {
		chas.brakeCoast();
      	chas.spinLeft(con.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) + con.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) * 0.5);
        chas.spinRight(con.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) + con.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) * 0.5);
    }
    else
    {
        chas.brakeHold();
    }
}

void tankDrive()
{
    if(abs(con.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) > 3 || abs(con.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)) > 3)
    {
		chas.brakeCoast();
        chas.spinLeft(con.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
        chas.spinRight(con.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
    }
    else
    {
        chas.brakeHold();
    }
}

// main function
void opcontrol() {
	pros::lcd::print(2, "Running Op Control");
	while (true)
	{ // Drive loop (there's an arcadeDrive() function and tankDrive() function.
		tankDrive();

		if(con.get_digital(pros::E_CONTROLLER_DIGITAL_A))
		{
			pneu.set_value(true);
		}
		if(con.get_digital(pros::E_CONTROLLER_DIGITAL_Y))
		{
			pneu.set_value(false);
		}
	}
}
