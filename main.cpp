#include "main.h"
#include "chassis.h"

#include "port_defines.h"

void initialize()
{
	pros::Controller con(E_CONTROLLER_MASTER);
	Chassis chas;
	pros::Imu inert(IMU_PORT);
	inert.reset();
}

 // auton functions =============================================================
 enum driveDirection {FORWARD, BACKWARD};
 void drive(int targetPos, driveDirection dir=FORWARD) // Need to implement inertial for straight driving
 {
	   chas.brakeCoast();

     int leftStartPos = chas.getLeftPos();
     int rightStartPos = chas.getRightPos();

		 // Distance error (only P)
		 double distError
		 double currentPos = 0.0;
		 float distKp = 0.2;

		 // Rotational Error - intertial (PID)
		 double initialRotation = inert.getRotation();
     double lastError, derivative;
     double error = 0.0;
     double integral = 0.0;

		 float kP = 0.2;
		 float kI = 0.07;
		 float kD = 0.03;

     while(currentPos < targetPos - 5)
     {
			 distError = targetPos - ((leftStarPos - chas.getLeftPos()) + (rightStartPos - chas.getRightPos()) / 2;

			 lastError = error;
			 error = initialRotation - inert.getRotation();
			 integral += error;
			 derivative = error - lastError;

			 rightSpeed = distError * distKp;
			 leftSpeed = rightSpeed + (error * kP) + (integral * kI) + (derivative * kD);

			 chas.spinLeft(leftSpeed)
     }
 }

 enum rotateDirection {CW, CCW};
 void rotate(int degrees, rotateDirection dir=CW)
 {

 }
// main auton function
void autonomous()
{
	while(inert.is_calibrating())
	{
		pros:delay(5);
	}
}

// driver control functions ====================================================
void arcadeDrive()
{
    if(abs(con.get_analog(E_CONTROLLER_ANALOG_LEFT_Y)) > 3 || abs(con.get_analog(E_CONTROLLER_ANALOG_RIGHT_X)) > 3)
    {
				chas.brakeCoast();
        chas.spinLeft(con.get_analog(E_CONTROLLER_ANALOG_LEFT_Y) + con.get_analog(E_CONTROLLER_ANALOG_RIGHT_X) * 0.5);
        chas.spinRight(con.get_analog(E_CONTROLLER_ANALOG_LEFT_Y) + con.get_analog(E_CONTROLLER_ANALOG_RIGHT_X) * 0.5);
    }
    else
    {
        chas.brakeHold();
    }
}

void tankDrive()
{
    if(abs(con.get_analog(E_CONTROPLLER_ANALOG_LEFT_Y)) > 3 || abs(con.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y)) > 3)
    {
			  chas.brakeCoast();
        chas.spinLeft(con.get_analog(E_CONTROPLLER_ANALOG_LEFT_Y));
        chas.spinRight(con.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y));
    }
    else
    {
        chas.brakeHold();
    }
}

// main function
void opcontrol() {
	while(inert.is_calibrating())
	{
		pros:delay(5);
	}

	while (true) {
		arcadeDrive();
	}
}
