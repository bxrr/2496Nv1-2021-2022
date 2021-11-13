#include "main.h"
#include "autons.h"
//autons has all includes through chains

#include <string.h>

using namespace pros;


// Globals
Chassis chas;

Piston frontPneu(FRONT_PNEUMATIC_PORT);
Piston backPneu(BACK_PNEUMATIC_PORT);
Piston frontSpecialPneu(FRONT_SPECIAL_PNUEMATIC_PORT);
Motor backLift(BACK_LIFT_PORT, E_MOTOR_GEARSET_18, false);
Motor frontLift(FRONT_LIFT_PORT, E_MOTOR_GEARSET_36, false);

Imu inert(INERT_PORT);
Controller con(E_CONTROLLER_MASTER);

Motor backLeft(BACK_LEFT_PORT);
Motor midLeft(MID_LEFT_PORT);
Motor frontLeft(FRONT_LEFT_PORT);
Motor backRight(BACK_RIGHT_PORT);
Motor midRight(MID_RIGHT_PORT);
Motor frontRight(FRONT_RIGHT_PORT);


int globalTime;		//time since code has initialized, used as a timer
bool disableAll = false;
double globalRotation = 0;


// misc functions ==============================================================


bool autonCurrentlySelecting = true;
int autonType = 1;
int autonColor = 1;
bool autonTypeSelected = false;
void autonSelector()
{
	static bool firstTime = true;
	static int localTime = 0;
	if(localTime > 150) {localTime = 0;}
	if(localTime == 50) {con.set_text(0,0, "Select Auton:");}

	if(localTime ==100)
	{
		if (autonType == 1){con.set_text(1,0, "Elevated Long        ");}
		else if (autonType == 2){con.set_text(1,0, "Elevated Short       ");}
		else if (autonType == 3){con.set_text(1,0, "De-elevated Long      ");}
		else if (autonType == 4){con.set_text(1,0, "De-elevated short     ");}
		else if (autonType == 5) {con.set_text(1,0, "Both               ");}
		else if (autonType == 6) {con.set_text(1,0, "Rush Neutral       ");}
		else if (autonType == 7){con.set_text(1,0, "Skills               ");}
		else if(autonType == 8) {con.set_text(1,0, "None                  ");}
	}

	if(autonTypeSelected)
	{
		if(localTime == 150)
		{
			if (autonColor == 1){con.set_text(2,0, "Red ");}
			else if (autonColor == 2){con.set_text(2,0, "Blue");}
		}
	}


	if (con.get_digital(E_CONTROLLER_DIGITAL_RIGHT))
	{
		con.clear();
		if(autonTypeSelected)
		{
			if(autonColor == 3){autonColor = 1;}
			else{autonColor++;}
		}
		else
		{
			if(autonType == 9) {autonType = 1;}
			else {autonType++;}
		}

		while (con.get_digital(E_CONTROLLER_DIGITAL_RIGHT)){}
	}

	else if (con.get_digital(E_CONTROLLER_DIGITAL_LEFT))
	{
		con.clear();
		if(autonTypeSelected)
		{
			if(autonColor == 0){autonColor = 2;}
			else{autonColor--;}
		}
		else
		{
			if(autonType == 0) {autonType = 8;}
			else {autonType--;}
		}
		while (con.get_digital(E_CONTROLLER_DIGITAL_LEFT)){}
	}


	if (con.get_digital(E_CONTROLLER_DIGITAL_A))
	{
		if(autonType >= 6) {autonCurrentlySelecting = false;}
		else
		{
			if(autonTypeSelected)
			{
				autonCurrentlySelecting = false;
				while(con.get_digital(E_CONTROLLER_DIGITAL_A)) {}
			}

			else
			{
				autonTypeSelected = true;
				while(con.get_digital(E_CONTROLLER_DIGITAL_A)) {}
			}
		}
	}

	localTime += 5;
	delay(5);
}





// auton functions =============================================================
double goalsPossessed = 0;
bool killAuton = false;


// driver control functions ====================================================

bool chasDisabled = false;
enum arcadeTypes {MANUAL, AUTO};
void arcadeDrive(arcadeTypes arcadeType)			//fully manual arcade drive
{

	if(arcadeType == MANUAL)
	{

		if((abs(con.get_analog(E_CONTROLLER_ANALOG_LEFT_Y)) > 10 || abs(con.get_analog(E_CONTROLLER_ANALOG_RIGHT_X)) > 10) && !chasDisabled)
		{
			double turnStick = (chas.reverseStatus()) ? (-con.get_analog(E_CONTROLLER_ANALOG_RIGHT_X)) : (con.get_analog(E_CONTROLLER_ANALOG_RIGHT_X));
			chas.spinLeft(con.get_analog(E_CONTROLLER_ANALOG_LEFT_Y) + turnStick);
			chas.spinRight(con.get_analog(E_CONTROLLER_ANALOG_LEFT_Y) - turnStick);
		}
		else
		{
		chas.stop();
		}

	}

	else	//automatically make the robot drive straight
	{
		static int autoStraightVal;
		static double inertialStart;					//starting inertial heading when driving straight
		static bool autoStraight = false; 				//boolean checking whether to implement autostraight assist
		double kPauto = chas.getVelocity() > 0 ? 4 : -4;		//kP is the magnitude of the effect of autostraight, reverse for when chassis is negative
		kPauto = chas.reverseStatus() ? -kPauto : kPauto;
		if(abs(con.get_analog(E_CONTROLLER_ANALOG_LEFT_Y)) > 10 || abs(con.get_analog(E_CONTROLLER_ANALOG_RIGHT_X)) > 10)
		{
				double turnStick = (chas.reverseStatus()) ? (-con.get_analog(E_CONTROLLER_ANALOG_RIGHT_X)) : (con.get_analog(E_CONTROLLER_ANALOG_RIGHT_X));
				if(abs(turnStick) < 5) 		//if not turning(initiate autostraight)
				{
					if(autoStraight) 			//if autostraight enabled, calculate error
					{
						autoStraightVal = (inert.get_heading() - inertialStart) * kPauto;

						if(autoStraightVal > 10) {autoStraight = false;} 		//if there is a big unexpected jerk, ie if the robot is hit by an external object, reset autostraight
					}

					else
					{
						autoStraight = true;					//since driving straight, set autostraight to true
						inert.set_heading(180);					//reset inertial valu
						inertialStart = inert.get_heading();	//set initial inertial value, this part of the loop will not be executed again until autostraight is reset
					}
				}

				else
				{
					autoStraight = false;			//disable autostraight calculations because turning is true
					autoStraightVal = 0;			//disable effect of autostraight
				}

				//chas controls(take the vertical stick +- the turn stick and apply autostraight effect, if autoStraightVal = 0, then autostraight is disabled)
				chas.spinLeft(con.get_analog(E_CONTROLLER_ANALOG_LEFT_Y) + turnStick - (autoStraightVal) * (chas.getVelocity()/127));
				chas.spinRight(con.get_analog(E_CONTROLLER_ANALOG_LEFT_Y) - turnStick + (autoStraightVal) * (chas.getVelocity()/127));
		}
		else
		{
			chas.stop();
			autoStraight = false;
		}
	}
}



void tankDrive()
{
	if((abs(con.get_analog(E_CONTROLLER_ANALOG_LEFT_Y)) > 10 || abs(con.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y)) > 10) && !chasDisabled)
	{
		chas.spinLeft(con.get_analog(E_CONTROLLER_ANALOG_LEFT_Y));
		chas.spinRight(con.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y));
	}
	else
	{
		chas.stop();
	}
}

void reverseToggle()
{
	if(con.get_digital(E_CONTROLLER_DIGITAL_Y))
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
	if(con.get_digital(E_CONTROLLER_DIGITAL_UP))
	{
		if(chas.getBrakeMode() == 0) {chas.changeBrake(chas.HOLD);}
		else {chas.changeBrake(chas.COAST);}
		disableAuto = true;
		startTime = globalTime;
		while(con.get_digital(E_CONTROLLER_DIGITAL_UP)) {}
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
			if (abs(con.get_analog(E_CONTROLLER_ANALOG_RIGHT_X)) > 10|| abs(con.get_analog(E_CONTROLLER_ANALOG_LEFT_Y)) > 10)
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
				if (globalTime - startTime > 1000)
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
	static bool front = true;
	static double backSpeed = 0;
	static bool firstTime = false;
	static bool changeVelocity = true;
	static int startTime = 0;
	// Check if 'X' is being pressed and set back lift to coast if it is
	if(con.get_digital(E_CONTROLLER_DIGITAL_X))
		backLift.set_brake_mode(E_MOTOR_BRAKE_COAST);
	else
		backLift.set_brake_mode(E_MOTOR_BRAKE_HOLD);

	// Check if driver is currently controlling pneumatics or lifts
	if(con.get_digital(E_CONTROLLER_DIGITAL_L2) == false)
	{

		// Check L1 to see if driver wants to control front lift or back lift

		if(con.get_digital(E_CONTROLLER_DIGITAL_L1)) {front = false;}
		else {front = true;}
		// Move lift up
		if(con.get_digital(E_CONTROLLER_DIGITAL_R1))
		{
			if(front)
			{
				frontLift.move(-127);
				backLift.move(0);
			}
			else
			{
				frontLift.move(0);
				/*
				if(backLift.get_position() <= 5)
				{
					backLift.move(0);
				}
				else
				{
					backLift.move(127);
				}
				*/
				backLift.move(127);
			}
			firstTime = true;
			changeVelocity = true;
			startTime = 0;
		}
		// Move lift down
		else if(con.get_digital(E_CONTROLLER_DIGITAL_R2))
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
			firstTime = true;
			changeVelocity = true;
			startTime = 0;
		}
		// No buttons are being pressed so stop both lifts
		else
		{
			if(firstTime)
			{
				startTime = globalTime;
				firstTime = false;
			}
			if(globalTime - startTime <= 200) {backSpeed = 0;}
			else if(globalTime - startTime > 200 && changeVelocity)
			{
				backSpeed = -backLift.get_actual_velocity();
				changeVelocity = false;
			}

			if(globalTime - startTime > 700 && abs(backLift.get_actual_velocity()) > 1) {changeVelocity = true;}
			frontLift.move(0);
			backLift.move(backSpeed);
		}
	}
}

// Pneumatics
void pneumaticControl()
{
	// Check if l2 is pressed down, meaning driver wants to control the pneumatic
	if(con.get_digital(E_CONTROLLER_DIGITAL_L2))
	{
		// firstPress1 boolean is used to make sure the front pneumatic
		// doesn't repeatedly toggle on and off when R1 is held
		static bool firstPress1 = true;
		if(con.get_digital(E_CONTROLLER_DIGITAL_R1))
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
		if(con.get_digital(E_CONTROLLER_DIGITAL_R2))
		{
			// Check if this is R2's initial press, and toggle the pneumatic if it is.
			if(firstPress2)
			{
				frontPneu.toggle();
				firstPress2 = false;
			}
		}

		else
			firstPress2 = true;

		static bool firstPress3 = true;
		if(con.get_digital(E_CONTROLLER_DIGITAL_L1))
		{
			// Check if this is R2's initial press, and toggle the pneumatic if it is.
			if(firstPress3)
			{
				frontSpecialPneu.toggle();
				firstPress3 = false;
			}
		}

		// Allow the pneumatic to be toggled again after the button has been released
		else
			firstPress3 = true;
	}
}

void killAllAuto()
{
	if(disableAll) {disableAuto = true;}
	if(con.get_digital(E_CONTROLLER_DIGITAL_LEFT) && con.get_digital(E_CONTROLLER_DIGITAL_RIGHT))
	{
		con.clear();
		disableAll = !disableAll;
		while(con.get_digital(E_CONTROLLER_DIGITAL_LEFT) && con.get_digital(E_CONTROLLER_DIGITAL_RIGHT)) {}
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

			//con.print(0,0,"inert: %.2f", inert.get_heading());
			con.print(0,0,"inert: %.2f", inert.get_heading());
		}
		if(counter == 20)
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
//rotate(degrees, timeout)

//autonomous(will be called by competition)
void autonomous()
{
	/*
	KEY:
	elevated = side with platform elevated
	de-elevated = side with platform not elevated
	both = starts from de-elevated side but utilizes both side, most likely use in quals
	rush: rushes the neutral goal, will win 99% of the time, no win point, most likely use in elims
	long = gets at least 2 goals
	short = only gets one win point from alliance mobile goal
	*/

	midRight.move(127);

	while(!killAuton)
	{

		if(autonType == 1)
		{
			if(autonColor == 1) {redElevatedLong();}
			else {blueElevatedLong();}
		}

		if(autonType == 2)	//only gets one win point on elevated side
		{
			if(autonColor == 1) {redElevatedShort();}
			else {blueElevatedShort();}
		}

		if(autonType == 3)
		{
			if(autonColor == 1) {redDeElevatedLong();}
			else {blueDeElevatedLong();}
		}

		if(autonType == 4)
		{
			if(autonColor == 1) {redDeElevatedShort();}
			else {blueDeElevatedShort();}
		}

		if(autonType == 5)
		{
			if(autonColor == 1) {redBoth();}
			else {blueBoth();}
		}


		if(autonType == 6) {neutralRush();}
		if(autonType == 7) {skills();}

		break;
	}
}

// main control functions ======================================================
void initialize()
{
	lcd::initialize();
	frontLift.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	backLift.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	chas.changeBrake(chas.COAST);
	disableAuto = false;
}



void disabled() {
	chas.changeBrake(chas.HOLD);
	backLift.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	frontLift.set_brake_mode(E_MOTOR_BRAKE_HOLD);
}

void competition_initialize() {}

void opcontrol()
{
	lcd::set_text(0, "aayush the goat");
	con.clear();
	while(autonCurrentlySelecting) { autonSelector(); }

	chas.changeBrake(chas.COAST);
	backLift.set_brake_mode(E_MOTOR_BRAKE_HOLD);

	backPneu.toggle();
	while (true)
	{

		// Drive loop (there's an arcadeDrive() function and tankDrive() function.
		if(con.get_digital(E_CONTROLLER_DIGITAL_LEFT) && con.get_digital(E_CONTROLLER_DIGITAL_UP))
		{
			chasDisabled = true;
		}
		arcadeDrive(MANUAL);
		liftControl();
		pneumaticControl();
		reverseToggle();
		brakeType();
		autoBrakeMode();
		killAllAuto();
		printInfo();

		if(con.get_digital(E_CONTROLLER_DIGITAL_DOWN)) { autonomous(); }

		delay(5);
		globalTime += 5;
	}
}
