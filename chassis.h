#include "main.h"
#include "port_defines.h"

class Chassis
{
private:
  pros::Motor backLeft;
	pros::Motor midLeft;
	pros::Motor frontLeft;
	pros::Motor backRight;
	pros::Motor midRight;
	pros::Motor frontRight;

public:
  Chassis() : backLeft(BACK_LEFT_PORT, pros::E_MOTOR_GEARSET_06, false),
              midLeft(MID_LEFT_PORT, pros::E_MOTOR_GEARSET_06, false),
              frontLeft(FRONT_LEFT_PORT, pros::E_MOTOR_GEARSET_06, false),
              backRight(BACK_RIGHT_PORT, pros::E_MOTOR_GEARSET_06, false),
              midRight(MID_RIGHT_PORT, pros::E_MOTOR_GEARSET_06, false),
              frontRight(FRONT_RIGHT_PORT, pros::E_MOTOR_GEARSET_06, false)
              {}

  // General spin functions ====================================================
  enum unitType {PCT, VOLT};
  void spinLeft(double speed, unitType unit=VOLT) // VOLT is -127 to 127, PCT is -100 to 100
  {
     if(unit == VOLT)
     {
         backLeft.move(speed);
         midLeft.move(speed);
         frontLeft.move(speed);
     }
     else
     {
         backLeft.move(speed * 127 / 100);
         midLeft.move(speed * 127 / 100);
         frontLeft.move(speed * 127 / 100);
     }
  }

  void spinRight(double speed, unitType unit=VOLT) // Spins right motors with accordance to the left motor spin function
  {
     if(unit == VOLT)
     {
         backRight.move(speed);
         midRight.move(speed);
         frontRight.move(speed);
     }
     else
     {
         backRight.move(speed * 127 / 100);
         midRight.move(speed * 127 / 100);
         frontRight.move(speed * 127 / 100);
     }
  }

  // Change brake type =========================================================
  void brakeHold()
  {
     backLeft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
     midLeft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
     frontLeft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
     backRight.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
     midRight.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
     frontRight.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  }

  void brakeCoast()
  {
    backLeft.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    midLeft.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    frontLeft.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    backRight.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    midRight.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    frontRight.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  }

  // Motor position functions ==================================================

  double getLeftPos() // Returns the current average encoding of all the left chassis motors
  {
     return (backLeft.get_position() +
            midLeft.get_position() +
            frontLeft.get_position()) / 3;
  }

  double getRightPos() // Returns the current average encoding of all the right chassis motors
  {
     return (backRight.get_position() +
             midRight.get_position() +
             frontRight.get_position()) / 3;
  }
};
