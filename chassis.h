#include "main.h"
#include "ports.h"

class Chassis
{
private:
    pros::Motor backLeft;
    pros::Motor midLeft;
    pros::Motor frontLeft;
    pros::Motor backRight;
    pros::Motor midRight;
    pros::Motor frontRight;
    bool reverse;
    bool reverseButton;

public:
    Chassis() : backLeft(BACK_LEFT_PORT, pros::E_MOTOR_GEARSET_06, false),
    midLeft(MID_LEFT_PORT, pros::E_MOTOR_GEARSET_06, false),
    frontLeft(FRONT_LEFT_PORT, pros::E_MOTOR_GEARSET_06, false),
    backRight(BACK_RIGHT_PORT, pros::E_MOTOR_GEARSET_06, false),
    midRight(MID_RIGHT_PORT, pros::E_MOTOR_GEARSET_06, false),
    frontRight(FRONT_RIGHT_PORT, pros::E_MOTOR_GEARSET_06, false),
    reverse(false),
    reverseButton(true)
    {}

    // Spin methods ======================================================
    enum unitType {PCT, VOLT};
    void spinLeft(double speed, unitType unit=VOLT) // VOLT is -127 to 127, PCT is -100 to 100
    {
        speed = (reverse) ? (-speed) : (speed);
        if(unit == VOLT)
        {
            backLeft.move(speed);
            midLeft.move(-speed);
            frontLeft.move(-speed);
        }
        else
        {
            backLeft.move(speed * 127 / 100);
            midLeft.move(-speed * 127 / 100);
            frontLeft.move(-speed * 127 / 100);
        }
    }

    void spinRight(double speed, unitType unit=VOLT) // Spins right motors with accordance to the left motor spin function
    {
        speed = (reverse) ? (-speed) : (speed);
        if(unit == VOLT)
        {
            backRight.move(-speed);
            midRight.move(speed);
            frontRight.move(speed);
        }
        else
        {
            backRight.move(-speed * 127 / 100);
            midRight.move(speed * 127 / 100);
            frontRight.move(speed * 127 / 100);
        }
    }

    // Change brake type/stop ===================================================
    void stop()
    {
        spinLeft(0);
        spinRight(0);
    }

    enum brakeTypes {COAST, HOLD, S_HOLD}; // special hold applies motor speed to prevent the robot from moving.
    void changeBrake(brakeTypes bT, double inertialv = 0, double kP = 1.45)
    {
      if(bT == COAST)
      {
        backLeft.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        backRight.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        frontLeft.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        frontRight.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        midLeft.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        midRight.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
      }
      else if(bT == HOLD)
      {
        backLeft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        backRight.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        frontLeft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        frontRight.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        midLeft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        midRight.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      }
      else
      {
        if (reverse) {kP = -kP;}
        double speed = inertialv * kP;

        spinLeft(speed);
        spinRight(speed);

        backLeft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        backRight.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        frontLeft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        frontRight.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        midLeft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        midRight.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      }
    }

    //HOLD = 1, COAST = 0, 2 = neither coast nor hold
    int getBrakeMode()
    {
      if(frontLeft.get_brake_mode() == pros::E_MOTOR_BRAKE_HOLD) {return(1);}
      else if(frontLeft.get_brake_mode() == pros::E_MOTOR_BRAKE_COAST) {return(0);}
      else {return(2);}
    }

    // Motor position functions ==================================================

    double getLeftPos() // Returns the current average encoding of all the left chassis motors
    {
      return (frontLeft.get_position() + midLeft.get_position() + backLeft.get_position()) / 3;
    }

    double getRightPos() // Returns the current average encoding of all the right chassis motors
    {
      return (frontRight.get_position() + midRight.get_position() + backRight.get_position()) / 3;
    }

    double getVelocity()
    {
      return (frontRight.get_actual_velocity() + frontLeft.get_actual_velocity() + midLeft.get_actual_velocity() + midRight.get_actual_velocity() + backRight.get_actual_velocity() + backLeft.get_actual_velocity()) / 6 ;
    }

    // Chassis reverse control ===================================================
    void reverseControls()
    {
      if(reverseButton)
      {
        reverse = (reverse) ? (false) : (true);
        reverseButton = false;
      }
    }

    void reverseReleased() { reverseButton = true; }

    bool reverseStatus()
    {
      return reverse;
    }

    // Temperature ===============================================================
    double leftTemp()
    {
      return (backLeft.get_temperature() +
              midLeft.get_temperature() +
              frontLeft.get_temperature()) / 3;
              
    }

    double rightTemp()
    {
      return (backRight.get_temperature() +
              midRight.get_temperature() +
              frontRight.get_temperature()) / 3;
    }
};
