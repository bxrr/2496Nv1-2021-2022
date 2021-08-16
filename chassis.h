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
        //maybe add brake type variable here?
    }

    enum brakeTypes {COAST, HOLD, S_HOLD}; // special hold applies motor speed to prevent the robot from moving.
    void changeBrake(brakeTypes bT)
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
      
      else if(bT == S_HOLD)
      {
        float kP = 1;
        static bool firstRun = true;
        static double leftStart;
        static double rightStart;
        static double lError;
        static double rError;
        if(firstRun)
        {
          leftStart = getLeftPos();
          rightStart = getRightPos();
          lError = 0;
          rError = 0;
          firstRun = false; 
          backLeft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
          backRight.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
          frontLeft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
          frontRight.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
          midLeft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
          midRight.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        }
        else
        {
          lError = leftStart - getLeftPos();
          rError = rightStart - getRightPos();
          if(lError > 50 || rError > 50)
          {
            spinLeft(-lError * kP);
            spinRight(-rError * kP);
          }
          else
          {
            spinLeft(0);
            spinRight(0);
          }
        }
        
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
