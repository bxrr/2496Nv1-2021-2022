#include "main.h"
#include "ports.h"
<<<<<<< HEAD
<<<<<<< HEAD
#include "globals.h"


PID drivePID(0.5,0.01,7);
PID autoStraightPID(2,0,0);
PID turnPID(1,1,1);
=======
>>>>>>> parent of 47d2a50 (october 18)
=======
>>>>>>> parent of 47d2a50 (october 18)

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
      else //brake type = S_HOLD
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


    void spinTo(double enc, int speedRaw)
    {
        int speed = speedRaw > 0 ? enc >= 0 ? speedRaw : -speedRaw : speedRaw;
        double startPos = frontLeft.get_position();
        changeBrake(COAST);
        while(abs(frontLeft.get_position() - startPos) < enc)
        {
          spinLeft(speed);
          spinRight(speed);
        }
        stop();
    }
<<<<<<< HEAD
<<<<<<< HEAD



    //pid functions
    void drive(double targetEnc, int timeout = 4000, double maxspeed = 1, double errorRange = 3) // timeout in milliseconds
    {
      reset();
      if(maxspeed > 1) {maxspeed /= 100;}
      int localTime = 0;
      double currentPosition = 0;
      double slewMult = 0;

      int withinRangeTime;
      bool withinRange = false;

      inert.set_heading(180);
      double initialRotation = inert.get_heading();
      double errorStartI = 10;


      while(localTime < timeout)
      {

        //currentPosition = (getLeftPos() - getRightPos()) / 2;
        currentPosition = (getRightPos() - getLeftPos()) / 2;

        //calculate(double currentPosition, double target, bool countIntegral)
        double speed = (slewMult * drivePID.calculate(currentPosition, targetEnc, abs(targetEnc - currentPosition) < errorStartI)) * maxspeed;
        if(abs(speed) > maxspeed * 127) {speed = speed > 0 ? maxspeed * 127 : -maxspeed * 127;}
        double autoStraight = autoStraightPID.calculate(inert.get_heading(), initialRotation, false);
        autoStraight = getVelocity() > 0 ? autoStraight : -autoStraight;
        if(localTime % 50 == 0) {con.print(1,0,"error: %.6f", targetEnc - currentPosition);}

        spinLeft(speed + (autoStraight * (speed/127)));
        spinRight(speed - (autoStraight * (speed/127)));


        if(abs(targetEnc - currentPosition) < errorRange)
        {
          if(!withinRange)
          {
            withinRangeTime = localTime;
            withinRange = true;
          }
          else if(localTime >= withinRangeTime + 250) { break; }
        }

        else { withinRange = false; }
        
        if(slewMult < 1) {slewMult += 0.025;}
        delay(5);
        localTime += 5;
      }

      changeBrake(HOLD);
      stop();
      reset();
      globalRotation += inert.get_heading() - initialRotation;
    }


    void rotate(double degrees, int timeout = 60000, double maxspeed = 1)
    {
      if(maxspeed > 1) {maxspeed /= 100;}
      int localTime = 0;

      if(degrees < 0) {inert.set_heading(350);}
      else if(degrees > 0) {inert.set_heading(10);}
        else {return;}

      double targetRotation = inert.get_heading() + degrees;
      double initialRotation = inert.get_heading();

      double currentRotation;
      int withinRangeTime;
      bool withinRange = false;
      double rotateStartI = 1;

      while(timeout > localTime)
      {
        currentRotation = inert.get_heading();
        bool integral = abs(initialRotation - currentRotation) <= rotateStartI;
        //calculate(double currentPosition, double target, bool countIntegral)
        double speed = turnPID.calculate(currentRotation - initialRotation, targetRotation, integral);


        if(abs(currentRotation - targetRotation) <= 0.5)
        {
          if(!withinRange)
          {
            withinRangeTime = localTime;
            withinRange = true;
          }
          else if(localTime >= withinRangeTime + 500) { break; }
        }
        else { withinRange = false; }


        spinLeft(speed * maxspeed);
        spinRight(-speed * maxspeed);

        localTime += 5;
        delay(5);


      }
      changeBrake(HOLD);
      stop();
      turnPID.resetI();
      reset();
    }

=======
>>>>>>> parent of 47d2a50 (october 18)
=======
>>>>>>> parent of 47d2a50 (october 18)
};
