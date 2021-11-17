#include "main.h"
#include "ports.h"
#include "globals.h"
#include <math.h>


PID drivePID(0.25, 0.01, 0); //tuned at all goals possessed
PID autoStraightPID(1.5, 0, 0);
PID turnPID(1.5, 0.085, 0);
PID curvePID(1.5, 0.08, 0);
//to try:
//PID turnPID(1.6,0.1,2);
//float kP = (1.5*(90/degrees) > 2 ? (2) : (1.5*(90/degrees) < 1.5 ? (1.5) : 1.5*(90/degrees)))

class Chassis
{
private:
    bool reverse;
    bool reverseButton;

public:


    Chassis() :
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
      return (frontRight.get_actual_velocity() - frontLeft.get_actual_velocity() - midLeft.get_actual_velocity() + midRight.get_actual_velocity() - backRight.get_actual_velocity() + backLeft.get_actual_velocity()) / 6 ;
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


    void reset()
    {
      frontLeft.tare_position();
      frontRight.tare_position();
      midLeft.tare_position();
      midRight.tare_position();
      backLeft.tare_position();
      backRight.tare_position();
      drivePID.resetI();
      turnPID.resetI();
    }

    void spinTo(double enc, int speed, int brakeType = 1, double timeout = 4000)
    {
      reset();
      int timer = 0;
      changeBrake(brakeType == 1 ? COAST : HOLD);
      while(abs(getRightPos()) < abs(enc))
      {
        spinLeft(speed);
        spinRight(speed);
        if (timer % 50 == 0) con.print(0,0,"inert: %.2f", getRightPos());
        timer += 5;
        pros::delay(5);
        if(timer >= timeout) break;
      }
      stop();
    }

    //pid/autonomous functions
    void turnPIDadjuster()
    {
      if(frontGoals == 0)
      {
        if(backGoals == 0)
        {
          turnPID.modify(1.6, 0.085, 2);
        }

        if(backGoals == 1)
        {
          turnPID.modify(1.72, 0.085, 2.2);
        }

        if(backGoals == 2)
        {
          turnPID.modify(1.66, 0.085, 2.2);
        }
      }

      if(frontGoals == 1)
      {
        if(backGoals == 0)
        {
          //need to fix smaller degree turns
          turnPID.modify(1.3, 0.095, 2);
        }

        if(backGoals == 1)
        {
          turnPID.modify(1.4, 0.085, 2);
        }

        if(backGoals == 2)
        {
          turnPID.modify(1.4, 0.085, 2);
        }
      }
    }


    void drive(double targetEnc, int timeout = 4000, double maxspeed = 1, double errorRange = 7)
    {
      reset();
      if(maxspeed > 5) {maxspeed /= 100;}
      int localTime = 0;
      double currentPosition = 0;
      double slewMult = 0;

      int withinRangeTime;
      bool withinRange = false;

      inert.set_heading(180);
      double initialRotation = inert.get_heading();
      double errorStartI = 30;
      bool countInt = false;


      while(localTime < timeout)
      {

        //currentPosition = (getLeftPos() - getRightPos()) / 2;
        currentPosition = (getRightPos() - getLeftPos()) / 2;

        if(abs(targetEnc - currentPosition) < errorStartI) {countInt = true;}

        //calculate(double currentPosition, double target, bool countIntegral)
        double speed = (slewMult * drivePID.calculate(currentPosition, targetEnc, countInt)) * maxspeed;
        if(abs(speed) >  maxspeed * 127) {speed = speed > 0 ? maxspeed * 127 : -maxspeed * 127;}
        double autoStraight = autoStraightPID.calculate(inert.get_heading(), initialRotation, false);
        autoStraight = getVelocity() > 0 ? autoStraight : -autoStraight;
        if(localTime % 50 == 0) {con.print(1,0,"error: %.2f", targetEnc - currentPosition);}

        spinLeft(speed + (autoStraight * speed/127));
        spinRight(speed - (autoStraight * speed/127));


        if(abs(targetEnc - currentPosition) < errorRange)
        {
          if(!withinRange)
          {
            withinRangeTime = localTime;
            withinRange = true;
          }
          else if(localTime >= withinRangeTime + 200) { break; }
        }

        else { withinRange = false; }

        if(slewMult < 1) {slewMult += 0.02;}
        delay(10);
        localTime += 10;
      }

      changeBrake(HOLD);
      stop();
      reset();
      globalRotation += inert.get_heading() - initialRotation;
    }


    void rotate(double degrees, int timeout = 60000, double maxspeed = 1)
    {
      reset();
      if(maxspeed > 10) {maxspeed /= 100;}
      int localTime = 0;

      if(degrees < 0) {inert.set_heading(350);}
      else if(degrees > 0) {inert.set_heading(10);}
        else {return;}

      double targetRotation = inert.get_heading() + degrees;
      double initialRotation = inert.get_heading();

      double currentRotation;
      int withinRangeTime;
      bool withinRange = false;
      double rotateStartI = 8;
      bool integral = false;

      double lastCheck = 1000000;
      bool temp = false;
      // turnPIDadjuster();
      //set kP between 1.5 and 2 depending on degree of rotation
      double kpAdj = pow(turnPID.getkP(), pow(90/abs(degrees), 0.6));
      // if(frontGoals == 1)
      //   tempAdj = pow(turnPID.getkP(), pow(90/abs(degrees), 1.12));
      turnPID.modify(kpAdj > 4 ? 4 : kpAdj < turnPID.getkP() - 0.1 ? turnPID.getkP() - 0.1 : kpAdj);

      double ogKI = turnPID.getkI();

      while(timeout > localTime)
      {
        currentRotation = inert.get_heading();
        if(abs(targetRotation - currentRotation) <= rotateStartI) { integral = true; }

        //calculate(double currentPosition, double target, bool countIntegral)
        double speed = turnPID.calculate(currentRotation, targetRotation, integral);
        {
          con.print(1,0,"Error: %.2f      ", (currentRotation - targetRotation));
        }


        if(abs(currentRotation - targetRotation) <= 0.75)
        {
          if(!withinRange)
          {
            withinRangeTime = localTime;
            withinRange = true;
          }
          else if(localTime >= withinRangeTime + 300) { break; }
        }
        else { withinRange = false; }


        spinLeft(speed * maxspeed);
        spinRight(-speed * maxspeed);

        localTime += 10;
        delay(10);


      }
      changeBrake(HOLD);
      stop();
      turnPID.resetI();
      turnPID.setkI(ogKI);
      reset();
      globalRotation += inert.get_heading() - initialRotation;
    }


    void curve(double degrees, bool clockwise=true, double amplifier=1, int timeout=5000)
    {
      if(degrees > 0) inert.set_heading(10);
      else inert.set_heading(350);

      double targetHeading = inert.get_heading() + degrees;
      double curPos = inert.get_heading();

      bool exitFirst = true;
      double exitRange = 0.5; // degrees
      double exitTimeout = 250; //ms
      double exitStartTime;

      int timer = 0;

      while(true)
      {
        curPos = inert.get_heading();
        double speed = curvePID.calculate(curPos, targetHeading);

        if(clockwise)
        {
          spinLeft(speed);
          spinRight(speed * 0.4 * amplifier);
        }
        else
        {
          spinLeft(speed * 0.4 * amplifier);
          spinRight(speed);
        }

        if(timer >= timeout)
        {
          break;
        }

        if(abs(targetHeading-curPos) < exitRange)
        {
          if(exitFirst)
          {
            exitFirst = false;
            exitStartTime = timer;
          }
          else if(exitStartTime + exitTimeout <= timer) break;
        }
        else
        {
          exitFirst = true;
        }

        delay(5);
        timer += 5;
      }
      stop();
    }


    void park(bool pistonUsed = false)
    {
      bool doOnce = true;
      //if(frontGoals + backGoals < 4) doOnce = false;
      bool parking = false;
      int localTime = 0;
      double lastPitch = 0;
      double maxPitch = 0;
      double pitch = 0;
      int parkStartTime;
      while(true)
      {
        pitch = abs(inert.get_pitch());
        double pitchVelocity = pitch - lastPitch;
        //prevent the back lift from falling down if holding mobile goal
        backLift.move(30);

        //prevent the front lift from inhibiting the park, if being used
        if(pitch > 1 && pistonUsed) {frontLift.move(-18);}


        if(pitch > maxPitch) { maxPitch = pitch; }


        if(pitch > 21)		//move from step 1 to 2
        {
          if(!parking) parkStartTime = localTime;
          parking = true;
          /*
          if(localTime - parkStartTime >= 1000 && localTime - parkStartTime >= 1100)
          {
            maxPitch = pitch;
          }
          */
        }

        if(pitch < 15 && !parking)		//step 1(initialize)
        {
          spinLeft(127);
          spinRight(127);
        }


        else if(pitch < maxPitch - 4 && parking)	//step 3(finalize park)
        {
          if(doOnce)
          {
            changeBrake(HOLD);
            stop();
            delay(100);
            spinTo(-30, -35, 2);
            doOnce = false;
          }
          backLift.move(0);
          frontLift.move(0);
        }

        else
        {
          //step 2(moving up the platform)
          doOnce = true;
          //changeBrake(S_HOLD, pitch, 6 + 0.7 *(frontGoals + backGoals));
          spinLeft(120);
          spinRight(120);
        }

        if(localTime % 50 == 0)
        {
          if(parking) con.print(0,0, "max: %.1f, %.1f", maxPitch, pitch);
          lastPitch = pitch;
        }
        localTime += 5;
        delay(5);

      }
    }
};
