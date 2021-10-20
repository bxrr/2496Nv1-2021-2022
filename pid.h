#include "main.h"
#include "ports.h"
using namespace pros;

>>>>>>> parent of 47d2a50 (october 18)
class PID
{
    private:

        //instance fields
        double kP;
        double kI;
        double kD;
    
    public:

        //constructor
        PID(double kP, double kI, double kD)
        {
            this->kP = kP;
            this->kI = kI;
            this->kD = kD;
        }


    //getters
    double getkP() {return kP;}
    double getkI() {return kI;}
    double getkD() {return kD;}

    //change value of object PID
    void modify(double kPnew, double kInew = 100000, double kDnew = 100000)
    {
        kP = kPnew;
        if(kInew != 100000) {kI = kInew;}
        if(kDnew != 100000) {kD = kDnew;}
    }


    //calculate speed to run chassis
    double calculate(double initialPosition, double currentPosition, double target, bool countIntegral, double positionDifference)
    {
        static double I = 0;
		double error = target - (initialPosition - currentPosition);
        double lastError = positionDifference + error;
        double D = lastError - error;
        if(countIntegral) {I += error;}
        return (kP * error) + (kI * I) + (kD * D);
    }

};