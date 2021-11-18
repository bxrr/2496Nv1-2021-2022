class PID
{
    private:

        //instance fields
        double kP;
        double kI;
        double kD;

        double lastError;
        double I;

    public:

        //constructor
        PID(double kP, double kI, double kD)
        {
            this->kP = kP;
            this->kI = kI;
            this->kD = kD;
        }

        PID(double kP) : kI(0), kD(0)
        {
            this->kP = kP;
        }


    //getters
    double getkP() {return kP;}
    double getkI() {return kI;}
    double getkD() {return kD;}
    double getLast() {return lastError;}
    double getCurI() {return I;}

    void setkI(double n) {kI = n;}

    void resetI() {I = 0;}

    //change value of object PID
    void modify(double kPnew, double kInew = -24204124, double kDnew = -24204124)
    {
        kP = kPnew;
        if(kInew != -24204124) {kI = kInew;}
        if(kDnew != -24204124) {kD = kDnew;}
    }


    //calculate speed to run chassis
    double calculate(double currentPosition, double target, bool countIntegral=false)
    {
        double error = target - currentPosition;
        double D = lastError - error;
        if(countIntegral) {I += error;}
        lastError = error;
        return (kP * error) + (kI * I) + (kD * D);
    }
};
