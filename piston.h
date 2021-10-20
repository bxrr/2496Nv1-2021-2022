class Piston // pneumatic class
{
private:
    pros::ADIDigitalOut pneu;
    bool extended;
public:
    Piston(int port) : pneu(port), extended(true) {pneu.set_value(true);}

    // general methods
    void toggle()
    {
        if(extended)
        {
            extended = false;
            pneu.set_value(extended);
        }
        else
        {
            extended = true;
            pneu.set_value(extended);
        }
    }

    bool status()
    {
        return extended;
    }
};