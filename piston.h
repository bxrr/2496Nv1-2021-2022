#include "main.h"
#include "ports.h"

class Piston // pneumatic class
{
private:
    pros::ADIDigitalOut pneu;
    bool extended;
public:
    Piston() : pneu(PNEUMATIC_PORT), extended(false) {}

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
