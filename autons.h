#include "main.h"
#include "chassis.h"

Chassis ch;

void drive(double targetEnc, int timeout = 4000, double maxspeed = 1, double errorRange = 3)  { ch.drive(targetEnc, timeout, maxspeed, errorRange); }
void rotate(double degrees, int timeout = 60000, double maxspeed = 1)  { ch.rotate(degrees, timeout, maxspeed); }
void rotateTo(double degrees, int timeout=100000, double speedM  = 1) { ch.rotate(degrees - globalRotation, timeout, speedM); }

void redElevatedLong()
{
	drive(600, 1000000, .99999);
}

void redElevatedShort()
{
}

void redDeElevatedLong()
{
}

void redDeElevatedShort()
{
}

void redBoth() // 10/11
{
}

void blueElevatedLong()
{
}

void blueElevatedShort()
{

}

void blueDeElevatedLong()
{

}

void blueDeElevatedShort()
{

}

void blueBoth()
{

}


void skillsFuture()
{

	
}

void skills()
{

}


void neutralRush()
{
}