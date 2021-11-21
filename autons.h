#include "main.h"
#include "chassis.h"

Chassis ch;

double backGoals = 0;
double frontGoals = 0;
void drive(double targetEnc, int timeout = 4000, double maxspeed = 1, double errorRange = 3)
{
	if(timeout == 4000) {timeout = abs(targetEnc)*4;}
	ch.drive(targetEnc, timeout, maxspeed, errorRange);
}
void curve(double degrees, bool backward=false, double amplifier=1, int timeout=5000) {ch.curve(degrees, backward, amplifier, timeout);}
void rotate(double degrees, int timeout = 5000, double maxspeed = .9)  { ch.rotate(degrees, timeout, maxspeed); }
void rotateTo(double degrees, int timeout = 5000, double speedM  = .9) { ch.rotate(degrees - globalRotation, timeout, speedM); }

void redElevatedLong()
{
	ch.spinTo(700,127);
	frontPneu.toggle();
	frontGoals = 1;
	frontLift.move_absolute(-800, -127);
	ch.spinTo(-700, -127);
	ch.spinTo(-500, -50, 1, 1500);
	drive(450, 2000);
	rotateTo(-90);
	backLift.move_absolute(-2400, -100);
	drive(-100, 2000);
	delay(500);
	drive(300);
	backLift.move_absolute(0, 127);
	rotateTo(-80, 900);
	drive(-500, 1200);
	backPneu.toggle();
	delay(200);
	drive(400);
}

void redElevatedShort()
{
	ch.spinTo(700,127);
	frontPneu.toggle();
	frontGoals = 1;
	frontLift.move_absolute(-800, -127);
	drive(-655, 2500, 1, 80);
	frontPneu.toggle();
	drive(-250);
	rotateTo(-39, 1200);
	frontLift.move_absolute(0, -127);
	drive(1000, 2000, 1, 50);
	frontPneu.toggle();
	frontGoals = 1;
	frontLift.move_absolute(-800, -127);
	drive(-1000);
	backLift.move_absolute(-3000, -127);
	rotateTo(-135, 1400);
	drive(-400, 1500);
	backLift.move_absolute(-1000, 127);
	delay(200);
	drive(200);
}

void redDeElevatedLong()
{
	ch.spinTo(760,127);
	frontPneu.toggle();
	frontGoals = 1;
	frontLift.move_absolute(-800, -127);
	ch.spinTo(-800, -127);
	ch.stop();
	rotateTo(-20);
	ch.spinTo(-400, -50, 1, 1500);
	drive(165, 2000);
	/*
	rotate(-80, 1500);
	backLift.move_absolute(-2400, -127);
	drive(-100, 1500);
	delay(500);
	drive(300);
	*/
	rotateTo(-81, 2000);
	backLift.move_absolute(-3000, -127);
	drive(150, 1500);
	drive(-500, 1500);
	backLift.move_absolute(-1000, 127);
	delay(500);

}

void redDeElevatedShort()
{
	generatePath(3,0,0, "A");
	move("A");
}

void redBoth()
{
	ch.park();
}

void blueElevatedLong()
{
	ch.spinTo(700,127);
	frontPneu.toggle();
	frontGoals = 1;
	frontLift.move_absolute(-800, -127);
	ch.spinTo(-700, -127);
	ch.spinTo(-500, -50, 1, 1500);
	drive(450, 2000);
	rotateTo(-90);
	backLift.move_absolute(-2400, -100);
	drive(-100, 2000);
	delay(500);
	drive(300);
	backLift.move_absolute(0, 127);
	rotateTo(-80, 900);
	drive(-500, 1200);
	backPneu.toggle();
	delay(200);
	drive(400);
}

void blueElevatedShort()
{
	ch.spinTo(700,127);
	frontPneu.toggle();
	frontGoals = 1;
	frontLift.move_absolute(-800, -127);
	drive(-725);
	rotateTo(50, 1500);
	frontPneu.toggle();
	drive(-125);
	rotateTo(-44, 1500);
	frontLift.move_absolute(0, -127);
	drive(700, 2000);
	frontPneu.toggle();
	frontGoals = 1;
	frontLift.move_absolute(-800, -127);
	drive(-800);
}

void blueDeElevatedLong()
{
	ch.spinTo(760,127);
	frontPneu.toggle();
	frontGoals = 1;
	frontLift.move_absolute(-800, -127);
	ch.spinTo(-800, -127);
	ch.stop();
	rotate(-20);
	ch.spinTo(-400, -50, 1, 1500);
	drive(165, 2000);
	rotate(-82, 1500);
	backLift.move_absolute(-2400, -127);
	drive(-85, 1500);
	delay(500);
	drive(300);
}

void blueDeElevatedShort()
{
	drive(200);
	rotate(45);
	drive(400);
	rotate(90);
	drive(100);
	rotate(45);
	drive(600);
}

void blueBoth()
{
	drive(500);
}

void neutralRush()
{
	ch.spinTo(760,127);
	frontPneu.toggle();
	frontLift.move_absolute(-600, -127);
	drive(-700);
}


void skills()
{
	//actually skills lol
	backLift.move_absolute(-3000, -127);
	drive(410);
	rotateTo(-90);
	drive(-300, 1500);
	backLift.move_absolute(-1200, 127);

	backGoals = 1;

	delay(500);
	drive(300);
	rotateTo(0);
	drive(400);
	frontPneu.toggle();
	delay(100);
	frontLift.move_absolute(-750, -127);

	frontGoals = 1;

	drive(150);
	rotateTo(30);
	drive(710);
	rotateTo(0);
	drive(300);
	frontLift.move_absolute(10, 127);
	delay(200);
	frontPneu.toggle();

	frontGoals = 0;

	delay(1000);
	drive(-250, 2000);
	rotateTo(-46);
	drive(475);
	frontPneu.toggle();
	delay(200);
	frontLift.move_absolute(-750, -127);

	frontGoals = 1;

	delay(650);
	drive(-435);
	rotateTo(-90);
	backLift.move_absolute(-3050, 127);
	drive(200);
	frontLift.move_absolute(-1200, -127);
	delay(300);

	backGoals = 0;

	drive(170);
	backLift.move_absolute(30, 127);
	//delay(500);
	rotateTo(90);
	backLift.move_absolute(-3000, -127);
	drive(-1000, 2000, 0.7);
	rotateTo(90, 500, 1.2);
	drive(-550, 1000);

	backGoals = 1;

	backLift.move_absolute(100, 127);
	delay(100);
	rotateTo(90, 800, 1.2);
	drive(250);
	rotateTo(0, 2000);
	backLift.move_absolute(-3000, -127);
	delay(650);
	drive(-525, 2000, 0.4);
	backLift.move_absolute(-1200, 127);

	frontGoals = 1;
	backGoals = 2;


	delay(200);
	rotateTo(35, 1000);
	drive(-300, 1500);
	rotateTo(180, 2000);
	frontLift.move_absolute(-4200, -127);
	drive(1130, 3100, 0.8);
	drive(-100, 800);
	rotateTo(90,2000, 1.05);
	drive(300, 1000);
	frontLift.move_absolute(-2000, -127);
	frontGoals = 2;
	backGoals = 2;

	rotateTo(88.5, 500, 2);
	ch.park();
}