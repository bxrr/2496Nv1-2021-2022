#include "api.h"
#include "okapi/api.hpp"
#include <string>

using namespace okapi;


std::shared_ptr<ChassisController> chassis =
ChassisControllerBuilder()
    .withMotors(
        { 1, 2, -3 }, //right side, front, mid, back
        { -4, -5, 6} //left side, front, mid, back
    )
    // Green gearset, 4 in wheel diam, 11.5 in wheel track
    .withDimensions({AbstractMotor::gearset::blue, (50.0 / 36.0)}, {{4_in, 11.5_in}, imev5GreenTPR})
    .build();


std::shared_ptr<AsyncMotionProfileController> profileController =
  AsyncMotionProfileControllerBuilder()
    .withLimits({
      1.0, // Maximum linear velocity of the Chassis in m/s
      2.0, // Maximum linear acceleration of the Chassis in m/s/s
      10.0 // Maximum linear jerk of the Chassis in m/s/s/s
    })
    .withOutput(chassis)
    .buildMotionProfileController();


void generatePath(double x, double y, double deg, std::string name)
{
    profileController->generatePath({
    {0_ft, 0_ft, 0_deg},  // Profile starting position, this will normally be (0, 0, 0)
    {1_ft * x, 1_ft * x, 1_deg * deg}}, // The next point in the profile, 3 feet forward
    name // Profile name
);
}

void move(std::string name) {
    profileController->setTarget(name);
    profileController->waitUntilSettled();
}