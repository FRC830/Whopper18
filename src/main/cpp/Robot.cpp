
#include "Robot.h"

#include <iostream>


#include <SmartDashboard/SmartDashboard.h>

static  const int FRONT_LEFT_WHEEL = 1;  //Random Values
static const int FRONT_RIGHT_WHEEL = 2;
static const int BOTTOM_LEFT_WHEEL = 3;
static const int BOTTOM_RIGHT_WHEEL = 4;

VictorSP frontl{FRONT_LEFT_WHEEL};
VictorSP frontr{FRONT_RIGHT_WHEEL};
VictorSP backl{BOTTOM_LEFT_WHEEL};
VictorSP backr{BOTTOM_RIGHT_WHEEL};
MecanumDrive m_drive{frontl, frontr, backl, backr};

static const GenericHID::JoystickHand LEFT = GenericHID::kLeftHand;
static const GenericHID::JoystickHand RIGHT = GenericHID::kRightHand;

XboxController pilot{0};

void Robot::RobotInit() {

}

void Robot::RobotPeriodic() {

}

void Robot::AutonomousInit() {
}

void Robot::AutonomousPeriodic() {
}

void Robot::TeleopInit() {

}

void Robot::TeleopPeriodic() {
    double x = pilot.GetX(LEFT);
    double y = pilot.GetY(LEFT);
    double turn = pilot.GetX(RIGHT);

    m_drive.DriveCartesian(x , y, turn);

}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
START_ROBOT_CLASS(Robot)
#endif
