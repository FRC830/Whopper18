
#include "Robot.h"
#include <Lib830.h>
// Including Lib830 using full path within build.gradle/wpilib_preferences.json
#include <iostream>



static  const int FRONT_LEFT_WHEEL = 0;  //Same values as Robot2018 Github
static const int FRONT_RIGHT_WHEEL = 6;
static const int BOTTOM_LEFT_WHEEL = 1;
static const int BOTTOM_RIGHT_WHEEL = 7;
static const int GYRO_ANALOG_IN = 0;
static const double DEADZONE_THRESHOLD = .05;
VictorSP frontl{FRONT_LEFT_WHEEL};
VictorSP frontr{FRONT_RIGHT_WHEEL};
VictorSP backl{BOTTOM_LEFT_WHEEL};
VictorSP backr{BOTTOM_RIGHT_WHEEL};
MecanumDrive m_drive{frontl, frontr, backl, backr};

static const GenericHID::JoystickHand LEFT = GenericHID::kLeftHand;
static const GenericHID::JoystickHand RIGHT = GenericHID::kRightHand;

XboxController pilot{0};
AnalogGyro gyro{GYRO_ANALOG_IN};

void Robot::RobotInit() {

}

void Robot::RobotPeriodic() {

}

void Robot::AutonomousInit() {
}

void Robot::AutonomousPeriodic() {
}

void Robot::TeleopInit() {
    // supposedly 'calibrates' the robot.
    gyro.Calibrate();

}

void Robot::TeleopPeriodic() {
    // using a ternary operator to set rawX/Y to 0 if value is below minimum threshold
    double rawX = abs(pilot.GetX(LEFT)) < DEADZONE_THRESHOLD ? 0 : pilot.GetX(LEFT)
    double rawY = abs(pilot.GetY(LEFT)) < DEADZONE_THRESHOLD ? 0 : pilot.GetY(LEFT)
    double turn = pilot.GetX(RIGHT);

    // Debug
    SmartDashboard::PutNumber("getAngle",gyro.GetAngle());
    SmartDashboard::PutData("Gyro",&gyro);

    double y = SmoothCosineTransformation(-rawY); // Testing, robot has inverse controls
    double x = SmoothCosineTransformation(rawX);
    m_drive.DriveCartesian(y , x, turn);

}
SmoothCosineTransformation(input double) {
    // Use a cosine function to activate a smooth start and stop
    double output = input;
    return output
}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
START_ROBOT_CLASS(Robot)
#endif
