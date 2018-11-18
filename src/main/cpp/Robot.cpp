
#include "Robot.h"
#include <iostream>
#include "Lib830.h"


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
Timer autonTimer{};
static const int TICKS_TO_ACCEL = 20;

void Robot::RobotInit() {
    gyro.Calibrate();

}

void Robot::RobotPeriodic() {

}

void Robot::AutonomousInit() {
    // Reset Timer
    autonTimer.Reset();
    autonTimer.Start();
}
void Robot::AutonomousPeriodic() {
    if (autonTimer.Get() < 8) {
        //m_drive.DriveCartesian(0.2, 0, 0);
        double gyroReading = gyro.GetAngle();
        SmartDashboard::PutNumber("getAngleAuto",gyroReading);
        double correctedGyro = - gyroReading / 90;
        m_drive.DriveCartesian(0.4, 0, correctedGyro);
    } else {
        autonTimer.Stop();
    }
    
    
}

void Robot::TeleopInit() {
    // supposedly 'calibrates' the robot, doesn't actually.
    gyro.Calibrate();

}

//unfinished code
double SmoothCosineTransformation(double input) {
    // Use a cosine function to activate a smooth start and stop
    double output = input;
    return output;
}

double prev_y;
double prev_x;

void Robot::TeleopPeriodic() {
    // using a ternary operator to set rawX/Y to 0 if value is below minimum threshold
    double rawX = abs(pilot.GetX(LEFT)) < DEADZONE_THRESHOLD ? 0 : pilot.GetX(LEFT);
    double rawY = abs(pilot.GetY(LEFT)) < DEADZONE_THRESHOLD ? 0 : pilot.GetY(LEFT);
    double turn = pilot.GetX(RIGHT);

    // Debug
    SmartDashboard::PutNumber("getAngle",gyro.GetAngle());

    SmartDashboard::PutData("Gyro",&gyro);

    // double y = SmoothCosineTransformation(-rawY); // Testing, robot has inverse controls
    // double x = SmoothCosineTransformation(rawX);
    m_drive.DriveCartesian(rawY, rawX, turn);

}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
START_ROBOT_CLASS(Robot)
#endif
