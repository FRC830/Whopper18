
#include "Robot.h"

void Robot::RobotInit() {
    gyro.Calibrate();
    sonic_sensor.SetEnabled(true);
    sonic_sensor.SetAutomaticMode(true);
    chooser.AddDefault("Nothing", AutoMode(NOTHING));
    chooser.AddObject("Side", AutoMode(SIDE));
    chooser.AddObject("Center", AutoMode(CENTER));
    SmartDashboard::PutData("Chooser", &chooser);
}

void Robot::RobotPeriodic() {

}

void Robot::AutonomousInit() {
    // Reset Timer
    autonTimer.Reset();
    autonTimer.Start();

}
void Robot::AutonomousPeriodic() {

    double rangeMeasurement = sonic_sensor.GetRangeInches();
    SmartDashboard::PutNumber("sonicRange (in)", rangeMeasurement);
    int distance_from_center = 70;
    int random_threshold = 5;
    if (rangeMeasurement <= random_threshold || rangeMeasurement >= distance_from_center) {
        double rawGyroReading = gyro.GetAngle();
        SmartDashboard::PutNumber("getAngleAuto",rawGyroReading);
        double correctedGyro = - rawGyroReading / 90;
        m_drive.DriveCartesian(0.25, 0, correctedGyro);
    } else {
        m_drive.DriveCartesian(0, 0, 0);
    }
    AutoMode mode = chooser.GetSelected();
    if (mode == NOTHING) {
         

    }
    else if (mode == SIDE) {

    }
    else if (mode == CENTER) {

    }
    
}

void Robot::TeleopInit() {
    // supposedly 'calibrates' the robot, doesn't actually.

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
