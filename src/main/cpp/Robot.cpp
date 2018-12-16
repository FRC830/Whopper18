
#include "Robot.h"
#include "Shooter.h"

void Robot::RobotInit() {
    gyro.Calibrate();
    sonic_sensor.SetEnabled(true);
    sonic_sensor.SetAutomaticMode(true);
    chooser.AddDefault("Nothing", AutoMode(NOTHING));
    chooser.AddObject("Side", AutoMode(SIDE));
    chooser.AddObject("Center", AutoMode(CENTER));
    SmartDashboard::PutData("Chooser", &chooser);
}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {
    // Reset Timer
    autonTimer.Reset();
    autonTimer.Start();
}

void Robot::AutonomousPeriodic() {

    //useful values
    // double rangeMeasurement = sonic_sensor.GetRangeMM();
    // int distance_from_center = 1778;
    // int random_threshold = 0;
    double rawGyroReading = gyro.GetAngle();
    //double gyroCorrectedTurn = - rawGyroReading / 90;
    double gyroCorrectedTurn = 0;
    double autonSpeed = 0.25;
    double maxTime = 4.5;
    double time = autonTimer.Get();

    //each case for auton starting position
    AutoMode mode = chooser.GetSelected();
    //Ultrassonic sensor auton
    // switch(mode){
    //     case NOTHING:
    //         m_drive.DriveCartesian(0, 0, 0);
    //         break;
    //     case SIDE:
    //         if (rangeMeasurement <= random_threshold || rangeMeasurement >= distance_from_center) {
    //             m_drive.DriveCartesian(0.25, 0, gyroCorrectedTurn);
    //         } 
    //         else {
    //             m_drive.DriveCartesian(0, 0, 0, 0);
    //         }
    //         break;
    //     case CENTER:
    //         if (rangeMeasurement <= random_threshold || rangeMeasurement >= distance_from_center) {
    //             m_drive.DriveCartesian(0.25, 0, gyroCorrectedTurn);
    //         } 
    //         else {
    //             m_drive.DriveCartesian(0, 0, 0, 0);
    //         }
    //         break;
    //     default:
    //         m_drive.DriveCartesian(0, 0, 0);

    //Timer Auton
    switch(mode){
    case NOTHING:
        m_drive.DriveCartesian(0, 0, 0);
        break;
    case SIDE:
        if (time < maxTime) {
            m_drive.DriveCartesian(autonSpeed, 0, gyroCorrectedTurn);
        } 
        else {
            m_drive.DriveCartesian(0, 0, 0, 0);
        }
        break;
    case CENTER:
        if (time < maxTime+2) {
            m_drive.DriveCartesian(autonSpeed, 0, gyroCorrectedTurn);
        } 
        else {
            m_drive.DriveCartesian(0, 0, 0, 0);
        }
        break;
    default:
        m_drive.DriveCartesian(0, 0, 0);


    // SmartDashboard::PutNumber("sonicRange (mm)", rangeMeasurement);
    SmartDashboard::PutNumber("getAngleAuto (raw)",rawGyroReading);
    SmartDashboard::PutNumber("getAngleAuto (corrected)",gyroCorrectedTurn);

    }
}    


void Robot::TeleopInit() {}

//preventing speedy acceleration
double prev_y;
double prev_x;

void Robot::TeleopPeriodic() {
    double rawGyroReading = gyro.GetAngle();
    double gyroCorrectedTurn = - rawGyroReading / 90;

    // using a ternary operator to set rawX/Y to 0 if value is below minimum threshold
    double rawX = fabs(pilot.GetX(LEFT)) < DEADZONE_THRESHOLD ? 0 : pilot.GetX(LEFT);
    double rawY = fabs(pilot.GetY(LEFT)) < DEADZONE_THRESHOLD ? 0 : pilot.GetY(LEFT);
    double turn = fabs(pilot.GetX(RIGHT)) < DEADZONE_THRESHOLD ? 0 : pilot.GetX(RIGHT);

    //stop gyro correct when turning
    // if (turn != 0){
    //     m_drive.DriveCartesian(rawY, rawX, turn);
    // } else{
    //     m_drive.DriveCartesian(rawY, rawX, gyroCorrectedTurn);
    // }
    m_drive.DriveCartesian(-rawY, rawX, turn);

    double shoot_threshold = 0.3;
    bool intake_trigger_pressed = copilot.GetTriggerAxis(LEFT)>shoot_threshold;
    bool shooting_trigger_pressed = copilot.GetTriggerAxis(RIGHT)>shoot_threshold;
    double winch_control_threshold = 0.1;
    // Control Intake & Outtake modes      
    if(!(intake_trigger_pressed && shooting_trigger_pressed)) {
        if (shooting_trigger_pressed){
            shooter.Flywheel(shooting_trigger_pressed);        
        } else {
            shooter.Intake(intake_trigger_pressed); 
        }
    }

    //shoots when trigger is pressed 
    shooter.Shoot(shooting_trigger_pressed);

    // Debug
    SmartDashboard::PutBoolean("Shooting Trigger", shooting_trigger_pressed);
    SmartDashboard::PutBoolean("Intake Trigger", intake_trigger_pressed);      
    
    //Control Winch Moving Up/Down
    double joystick_value = copilot.GetY(LEFT);
    if (joystick_value > winch_control_threshold || joystick_value < -(winch_control_threshold)) {
        shooter.Angle(-joystick_value);
        // Debug
        SmartDashboard::PutNumber("Winch Joystick (Neg)", -joystick_value);
    } else {
        shooter.Angle(0);
    }

    // Debug
    SmartDashboard::PutNumber("getAngle",gyro.GetAngle());
    SmartDashboard::PutData("Gyro",&gyro);
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
START_ROBOT_CLASS(Robot)
#endif
