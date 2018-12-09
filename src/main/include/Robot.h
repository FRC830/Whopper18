/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>
#include <VictorSP.h>
#include <IterativeRobot.h>
#include <SmartDashboard/SendableChooser.h>
#include <WPILib.h>
#include "Shooter.h"
class Robot : public frc::IterativeRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
  static  const int FRONT_LEFT_WHEEL = -1;  //Same values as Robot2018 Github
  static const int FRONT_RIGHT_WHEEL = -1;
  static const int BACK_LEFT_WHEEL = -1;
  static const int BACK_RIGHT_WHEEL = -1;
  static const int GYRO_ANALOG_IN = 0;
  static constexpr double DEADZONE_THRESHOLD = .05;
  static const int ULTRASONIC_PING_PIN = 8;
  static const int ULTRASONIC_ECHO_PIN = 9;
  static const int WINCH_PIN = -1; //fix these
  static const int INTAKE_PIN = -1;
  static const int SERVO_PIN = -1;

  Ultrasonic sonic_sensor{ULTRASONIC_PING_PIN, ULTRASONIC_ECHO_PIN};

  WPI_TalonSRX frontl{FRONT_LEFT_WHEEL};
  WPI_TalonSRX frontr{FRONT_RIGHT_WHEEL};
  WPI_TalonSRX backl{BACK_LEFT_WHEEL};
  WPI_TalonSRX backr{BACK_RIGHT_WHEEL};
  MecanumDrive m_drive{frontl, frontr, backl, backr};
 
  WPI_TalonSRX winch{WINCH_PIN};
  WPI_TalonSRX intake{INTAKE_PIN};
  Servo servo{SERVO_PIN};
  Shooter shooter{intake, winch, servo}; 


  static const GenericHID::JoystickHand LEFT = GenericHID::kLeftHand;
  static const GenericHID::JoystickHand RIGHT = GenericHID::kRightHand; 

  XboxController pilot{0};
  XboxController copilot{1};
  AnalogGyro gyro{GYRO_ANALOG_IN};
  Timer autonTimer{};
  static const int TICKS_TO_ACCEL = 20;
  MecanumDrive drive{frontl, backl, frontr, backr};
 private:
  enum AutoMode {NOTHING = 0, SIDE, CENTER};
  SendableChooser<AutoMode> chooser;
};
