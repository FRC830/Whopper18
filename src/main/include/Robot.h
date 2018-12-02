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
class Robot : public frc::IterativeRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
  static  const int FRONT_LEFT_WHEEL = 0;  //Same values as Robot2018 Github
  static const int FRONT_RIGHT_WHEEL = 6;
  static const int BOTTOM_LEFT_WHEEL = 1;
  static const int BOTTOM_RIGHT_WHEEL = 7;
  static const int GYRO_ANALOG_IN = 0;
  static constexpr double DEADZONE_THRESHOLD = .05;
  static const int ULTRASONIC_PING_PIN = 8;
  static const int ULTRASONIC_ECHO_PIN = 9;
  Ultrasonic sonic_sensor{ULTRASONIC_PING_PIN, ULTRASONIC_ECHO_PIN};

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
  MecanumDrive drive{frontl, backl, frontr, backr};
 private:
  enum AutoMode {NOTHING = 0, SIDE, CENTER};
  SendableChooser<AutoMode> chooser;
};
