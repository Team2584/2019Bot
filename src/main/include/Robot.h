/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include <string>
#include "ctre/phoenix.h"
#include <frc/IterativeRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <PWMVictorSPX.h>
#include <frc/WPILib.h>
#include <DigitalInput.h>
#include <DigitalSource.h>

class Robot : public frc::IterativeRobot {
    frc::DigitalInput* limitSwitch;
    TalonSRX * TalonTest;
    VictorSPX * VictorTest;
 public:

  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

 private:

  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
};
