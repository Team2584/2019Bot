/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/commands/Subsystem.h>
#include <frc/Spark.h>
#include <frc/SpeedControllerGroup.h>
using namespace frc;
class Drive : public frc::Subsystem {
 private:
  // It's desirable that everything possible under private except
  // for methods that implement subsystem capabilities

 public:
  Drive();
  void InitDefaultCommand() override;
  void Periodic() override;
  void ArcadeDrive() override;
  void TankDrive() override;
  




  SpeedControllerGroup* speedController = new SpeedController();
};
