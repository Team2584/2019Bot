#pragma once

#include "commands/IntakeCommand.h"
#include <frc/SpeedController.h>
#include <frc/commands/Subsystem.h>

class Intake : public frc::Subsystem {
 private:
  

 public:
 Intake();
  void InitDefaultCommand() ;
  void Periodic() ;
  void Intake_control() ;

}

