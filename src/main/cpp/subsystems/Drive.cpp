/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/Drive.h"
#include "frc/WPILib.h"
#include "Robot.h"
#include "ctre/Phoenix.h"
#include <frc/encoder.h>
#include <iostream>
#include <frc/SmartDashboard/SmartDashboard.h>
#include <frc/drive/DifferentialDrive.h>
#include "rev/CANSparkMax.h"
#include <frc/PWMVictorSPX.h>
#include <frc/Solenoid.h>
#include <frc/DigitalInput.h>
#include <frc/DigitalSource.h>
#include "Constants.h"
#include <frc/Timer.h>

Drive::Drive() : Subsystem("Drive") {

  static const int leftLeadDeviceID = 1, rightLeadDeviceID = 2, leftFollowDeviceID = 3, rightFollowDeviceID = 4;
  CANSparkMax m_leftLeadMotor{leftLeadDeviceID, CANSparkMax::MotorType::kBrushless};
  CANSparkMax m_rightLeadMotor{rightLeadDeviceID, CANSparkMax::MotorType::kBrushless};
  CANSparkMax m_leftFollowMotor{leftFollowDeviceID, CANSparkMax::MotorType::kBrushless};
  CANSparkMax m_rightFollowMotor{rightFollowDeviceID, CANSparkMax::MotorType::kBrushless};

  Joystick m_stick{3};
  DifferentialDrive m_robotDrive{m_leftLeadMotor, m_rightLeadMotor};

}

void Drive::InitDefaultCommand() {
  // Set the default command for a subsystem here.
  // SetDefaultCommand(new MySpecialCommand());
}

// Put methods for controlling this subsystem
// here. Call these from Commands.
