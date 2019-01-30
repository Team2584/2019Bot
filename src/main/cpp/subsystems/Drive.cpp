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
#include <rev/CANPIDController.h>
#include <frc/PWMVictorSPX.h>
#include <frc/Solenoid.h>
#include <frc/DigitalInput.h>
#include <frc/DigitalSource.h>
#include <frc/Timer.h>
#include "RobotMap.h"
#include "subsystems/Drive.h"
#include "OI.h"
Drive::Drive() : Subsystem("Drive") {

  CANSparkMax m_leftLeadMotor{leftLeadDeviceID, CANSparkMax::MotorType::kBrushless};
  CANSparkMax m_rightLeadMotor{rightLeadDeviceID, CANSparkMax::MotorType::kBrushless};
  CANSparkMax m_leftFollowMotor{leftFollowDeviceID, CANSparkMax::MotorType::kBrushless};
  CANSparkMax m_rightFollowMotor{rightFollowDeviceID, CANSparkMax::MotorType::kBrushless};

  m_leftFollowMotor.Follow(m_leftLeadMotor);
  m_rightFollowMotor.Follow(m_rightLeadMotor);

  Joystick m_stick{3};
  DifferentialDrive m_robotDrive{m_leftLeadMotor, m_rightLeadMotor};

  
}

void Drive::InitDefaultCommand() {
  // Set the default command for a subsystem here.
  // SetDefaultCommand(new MySpecialCommand());
}

void Drive::Periodic(){
    ArcadeDrive();
    //TankDrive():
    //CurvatureDrive();
}

void Drive::ArcadeDrive(){
  m_robotDrive.ArcadeDrive(-m_stick.GetX(), -m_stick.GetY();)
}

void Drive::TankDrive(){
  m_robotDrive.TankDrive(-m_stick.GetRawAxis(1), m_stick.GetRawAxis(4));
}

void Drive::CurvatureDrive(){
  m_robotDrive.CurvatureDrive(-m_stick.GetRawAxis(1), m_stick.GetRawAxis(5), true);
}



// Put methods for controlling this subsystem
// here. Call these from Commands.
