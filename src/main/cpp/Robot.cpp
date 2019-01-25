/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

#include <frc/drive/DifferentialDrive.h>

#include "rev/CANSparkMax.h"

#include <frc/PWMVictorSPX.h>

#include <frc/Solenoid.h>

#include <DigitalInput.h>

#include <DigitalSource.h>

#include "ctre/Phoenix.h"

#include <frc/Encoder.h>

#include "frc/WPILib.h"

//#include "MotorSetup.h"

VictorSPX * topRoller;
TalonSRX * hatchMotor;
TalonSRX * platformLead;
VictorSPX * platformFollower;
VictorSPX * crawlMotor;

void Robot::RobotInit() {
  topRoller = new VictorSPX(1);
  hatchMotor = new TalonSRX(2);
  platformLead = new TalonSRX(3);
  platformFollower = new VictorSPX(4);
  crawlMotor = new VictorSPX(5);

  m_set = new MotorSetup();
  m_set->SetTopRoller(topRoller);
  m_set->SetHatchMotor(hatchMotor);
  m_set->SetPlatformLead(platformLead);
  m_set->SetPlatformFollower(platformFollower);
  m_set->SetCrawlMotor(crawlMotor);

  m_set->InitializeMotors();

  m_set->pidSetupShoulder();
  m_set->pidSetupWrist();

  m_set->printValues();

  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString(
  //     "Auto Selector", kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

frc::Joystick m_stick{3};
frc::DifferentialDrive m_robotDrive{m_leftLeadMotor, m_rightLeadMotor};

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {

  double encoderPosS = m_ShoulderEncoder.GetPosition();
  double encoderVelS = m_ShoulderEncoder.GetVelocity();
  double encoderPosW = m_WristEncoder.GetPosition();
  double encoderVelW = m_WristEncoder.GetVelocity();

  buttonValueOne = m_stick.GetRawButtonPressed(1);
  buttonValueTwo = m_stick.GetRawButtonPressed(2);
  buttonValueThree = m_stick.GetRawButtonPressed(3);
  buttonValueFour = m_stick.GetRawButtonPressed(4);
  buttonValueFive = m_stick.GetRawButtonPressed(5);
  buttonValueSix = m_stick.GetRawButtonPressed(6);

  if(buttonValueOne == true&&positionS!=maxPos){
     positionS++
    }
  else if(buttonValueThree == true&&positionS!=minPos){
     positionS--;
    }
  if(positionS == 0){
    rotationS = 0.125;
  }
  else if(positionS == 1){
    rotationS = 5;
  }
  else if(positionS == 2){
    rotationS = 10;
  }

  if(buttonValueTwo == true&&positionW!=maxPos){
     positionW++
    }
  else if(buttonValueThree == true&&positionW!=minPos){
     positionW--;
    }
  if(positionW == 0){
    rotationW = 0.125;
  }
  else if(positionW == 1){
    rotationW = 5;
  }
  else if(positionW == 2){
    rotationW = 10;
  }  

  m_robotDrive.ArcadeDrive(m_stick.GetY(), m_stick.GetX());

  m_pidControllerShoulder.SetReference(rotationS, rev::ControlType::kPosition);
  m_pidControllerWrist.SetReference(rotationW, rev::ControlType::kPosition);

}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
