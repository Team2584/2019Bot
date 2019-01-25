/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "frc/WPILib.h"
#include "Robot.h"
#include "ctre/Phoenix.h"
#include <frc/encoder.h>
#include <iostream>
//#include <rev/SparkMax.h>
#include <frc/SmartDashboard/SmartDashboard.h>
#include <frc/drive/DifferentialDrive.h>
#include "rev/CANSparkMax.h"
#include <PWMVictorSPX.h>
#include <frc/Solenoid.h>
#include <DigitalInput.h>
#include <DigitalSource.h>

  static const int leftLeadDeviceID = 1, rightLeadDeviceID = 2, leftFollowDeviceID1 = 3, rightFollowDeviceID1 = 4, leftFollowDeviceID2 = 5, rightFollowDeviceID2 = 6;
  rev::CANSparkMax m_leftLeadMotor{leftLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightLeadMotor{rightLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_leftFollowMotor1{leftFollowDeviceID1, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightFollowMotor1{rightFollowDeviceID1, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_leftFollowMotor2{leftFollowDeviceID2, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightFollowMotor2{rightFollowDeviceID2, rev::CANSparkMax::MotorType::kBrushless};  

  static const int shoulderID = 7, wristID = 8;
  rev::CANSparkMax m_shoulder{shoulderID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_wrist{wristID, rev::CANSparkMax::MotorType::kBrushed};
  //rev::CANSparkMax m_motorFollower{motorFollowerDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANPIDController m_pidShoulder = m_shoulder.GetPIDController();
  rev::CANPIDController m_pidWrist = m_wrist.GetPIDController();
  rev::CANEncoder m_encoder = m_motor.GetEncoder();

  double shoulderPos = m_shoulder.GetPosition();
  double shoulderVel = m_shoulder.GetVelocity();

  double wristPos = m_wrist.GetPosition();
  double wristVel = m_wrist.GetVelocity();

void Robot::RobotInit() {
  topRoller = new VictorSPX(1);
  hatchMotor = new TalonSRX(2);
  platformLead = new TalonSRX(3);
  platformFollower = new VictorSPX(4);
  crawlMotor = new VictorSPX(5);

  m_leftFollowMotor1.Follow(m_leftLeadMotor);  
  m_leftFollowMotor2.Follow(m_leftLeadMotor);
  m_rightFollowMotor1.Follow(m_rightLeadMotor);
  m_rightFollowMotor2.Follow(m_rightLeadMotor);

  m_pidShoulder.SetP(kP);
  m_pidShoulder.SetI(kI);
  m_pidShoulder.SetD(kD);
  m_pidShoulder.SetIZone(kIz);
  m_pidShoulder.SetFF(kFF);
  m_pidShoulder.SetOutputRange(kMinOutput, kMaxOutput);

  m_pidWrist.SetP(kP);
  m_pidWrist.SetI(kI);
  m_pidWrist.SetD(kD);
  m_pidWrist.SetIZone(kIz);
  m_pidWrist.SetFF(kFF);
  m_pidWrist.SetOutputRange(kMinOutput, kMaxOutput);

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

void Robot::TeleopInit() {}

double positionS = 0;
double positionW = 0;
double positionH = 0;
double maxPosSW = 2;
double minPosSW = 0;
double maxPosPH = 1;
double minPosPH = 0;

void Robot::TeleopPeriodic() {
    double speedTopRoller;
    double speedV;
    double rotations;
    double rotationsH;
    //double rotations = frc::SmartDashboard::GetNumber("SetPoint", rotations);
    bool buttonValueOne;
    buttonValueOne = m_stick.GetRawButtonPressed(1);
    bool buttonValueTwo;
    buttonValueTwo = m_stick.GetRawButtonPressed(2);
    bool buttonValueThree;
    buttonValueThree = m_stick.GetRawButtonPressed(3);
    bool buttonValueFour;
    buttonValueFour = m_stick.GetRawButtonPressed(4);
    bool buttonValueFive;
    buttonValueFour = m_stick.GetRawButtonPressed(5);
    if(buttonValueOne == true&&positionS!=maxPos){
     positionS++;
    }
    else if(buttonValueThree == true&&positionS!=minPos){
      positionS--;
    }
    if(buttonValueTwo == true&&positionH!=maxPosPH){
      positionH++;
    }  
    else if(buttonValueFour == true&&positionH!=minPosPH){
      positionH--;
    } 
    if(positionS == 0){
      rotations = 0.125;
    }
    else if(positionS == 1){
      rotations = 5;
    }
    else if(positionS == 2){
      rotations = 10;
    }
    if(positionH == 0){
      rotationsH = 0;
    }
    else if(positionH == 1){
      rotationsH = 0.5;
    }

    m_pidShoulder.SetReference(rotations, rev::ControlType::kPosition);
    m_pidWrist.SetReference(rotations, rev::ControlType::kPosition);
    topRoller->Set(ControlMode::PercentOutput, speedTopRoller);
    hatchMotor->Set(ControlMode::Position, rotationsH);
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
