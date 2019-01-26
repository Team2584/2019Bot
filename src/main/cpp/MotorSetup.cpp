/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "MotorSetup.h"

#include "rev/CANSparkMax.h"

#include <frc/PWMVictorSPX.h>

#include "ctre/Phoenix.h"

MotorSetup::MotorSetup() {}

void MotorSetup::SetHatchMotor(TalonSRX * hatchMotor){
    this->hatchMotor = hatchMotor;
}

void MotorSetup::SetCrawlMotor(VictorSPX * crawlMotor){
    this->crawlMotor = crawlMotor;
}

void MotorSetup::SetPlatformLead(TalonSRX * platformLead){
    this->platformLead = platformLead;
}

void MotorSetup::SetPlatformFollower(VictorSPX * platformFollower){
    this->platformFollower = platformFollower;
}

void MotorSetup::SetTopRoller(VictorSPX * topRoller){
    this->topRoller = topRoller;
}

void MotorSetup::InitializeMotors(){
    rev::CANSparkMax m_leftLeadMotor{leftLeadID, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax m_rightLeadMotor{rightLeadID, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax m_leftFollowMotor1{leftFollowID_1, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax m_rightFollowMotor1{rightFollowID_1, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax m_leftFollowMotor2{leftFollowID_2, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax m_rightFollowMotor2{rightFollowID_2, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax m_shoulderMotor{shoulderID, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax m_wristMotor{wristID, rev::CANSparkMax::MotorType::kBrushed};

    rev::CANPIDController m_pidControllerShoulder = m_shoulderMotor.GetPIDController();
    rev::CANPIDController m_pidControllerWrist = m_wristMotor.GetPIDController();
    rev::CANEncoder m_shoulderEncoder = m_shoulderMotor.GetEncoder();
    rev::CANEncoder m_wristEncoder = m_wristMotor.GetEncoder();

    m_leftFollowMotor1.Follow(m_leftLeadMotor);
    m_leftFollowMotor2.Follow(m_leftLeadMotor);
    m_rightFollowMotor1.Follow(m_rightLeadMotor);
    m_rightFollowMotor2.Follow(m_rightLeadMotor);            
}

void MotorSetup::pidSetupShoulder(){
    m_pidControllerShoulder.SetP(kPs);
    m_pidControllerShoulder.SetI(kIs);
    m_pidControllerShoulder.SetD(kDs);
    m_pidControllerShoulder.SetIZone(kIzs);
    m_pidControllerShoulder.SetFF(kFFs);
    m_pidControllerShoulder.SetOutputRange(kMinOutputs, kMaxOutputs);    
}

void MotorSetup::pidSetupWrist(){
    m_pidControllerWrist.SetP(kPw);
    m_pidControllerWrist.SetI(kIw);
    m_pidControllerWrist.SetD(kDw);
    m_pidControllerWrist.SetIZone(kIzw);
    m_pidControllerWrist.SetFF(kFFw);
    m_pidControllerWrist.SetOutputRange(kMinOutputw, kMaxOutputw);
}

void MotorSetup::printValues(){
    frc::SmartDashboard::PutNumber("Shoulder Encoder Position", encoderPosS);
    frc::SmartDashboard::PutNumber("Shoulder Encoder Velocity", encoderVelS);
    frc::SmartDashboard::PutNumber("Wrist Encoder Position", encoderPosW);
    frc::SmartDashboard::PutNumber("Wrist Encoder Velocity", encoderVelW);
}





