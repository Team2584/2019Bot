/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include "ctre/Phoenix.h"

#include <frc/PWMVictorSPX.h>

#include "rev/CANSparkMax.h"

#pragma once

#ifndef SRC_MOTORSETUP_H_
#define SRC_MOTORSETUP_H_


#define leftLeadID 1;
#define leftFollowID_1 3;
#define leftFollowID_2 5;
#define rightLeadID 2;
#define rightFollowID_1 4;
#define rightFollowID_2 6;
#define shoulderID 7;
#define wristID 8;
#define hatchMotorID 1;
#define topRollerID 2;
#define platformLeadID 3;
#define platformFollowID 4;
#define crawlMotorID 5;

class MotorSetup {
  private:
  TalonSRX * hatchMotor = NULL;
  VictorSPX * topRoller = NULL;
  TalonSRX * platformLead = NULL;
  VictorSPX * platformFollower = NULL;
  VictorSPX * crawlMotor = NULL;

 public:
  double kPs = 0.1001, kIs = 0.00001, kDs = 0.5, kIzs = 0, kFFs = 0, kMaxOutputs = 1, kMinOutputs = -1;
  double kPw = 0.1001, kIw = 0.00001, kDw = 0.5, kIzw = 0, kFFw = 0, kMaxOutputw = 1, kMinOutputw = -1;
  MotorSetup();
  double rotationS = 0;
  double rotationW = 0;
  void SetHatchMotor(TalonSRX *);
  void SetTopRoller(VictorSPX *);
  void SetPlatformLead(TalonSRX *);
  void SetPlatformFollower(VictorSPX *);
  void SetCrawlMotor(VictorSPX *);

  void InitializeMotors();

  void pidSetupShoulder();
  void pidSetupWrist();

  void printValues();

  void pidCorrectShoulder();
  void pidCorrectWrist();



  double maxPos = 2;
  double minPos = 0;
  double positionS = 0;
  double positionW = 0;

  bool buttonValueOne;
  bool buttonValueTwo;
  bool buttonValueThree;
  bool buttonValueFour;
  bool buttonValueFive;
  bool buttonValueSix;
};

#endif
