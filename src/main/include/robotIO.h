/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include "frc/WPILib.h"

#pragma once

class robotIO {
 public:
  robotIO();

  void joystickInit();

  void setButtonOne();
  void setButtonTwo();
  void setButtonThree();
  void setButtonFour();
  void setButtonFive();
  void setButtonSix();
  void setButtonSeven();
  void setButtonEight();
  void drive();

  bool getButtonOne();
  bool getButtonTwo();
  bool getButtonThree();
  bool getButtonFour();
  bool getButtonFive();
  bool getButtonSix();
  bool getButtonSeven();
  bool getButtonEight();
  double getY();
  double getX();
  double getAxisFour();
  double getAxisFive();

  bool getButtonOnePressed();
  bool getButtonFourPressed();

  bool getButtonOnePartner();
  bool getButtonTwoPartner();
  bool getButtonThreePartner();
  bool getButtonFourPartner();
  bool getButtonFivePartner();
  bool getButtonSixPartner();
  bool getButtonSevenPartner();
  bool getButtonEightPartner();
  double getYPartner();
  double getAxisFivePartner();
  double getAxisFourPartner();
  
};
