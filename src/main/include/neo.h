/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include "string"
#pragma once

class neo {
 public:
 void setupMotor(std::string name, int ID){};
 void setupEncoder(std::string name, std::string motor){};
  neo();
};
