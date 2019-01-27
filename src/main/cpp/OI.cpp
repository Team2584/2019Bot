/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "OI.h"
#include <frc/Joystick.h>
#include <frc/WPILib.h>
using namespace frc;

OI::OI() {
  Joystick::Joystick *joy = new Joystick(0);

	JoystickButton * ShoulderUp = new JoystickButton(joy, 1);
	JoystickButton * WristUp = new JoystickButton(joy, 2);
	JoystickButton * ShoulderDown = new JoystickButton(joy, 3);
	JoystickButton * WristDown = new JoystickButton(joy, 4);
	JoystickButton * HatchUp = new JoystickButton(joy, 5);
	JoystickButton * RollerIn = new JoystickButton(joy, 6);
	JoystickButton * HatchDown = new JoystickButton(joy, 7);
	JoystickButton * RollerDown = new JoystickButton(joy, 8);
  // Process operator interface input here.
}
