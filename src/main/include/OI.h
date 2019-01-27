/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/Joystick.h>
#include <frc/WPILib.h>


class OI {
 public:
  OI();
  //// CREATING BUTTONS
	// One type of button is a joystick button which is any button on a
	//// joystick.
	// You create one by telling it which joystick it's on and which button
	// number it is.
	// Joystick stick = new Joystick(port);
	// Button button = new JoystickButton(stick, buttonNumber);

	// There are a few additional built in buttons you can use. Additionally,
	// by subclassing Button you can create custom triggers and bind those to
	// commands the same as any other Button.

	//// TRIGGERING COMMANDS WITH BUTTONS
	// Once you have a button, it's trivial to bind it to a button in one of
	// three ways:

	// Start the command when the button is pressed and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenPressed(new ExampleCommand());

	// Run the command while the button is being held down and interrupt it once
	// the button is released.
	// button.whileHeld(new ExampleCommand());

	// Start the command when the button is released and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenReleased(new ExampleCommand());

	frc::Joystick * m_stick;
  frc::JoystickButton * buttone;
  frc::JoystickButton * butttwo;
  frc::JoystickButton * buttthree;
  frc::JoystickButton * buttfour;
  



  bool buttonValueOne;
  bool buttonValuetwo;
  bool buttonValuethree;
  bool buttonValuefour;



	

	 double thresholdedX;
	 double thresholdedY;
	 double thresholdedTwist;

	 const double thresholdX = 0; // Added to make sure the drive isn't
										// too sensitive
	 const double thresholdY = 0; // As above
	 const  double thresholdTwist = 0; // As above

  static const int control = 1;
   static const int leftjoy = 1;
   static const int rightjoy = 2;
   static const int but1 = 1;
   static const int but2 = 2;
   static const int but3 = 3;
   static const int but4 = 4;
   static const int trig1 = 1;
   static const int trig2 = 2;
  
  frc::Joystick::Joystick * m_stick = new frc::Joystick::Joystick(control);


  frc::JoystickButton * shoulderup = new frc::JoystickButton(m_stick,but1);
  frc::JoystickButton * wristup = new frc::JoystickButton(m_stick,but1);
  frc::JoystickButton * shouldermid = new frc::JoystickButton(m_stick,but2);
  frc::JoystickButton * wristmid = new frc::JoystickButton(m_stick,but2);
  frc::JoystickButton * shoulderlow = new frc::JoystickButton(m_stick,but3);
  frc::JoystickButton * wristlow = new frc::JoystickButton(m_stick,but3);
  frc::Joystick::Joystick * rightstick = new frc::Joystick::Joystick(rightjoy);
  frc::Joystick::Joystick * leftstick = new frc::Joystick::Joystick(leftjoy);
  frc::AnalogTrigger * righttrig = new frc::AnalogTrigger(trig1);
  frc::AnalogTrigger * lefttrig = new frc::AnalogTrigger(trig2);


};
