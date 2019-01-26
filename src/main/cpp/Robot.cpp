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
#include <frc/SmartDashboard/SmartDashboard.h>
#include <frc/drive/DifferentialDrive.h>
#include "rev/CANSparkMax.h"
#include <frc/PWMVictorSPX.h>
#include <frc/Solenoid.h>
#include <frc/DigitalInput.h>
#include <frc/DigitalSource.h>
#include "Constants.h"
#include <frc/Timer.h>

using namespace frc;
using namespace std;
using namespace rev;

  static const int leftLeadDeviceID = 1, rightLeadDeviceID = 2, leftFollowDeviceID = 3, rightFollowDeviceID = 4;
  CANSparkMax m_leftLeadMotor{leftLeadDeviceID, CANSparkMax::MotorType::kBrushless};
  CANSparkMax m_rightLeadMotor{rightLeadDeviceID, CANSparkMax::MotorType::kBrushless};
  CANSparkMax m_leftFollowMotor{leftFollowDeviceID, CANSparkMax::MotorType::kBrushless};
  CANSparkMax m_rightFollowMotor{rightFollowDeviceID, CANSparkMax::MotorType::kBrushless};
  static const int ShoulderID = 1, WristID = 2;
  string _sb;
  int _loops = 0;
  bool _lastButton1 = false;
  bool _lastButton2 = true;

  double targetPositionRotations;
  double targetPositionRotationsW = 0;

void Robot::RobotInit() {
  Shoulder = new WPI_TalonSRX(ShoulderID);
  Wrist = new WPI_TalonSRX(WristID);

  int absolutePosition = Shoulder->GetSelectedSensorPosition(0) & 0xFFF;
  Shoulder->SetSelectedSensorPosition(absolutePosition, kPIDLoopIdx,
		kTimeoutMs);
  Shoulder->ConfigSelectedFeedbackSensor(
	  FeedbackDevice::CTRE_MagEncoder_Relative, kPIDLoopIdx,
	  kTimeoutMs);

   Shoulder->ConfigNominalOutputForward(0, kTimeoutMs);
		Shoulder->ConfigNominalOutputReverse(0, kTimeoutMs);
		Shoulder->ConfigPeakOutputForward(1, kTimeoutMs);
		Shoulder->ConfigPeakOutputReverse(-1, kTimeoutMs); 

  //These are the PID values
   Shoulder->Config_kF(kPIDLoopIdx, 0.0, kTimeoutMs);
		Shoulder->Config_kP(kPIDLoopIdx, 0.1, kTimeoutMs);
		Shoulder->Config_kI(kPIDLoopIdx, 0.0, kTimeoutMs);
		Shoulder->Config_kD(kPIDLoopIdx, 0.0, kTimeoutMs);

  int absolutePositionW = Wrist->GetSelectedSensorPosition(0) & 0xFFF;
  Wrist->SetSelectedSensorPosition(absolutePositionW, kPIDLoopIdx,
		kTimeoutMs);
  Wrist->ConfigSelectedFeedbackSensor(
	  FeedbackDevice::CTRE_MagEncoder_Relative, kPIDLoopIdx,
	  kTimeoutMs);

   Wrist->ConfigNominalOutputForward(0, kTimeoutMs);
		Wrist->ConfigNominalOutputReverse(0, kTimeoutMs);
		Wrist->ConfigPeakOutputForward(1, kTimeoutMs);
		Wrist->ConfigPeakOutputReverse(-1, kTimeoutMs); 

  //These are the PID values
   Wrist->Config_kF(kPIDLoopIdx, 0.0, kTimeoutMs);
		Wrist->Config_kP(kPIDLoopIdx, 0.1, kTimeoutMs);
		Wrist->Config_kI(kPIDLoopIdx, 0.0, kTimeoutMs);
		Wrist->Config_kD(kPIDLoopIdx, 0.0, kTimeoutMs);


  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  SmartDashboard::PutData("Auto Modes", &m_chooser);
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
  cout << "Auto selected: " << m_autoSelected << endl;

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
Joystick m_stick{3};
DifferentialDrive m_robotDrive{m_leftLeadMotor, m_rightLeadMotor};
void Robot::TeleopInit() {}

double speedShoulder = 10;

void Robot::TeleopPeriodic() {

  double motorOutput = Shoulder->GetMotorOutputPercent();

  double motorOutputW = Wrist->GetMotorOutputPercent();

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

    /*if(buttonValueOne == true){
      speedShoulder = speedShoulder;
    }

    else if(buttonValueThree == true){
      speedShoulder = -speedShoulder;
    }*/

    _sb.append("\tout:");
		_sb.append(to_string(motorOutput));
		_sb.append("\tpos:");
		_sb.append(to_string(Shoulder->GetSelectedSensorPosition(kPIDLoopIdx)));

    _sb.append("\tout:");
		_sb.append(to_string(motorOutputW));
		_sb.append("\tpos:");
		_sb.append(to_string(Wrist->GetSelectedSensorPosition(kPIDLoopIdx)));  

    if (buttonValueOne && !_lastButton1) {
			/* Position mode - button just pressed */
      Timer(push);
			targetPositionRotations = 10.0 * 4096; /* 10 Rotations in either direction */
		}

    else if(!_lastButton2 && buttonValueTwo){
      targetPositionRotations = 0 *4096;
    }

    if (Shoulder->GetControlMode() == ControlMode::Position) {
			/* append more signals to print when in speed mode. */
			_sb.append("\terrNative:");
			_sb.append(to_string(Shoulder->GetClosedLoopError(kPIDLoopIdx)));
			_sb.append("\ttrg:");
			_sb.append(to_string(targetPositionRotations));
    }

    if (Wrist->GetControlMode() == ControlMode::Position) {
			/* append more signals to print when in speed mode. */
			_sb.append("\terrNative:");
			_sb.append(to_string(Wrist->GetClosedLoopError(kPIDLoopIdx)));
			_sb.append("\ttrg:");
			_sb.append(to_string(targetPositionRotationsW));
    }



    Shoulder->Set(ControlMode::Position, targetPositionRotations);
    Wrist->Set(ControlMode::Position, targetPositionRotationsW);
    
    m_robotDrive.ArcadeDrive(-m_stick.GetY(), -m_stick.GetX());

    if (++_loops >= 10) {
			_loops = 0;
			printf("%s\n", _sb.c_str());
		}
		_sb.clear();
		/* save button state for on press detect */
		_lastButton1 = buttonValueOne;
    _lastButton2 = buttonValueTwo;
}

void Robot::TestPeriodic() {
}

#ifndef RUNNING_FRC_TESTS
int main() { return StartRobot<Robot>(); }
#endif
