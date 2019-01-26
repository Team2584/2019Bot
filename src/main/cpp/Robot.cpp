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
#include <frc/PWMVictorSPX.h>
#include <frc/Solenoid.h>
#include <frc/DigitalInput.h>
#include <frc/DigitalSource.h>
#include "Constants.h"


  static const int leftLeadDeviceID = 1, rightLeadDeviceID = 2, leftFollowDeviceID = 3, rightFollowDeviceID = 4;
  rev::CANSparkMax m_leftLeadMotor{leftLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightLeadMotor{rightLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_leftFollowMotor{leftFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightFollowMotor{rightFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  static const int ShoulderID = 1;
  std::string _sb;
  int _loops = 0;
  bool _lastButton1 = false;
  bool _lastButton2 = true;

  double targetPositionRotations;

void Robot::RobotInit() {
  Shoulder = new WPI_TalonSRX(ShoulderID);

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

   Shoulder->Config_kF(kPIDLoopIdx, 0.0, kTimeoutMs);
		Shoulder->Config_kP(kPIDLoopIdx, 0.1, kTimeoutMs);
		Shoulder->Config_kI(kPIDLoopIdx, 0.0, kTimeoutMs);
		Shoulder->Config_kD(kPIDLoopIdx, 0.0, kTimeoutMs);


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

double speedShoulder = 10;

void Robot::TeleopPeriodic() {

  double motorOutput = Shoulder->GetMotorOutputPercent();

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

    if(buttonValueOne == true){
      speedShoulder = speedShoulder;
    }

    else if(buttonValueThree == true){
      speedShoulder = -speedShoulder;
    }

    _sb.append("\tout:");
		_sb.append(std::to_string(motorOutput));
		_sb.append("\tpos:");
		_sb.append(std::to_string(Shoulder->GetSelectedSensorPosition(kPIDLoopIdx)));

    if (!_lastButton1 && buttonValueTwo) {
			/* Position mode - button just pressed */
			targetPositionRotations = 10.0 * 4096; /* 10 Rotations in either direction */
		}

    else if(!_lastButton2 && buttonValueFour){
      targetPositionRotations = 0 *4096;
    }

    if (Shoulder->GetControlMode() == ControlMode::Position) {
			/* append more signals to print when in speed mode. */
			_sb.append("\terrNative:");
			_sb.append(std::to_string(Shoulder->GetClosedLoopError(kPIDLoopIdx)));
			_sb.append("\ttrg:");
			_sb.append(std::to_string(targetPositionRotations));
    }



    Shoulder->Set(ControlMode::Position, targetPositionRotations);
    
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
int main() { return frc::StartRobot<Robot>(); }
#endif
