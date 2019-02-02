/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include "ctre/Phoenix.h"
#include <iostream>
#include <frc/encoder.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DigitalInput.h>
#include <frc/DigitalSource.h>
#include "Constants.h"
#include "frc/WPILib.h"
#include <memory>
#include <chrono>
#include <thread>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

using namespace frc;
using namespace std;

static const int ShoulderID = 1, WristID = 5;
string _sb;
int _loops = 0;

double targetPositionRotationsS = 0;
double targetPositionRotationsW = 0;

double absolutePositionS = 0;
double absolutePositionW = 0;

static void VisionThread()
    {
        cs::UsbCamera camera = CameraServer::GetInstance()->StartAutomaticCapture();
        camera.SetResolution(280, 200);
    }

void Robot::RobotInit() {
  //CameraServer::GetInstance()->StartAutomaticCapture();
  std::thread visionThread(VisionThread);
  visionThread.detach();
  /*cs::UsbCamera camera = CameraServer::GetInstance()->StartAutomaticCapture();
  camera.SetResolution(640, 480);*/
  const bool kInvert = false;
  const bool kSensorPhase = false;
  Shoulder = new WPI_TalonSRX(ShoulderID);
  Wrist = new WPI_TalonSRX(WristID);
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  Shoulder->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, kTimeoutMs);
  Shoulder->SetStatusFramePeriod(StatusFrame::Status_1_General_, 5, kTimeoutMs);
  Shoulder->SetSensorPhase(kSensorPhase);
  Shoulder->SetInverted(kInvert);
  SmartDashboard::PutData("Auto Modes", &m_chooser);

  absolutePositionS = Shoulder->GetSelectedSensorPosition(0) & 0xFFF;
  Shoulder->SetSelectedSensorPosition(absolutePositionS, kPIDLoopIdx,
		kTimeoutMs);
  Shoulder->ConfigSelectedFeedbackSensor(
	  FeedbackDevice::CTRE_MagEncoder_Relative, kPIDLoopIdx,
	  kTimeoutMs);

    Shoulder->ConfigNominalOutputForward(0, kTimeoutMs);
	  Shoulder->ConfigNominalOutputReverse(0, kTimeoutMs);
  	Shoulder->ConfigPeakOutputForward(0.5, kTimeoutMs);
		Shoulder->ConfigPeakOutputReverse(-0.5, kTimeoutMs); 

    Shoulder->Config_kF(kPIDLoopIdx, 0.0, kTimeoutMs);
		Shoulder->Config_kP(kPIDLoopIdx, 0.75, kTimeoutMs);
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
		Wrist->ConfigPeakOutputForward(0.5, kTimeoutMs);
		Wrist->ConfigPeakOutputReverse(-0.5, kTimeoutMs); 

   Wrist->Config_kF(kPIDLoopIdx, 0.0, kTimeoutMs);
		Wrist->Config_kP(kPIDLoopIdx, 0.1, kTimeoutMs);
		Wrist->Config_kI(kPIDLoopIdx, 0.0, kTimeoutMs);
		Wrist->Config_kD(kPIDLoopIdx, 0.0, kTimeoutMs);
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

void Robot::TeleopInit() {}

double lastButtonPressed4 = false;
double lastButtonPressed1 = true;
double lastButtonPressed2 = false;
double lastButtonPressed3 = true;

void Robot::TeleopPeriodic() {

  bool buttonValueOne;
    buttonValueOne = m_stick.GetRawButtonPressed(1);

  bool buttonValueTwo;
    buttonValueTwo = m_stick.GetRawButtonPressed(2);

  bool buttonValueThree;
    buttonValueThree = m_stick.GetRawButtonPressed(3);

  bool buttonValueFour;
    buttonValueFour = m_stick.GetRawButtonPressed(4);

  bool buttonValueFive;
    buttonValueFive = m_stick.GetRawButtonPressed(5);

    SmartDashboard::PutNumber("TargetS", targetPositionRotationsS);
    SmartDashboard::PutNumber("TargetW", targetPositionRotationsW);
    SmartDashboard::PutNumber("Shoulder Encoder", Shoulder->GetSelectedSensorPosition());

  if (buttonValueFour /*&& !lastButtonPressed4*/) {
			/* Position mode - button just pressed */
			targetPositionRotationsS = 25.0 * 4096; /* 10 Rotations in either direction */
		}

  else if(/*!lastButtonPressed1 &&*/ buttonValueOne){
      targetPositionRotationsS = 7.5 *4096;
    }
  
  else if(buttonValueFive){
    targetPositionRotationsS = 85 * 4096;
    targetPositionRotationsW = 45 * 4096;
  }

  if (buttonValueTwo /*&& !lastButtonPressed2*/) {
			/* Position mode - button just pressed */
			targetPositionRotationsW = 180.0 * 4096; /* 10 Rotations in either direction */
		}

  else if(/*!lastButtonPressed3 &&*/ buttonValueThree){
      targetPositionRotationsW = 0 *4096;
    }

    Wrist->Set(ControlMode::Position, targetPositionRotationsW);
    Shoulder->Set(ControlMode::Position, targetPositionRotationsS);

    double lastButtonPressed4 = buttonValueFour;
    double lastButtonPressed1 = buttonValueOne;
    double lastButtonPressed2 = buttonValueTwo;
    double lastButtonPressed3 = buttonValueThree;

}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return StartRobot<Robot>(); }
#endif
