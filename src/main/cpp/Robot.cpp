/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include "ctre/Phoenix.h"
#include "rev/CANSparkMax.h"
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
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"

using namespace frc;
using namespace std;

static const int ShoulderID = 1, WristID = 5, LeadID = 3, FollowID = 4, HatchID = 7, RollerID = 6, CrawlID = 2;
string _sb;
int _loops = 0;

static const int leftLeadDeviceID = 3, rightLeadDeviceID = 1, leftFollowDeviceID = 4, rightFollowDeviceID = 2;
  rev::CANSparkMax m_leftLeadMotor{leftLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightLeadMotor{rightLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_leftFollowMotor{leftFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightFollowMotor{rightFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless};

double absolutePositionS = 0;
double absolutePositionW = 0;



  double kPgain = 0.04; /* percent throttle per degree of error */
	double kDgain = 0.0004; /* percent throttle per angular velocity dps */
	double kMaxCorrectionRatio = 0.30; /* cap corrective turning throttle to 30 percent of forward throttle */
	/** holds the current angle to servo to */
	double _targetAngle = 0;
	/** count loops to print every second or so */
	int _printLoops = 0;

static void VisionThread()
    {
        cs::UsbCamera camera = CameraServer::GetInstance()->StartAutomaticCapture();
        camera.SetResolution(280, 200);
    }

void Robot::RobotInit() {
  std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
  double targetOffsetAngle_Horizontal = table->GetNumber("tx",0.0);
  double targetOffsetAngle_Vertical = table->GetNumber("ty",0.0);
  double targetArea = table->GetNumber("ta",0.0);
  double targetSkew = table->GetNumber("ts",0.0);
  nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("pipeline",0);
  //CameraServer::GetInstance()->StartAutomaticCapture();
  std::thread visionThread(VisionThread);
  visionThread.detach();
  /*cs::UsbCamera camera = CameraServer::GetInstance()->StartAutomaticCapture();
  camera.SetResolution(640, 480);*/
  const bool kInvert = false;
  const bool kSensorPhase = false;

  m_leftFollowMotor.Follow(m_leftLeadMotor);
  m_rightFollowMotor.Follow(m_rightLeadMotor);

  Counter *hatchEncoder = new Counter(0);
  Shoulder = new WPI_TalonSRX(ShoulderID);
  Wrist = new WPI_TalonSRX(WristID);
  Hatch = new WPI_TalonSRX(HatchID);
  ClimbLead = new WPI_TalonSRX(LeadID);
  ClimbFollow = new WPI_VictorSPX(FollowID);
  Roller = new WPI_VictorSPX(RollerID);
  ClimbFollow->Follow(*ClimbLead);
  Crawl = new WPI_VictorSPX(CrawlID);


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
		Shoulder->Config_kP(kPIDLoopIdx, 1, kTimeoutMs);
		Shoulder->Config_kI(kPIDLoopIdx, 0.0001, kTimeoutMs);
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
		Wrist->Config_kP(kPIDLoopIdx, 0.02, kTimeoutMs);
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

Joystick * m_stick = new Joystick(3);
Joystick * m_partner = new Joystick(4);
frc::DifferentialDrive m_robotDrive{m_leftLeadMotor, m_rightLeadMotor};


void Robot::TeleopInit() {}

double lastButtonPressed4 = false;
double lastButtonPressed1 = true;
double lastButtonPressed2 = false;
double lastButtonPressed3 = true;

double climbSpeed = 0;
double hatchSpeed = 0;
double rollerSpeed = 0;
double crawlSpeed = 0;

double pos;

double targetPositionRotationsS = 0;
double targetPositionRotationsW = 0;

double targetVelocity_UnitsPer100ms;

double wristSpeed;

void Robot::TeleopPeriodic() {

  bool buttonValueOne;
    buttonValueOne = m_stick->GetRawButtonPressed(1);

  bool buttonValueTwo;
    buttonValueTwo = m_stick->GetRawButtonPressed(2);

  bool buttonValueThree;
    buttonValueThree = m_stick->GetRawButtonPressed(3);

  bool buttonValueFour;
    buttonValueFour = m_stick->GetRawButtonPressed(4);

  bool buttonValueFive;
    buttonValueFive = m_stick->GetRawButton(5);

  bool buttonValueSix;
    buttonValueSix = m_stick->GetRawButton(6);

  bool buttonValueSeven;
    buttonValueSeven = m_stick->GetRawButton(7);
    
  bool buttonValueEight;
    buttonValueEight = m_stick->GetRawButton(8);

  bool buttonValueOneP;
    buttonValueOne = m_partner->GetRawButtonPressed(1);

  bool buttonValueFiveP;
    buttonValueFour = m_partner->GetRawButtonPressed(5);

  bool buttonValueSixP;
    buttonValueTwo = m_partner->GetRawButtonPressed(6);

  bool buttonValueThreeP;
    buttonValueThree = m_partner->GetRawButtonPressed(3);

  bool buttonValueSevenP;
    buttonValueSevenP = m_partner->GetRawButton(7);

  bool buttonValueEightP;
    buttonValueEightP = m_partner->GetRawButton(8);

    int hatchPos = hatchEncoder->Get();

    SmartDashboard::PutNumber("TargetS", targetPositionRotationsS);
    SmartDashboard::PutNumber("TargetW", targetPositionRotationsW);
    SmartDashboard::PutNumber("Shoulder Encoder", Shoulder->GetSelectedSensorPosition());
    //SmartDashboard::PutNumber("Hatch Encoder", hatchEncoder->Get());

  if (buttonValueOne /*&& !lastButtonPressed4*/) {
			/* Position mode - button just pressed */
			targetPositionRotationsS = 12.5 * 4096; /* 10 Rotations in either direction */
		}
  else if(/*!lastButtonPressed1 &&*/ buttonValueFour){
      targetPositionRotationsS = 3.5 *4096;
    }
  /*if(buttonValueFive){
    targetPositionRotationsS = 85 * 4096;
    targetPositionRotationsW = 45 * 4096;
  }*/
  if(buttonValueSevenP){
    pos++;
    targetPositionRotationsW = pos * 4096;
    
    Wrist->Set(ControlMode::Position, targetPositionRotationsW);
  }
  else if(buttonValueEightP){
    pos--;
    targetPositionRotationsW = pos * 4096;    
    Wrist->Set(ControlMode::Position, targetPositionRotationsW);
  }
  else{
    Wrist->Set(ControlMode::Position, targetPositionRotationsW);
  }

  /*if (buttonValueTwo && !lastButtonPressed2) {
			//Position mode - button just pressed 
			targetPositionRotationsW = 27.0 * 4096; // 10 Rotations in either direction 
		}
  else if(!lastButtonPressed3 && buttonValueThree){
      targetPositionRotationsW = 0 *4096;
    }
  else if(buttonValueFive){
    double wristSpeed = -0.2;
    Wrist->Set(ControlMode::PercentOutput, wristSpeed);
  }
  else if(buttonValueSix){
    double wristSpeed = 0.2;
    Wrist->Set(ControlMode::PercentOutput, wristSpeed);
  }*/

  if(buttonValueSeven){
      climbSpeed = -0.4;
      //currentlyUp = 0;
    }
  else if(buttonValueEight){
      climbSpeed = 0.4;
      //currentlyUp = 1;
    }
  else{
      climbSpeed = 0;
    }

    if(buttonValueFiveP){
      rollerSpeed = 0.75;
    }
    else if(buttonValueSixP){
      rollerSpeed = -0.75;
    }

  if(buttonValueFive){
      hatchSpeed = 1;
    }
  else if(buttonValueSix){
      hatchSpeed = -1;
    }
  else{
      hatchSpeed = 0;
    }

    //crawlSpeed = abs(m_partner->GetRawAxis(5))*-0.20;

    Shoulder->Set(ControlMode::Position, targetPositionRotationsS);
    ClimbLead->Set(ControlMode::PercentOutput, climbSpeed);
    ClimbFollow->Set(ControlMode::PercentOutput, climbSpeed);
    Roller->Set(ControlMode::PercentOutput, rollerSpeed);
    Hatch->Set(ControlMode::PercentOutput, hatchSpeed);
    Crawl->Set(ControlMode::PercentOutput, crawlSpeed);

    double lastButtonPressed4 = buttonValueFour;
    double lastButtonPressed1 = buttonValueOne;
    double lastButtonPressed2 = buttonValueTwo;
    double lastButtonPressed3 = buttonValueThree;

    m_robotDrive.ArcadeDrive((-m_stick->GetY()*0.5), (m_stick->GetX()*0.5));

}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return StartRobot<Robot>(); }
#endif
