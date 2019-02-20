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
#include "frc/WPILib.h"
#include <stdio.h>
#include <memory>
#include <chrono>
#include <thread>
#include <stdlib.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "robotIO.h"
#include "Constants.h"

using namespace frc;
using namespace std;



  /*static const int leftLeadDeviceID = 1, rightLeadDeviceID = 3, leftFollowDeviceID = 2 , rightFollowDeviceID = 4, shoulderID = 5;
  rev::CANSparkMax m_leftLeadMotor{leftLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightLeadMotor{rightLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_leftFollowMotor{leftFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightFollowMotor{rightFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless};*/
    static const int leftLeadDeviceID = 1, rightLeadDeviceID = 3, leftFollowDeviceID = 2 , rightFollowDeviceID = 4;
    rev::CANSparkMax m_leftLeadMotor{leftLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax m_rightLeadMotor{rightLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax m_leftFollowMotor{leftFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax m_rightFollowMotor{rightFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless};



  static const int shoulderID = 5;
  rev::CANSparkMax m_shoulder{shoulderID, rev::CANSparkMax::MotorType::kBrushless};


  //SETS DRIVE MODE
  //1 = Single Controller
  //2 = Partner Control
  static const int DriverMode = 1;

//INIT INPUTS CLASS
robotIO* inputs = new robotIO;

string _sb;

//WAIT INIT
void wait ( int seconds )
{
	clock_t endwait;
	endwait = clock () + seconds * CLOCKS_PER_SEC ;
	while (clock() < endwait) {}
}

//AUTON VARIABLES INIT FOR WRIST DEPLOY
bool shoulderUp = false;
bool wristDeployed = false;
bool fullyDeployed = false;
double autonSPos = 0;

//PID POSITION INT SETUP 
double absolutePositionS = 0;
double absolutePositionW = 0;
double kPgain = 0.04; /* percent throttle per degree of error */
double kDgain = 0.0004; /* percent throttle per angular velocity dps */
double kMaxCorrectionRatio = 0.30; /* cap corrective turning throttle to 30 percent of forward throttle */
/** holds the current angle to servo to */
double _targetAngle = 0;
/** count loops to print every second or so */
int _printLoops = 0;

  rev::CANPIDController m_pidController = m_shoulder.GetPIDController();

  // Encoder object created to display position values
  rev::CANEncoder m_encoder = m_shoulder.GetEncoder();

    // PID coefficients
  double kP = 0.85, kI = 0.00005, kD = 0.05, kIz = 0, kFF = 0, kMaxOutput = 0.2, kMinOutput = -0.2;

//LOGITECH CAMERA INIT
static void VisionThread()
    {
        cs::UsbCamera camera = CameraServer::GetInstance()->StartAutomaticCapture();
        camera.SetResolution(280, 200);
    }

void Robot::RobotInit() {
  static const int WristID = 6;
  static const int LeadID = 1, FollowID = 2, HatchID = 3, RollerID = 4, CrawlID = 7;
  Hatch = new WPI_TalonSRX(HatchID);
  ClimbLead = new WPI_TalonSRX(LeadID);
  ClimbFollow = new WPI_VictorSPX(FollowID);
  Roller = new WPI_VictorSPX(RollerID);
  ClimbFollow->Follow(*ClimbLead);
  Crawl = new WPI_TalonSRX(CrawlID);

  Wrist = new WPI_TalonSRX(WristID);

      //SET FOLLOWER MOTORS FOR DRIVE
    m_leftFollowMotor.Follow(m_leftLeadMotor);
    m_rightFollowMotor.Follow(m_rightLeadMotor);

      // set PID coefficients
    m_pidController.SetP(kP);
    m_pidController.SetI(kI);
    m_pidController.SetD(kD);
    m_pidController.SetIZone(kIz);
    m_pidController.SetFF(kFF);
    m_pidController.SetOutputRange(kMinOutput, kMaxOutput);

  //CAMERA INIT
  std::thread visionThread(VisionThread);
  visionThread.detach();

  //TALON AND SENSOR BOOL INIT
  const bool kInvert = false;
  const bool kSensorPhase = false;

  //SET FOLLOWER MOTORS FOR DRIVE
  m_leftFollowMotor.Follow(m_leftLeadMotor);
  m_rightFollowMotor.Follow(m_rightLeadMotor);

    //CREATE WRIST PID
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
		Wrist->Config_kP(kPIDLoopIdx, 0.01, kTimeoutMs);
		Wrist->Config_kI(kPIDLoopIdx, 0.0, kTimeoutMs);
		Wrist->Config_kD(kPIDLoopIdx, 0.0, kTimeoutMs);

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
    
  } else {
  }
}

frc::DifferentialDrive m_robotDrive{m_leftLeadMotor, m_rightLeadMotor};

void Robot::TeleopInit() {}

  //SPEED SETUP
  double climbSpeed = 0;
  double hatchSpeed = 0;
  double rollerSpeed = 0;
  double crawlSpeed = 0;  
  double wristSpeed = 0;
  bool hatchHeld = 0;
  bool isUp = 0;
  double shoulderManual;

  //POSITION SETUP
  double pos;
  double rotations = 0.2;

  //ROTATIONS SETUP
  double targetPositionRotationsW;

  int shoulderPos = 0;

  int maxPos = 2;
  int minPos = 0;

void Robot::TeleopPeriodic() {

  //ROLLER SPEED SET
  if(inputs->getButtonTwo()||inputs->getButtonTwoPartner()){
    rollerSpeed = 1;
    Roller->Set(ControlMode::PercentOutput, rollerSpeed);
  }

  else if(inputs->getButtonThree()||inputs->getButtonThreePartner()){
    rollerSpeed = -1;
    Roller->Set(ControlMode::PercentOutput, rollerSpeed);    
  }

  else{
    rollerSpeed = -0.1;
    Roller->Set(ControlMode::PercentOutput, rollerSpeed);
  }

  //HATCH SPEED SET
  if(inputs->getButtonFivePartner()||inputs->getButtonFive()){
    hatchSpeed = 0.25;
    bool hatchHeld = true;
  }

 // else if(inputs->getButtonSixPartner()||inputs->getButtonSix()){
   // hatchSpeed = -0.25;
  //}

  else if(hatchHeld == true){
    hatchSpeed = 0.05;
  }

  else{
    hatchSpeed = 0;
  }

  //SHOULDER POSITION SET
  /*if(inputs->getButtonFourPressed() && shoulderPos < maxPos){
    shoulderPos = shoulderPos + 1;
  }
  else if(inputs->getButtonOnePressed() && shoulderPos > minPos){
    shoulderPos = shoulderPos - 1;
  }
  else{
    shoulderPos = shoulderPos;
  }*/

  if(abs(inputs->getYPartner()) > 0.15){
    shoulderManual = inputs->getYPartner();
  }

  if(shoulderPos < 2 && inputs->getButtonOnePressed()){
    rotations = -17;
    targetPositionRotationsW = -1000;
    m_pidController.SetReference(rotations, rev::ControlType::kPosition);
    Wrist->Set(ControlMode::Position, targetPositionRotationsW);
    shoulderPos = 0;
  }
  else if(shoulderPos < 1 && inputs->getButtonFourPressed()){
    rotations = -32;
    targetPositionRotationsW = 20000;
    m_pidController.SetReference(rotations, rev::ControlType::kPosition);
    Wrist->Set(ControlMode::Position, targetPositionRotationsW);
    shoulderPos = 1;
  }
  else if(shoulderPos > 1 && inputs->getButtonOnePressed()){
    rotations = -32;
    //targetPositionRotationsW = -15000;
    m_pidController.SetReference(rotations, rev::ControlType::kPosition);
    Wrist->Set(ControlMode::Position, targetPositionRotationsW);
    shoulderPos = 1;
  }
  else if(shoulderPos < 2 && inputs->getButtonFourPressed()){
    rotations = -42;
    //targetPositionRotationsW = -45000;
    m_pidController.SetReference(rotations, rev::ControlType::kPosition);
    Wrist->Set(ControlMode::Position, targetPositionRotationsW);
    shoulderPos = 2;
  }
  else{
    targetPositionRotationsW = targetPositionRotationsW + -(inputs->getAxisFive() * 2000);
    //rotations = rotations + (shoulderManual);
    m_pidController.SetReference(rotations, rev::ControlType::kPosition);
    Wrist->Set(ControlMode::Position, targetPositionRotationsW);
  }  






//SHOULDER MANUAL CONTROL
  /*if(inputs->getButtonFour()){
    m_shoulder.Set(-0.5);
  }

  else if(inputs->getButtonOne()){
    m_shoulder.Set(0.5);
  }

  else{
    m_shoulder.Set(0.0);
  }*/

  //WRIST POSITION SET
  /*if(inputs->getButtonSeven()){
    pos++;
    //targetPositionRotationsW = pos * 4096;
    wristSpeed = -0.5;
    
    Wrist->Set(ControlMode::Position, targetPositionRotationsW);
  }
  else if(inputs->getButtonEight()){
    pos--;
    //targetPositionRotationsW = pos * 4096;    
    wristSpeed = 0.5;
    //Wrist->Set(ControlMode::Position, targetPositionRotationsW);
  }
  else{
    wristSpeed = -0.0;
    //Wrist->Set(ControlMode::Position, targetPositionRotationsW);
  }*/

  //ClIMB SPEED
  if(inputs->getButtonSevenPartner()){
    climbSpeed = 0.75;
  }
  else if(inputs->getButtonEightPartner()){
    climbSpeed = 0.075;
  }
  else if(inputs->getButtonSixPartner()){
    climbSpeed = -0.75;
  }
  else{
    climbSpeed = 0.0;
  }

  //CRAWL SPEED
  //crawlSpeed = inputs->getYPartner();
if(inputs->getButtonOnePartner()){
  crawlSpeed = -0.25;
}
else{
  crawlSpeed = 0;
}

  //double shoulderPos = m_shoulder.GetPosition();
  //MOTOR MODE SETUP
  ClimbLead->Set(ControlMode::PercentOutput, climbSpeed);
  ClimbFollow->Set(ControlMode::PercentOutput, climbSpeed);
  Hatch->Set(ControlMode::PercentOutput, hatchSpeed);
  Crawl->Set(ControlMode::PercentOutput, crawlSpeed);
  Wrist->Set(ControlMode::Position, targetPositionRotationsW);

  SmartDashboard::PutNumber("YMain", inputs->getY());
  SmartDashboard::PutNumber("XMain", inputs->getX());
  SmartDashboard::PutNumber("WristPos", Wrist->GetSelectedSensorPosition(0));
  SmartDashboard::PutNumber("ShoulderPos", m_encoder.GetPosition());
  SmartDashboard::PutNumber("ShoulderPositionSet", rotations);  

  m_robotDrive.ArcadeDrive(-(inputs->getYPartner()*0.40), (inputs->getAxisFourPartner()*0.475));

  //ROBOT DRIVE
  //m_robotDrive.ArcadeDrive(-(driveForward), -(driveTurn));

}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
