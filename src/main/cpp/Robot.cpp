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
#include "frc/Watchdog.h"

using namespace frc;
using namespace std;

  //Drive Setup
  static const int leftLeadDeviceID = 1, rightLeadDeviceID = 3, leftFollowDeviceID = 2 , rightFollowDeviceID = 4;
  rev::CANSparkMax m_leftLeadMotor{leftLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightLeadMotor{rightLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_leftFollowMotor{leftFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightFollowMotor{rightFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless};

  //Arm Setup
  static const int shoulderID = 5;
  rev::CANSparkMax m_shoulder{shoulderID, rev::CANSparkMax::MotorType::kBrushless};

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
double absolutePositionH = 0;
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
  double kP = 0.60, kI = 0.00005, kD = 0.05, kIz = 0, kFF = 0, kMaxOutput = 0.3, kMinOutput = -0.80;

//LOGITECH CAMERA INIT
static void VisionThread()
    {
        cs::UsbCamera camera = CameraServer::GetInstance()->StartAutomaticCapture();
        camera.SetResolution(272, 204);
        camera.SetFPS(22);
    }

void Robot::RobotInit() {
  //ID's Setup
  static const int LeadID = 1, FollowID = 2, HatchID = 3, RollerID = 4, WristID = 6, CrawlID = 7;
  //Hatch Setup
  Hatch = new WPI_TalonSRX(HatchID);
  //Climb Setup
  ClimbLead = new WPI_TalonSRX(LeadID);
  ClimbFollow = new WPI_VictorSPX(FollowID);
  ClimbFollow->Follow(*ClimbLead);
  //Roller Setup
  Roller = new WPI_VictorSPX(RollerID);
  //Crawler Setup
  Crawl = new WPI_TalonSRX(CrawlID);
  //Wrist Setup
  Wrist = new WPI_TalonSRX(WristID);

  limitSwitch = new DigitalInput(1);

  //SET FOLLOWER MOTORS FOR DRIVE
  m_leftFollowMotor.Follow(m_leftLeadMotor);
  m_rightFollowMotor.Follow(m_rightLeadMotor);

  //Set PID coefficients
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

  //Create Hatch PID
  int absolutePositionH = Hatch->GetSelectedSensorPosition(0) & 0xFFF;
  Hatch->SetSelectedSensorPosition(absolutePositionH, kPIDLoopIdx, kTimeoutMs);
  Hatch->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, kPIDLoopIdx, kTimeoutMs);

  Hatch->ConfigNominalOutputForward(0, kTimeoutMs);
	Hatch->ConfigNominalOutputReverse(0, kTimeoutMs);
	Hatch->ConfigPeakOutputForward(1, kTimeoutMs);
	Hatch->ConfigPeakOutputReverse(-1, kTimeoutMs); 

  Hatch->Config_kF(kPIDLoopIdx, 0.0, kTimeoutMs);
	Hatch->Config_kP(kPIDLoopIdx, 0.8, kTimeoutMs);
	Hatch->Config_kI(kPIDLoopIdx, 0.0, kTimeoutMs);
	Hatch->Config_kD(kPIDLoopIdx, 0.0, kTimeoutMs);

  //CREATE WRIST PID
  /*int absolutePositionW = Wrist->GetSelectedSensorPosition(0) & 0xFFF;
  Wrist->SetSelectedSensorPosition(absolutePositionW, kPIDLoopIdx,
	  kTimeoutMs);
  Wrist->ConfigSelectedFeedbackSensor(
	FeedbackDevice::CTRE_MagEncoder_Relative, kPIDLoopIdx,
	  kTimeoutMs);

  //Wrist PID Coefficients Setup
  Wrist->ConfigNominalOutputForward(0, kTimeoutMs);
	Wrist->ConfigNominalOutputReverse(0, kTimeoutMs);
	Wrist->ConfigPeakOutputForward(0.66, kTimeoutMs);
	Wrist->ConfigPeakOutputReverse(-0.66, kTimeoutMs); 

  Wrist->Config_kF(kPIDLoopIdx, 0.0, kTimeoutMs);
	Wrist->Config_kP(kPIDLoopIdx, 0.1, kTimeoutMs);
	Wrist->Config_kI(kPIDLoopIdx, 0.001, kTimeoutMs);
	Wrist->Config_kD(kPIDLoopIdx, 0.0, kTimeoutMs);*/

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
  Robot::TeleopPeriodic();
}

//Setup Drive Function
frc::DifferentialDrive m_robotDrive{m_leftLeadMotor, m_rightLeadMotor};

//Setup Starting Position Ints for PID
int shoulderPos;
double wristStart;
double hatchStart;
int hatchPos;
double targetPositionRotationsW; 
double wristGoal;
double wristCurrent;
double wristPosition;
double wristLast;
bool isDeployed = false;
bool climbed = false;
double rotations;
bool hatchStarted;
double hatchGoal;
double hatchCurrent;
double hatchPosition;
double hatchLast;
double wristHold;



void Robot::TeleopInit() {
  //Setup Starting Positions for PID
  wristStart = Wrist->GetSelectedSensorPosition(0);
  hatchPos = 0;
  shoulderPos = 0;
  hatchLast = Hatch->GetSelectedSensorPosition(0);
  targetPositionRotationsW = wristStart;
  wristLast  = Wrist->GetSelectedSensorPosition(0);
  rotations = 0.2;
  hatchStarted = false;
} 

  //SPEED SETUP
  double climbSpeed = 0;
  double hatchSpeed = 0;
  double rollerSpeed = 0;
  double crawlSpeed = 0;  
  double wristSpeed = 0;
  bool isUp = 0;
  double shoulderManual = 0;

  //POSITION SETUP
  double pos;

  //ROTATIONS SETUP
  double targetPositionRotationsH; // Hatch positions
 
  int maxPos = 2;
  int minPos = 0;

void Robot::TeleopPeriodic() {

  //ROLLER SPEED SET

  ////////  ////////  //        //       //////// ////////
  //    //  //    //  //        //       //       //    //
  ///////   //    //  //        //       //////   ///////
  //  //    //    //  //        //       //       //  //
  //    //  ///////   ////////  //////// //////// //    //

  if(inputs->getButtonThree()||inputs->getButtonThreePartner()){
    //Roller Speed Max
    rollerSpeed = 1;
    Roller->Set(ControlMode::PercentOutput, rollerSpeed);
  }

  else if(inputs->getButtonTwo()||inputs->getButtonTwoPartner()){
    //Roller Speed Max Reversed
    rollerSpeed = -1;
    Roller->Set(ControlMode::PercentOutput, rollerSpeed);    
  }

  else{
    //Passive Speed -10%
    rollerSpeed = -0.1;
    Roller->Set(ControlMode::PercentOutput, rollerSpeed);
  }


  //Hatch PID

  //    //  ////////  //////////  ////////  //    //
  //    //  //    //      //      //        //    //
  ////////  ////////      //      //        ////////
  //    //  //    //      //      //        //    //
  //    //  //    //      //      ////////  //    //

  //Sets Boolean for Passive Hatch Power
  bool hatchHeld = false;

  if(inputs->getButtonSix() == 1){
        Hatch->Set(ControlMode::PercentOutput, 0.35);
        hatchHeld = false;
    }
    else if(inputs->getButtonFive() == 1){
        Hatch->Set(ControlMode::PercentOutput, -0.35);
        hatchHeld = true;
    }
    else if(hatchHeld == true){
      Hatch->Set(ControlMode::PercentOutput, 0.1);
    }
    else{
      Hatch->Set(ControlMode::PercentOutput, 0);
    }

  //Shoulder & Wrist PID Cargo

  //////    //  //////          //////
  //    //  //  //    //      //
  /////     //  //    //  //  //
  //        //  //    //      //
  //        //  /////           //////

  if(shoulderPos < 2 && inputs->getButtonOnePressed()){
    //Ball Grab Position
    //Amount of NEO Rotations
    rotations = -22;
    m_pidController.SetReference(rotations, rev::ControlType::kPosition);
    //Sets Current Shoulder Position Value
    shoulderPos = 0;
  }
  else if(shoulderPos < 1 && inputs->getButtonFourPressed()){
    //Rocket Low Cargo
    //Amount of NEO Rotations
    rotations = -27;
    m_pidController.SetReference(rotations, rev::ControlType::kPosition);
    //Sets Current Shoulder Position Value
    shoulderPos = 1;
  }
  else if(shoulderPos == 2 && inputs->getButtonOnePressed()){
    //Rocket Low Cargo
    //Amount of NEO Rotations
    rotations = -27;
    m_pidController.SetReference(rotations, rev::ControlType::kPosition);
    //Sets Current Shoulder Position Value
    shoulderPos = 1;
  }
  else if(shoulderPos == 1 && inputs->getButtonFourPressed()){
    //Cargo Ship
    //Amount of NEO Rotations
    rotations = -60;
    m_pidController.SetReference(rotations, rev::ControlType::kPosition);
    //Sets Current Shoulder Position Value
    shoulderPos = 2;
  }
  else if(shoulderPos == 3 && inputs->getButtonOnePressed()){
    //Cargo Ship
    //Amount of NEO Rotations
    rotations = -60;
    m_pidController.SetReference(rotations, rev::ControlType::kPosition);
    //Sets Current Shoulder Position Value
    shoulderPos = 2;
  }
  else if(shoulderPos == 4 && inputs->getButtonOnePressed()){
    //Rocket Mid Cargo
    //Amount of NEO Rotations
    rotations = -67.7;
    m_pidController.SetReference(rotations, rev::ControlType::kPosition);
    //Sets Current Shoulder Position Value
    shoulderPos = 3; 
  }
  else if(shoulderPos == 2 && inputs->getButtonFourPressed()){
    //Rocket Mid Cargo
    //Amount of NEO Rotations
    rotations = -67.7;
    m_pidController.SetReference(rotations, rev::ControlType::kPosition);
    //Sets Current Shoulder Position Value
    shoulderPos = 3; 
  }
  else if(shoulderPos == 3 && inputs->getButtonFourPressed()){
    //Rocket High Cargo
    //Amount of NEO Rotations
    rotations = -93.75;
    m_pidController.SetReference(rotations, rev::ControlType::kPosition);
    //Sets Current Shoulder Position Value
    shoulderPos = 4; 
  }
  else{
    //Manual Shoulder Control
    if(abs(inputs->getY()) > 0.08){
      //Amount of Wrist Rotations Changes as Stick is held
      shoulderManual = (inputs->getY() * .6); // Set arm target rate the same as target movement
    }
    else{
    //Else Shoulder not Moving
    shoulderManual = 0;
    }

    //Sets Shoulder Position
    rotations = rotations + (shoulderManual);

    //Arm max and min limits
    if (rotations < -93.75){ 
      //Go Back if Above Max
      rotations = -93.75;      
    }
    else if (rotations > 2.5){ 
      //Go Back if Above Max
      rotations = 2.5;            
    }
    
    //Setup PID Controller
    m_pidController.SetReference(rotations, rev::ControlType::kPosition);
    //Set Wrist Position
  }  
  
    //Manual Wrist Control
    if(inputs->getAxisFive() > 0.1 && limitSwitch->Get() == 1){
      wristCurrent = (inputs->getAxisFive());
        Wrist->Set(ControlMode::Current, 4 * wristCurrent);
        wristLast = Wrist->GetSelectedSensorPosition(0);
    }
    else if(inputs->getAxisFive() < -0.1){
      wristCurrent = (inputs->getAxisFive());
        Wrist->Set(ControlMode::Current, 10 * wristCurrent);
        wristLast  = Wrist->GetSelectedSensorPosition(0);
    }
    else if(limitSwitch->Get() == 0){

      if(inputs->getAxisFive() > 0.1){
      wristCurrent = (inputs->getAxisFive());
        Wrist->Set(ControlMode::Current, 4 * wristCurrent);
        wristLast = Wrist->GetSelectedSensorPosition(0);
        }
      else if(inputs->getAxisFive() < -0.1){
      wristCurrent = (inputs->getAxisFive());
        Wrist->Set(ControlMode::Current, 0.5 * wristCurrent);
        wristLast  = Wrist->GetSelectedSensorPosition(0);
        }
        else if(abs(wristCurrent) > 20){
       wristCurrent = 0;
        }
    }
    else if(abs(wristCurrent) > 20){
      wristCurrent = 0;
    }
    else{
      wristPosition = Wrist->GetSelectedSensorPosition(0);
      wristHold = (wristLast - wristPosition) * 1/220000;
      wristHold != 0;
      Wrist->Set(ControlMode::Current, -7.5 * wristHold);
    }

    /*if(isDeployed == false && inputs->getButtonEightPartner()){
      rotations = 6;
      m_pidController.SetReference(rotations, rev::ControlType::kPosition);
      int shoulderPos = m_shoulder->GetEncoder();
      if(shoulderPos > 4 && shoulderPos < 5.5){
        m_pidController.SetReference(rotations, rev::ControlType::kPosition);
        wristCurrent = 1;
        Wrist->Set(ControlMode::Current, 5 * wristCurrent);
      }
    }*/


    /*if(abs(inputs->getAxisFive()) > .1 && limitSwitch->Get() == 1){
    //Amount of Wrist Rotations Changes as Stick is held
    targetPositionRotationsW = wristStart - (inputs->getAxisFive() * 4096);
    //Wrist->Set(ControlMode::Position, targetPositionRotationsW);
    }
    else if(inputs->getAxisFive() > .1 && limitSwitch->Get() == 0){
    //Amount of Wrist Rotations Changes as Stick is held
    targetPositionRotationsW = wristStart - (inputs->getAxisFive() * 4096);
    //Wrist->Set(ControlMode::Position, targetPositionRotationsW);
    }*/

  //Shoulder & Wrist PID Hatch

  //////    //  //////        //    //
  //    //  //  //    //      //    //
  /////     //  //    //  //  ////////
  //        //  //    //      //    //
  //        //  /////         //    //

  if(shoulderPos < 2 && inputs->getPOV() == 180){
    //Low Hatch Position
    rotations = -15;
    m_pidController.SetReference(rotations, rev::ControlType::kPosition);
    shoulderPos = 0;
  }
  else if((shoulderPos == 0 || shoulderPos == 2) && (inputs->getPOV() == 0 || inputs->getPOV() == 180)){
    //Mid Hatch Position
    rotations = -53;
    m_pidController.SetReference(rotations, rev::ControlType::kPosition);
    shoulderPos = 1;
  }
  else if((shoulderPos == 1 || shoulderPos == 3) && inputs->getPOV() == 0){
    //High Hatch Position
    rotations = -89.5;
    m_pidController.SetReference(rotations, rev::ControlType::kPosition);
    shoulderPos = 2;
  }

  //ClIMB SPEED

    //////  //        //  //      //  //////
  //        //        //  ////  ////  //    //
  //        //        //  //  //  //  //////
  //        //        //  //      //  //    //
    //////  ////////  //  //      //  //////

  if(inputs->getButtonSevenPartner()){
    //Climb Speed 75%
    climbSpeed = 0.75;
    climbed = true;
  }
  else if(inputs->getButtonEightPartner()){
    //Climb Speed -15%
    climbSpeed = -0.15;
    climbed = false;
  }
  else if(climbed == false){
    //Climber Off
    climbSpeed = 0.0;
  }
  else if(climbed == true){
    climbSpeed = 0.15;
  }

  //CRAWL SPEED
  if(inputs->getButtonOnePartner()){
    //Crawl Speed -25%
    crawlSpeed = -0.20;
  }
  else{
    //Crawler Off
    crawlSpeed = 0;
  }

  //MOTOR MODE SETUP
  ClimbLead->Set(ControlMode::PercentOutput, climbSpeed);
  ClimbFollow->Set(ControlMode::PercentOutput, climbSpeed);
  Crawl->Set(ControlMode::PercentOutput, crawlSpeed);

  //Smart Dashboard outputs
  SmartDashboard::PutNumber("YMain", inputs->getYPartner());
  SmartDashboard::PutNumber("XMain", inputs->getAxisFourPartner());
  SmartDashboard::PutNumber("WristPosition", Wrist->GetSelectedSensorPosition(0));
  SmartDashboard::PutNumber("WristStart", wristStart);
  SmartDashboard::PutNumber("WristLast", wristLast);
  SmartDashboard::PutNumber("WristCurrent", wristCurrent);
  SmartDashboard::PutNumber("HatchValue", Hatch->GetSelectedSensorPosition(0));
  SmartDashboard::PutNumber("HatchTargetPos", targetPositionRotationsH);
  SmartDashboard::PutNumber("HatchPos", hatchPos);
  SmartDashboard::PutNumber("ShoulderPos", shoulderPos);
  SmartDashboard::PutNumber("ShoulderValue", m_encoder.GetPosition());
  SmartDashboard::PutNumber("ShoulderTargetPos", rotations);  

  //Driver Control Inputs
  //Deadband modifier
  double joyStickYAxis = 0.0, joyStickXAxis = 0.0;
  double deadBandY = 0.0, deadBandX = 0.0;

  //Deadband Setup
  if (abs(inputs->getYPartner()) < deadBandY) {
    joyStickYAxis = 0.0;
    }
  else {
   if (inputs->getYPartner()>0.0) {
      joyStickYAxis = (inputs->getYPartner() - deadBandY) / (1.0 - deadBandY);
      }
   else {
      joyStickYAxis = (inputs->getYPartner() - -deadBandY) / (1.0 - deadBandY);
      }
    }

  if (abs(inputs->getAxisFourPartner()) < deadBandX) {
    joyStickXAxis = 0.0;
    }
  else {
   if (inputs->getAxisFourPartner() >0.0) {
      joyStickXAxis = (inputs->getAxisFourPartner() - deadBandX) / (1.0 - deadBandX);
      }
   else {
      joyStickXAxis = (inputs->getAxisFourPartner() - -deadBandX) / (1.0 - deadBandX);
      }
    }

  //dPad Steer
  if(inputs->getPOVPartner() == 90){
    m_robotDrive.ArcadeDrive(0, .25);
  }
  else if(inputs->getPOVPartner() == 270){
    m_robotDrive.ArcadeDrive(0, -.25);
  }
  else if(inputs->getPOVPartner() == 0){
    m_robotDrive.ArcadeDrive(.20,0);
  }
  else if(inputs->getPOVPartner() == 180){
    m_robotDrive.ArcadeDrive(-.20,0);
  }
  else{
    //m_robotDrive.ArcadeDrive(-(inputs->getYPartner()*0.40), (inputs->getAxisFourPartner()*0.475));
    m_robotDrive.ArcadeDrive(-(joyStickYAxis*0.65), (joyStickXAxis*0.5));
  }

}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
