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
  double kP = 0.85, kI = 0.00005, kD = 0.05, kIz = 0, kFF = 0, kMaxOutput = 0.3, kMinOutput = -0.6;

//LOGITECH CAMERA INIT
static void VisionThread()
    {
        cs::UsbCamera camera = CameraServer::GetInstance()->StartAutomaticCapture();
        camera.SetResolution(272, 204);
        camera.SetFPS(22);
    }

void Robot::RobotInit() {
  //Manipulator and lift setup
  static const int LeadID = 1, FollowID = 2, HatchID = 3, RollerID = 4, WristID = 6, CrawlID = 7;
  Hatch = new WPI_TalonSRX(HatchID);
  ClimbLead = new WPI_TalonSRX(LeadID);
  ClimbFollow = new WPI_VictorSPX(FollowID);
  ClimbFollow->Follow(*ClimbLead);
  Roller = new WPI_VictorSPX(RollerID);
  Crawl = new WPI_TalonSRX(CrawlID);
  Wrist = new WPI_TalonSRX(WristID);

  limitSwitch = new DigitalInput(1);

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
    int absolutePositionW = Wrist->GetSelectedSensorPosition(0) & 0xFFF;
  Wrist->SetSelectedSensorPosition(absolutePositionW, kPIDLoopIdx,
		kTimeoutMs);
  Wrist->ConfigSelectedFeedbackSensor(
	  FeedbackDevice::CTRE_MagEncoder_Relative, kPIDLoopIdx,
	  kTimeoutMs);

   Wrist->ConfigNominalOutputForward(0, kTimeoutMs);
		Wrist->ConfigNominalOutputReverse(0, kTimeoutMs);
		Wrist->ConfigPeakOutputForward(0.66, kTimeoutMs);
		Wrist->ConfigPeakOutputReverse(-0.66, kTimeoutMs); 

   Wrist->Config_kF(kPIDLoopIdx, 0.0, kTimeoutMs);
		Wrist->Config_kP(kPIDLoopIdx, 0.1, kTimeoutMs);
		Wrist->Config_kI(kPIDLoopIdx, 0.001, kTimeoutMs);
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

int shoulderPos;
double wristStart;
double hatchStart;
int hatchPos;

/*V V V V PUT STUFF IN HERE V V V V*/
/*V V V V V V V V V V V V V V V V V*/
void Robot::TeleopInit() {
  wristStart = Wrist->GetSelectedSensorPosition(0);
  hatchPos = 0;
  shoulderPos = 0;
  hatchStart = Hatch->GetSelectedSensorPosition(0);
} 
/*^ ^ ^ ^ PUT STUFF IN HERE ^ ^ ^ ^*/
/*^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^*/

  //SPEED SETUP
  double climbSpeed = 0;
  double hatchSpeed = 0;
  double rollerSpeed = 0;
  double crawlSpeed = 0;  
  double wristSpeed = 0;
  bool isUp = 0;
  double shoulderManual = 0;
  //double hatchStart = 0;

  //POSITION SETUP
  double pos;
  double rotations = 0.2;

  //ROTATIONS SETUP
  double targetPositionRotationsW; // Wrist positions
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


  //Hatch PID

  //    //  ////////  //////////  ////////  //    //
  //    //  //    //      //      //        //    //
  ////////  ////////      //      //        ////////
  //    //  //    //      //      //        //    //
  //    //  //    //      //      ////////  //    //

  bool hatchHeld = false;
      //Manual hatch control
  if(inputs->getLT() == 1){
    hatchSpeed = targetPositionRotationsH - 500;
    hatchHeld = true;
    Hatch->Set(ControlMode::Position, hatchSpeed);
  }
  else if(inputs->getRT() == 1){
    hatchSpeed = targetPositionRotationsH + inputs->getRT() * 500;
    hatchHeld = true;
    Hatch->Set(ControlMode::Position, hatchSpeed);
    }
  else if(hatchPos == 0 && inputs->getButtonFive()){
    targetPositionRotationsH = hatchStart - 2150/.73;
    hatchStart = targetPositionRotationsH;
    Hatch->Set(ControlMode::Position, targetPositionRotationsH);
    hatchPos = 2;
  }
  else if (hatchPos == 1 && inputs->getButtonFive()){
    if (hatchHeld){
      targetPositionRotationsH = hatchStart; // Resets hatch after manual control
    }
    targetPositionRotationsH =  targetPositionRotationsH - 1350/.73;
    Hatch->Set(ControlMode::Position, targetPositionRotationsH);
    hatchPos = 2;
  }
  else if(hatchPos == 2 && inputs->getButtonSix()){ 
    if (hatchHeld){
      targetPositionRotationsH = hatchStart + 1350/.73; // resets hatch after manual control
    }   
    targetPositionRotationsH = targetPositionRotationsH + 1350/.73;
    Hatch->Set(ControlMode::Position, targetPositionRotationsH);
    hatchPos = 1;
  }
/*// Old Manual Control
  if(inputs->getButtonFivePartner() || inputs->getLT()){
      hatchSpeed = -.5;
      hatchHeld = false;
      Hatch->Set(ControlMode::PercentOutput, hatchSpeed);
    }

    else if(inputs->getButtonSixPartner() || inputs->getRT()){
      hatchSpeed = .5;
      hatchHeld = true;
      Hatch->Set(ControlMode::PercentOutput, hatchSpeed);
    }
    else if (hatchHeld == true){
      hatchSpeed = .1;
      Hatch->Set(ControlMode::PercentOutput, hatchSpeed);
    }
    else{
      hatchSpeed = 0;
      Hatch->Set(ControlMode::PercentOutput, hatchSpeed);
    }*/

  //Shoulder & Wrist PID Cargo

  //////    //  //////          //////
  //    //  //  //    //      //
  /////     //  //    //  //  //
  //        //  //    //      //
  //        //  /////           //////

  if(shoulderPos < 2 && inputs->getButtonOnePressed()){
    //Ball Grab
    rotations = -22; //Use for 50:1 Arm gearbox (Comp Bot)
    //targetPositionRotationsW = -900; //Use for 100:1 Wrist gearbox (Comp Bot)
    m_pidController.SetReference(rotations, rev::ControlType::kPosition);
    //Wrist->Set(ControlMode::Position, targetPositionRotationsW);
    shoulderPos = 0;
  }
  else if(shoulderPos < 1 && inputs->getButtonFourPressed()){
    //Rocket Low Cargo
    rotations = -27; //Use for 50:1 Arm gearbox (Comp Bot)
    //targetPositionRotationsW = 20000; //Use for 100:1 Wrist gearbox (Comp Bot)
    m_pidController.SetReference(rotations, rev::ControlType::kPosition);
    //Wrist->Set(ControlMode::Position, targetPositionRotationsW);
    shoulderPos = 1;
  }
  else if(shoulderPos == 2 && inputs->getButtonOnePressed()){
    //Rocket Low Cargo
    rotations = -27;  //Use for 50:1 Arm gearbox (Comp Bot)
    //targetPositionRotationsW = -15000;
    m_pidController.SetReference(rotations, rev::ControlType::kPosition);
    //Wrist->Set(ControlMode::Position, targetPositionRotationsW);
    shoulderPos = 1;
  }
  else if(shoulderPos == 1 && inputs->getButtonFourPressed()){
    //Cargo Ship
    rotations = -60; //Use for 50:1 Arm gearbox (Comp Bot)
    //targetPositionRotationsW = -45000;
    m_pidController.SetReference(rotations, rev::ControlType::kPosition);
    //Wrist->Set(ControlMode::Position, targetPositionRotationsW);
    shoulderPos = 2;
  }
  else if(shoulderPos == 3 && inputs->getButtonOnePressed()){
    //Cargo Ship
    rotations = -60; //Use for 50:1 Arm gearbox (Comp Bot)
    //targetPositionRotationsW = -45000;
    m_pidController.SetReference(rotations, rev::ControlType::kPosition);
    //Wrist->Set(ControlMode::Position, targetPositionRotationsW);
    shoulderPos = 2;
  }
  else if(shoulderPos == 4 && inputs->getButtonOnePressed()){
    //Rocket Mid Cargo
    rotations = -67.7; //Use for 50:1 Arm gearbox (Comp Bot)
    //targetPositionRotationsW = -45000;
    m_pidController.SetReference(rotations, rev::ControlType::kPosition);
    //Wrist->Set(ControlMode::Position, targetPositionRotationsW);
    shoulderPos = 3; 
  }
  else if(shoulderPos == 2 && inputs->getButtonFourPressed()){
    //Rocket Mid Cargo
    rotations = -67.7; //Use for 50:1 Arm gearbox (Comp Bot)
    //targetPositionRotationsW = -45000;
    m_pidController.SetReference(rotations, rev::ControlType::kPosition);
    //Wrist->Set(ControlMode::Position, targetPositionRotationsW);
    shoulderPos = 3; 
  }
  else if(shoulderPos == 3 && inputs->getButtonFourPressed()){
    //Rocket High Cargo
    rotations = -93.75; //Use for 50:1 Arm gearbox (Comp Bot)
    //targetPositionRotationsW = -45000;
    m_pidController.SetReference(rotations, rev::ControlType::kPosition);
    //Wrist->Set(ControlMode::Position, targetPositionRotationsW);
    shoulderPos = 4; 
  }
  else{
    //Manual Wrist Control
    /*if(abs(inputs->getAxisFive()) > .1){
    targetPositionRotationsW = targetPositionRotationsW - (inputs->getAxisFive() * 4096);
    }*/
    //Manual Shoulder Control
    if(abs(inputs->getY()) > 0.08){
      shoulderManual = (inputs->getY() * .6); // Set arm target rate the same as target movement
    }
    else{
    shoulderManual = 0;
    }

    rotations = rotations + (shoulderManual);

    //Arm max and min limits
    if (rotations < -93.75){ 
      rotations = -93.75;      
    }
    else if (rotations > 2.5){ 
      rotations = 2.5;            
    }

    //Wrist max and min limits
    /*
    if (targetPositionRotationsW > Wrist->GetSelectedSensorPosition(0) + 700){
      targetPositionRotationsW = Wrist->GetSelectedSensorPosition(0);
    }
    else if (targetPositionRotationsW < Wrist->GetSelectedSensorPosition(0) - 700){
      targetPositionRotationsW = Wrist->GetSelectedSensorPosition(0);
    }*/
    
    m_pidController.SetReference(rotations, rev::ControlType::kPosition);
    Wrist->Set(ControlMode::Position, targetPositionRotationsW);
  }  

  //Shoulder & Wrist PID Hatch

  //////    //  //////        //    //
  //    //  //  //    //      //    //
  /////     //  //    //  //  ////////
  //        //  //    //      //    //
  //        //  /////         //    //

  if(shoulderPos < 2 && inputs->getPOV() == 180){
    //Low Hatch Position
    rotations = -15; //Use for 50:1 Arm gearbox (Comp Bot)
    //targetPositionRotationsW = -1000; //Use for 100:1 Wrist gearbox (Comp Bot)
    m_pidController.SetReference(rotations, rev::ControlType::kPosition);
    //Wrist->Set(ControlMode::Position, targetPositionRotationsW);
    shoulderPos = 0;
  }
  else if((shoulderPos == 0 || shoulderPos == 2) && (inputs->getPOV() == 0 || inputs->getPOV() == 180)){
    //Mid Hatch Position
    rotations = -53; //Use for 50:1 Arm gearbox (Comp Bot)
    //targetPositionRotationsW = 20000; //Use for 100:1 Wrist gearbox (Comp Bot)
    m_pidController.SetReference(rotations, rev::ControlType::kPosition);
    //Wrist->Set(ControlMode::Position, targetPositionRotationsW);
    shoulderPos = 1;
  }
 /* else if(shoulderPos == 2 && inputs->getPOV() == 180){
    //Mid Hatch Position
    rotations = -53;  //Use for 50:1 Arm gearbox (Comp Bot)
    //targetPositionRotationsW = -15000;
    m_pidController.SetReference(rotations, rev::ControlType::kPosition);
    Wrist->Set(ControlMode::Position, targetPositionRotationsW);
    shoulderPos = 1;
  }*/
  else if((shoulderPos == 1 || shoulderPos == 3) && inputs->getPOV() == 0){
    //High Hatch Position
    rotations = -89.5; //Use for 50:1 Arm gearbox (Comp Bot)
    //targetPositionRotationsW = -45000;
    m_pidController.SetReference(rotations, rev::ControlType::kPosition);
    //Wrist->Set(ControlMode::Position, targetPositionRotationsW);
    shoulderPos = 2;
  }
  else{
    //Manual Wrist Control
    if(abs(inputs->getAxisFive()) > .1 && limitSwitch->Get() == 1){
    targetPositionRotationsW = targetPositionRotationsW + (inputs->getAxisFive() * 4096); //* 2000);
    }
    //Manual Shoulder Control
    if(abs(inputs->getY()) > 0.05){
      shoulderManual = (inputs->getY() * .6); // Set arm target rate the same as target movement
    }
    else{
    shoulderManual = 0;
    }

    rotations = rotations + (shoulderManual);

    //Arm max and min limits
    if (rotations < -93.75){ 
      rotations = -93.75;      
    }
    else if (rotations > 2.5){ 
      rotations = 2.5;            
    }

    //Wrist max and min limits
    /*if (targetPositionRotationsW > wristStart + 87000){
      targetPositionRotationsW = wristStart + 87000;
    }/*
    else if (targetPositionRotationsW < -400000){
      targetPositionRotationsW = -400000;
    }
    
    m_pidController.SetReference(rotations, rev::ControlType::kPosition);
    Wrist->Set(ControlMode::Position, targetPositionRotationsW);*/
  }  
  

  //ClIMB SPEED

    //////  //        //  //      //  //////
  //        //        //  ////  ////  //    //
  //        //        //  //  //  //  //////
  //        //        //  //      //  //    //
    //////  ////////  //  //      //  //////

  if(inputs->getButtonSevenPartner()){
    climbSpeed = 0.75;
  }
  /*else if(inputs->getButtonEightPartner()){
    climbSpeed = 0.075;
  }*/
  else if(inputs->getButtonEightPartner()){
    climbSpeed = -0.75;
  }
  else{
    climbSpeed = 0.0;
  }

  //CRAWL SPEED
if(inputs->getButtonOnePartner()){
  crawlSpeed = -0.25;
}
else{
  crawlSpeed = 0;
}

  //MOTOR MODE SETUP
  ClimbLead->Set(ControlMode::PercentOutput, climbSpeed);
  ClimbFollow->Set(ControlMode::PercentOutput, climbSpeed);
  //Hatch->Set(ControlMode::PercentOutput, hatchSpeed);
  Crawl->Set(ControlMode::PercentOutput, crawlSpeed);

  //Smart Dashboard outputs
  SmartDashboard::PutNumber("YMain", inputs->getYPartner());
  SmartDashboard::PutNumber("XMain", inputs->getAxisFourPartner());
  SmartDashboard::PutNumber("WristPos", Wrist->GetSelectedSensorPosition(0));
  SmartDashboard::PutNumber("WristTargetPos", targetPositionRotationsW);
  SmartDashboard::PutNumber("HatchValue", Hatch->GetSelectedSensorPosition(0));
  SmartDashboard::PutNumber("HatchTargetPos", targetPositionRotationsH);
  SmartDashboard::PutNumber("HatchPos", hatchPos);
  SmartDashboard::PutNumber("ShoulderPos", shoulderPos);
  SmartDashboard::PutNumber("ShoulderValue", m_encoder.GetPosition());
  SmartDashboard::PutNumber("ShoulderTargetPos", rotations); 
  SmartDashboard::PutNumber("LimitSwitch", limitSwitch->Get()); 

  //Driver Control Inputs
  //Deadband modifier
  double joyStickYAxis = 0.0, joyStickXAxis = 0.0;
  double deadBandY = 0.1, deadBandX = 0.12;

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
    m_robotDrive.ArcadeDrive(0, .2);
  }
  else if(inputs->getPOVPartner() == 270){
    m_robotDrive.ArcadeDrive(0, -.2);
  }
  else{
    //m_robotDrive.ArcadeDrive(-(inputs->getYPartner()*0.40), (inputs->getAxisFourPartner()*0.475));
    m_robotDrive.ArcadeDrive(-(joyStickYAxis*0.5), (joyStickXAxis*0.5));
  }

}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
