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
#include <PWMVictorSPX.h>
#include <frc/Solenoid.h>

//constexpr double kPi = 3.14159265358979323846264338327950288419716939937510;
  double kP = 0.1001, kI = 0.00001, kD = 0.5, kIz = 0, kFF = 0, kMaxOutput = 1, kMinOutput = -1;
  static const int leftLeadDeviceID = 1, rightLeadDeviceID = 2, leftFollowDeviceID = 3, rightFollowDeviceID = 4;
  rev::CANSparkMax m_leftLeadMotor{leftLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightLeadMotor{rightLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_leftFollowMotor{leftFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightFollowMotor{rightFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  //SETUP FOR PID MOTORS
  static const int deviceID = 5;
  static const int pidID = 6;
  rev::CANSparkMax m_motor{deviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_slaveMotor{pidID, rev::CANSparkMax::MotorType::kBrushless};
  //rev::CANSparkMax m_motorFollower{motorFollowerDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANPIDController m_pidController = m_motor.GetPIDController();
  rev::CANPIDController m_pidController2 = m_slaveMotor.GetPIDController();
  rev::CANEncoder m_encoder = m_motor.GetEncoder();
  double encoderPos = m_encoder.GetPosition();
  double encoderVel = m_encoder.GetVelocity();
  //soleniod set up
  static const int m_chanel = 1, mod_num = 3, pulsedur = 1;
  frc::Solenoid solen{m_chanel};

void Robot::RobotInit() {
    TalonTest = new TalonSRX(3);
    VictorTest = new VictorSPX(4);
  m_leftFollowMotor.Follow(m_leftLeadMotor);
  m_rightFollowMotor.Follow(m_rightLeadMotor);

    m_pidController.SetP(kP);
    m_pidController.SetI(kI);
    m_pidController.SetD(kD);
    m_pidController.SetIZone(kIz);
    m_pidController.SetFF(kFF);
    m_pidController.SetOutputRange(kMinOutput, kMaxOutput);

    m_pidController2.SetP(kP);
    m_pidController2.SetI(kI);
    m_pidController2.SetD(kD);
    m_pidController2.SetIZone(kIz);
    m_pidController2.SetFF(kFF);
    m_pidController2.SetOutputRange(kMinOutput, kMaxOutput);
      // set PID coefficients

    // display PID coefficients on SmartDashboard
    frc::SmartDashboard::PutNumber("P Gain", kP);
    frc::SmartDashboard::PutNumber("I Gain", kI);
    frc::SmartDashboard::PutNumber("D Gain", kD);
    frc::SmartDashboard::PutNumber("I Zone", kIz);
    frc::SmartDashboard::PutNumber("Feed Forward", kFF);
    frc::SmartDashboard::PutNumber("Max Output", kMaxOutput);
    frc::SmartDashboard::PutNumber("Min Output", kMinOutput);
    frc::SmartDashboard::PutNumber("Encoder Position", encoderPos);
    frc::SmartDashboard::PutNumber("Encoder Velocity", encoderVel);

  // set up soleniod pulse 
  solen.SetPulseDuration(pulsedur);
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
 
//TalonSRX srx = {0};
frc::Joystick m_stick{3};
frc::DifferentialDrive m_robotDrive{m_leftLeadMotor, m_rightLeadMotor};

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
    // read PID coefficients from SmartDashboard
    double p = frc::SmartDashboard::GetNumber("P Gain", 0);
    double i = frc::SmartDashboard::GetNumber("I Gain", 0);
    double d = frc::SmartDashboard::GetNumber("D Gain", 0);
    double iz = frc::SmartDashboard::GetNumber("I Zone", 0);
    double ff = frc::SmartDashboard::GetNumber("Feed Forward", 0);
    double max = frc::SmartDashboard::GetNumber("Max Output", 0);
    double min = frc::SmartDashboard::GetNumber("Min Output", 0);
    double speedT;
    double speedV;
    double rotations;
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
     rotations = 8.68;
    }
    else if(buttonValueTwo == true){
     rotations = 17.36;
    }
    else if(buttonValueThree == true){
      rotations = 0;
    }
    else if(buttonValueFour == true){
      speedT = 100;
    }
    else if(buttonValueFive == true){
      speedV = 100;
    }

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { m_pidController.SetP(p); kP = p; }
    if((i != kI)) { m_pidController.SetI(i); kI = i; }
    if((d != kD)) { m_pidController.SetD(d); kD = d; }
    if((iz != kIz)) { m_pidController.SetIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_pidController.SetFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_pidController.SetOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; }

    if((p != kP)) { m_pidController2.SetP(p); kP = p; }
    if((i != kI)) { m_pidController2.SetI(i); kI = i; }
    if((d != kD)) { m_pidController2.SetD(d); kD = d; }
    if((iz != kIz)) { m_pidController2.SetIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_pidController2.SetFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_pidController2.SetOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; }
    
m_robotDrive.TankDrive(-m_stick.GetRawAxis(1), -m_stick.GetRawAxis(5));
//m_robotDrive.ArcadeDrive(-m_stick.GetRawAxis(0), -m_stick.GetRawAxis(5);)
m_pidController.SetReference(rotations, rev::ControlType::kPosition);
m_pidController2.SetReference(rotations, rev::ControlType::kPosition);
TalonTest->Set(ControlMode::PercentOutput, speedT);
VictorTest->Set(ControlMode::PercentOutput, speedV);

//frc::SmartDashboard::PutNumber("SetPoint", rotations);
    frc::SmartDashboard::PutNumber("ProcessVariable", m_encoder.GetPosition());
 
  //start pulse
  //solen.StartPulse();
  //single soleniod trigger
  bool buttonValueSix;
  buttonValueSix = m_stick.GetRawButton(6);
  if(buttonValueSix == true){
    solen.Set(1);
  }
  else{
    solen.Set(0);
  }
  
};

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
