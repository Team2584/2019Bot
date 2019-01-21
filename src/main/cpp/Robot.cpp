/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include <frc/WPILib.h>
#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/SmartDashboard/SmartDashboard.h>
#include "rev/CANSparkMax.h"
#include "stdlib.h"
#include <time.h>
#include <stdio.h>
#include <iostream>
using namespace std; 

class Robot : public frc::TimedRobot {
  // initialize motor
  static const int deviceID = 1;
  //static const int followerID = 4;
  rev::CANSparkMax m_motor{deviceID, rev::CANSparkMax::MotorType::kBrushless};
  //rev::CANSparkMax m_follower{followerID, rev::CANSparkMax::MotorType::kBrushless};

  /**
   * In order to use PID functionality for a controller, a CANPIDController object
   * is constructed by calling the GetPIDController() method on an existing
   * CANSparkMax object
   */
  rev::CANPIDController m_pidController = m_motor.GetPIDController();

  // Encoder object created to display position values
  rev::CANEncoder m_encoder = m_motor.GetEncoder();


  // PID coefficients
  double kP = 0.85, kI = 0.00005, kD = 0.05, kIz = 0, kFF = 0, kMaxOutput = 0.2, kMinOutput = -0.2;
  frc::Joystick m_stick{3};
 public:
  void RobotInit() {
    // set PID coefficients
    m_pidController.SetP(kP);
    m_pidController.SetI(kI);
    m_pidController.SetD(kD);
    m_pidController.SetIZone(kIz);
    m_pidController.SetFF(kFF);
    m_pidController.SetOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    frc::SmartDashboard::PutNumber("P Gain", kP);
    frc::SmartDashboard::PutNumber("I Gain", kI);
    frc::SmartDashboard::PutNumber("D Gain", kD);
    frc::SmartDashboard::PutNumber("I Zone", kIz);
    frc::SmartDashboard::PutNumber("Feed Forward", kFF);
    frc::SmartDashboard::PutNumber("Max Output", kMaxOutput);
    frc::SmartDashboard::PutNumber("Min Output", kMinOutput);
    frc::SmartDashboard::PutNumber("Set Rotations", 0);
  }
  double rotations = 0;
  int position = 0;
  void TeleopPeriodic() {
    // read PID coefficients from SmartDashboard
    double p = frc::SmartDashboard::GetNumber("P Gain", 0);
    double i = frc::SmartDashboard::GetNumber("I Gain", 0);
    double d = frc::SmartDashboard::GetNumber("D Gain", 0);
    double iz = frc::SmartDashboard::GetNumber("I Zone", 0);
    double ff = frc::SmartDashboard::GetNumber("Feed Forward", 0);
    double max = frc::SmartDashboard::GetNumber("Max Output", 0);
    double min = frc::SmartDashboard::GetNumber("Min Output", 0);

    //double rotations = frc::SmartDashboard::GetNumber("Set Rotations", 0);
    
    int maxPos = 2;
    int minPos = 0;
    bool buttonValueOne;
    buttonValueOne = m_stick.GetRawButton(1);
    bool buttonValueTwo;
    buttonValueTwo = m_stick.GetRawButton(2);
    int button;
    bool buttonValueThree;
    buttonValueThree = m_stick.GetRawButton(3);
    //position = 0;
    if(buttonValueOne == true){
     position = 1;
    }
    else if(buttonValueTwo == true){
     position = 0;
    }
    else if(buttonValueThree == true){
      position = 2;
    }
    if(position == 1){
      rotations = 5;
    }
    else if(position == 2){
      rotations = 10;
    }
    else if(position == 0){
      rotations = 0.2;
    }
    
    /*switch(position){
      case 0:
      rotations = 0.2;
      break;
      case 1:
      rotations = 5;
      break;
      case 2:
      rotations = 10;
      break;
    }*/

    
    /*else if(buttonValueOne == false && buttonValueTwo == false)
    {
      button = 0;
    }*/
    /*switch(button){
      default:
      m_pidController.SetReference(0, rev::ControlType::kVelocity);m_motor.StopMotor();
      break;
      case 1: 
      rotations = 8.68;
      break;
      case 2: 
      rotations = 0;

    }*/
        frc::SmartDashboard::PutNumber("Rotations", rotations);
        frc::SmartDashboard::PutNumber("Pos", position);


    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { m_pidController.SetP(p); kP = p; }
    if((i != kI)) { m_pidController.SetI(i); kI = i; }
    if((d != kD)) { m_pidController.SetD(d); kD = d; }
    if((iz != kIz)) { m_pidController.SetIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_pidController.SetFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_pidController.SetOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }

    /**
     * PIDController objects are commanded to a set point using the 
     * SetReference() method.
     * 
     * The first parameter is the value of the set point, whose units vary
     * depending on the control type set in the second parameter.
     * 
     * The second parameter is the control type can be set to one of four 
     * parameters:
     *  rev::ControlType::kDutyCycle
     *  rev::ControlType::kPosition
     *  rev::ControlType::kVelocity
     *  rev::ControlType::kVoltage
     */
    m_pidController.SetReference(rotations, rev::ControlType::kPosition);
    
    frc::SmartDashboard::PutNumber("SetPoint", rotations);
    frc::SmartDashboard::PutNumber("ProcessVariable", m_encoder.GetPosition());
  }
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
