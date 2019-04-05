/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include "frc/WPILib.h"
#include "rev/CANSparkMax.h"
#include "robotIO.h"

using namespace frc;
using namespace std;

    //Joystick Setup
    Joystick * m_stick = new Joystick(3);
    Joystick * m_partner = new Joystick(4);

    //CONTROLLER TYPE
    //1 = LOGITECH
    //2 = PS4
    int CONTROLLER_TYPE = 1;

    //Setup Button Checks Main Controller
    bool buttonOne;
    bool buttonTwo;
    bool buttonThree;
    bool buttonFour;
    bool buttonFive;
    bool buttonSix;
    bool buttonSeven;
    bool buttonEight;
    int dPad;
    double lTrigger;
    double rTrigger;
    double yAxis;
    double xAxis;
    bool axisFour;
    double axisFive;
    double axisThree;
    double axisFire;
    bool buttonNine;

    //Setup Button Checks Partner Controller
    double axisFivePartner;
    bool buttonOnePartner;
    bool buttonFourPartner;
    bool buttonFivePartner;
    bool buttonSixPartner;
    bool buttonTwoPartner;
    int dPadPartner;
    double lTriggerPartner;
    double rTriggerPartner;
    double YPartner;
    bool buttonFourPressed;
    bool buttonOnePressed;
    bool buttonThreePartner;
    bool buttonEightPartner;
    double axisFourPartner;
    bool buttonSevenPartner;
    double axisThreePartner;
    
    //Controller Setup Function
robotIO::robotIO() {
    Joystick * m_stick = new Joystick(3);
    Joystick * m_partner = new Joystick(4);
}

    //Button Get Functions
bool robotIO::getButtonOne(){
    buttonOne = m_stick->GetRawButton(2);
    return buttonOne;
}

bool robotIO::getButtonTwo(){
    buttonTwo = m_stick->GetRawButton(1);
    return buttonTwo;
}

bool robotIO::getButtonThree(){
    buttonThree = m_stick->GetRawButton(3);
    return buttonThree;
}

bool robotIO::getButtonFour(){
    buttonFour = m_stick->GetRawButton(4);
    return buttonFour;
}

bool robotIO::getButtonFive(){
    buttonFive = m_stick->GetRawButton(5);
    return buttonFive;
}

bool robotIO::getButtonSix(){
    buttonSix = m_stick->GetRawButton(6);
    return buttonSix;
}

bool robotIO::getButtonSeven(){
    buttonSeven = m_stick->GetRawButton(9);
    return buttonSeven;
}

bool robotIO::getButtonEight(){
    buttonEight = m_stick->GetRawButton(10);
    return buttonEight;
}

int robotIO::getPOV(){
    dPad = m_stick->GetPOV(); 
    return dPad;
}

double robotIO::getLT(){
    lTrigger = m_stick->GetTwist();
    return lTrigger;
}

double robotIO::getRT(){
    rTrigger = m_stick->GetThrottle();
    return rTrigger;
}

double robotIO::getY(){
    yAxis = m_stick->GetY();
    return yAxis;
}

double robotIO::getX(){
    xAxis = m_stick->GetX();
    return xAxis;
}

double robotIO::getAxisFive(){
    axisFive = m_stick->GetRawAxis(5);
    return axisFive;
}

double robotIO::getAxisFour(){
    axisFour = m_stick->GetRawAxis(5);
    return axisFour;
}

double robotIO::getAxisFire(){
    axisFire = m_stick->GetRawAxis(4);
    return axisFire;
}

double robotIO::getAxisThree(){
    axisThree = m_stick->GetRawAxis(3);
    return axisThree;
}

bool robotIO::getButtonOnePartner(){
    buttonOnePartner = m_partner->GetRawButton(2);
        return buttonOnePartner;
    //EXAMPLE CASE FUNCTION FOR PS4 CONTROLLER USAGE
    /*switch(CONTROLLER_TYPE){
        case 1:
        buttonOnePartner = m_partner->GetRawButton(1);
        return buttonOnePartner;
        break;
        case 2:
        buttonOnePartner = m_partner->GetRawButton(3);
        return buttonOnePartner;
        break;
    }*/
}

bool robotIO::getButtonFourPartner(){
    buttonFourPartner = m_partner->GetRawButton(4);
    return buttonFourPartner;
}

bool robotIO::getButtonFivePartner(){
    buttonFivePartner = m_partner->GetRawButton(5);
    return buttonFivePartner;
}

bool robotIO::getButtonSixPartner(){
    buttonSixPartner = m_partner->GetRawButton(6);
    return buttonSixPartner;
}

int robotIO::getPOVPartner(){
    dPadPartner = m_partner->GetPOV(); 
    return dPadPartner;
}

double robotIO::getLTPartner(){
    lTriggerPartner = m_partner->GetTwist();
    return lTriggerPartner;
}

double robotIO::getRTPartner(){
    rTriggerPartner = m_partner->GetThrottle();
    return rTriggerPartner;
}

double robotIO::getYPartner(){
    YPartner = m_partner->GetY();
    return YPartner;
}

bool robotIO::getButtonTwoPartner(){
    buttonTwoPartner = m_partner->GetRawButton(1);
    return buttonTwoPartner;
}

bool robotIO::getButtonFourPressed(){
    buttonFourPressed = m_stick->GetRawButtonPressed(4);
    return buttonFourPressed;
}

bool robotIO::getButtonOnePressed(){
    buttonOnePressed = m_stick->GetRawButtonPressed(2);
    return buttonOnePressed;
}

bool robotIO::getButtonThreePartner(){
    buttonThreePartner = m_partner->GetRawButton(3);
    return buttonThreePartner;
}

double robotIO::getAxisFivePartner(){
    axisFivePartner = m_partner->GetRawAxis(5);
    return axisFivePartner;
}

bool robotIO::getButtonEightPartner(){
    buttonEightPartner = m_partner->GetRawButton(10);
    return buttonEightPartner;
}

bool robotIO::getButtonSevenPartner(){
    buttonSevenPartner = m_partner->GetRawButton(9);
    return buttonSevenPartner;
}

double robotIO::getAxisFourPartner(){
    axisFourPartner = m_partner->GetRawAxis(2);
    return axisFourPartner;
}

double robotIO::getAxisThreePartner(){
    axisThreePartner = m_partner->GetRawAxis(3);
    return axisThreePartner;
}