#ifndef ROBOT_h
#define ROBOT_h

#include <YETI_YUKON.h>
#include <STATE.h>

#include <XBOXRECV.h>

#include <YUKON_DBH12V.h>
#include <YUKON_RC_ESC.h>


#include "Subsystems\Drive.h"
#include "Subsystems\Lift.h"
#include "Subsystems\Claw.h"

#include "Autonomous\AUTONOMOUS.h"

class ROBOT
{
  public:
    ROBOT(YETI_YUKON &rYukon);

    void Setup();
    void GeneralTask();

    void WriteRobot();
    void USBOIGYRO();
    void OLEDLoop();

    YETI_YUKON &Yukon;

    STATE State;

    //Motors
    YUKON_DBH12V DriveLeft;
    YUKON_DBH12V DriveRight;
    YUKON_DBH12V LiftMotor1;
    YUKON_DBH12V LiftMotor2;
    YUKON_RC_ESC ClawMotor2;
    YUKON_RC_ESC ClawMotor1;

    //Sensors

    //Subsystems
    DRIVE Drive;
    LIFT Lift;
    CLAW Claw;
    AUTONOMOUS Auton;

    //USB Items
    USB Usb;
    
    XBOXRECV Xbox;
    
    
   

    //PIN Declarations
    static const int _DriveLeftPWM1 = 2;
    static const int _DriveLeftPWM2 = 3;
    static const bool _DriveLeftReversed = true;
        
    static const int _DriveRightPWM1 = 0;
    static const int _DriveRightPWM2 = 1;
    static const bool _DriveRightReversed = false;
        
    static const int _LiftMotor1PWM1 = 4;
    static const int _LiftMotor1PWM2 = 5;
    static const bool _LiftMotor1Reversed = false;
    
    static const int _LiftMotor2PWM1 = 6;
    static const int _LiftMotor2PWM2 = 7;
    static const bool _LiftMotor2Reversed = false;
        
    static const int _ClawMotor1PWM1 = 9;
    static const bool _ClawMotor1Reversed = true;
        
    static const int _ClawMotor2PWM1 = 8;
    static const bool _ClawMotor2Reversed = false;
    
    int ClawPosition = 0;
    static const int _CM1P0 = 0; //0
    static const int _CM1P1 = 258; //64
    static const int _CM1P2 = 168; //128
    static const int _CM2P0 = 0; //0
    static const int _CM2P1 = 100; //64
    static const int _CM2P2 = 160; //128

    static const uint8_t _Button0 = 0;
    static const uint8_t _LEDBuiltIn = 25;
    static const uint8_t _AutonLightSensor = 0;
    
    static const uint16_t _AutonLightSensorThreshold = 150; //Value 0 - 1024

  private:
    uint16_t LightSensorVal = 0;
    uint16_t MaxLightSensorVal = 0;
};

#endif