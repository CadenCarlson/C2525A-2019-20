
#pragma once

#include <Arduino.h>
#include <YUKON_COMMAND.h>

class ROBOT;

class DRIVE : public YUKON_COMMAND
{

    public:
        DRIVE(ROBOT &refRobot);
        ROBOT &Robot;

        void Loop();

        void OISetSpeed(int16_t DriveLeftSpeed,int16_t DriveRightSpeed);

        bool ForAsync(long durationMS, int16_t DriveLeftSpeed,int16_t DriveRightSpeed, float SlowDown, uint8_t HoldPercent);

        //Each Sensor
        

            
            //Gyro Functions
            bool TurnAsync(float degrees, bool AbsoluteSetpoint, int16_t Speed1, int16_t Speed2, long Timeout, float SlowDown, uint8_t ifldPercent);
            bool ForAsyncWithHeadingLock(long durationMS, int16_t DriveSpeed, float SlowDown, uint8_t HoldPercent, float HeadingLockValue = -123456789);
            bool ToAsyncWithHeadingLock(int32_t Position, bool AbsoluteSetpoint, int16_t DriveSpeed, long Timeout, float SlowDown, uint8_t HoldUntilPercent, float HeadingLockValue = -123456789);
            

    private:
        int16_t _CmdDriveLeftSpeed = 0;
        int16_t _OIDriveLeftSpeed = 0;
        int16_t _CmdDriveRightSpeed = 0;
        int16_t _OIDriveRightSpeed = 0;

            
             //HasGyro
             bool _HeadingLockEnabled = false;
             float _HeadingLockValue = 0;
            

};
