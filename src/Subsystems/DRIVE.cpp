
#include "DRIVE.h"
#include "ROBOT.h"

DRIVE::DRIVE(ROBOT &refRobot) : Robot(refRobot)
{
}

void DRIVE::Loop()
{
    //Only Run Command Code When Valid
    if (!CmdTimedOut() && CmdIsRunning())
    {
        if (CmdName() == "ForAsync")
        {
            Robot.State.DriveLeftSpeed = _CmdDriveLeftSpeed;
            Robot.State.DriveRightSpeed = _CmdDriveRightSpeed;
            while (CmdUpdatePercent(millis()))
            {
                delay(20);
            }
        }
    }
    Robot.State.DriveLeftSpeed = _OIDriveLeftSpeed;
    Robot.State.DriveRightSpeed = _OIDriveRightSpeed;

    delay(20);

    long nextLoopTime = millis()+20;
    //Only Run Command Code When Valid
    if (!CmdTimedOut() && CmdIsRunning())
    {
        if (CmdName() == "ForAsync")
        {
            Robot.State.DriveLeftSpeed = _CmdDriveLeftSpeed;
            Robot.State.DriveRightSpeed = _CmdDriveRightSpeed;
            long nextTime = millis();

            while (CmdUpdatePercent(millis()))
            {
                delay(20);
                nextTime = millis() + 25;

                int16_t tmpDriveLeftSpeed = BurnoutControl(CalcControlLoop(_CmdDriveLeftSpeed), Robot.State.DriveLeftSpeed);
                int16_t tmpDriveRightSpeed = BurnoutControl(CalcControlLoop(_CmdDriveRightSpeed), Robot.State.DriveRightSpeed);

                
                if (_HeadingLockEnabled)
                {
                    // float HeadingError = Robot.Yukon.GYRO.Heading() - _HeadingLockValue;
                    // float ErrMult = map(abs(HeadingError*100), 0, 4500, 0, 100) / 100;
                    // if(ErrMult < 0)
                    //     ErrMult = 0;
                    // if(ErrMult > 1)
                    //     ErrMult = 1;

                    // if(_CmdDriveRightSpeed > 0)
                    // {
                    //     //Moving Forward
                    //     if (HeadingError < 0)
                    //     {
                    //         //remove speed from left side
                    //         tmpDriveLeftSpeed *= ErrMult; //abs(HeadingError) * 10;
                    //     }
                    //     else if (HeadingError > 0)
                    //     {
                    //         //remove speed from right side
                    //         tmpDriveRightSpeed *= ErrMult; //-= abs(HeadingError) * 10;
                    //     }
                    // }
                    // else
                    // {
                    //     //Moving Backward
                    //     if (HeadingError < 0)
                    //     {
                    //         //remove speed from right side
                    //         tmpDriveRightSpeed *= ErrMult; //abs(HeadingError) * 10;
                    //     }
                    //     else if (HeadingError > 0)
                    //     {
                    //         //remove speed from left side
                    //         tmpDriveLeftSpeed *= ErrMult; //-= abs(HeadingError) * 10;
                    //     }
                    // }
                }
                

                Robot.State.DriveLeftSpeed = tmpDriveLeftSpeed;
                Robot.State.DriveRightSpeed = tmpDriveRightSpeed;

                while (millis() < nextTime)
                {
                    delay(1);
                }
            }
        }
                    
        else if (CmdName() == "TurnAsync")
        {
            long nextTime = millis();

            while (CmdUpdatePercent(Robot.Yukon.GYRO.Heading()))
            {
                nextTime = millis() + 25;

                Robot.State.DriveLeftSpeed = BurnoutControl(CalcControlLoop(_CmdDriveLeftSpeed), Robot.State.DriveLeftSpeed);
                Robot.State.DriveRightSpeed = BurnoutControl(CalcControlLoop(_CmdDriveRightSpeed), Robot.State.DriveRightSpeed);
                
                while (millis() < nextTime)
                {
                    delay(1);
                }
            }
        }
        
                
        _HeadingLockEnabled = false;
        
        _SlowDown = 0;
    }

    int16_t calculatedDriveLeftSpeed = _OIDriveLeftSpeed;
    int16_t calculatedDriveRightSpeed = _OIDriveRightSpeed;
    
    if (BurnoutControlEnabled)
    {
        calculatedDriveLeftSpeed = BurnoutControl(calculatedDriveLeftSpeed, Robot.State.DriveLeftSpeed);
        calculatedDriveRightSpeed = BurnoutControl(calculatedDriveRightSpeed, Robot.State.DriveRightSpeed);
        
    }
    Robot.State.DriveLeftSpeed = calculatedDriveLeftSpeed;
    Robot.State.DriveRightSpeed = calculatedDriveRightSpeed;

    long untilNextRun = nextLoopTime - millis();
    if(untilNextRun > 0)
        delay(untilNextRun);
}

void DRIVE::OISetSpeed(int16_t DriveLeftSpeed,int16_t DriveRightSpeed)
{
    _OIDriveLeftSpeed = DriveLeftSpeed;
    _OIDriveRightSpeed = DriveRightSpeed;
}

bool DRIVE::ForAsync(long durationMS, int16_t DriveLeftSpeed,int16_t DriveRightSpeed, float SlowDown, uint8_t HoldUntilPercent)
{
     _SlowDown = SlowDown;

    _CmdDriveLeftSpeed = DriveLeftSpeed;
    _CmdDriveRightSpeed = DriveRightSpeed;
    
    bool retVal = CmdStart("ForAsync", millis(), millis() + durationMS, durationMS + 500);

    while (CmdPercentComplete() < HoldUntilPercent && retVal)
    {
        delay(20);
    }

    return retVal;
}



    
bool DRIVE::TurnAsync(float degrees, bool AbsoluteSetpoint, int16_t Speed1, int16_t Speed2, long Timeout, float SlowDown, uint8_t HoldPercent)
{
    _SlowDown = SlowDown;

    int16_t StartPoint = Robot.Yukon.GYRO.Heading();
    int16_t SetPoint = ((AbsoluteSetpoint ? 0 : Robot.Yukon.GYRO.Heading()) + degrees);

    if(SetPoint > StartPoint)
    {
        //Turn CCW
        _CmdDriveRightSpeed = Speed1 > Speed2 ? Speed1 : Speed2; //Faster Speed
        _CmdDriveLeftSpeed = Speed1 < Speed2 ? Speed1 : Speed2; //Slower Speed
    }
    else
    {
        //Turn CW
        _CmdDriveRightSpeed = Speed1 < Speed2 ? Speed1 : Speed2;//Slower Speed
        _CmdDriveLeftSpeed = Speed1 > Speed2 ? Speed1 : Speed2;//Faster Speed
    }


    bool retVal = CmdStart("TurnAsync", StartPoint, SetPoint, Timeout);

    while (CmdPercentComplete() < HoldPercent && retVal)
    {
        delay(20);
    }

    return retVal;
}


//bool DRIVE::ToAsyncWithHeadingLock(int32_t Position, bool AbsoluteSetpoint, int16_t DriveSpeed, long Timeout, float SlowDown, uint8_t HoldUntilPercent, float HeadingLockValue)
//{
//    _HeadingLockEnabled = true;
//    _HeadingLockValue = (HeadingLockValue == -123456789 ? Robot.Yukon.GYRO.Heading() : HeadingLockValue);

//    bool retVal = ToAsync(Position, AbsoluteSetpoint, DriveSpeed, DriveSpeed, Timeout, SlowDown, HoldUntilPercent);

//    return retVal;
//}

bool DRIVE::ForAsyncWithHeadingLock(long durationMS, int16_t DriveSpeed, float SlowDown, uint8_t HoldPercent, float HeadingLockValue)
{
    _HeadingLockEnabled = true;
    _HeadingLockValue = (HeadingLockValue == -123456789 ? Robot.Yukon.GYRO.Heading() : HeadingLockValue);
    bool retVal = ForAsync(durationMS, DriveSpeed, DriveSpeed, SlowDown, HoldPercent);

    return retVal;
}    

