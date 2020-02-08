
#include "LIFT.h"
#include "ROBOT.h"

LIFT::LIFT(ROBOT &refRobot) : Robot(refRobot)
{
}

void LIFT::Loop()
{
    //Only Run Command Code When Valid
    if (!CmdTimedOut() && CmdIsRunning())
    {
        if (CmdName() == "ForAsync")
        {
            Robot.State.LiftMotorSpeed = _CmdLiftMotorSpeed;
            while (CmdUpdatePercent(millis()))
            {
                delay(20);
            }
        }
    }
    Robot.State.LiftMotorSpeed = _OILiftMotorSpeed;

    delay(20);

    long nextLoopTime = millis()+20;
    //Only Run Command Code When Valid
    if (!CmdTimedOut() && CmdIsRunning())
    {
        if (CmdName() == "ForAsync")
        {
            Robot.State.LiftMotorSpeed = _CmdLiftMotorSpeed;
            long nextTime = millis();

            while (CmdUpdatePercent(millis()))
            {
                delay(20);
                nextTime = millis() + 25;

                int16_t tmpLiftMotorSpeed = BurnoutControl(CalcControlLoop(_CmdLiftMotorSpeed), Robot.State.LiftMotorSpeed);


                Robot.State.LiftMotorSpeed = tmpLiftMotorSpeed;

                while (millis() < nextTime)
                {
                    delay(1);
                }
            }
        }
                        _SlowDown = 0;
    }

    int16_t calculatedLiftMotorSpeed = _OILiftMotorSpeed;
    
    if (BurnoutControlEnabled)
    {
        calculatedLiftMotorSpeed = BurnoutControl(calculatedLiftMotorSpeed, Robot.State.LiftMotorSpeed);
        
    }
    Robot.State.LiftMotorSpeed = calculatedLiftMotorSpeed;

    long untilNextRun = nextLoopTime - millis();
    if(untilNextRun > 0)
        delay(untilNextRun);
}

void LIFT::OISetSpeed(int16_t LiftMotorSpeed)
{
    _OILiftMotorSpeed = LiftMotorSpeed;
}

bool LIFT::ForAsync(long durationMS, int16_t LiftMotorSpeed, float SlowDown, uint8_t HoldUntilPercent)
{
     _SlowDown = SlowDown;

    _CmdLiftMotorSpeed = LiftMotorSpeed;
    
    bool retVal = CmdStart("ForAsync", millis(), millis() + durationMS, durationMS + 500);

    while (CmdPercentComplete() < HoldUntilPercent && retVal)
    {
        delay(20);
    }

    return retVal;
}



