
#include "CLAW.h"
#include "ROBOT.h"

CLAW::CLAW(ROBOT &refRobot) : Robot(refRobot)
{
}

void CLAW::Loop()
{
    //Only Run Command Code When Valid
    if (!CmdTimedOut() && CmdIsRunning())
    {
        if (CmdName() == "ForAsync")
        {
            Robot.State.ClawMotorSpeed = _CmdClawMotorSpeed;
            while (CmdUpdatePercent(millis()))
            {
                delay(20);
            }
        }
    }
    Robot.State.ClawMotorSpeed = _OIClawMotorSpeed;

    delay(20);

    long nextLoopTime = millis()+20;
    //Only Run Command Code When Valid
    if (!CmdTimedOut() && CmdIsRunning())
    {
        if (CmdName() == "ForAsync")
        {
            Robot.State.ClawMotorSpeed = _CmdClawMotorSpeed;
            long nextTime = millis();

            while (CmdUpdatePercent(millis()))
            {
                delay(20);
                nextTime = millis() + 25;

                int16_t tmpClawMotorSpeed = BurnoutControl(CalcControlLoop(_CmdClawMotorSpeed), Robot.State.ClawMotorSpeed);


                Robot.State.ClawMotorSpeed = tmpClawMotorSpeed;

                while (millis() < nextTime)
                {
                    delay(1);
                }
            }
        }
                        _SlowDown = 0;
    }

    int16_t calculatedClawMotorSpeed = _OIClawMotorSpeed;
    
    if (BurnoutControlEnabled)
    {
        calculatedClawMotorSpeed = BurnoutControl(calculatedClawMotorSpeed, Robot.State.ClawMotorSpeed);
        
    }
    Robot.State.ClawMotorSpeed = calculatedClawMotorSpeed;

    long untilNextRun = nextLoopTime - millis();
    if(untilNextRun > 0)
        delay(untilNextRun);
}

void CLAW::OISetSpeed(int16_t ClawMotorSpeed)
{
    _OIClawMotorSpeed = ClawMotorSpeed;
}

bool CLAW::ForAsync(long durationMS, int16_t ClawMotorSpeed, float SlowDown, uint8_t HoldUntilPercent)
{
     _SlowDown = SlowDown;

    _CmdClawMotorSpeed = ClawMotorSpeed;
    
    bool retVal = CmdStart("ForAsync", millis(), millis() + durationMS, durationMS + 500);

    while (CmdPercentComplete() < HoldUntilPercent && retVal)
    {
        delay(20);
    }

    return retVal;
}



