#include "STATE.h"

STATE::STATE()
{
}

bool STATE::AllSystemsIdle()
{
    return DriveLeftSpeed == 0 && DriveRightSpeed == 0 && LiftMotorSpeed == 0 && ClawMotorSpeed == 0;
}