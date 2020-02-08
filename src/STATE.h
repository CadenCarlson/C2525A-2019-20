#ifndef STATE_h
#define STATE_h

#include <Arduino.h>

class STATE
{

  public:
	STATE();             
    
    bool AllSystemsIdle();

    volatile int16_t DriveLeftSpeed = 0;
    volatile int16_t DriveRightSpeed = 0;
    volatile int16_t LiftMotorSpeed = 0;
    volatile int16_t ClawMotorSpeed = 0;
    
  volatile bool AutonLightSensorActive = false;
  volatile int16_t AutonLightSensorState = 0; 

  private:

};

#endif