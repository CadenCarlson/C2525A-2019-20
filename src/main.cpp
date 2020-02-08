#include <Arduino.h>
#include <Wire.h>

#include <YETI_YUKON.h>
#include <ROBOT.h>

#include "secrets.h"

YETI_YUKON Yukon("HS2020", "cb7dcd8e-99b7-4849-8bc9-61abb1bee63e");
ROBOT Robot(Yukon);

SemaphoreHandle_t I2Cbaton;

void WriteRobotTask(void *pvParameters);
void USBOIGYROTask(void *pvParameters);
void GeneralTask(void *pvParameters);
void OLEDTask(void *pvParameters);

void AutonTask(void *pvParameters);  
void DriveTask(void *pvParameters);
void LiftTask(void *pvParameters);
void ClawTask(void *pvParameters);
void WatchdogTask(void *pvParameters);

void setup()
{
    Serial.begin(115200);

    Wire.begin(4, 15, 400000);

    //To Set up wifi
    // You need to modify the secrets.h file yourself, and put the following into it:
    // const char* ssid[3] = {"yourwifi", "secondwifi", "3rd..."};  //  your network SSID (name)
    // const char* pass[3] = {"yourpwd", "secondpwd", "3rdpwd..."};       // your network password
    for (int i = 0; i < (sizeof(ssid) / sizeof(ssid[0])); i++)
    {
        Yukon.wifiMulti.addAP(ssid[i], pass[i]);
    }

    Yukon.Setup();
    Robot.Setup();
    
    I2Cbaton = xSemaphoreCreateMutex();

    Serial.print("MainLoop: Executing on core ");
    Serial.println(xPortGetCoreID());

    //CORE 1
    xTaskCreatePinnedToCore(WriteRobotTask, "WriteRobotTask", 10000, NULL, 3, NULL, 1); // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.

    xTaskCreatePinnedToCore(USBOIGYROTask, "USBOIGYROTask", 10000, NULL, 2, NULL, 1);

    xTaskCreatePinnedToCore(GeneralTask, "GeneralTask", 10000, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(OLEDTask, "OLEDTask", 10000, NULL, 2, NULL, 1);

    //CORE 0
    xTaskCreatePinnedToCore(WatchdogTask, "WatchdogTask", 10000, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(AutonTask, "AutonTask", 10000, NULL, 2, NULL, 0);

    xTaskCreatePinnedToCore(DriveTask, "DriveTask", 10000, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(LiftTask, "LiftTask", 10000, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(ClawTask, "ClawTask", 10000, NULL, 2, NULL, 0);
    
}

void loop()
{
    // Empty. Things are done in Tasks.
    delay(1000);
}

//CORE 1
void WriteRobotTask(void *pvParameters)
{
    Serial.print("WriteRobotTask: Executing on core ");
    Serial.println(xPortGetCoreID());

    while (true)
    {
        xSemaphoreTake(I2Cbaton, portMAX_DELAY);
        Robot.WriteRobot();
        xSemaphoreGive(I2Cbaton);
        delay(10);
    }
}
void USBOIGYROTask(void *pvParameters)
{
    Serial.print("USBOIGYROTask: Executing on core ");
    Serial.println(xPortGetCoreID());

    delay(1);
    yield();
    while (true)
    {
        Yukon.PatTheDog();
        xSemaphoreTake(I2Cbaton, portMAX_DELAY);
        Robot.USBOIGYRO();
        xSemaphoreGive(I2Cbaton);
        delay(2);
    }
}
void GeneralTask(void *pvParameters)
{
    Serial.print("GeneralTask: Executing on core ");
    Serial.println(xPortGetCoreID());

    while (true)
    {
        xSemaphoreTake(I2Cbaton, portMAX_DELAY);
        Yukon.GeneralTask();
        Robot.GeneralTask();
        xSemaphoreGive(I2Cbaton);
        delay(20);
    }
}
void OLEDTask(void *pvParameters)
{
    Serial.print("OLEDTask: Executing on core ");
    Serial.println(xPortGetCoreID());

    while (true)
    {
        //Updating the OLED ties up the serial line for longer than we want
        //Only update the OLED when the robot is idle.
        //TODO: Speed up the OLED lib.
      
            if(Robot.State.AllSystemsIdle())
            {
                xSemaphoreTake(I2Cbaton, portMAX_DELAY);
                Robot.OLEDLoop();
                xSemaphoreGive(I2Cbaton);
            }
        
        delay(250);
    }
}

//CORE 0
void WatchdogTask(void *pvParameters)
{
    Serial.print("WatchdogTask: Executing on core ");
    Serial.println(xPortGetCoreID());

        Yukon.EnableWatchdog();
        while (true)
        {
            Yukon.WatchdogLoop();
            delay(100);
        }
    Yukon.EnableWatchdog();
    while (true)
    {
        Yukon.WatchdogLoop();
        delay(100);
    }
}
void AutonTask(void *pvParameters)
{
    Serial.print("AutonTask: Executing on core ");
    Serial.println(xPortGetCoreID());
    Robot.Auton.Setup();

    while (true)
    {
        Robot.Auton.Loop();
    }
}

void DriveTask(void *pvParameters)
{
    Serial.print("DriveTask: Executing on core ");
    Serial.println(xPortGetCoreID());

        while (true)
        {
            Robot.Drive.Loop();
        }
    }
    
void LiftTask(void *pvParameters)
{
    Serial.print("LiftTask: Executing on core ");
    Serial.println(xPortGetCoreID());

        while (true)
        {
            Robot.Lift.Loop();
        }
    }
    
void ClawTask(void *pvParameters)
{
    Serial.print("ClawTask: Executing on core ");
    Serial.println(xPortGetCoreID());

        while (true)
        {
            Robot.Claw.Loop();
        }
    }
    
