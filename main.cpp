#include "MeSmartServo.h"
#include "MeSerial.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include<unistd.h>

int main(int argc, char* argv[])
{
    MeSmartServo servo(6,7);

    //Setup
    servo.begin(115200);
    sleep_ms(100);
    servo.assignDevIdRequest();
    sleep_ms(500);
    servo.setInitAngle(1);
    sleep_ms(100);

    while(true)
    {
        for(int i = 0; i < 50; i++)
        {
            servo.setPwmMove(1,i); //Device ID und Speed (max. Speed = 70)
            sleep_ms(50);
        }
        for(int i = 49; i > 0; i--)
        {
            servo.setPwmMove(1,i);
            sleep_ms(50);
        }
    }


    return 0;
}
