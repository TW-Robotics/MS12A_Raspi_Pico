#include "MeSmartServo.h"
#include "MeSerial.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include <unistd.h>


void blink();

int main(int argc, char* argv[])
{
    //LED
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    


    gpio_put(LED_PIN, 1);
    sleep_ms(100);
    gpio_put(LED_PIN, 0);
    sleep_ms(100);
    gpio_put(LED_PIN, 1);
    sleep_ms(100);
    gpio_put(LED_PIN, 0);

    
    sleep_ms(1000);
    MeSmartServo servo(6,7);
    sleep_ms(1000);
    //Setup
    servo.begin(115200);
    
    gpio_put(LED_PIN, 1);
    sleep_ms(100);
    gpio_put(LED_PIN, 0);
    sleep_ms(100);
    gpio_put(LED_PIN, 1);
    sleep_ms(100);
    gpio_put(LED_PIN, 0);
    sleep_ms(100);
    
    if(servo.assignDevIdRequest())
    {    
    gpio_put(LED_PIN, 1);
    sleep_ms(100);
    gpio_put(LED_PIN, 0);
    sleep_ms(100);
    gpio_put(LED_PIN, 1);
    sleep_ms(100);
    gpio_put(LED_PIN, 0);
    sleep_ms(500);
    }
    else
    {
        gpio_put(LED_PIN, 1);
        sleep_ms(100);
    }
    
    servo.setInitAngle(1);
    
    gpio_put(LED_PIN, 1);
    sleep_ms(100);
    gpio_put(LED_PIN, 0);
    sleep_ms(100);
    gpio_put(LED_PIN, 1);
    sleep_ms(100);
    gpio_put(LED_PIN, 0);
    sleep_ms(100);


    while(true)
    {
        gpio_put(LED_PIN, 1);
        for(int i = 0; i < 50; i++)
        {
            servo.setPwmMove(1,i); //Device ID und Speed (max. Speed = 70)
            sleep_ms(50);
        }
        gpio_put(LED_PIN, 0);
        for(int i = 49; i > 0; i--)
        {
            servo.setPwmMove(1,i);
            sleep_ms(50);
        }
        
    }


    return 0;
}



