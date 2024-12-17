#include "Arduino.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "bno085.h"
#include "TinyGPS++.h"

extern "C" void app_main(void)
{
    initArduino();

    while(true)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}