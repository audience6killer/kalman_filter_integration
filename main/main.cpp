#include "Arduino.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "bno085.h"
#include "neo_m8n.h"
#include "bno085_test.h"

extern "C" void app_main(void)
{
    initArduino();

    test_bno085_task_start();

    while(true)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}