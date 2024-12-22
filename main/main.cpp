#include "Arduino.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "bno085.h"
#include "neo_m8n.h"

extern "C" void app_main(void)
{
    initArduino();

    neo_m8n_task_start();

    while(true)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}