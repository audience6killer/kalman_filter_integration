#include "Arduino.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "bno085.h"
#include "neo_m8n_test.h"
#include "neo_m8n.h"
#include "bno085_test.h"
#include "kalman_test.h"

extern "C" void app_main(void)
{
    initArduino();

    // Serial.begin(115200);

    imu_start_task();

    neo_m8n_task_start();

    test_kalman_task_start();

    //test_bno085_task_start();

    // test_neo_m8n_task_start();

    while(true)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}