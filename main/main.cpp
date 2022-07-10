#include "freertos/FreeRTOS.h"
#include "sdkconfig.h"
#include "freertos/task.h"
#include <iostream>
#include <Eigen/Dense>
 
using Eigen::MatrixXd;

extern "C" {
	void app_main(void);
}

extern void task_initI2C(void*);
extern void task_display(void*);

void app_main(void)
{
    // MatrixXd m(2,2);
    // m(0,0) = 3;
    // m(1,0) = 2.5;
    // m(0,1) = -1;
    // m(1,1) = m(1,0) + m(0,1);
    // std::cout << m << std::endl;

    xTaskCreate(&task_initI2C, "mpu_task", 2048, NULL, 5, NULL);
    // task_initI2C(NULL);
    vTaskDelay(500/portTICK_PERIOD_MS);
    // task_display(NULL);
    xTaskCreate(&task_display, "disp_task", 8192, NULL, 5, NULL);
}
