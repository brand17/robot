#include "freertos/FreeRTOS.h"
#include "sdkconfig.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include <esp_log.h>
#include <esp_err.h>
#include "iot_servo.h"
#include "robot.hpp"
#include "esp_timer.h"

#define GPIO_TILT GPIO_NUM_36
#define GPIO_SERVO GPIO_NUM_32
#define TAG "example"

#include <iostream>

SemaphoreHandle_t xBinarySemaphoreMpuInterrupt = xSemaphoreCreateBinary();
SemaphoreHandle_t xMutexMpu = xSemaphoreCreateMutex();
 
extern "C" {
	void app_main(void);
}

void Engine::initServo(){
    // ESP_LOGI("initServo", "started");
    servo_config_t servo_cfg = {
        .max_angle = 180,
        .min_width_us = 500,
        .max_width_us = 2500,
        .freq = 50,
        .timer_number = LEDC_TIMER_0,
        .channels = {
            .servo_pin = {
                GPIO_SERVO,
            },
            .ch = {
                LEDC_CHANNEL_0,
            },
        },
        .channel_number = 1,
    } ;
    ESP_ERROR_CHECK(iot_servo_init(LEDC_LOW_SPEED_MODE, &servo_cfg));
    writeServo(90);
}

void Engine::writeServo(int pos){
    // ESP_LOGI("writeServo", "Servo angle: %i", pos);
    ESP_ERROR_CHECK(iot_servo_write_angle(LEDC_LOW_SPEED_MODE, 0, pos));
}

#include "driver/adc.h"
#include "esp_adc_cal.h"
static esp_adc_cal_characteristics_t adc1_chars;

float Sensor::angle(){
    xSemaphoreTake(xMutexMpu, portMAX_DELAY);
    // printf("core is %i ", xPortGetCoreID());

    auto voltage = esp_adc_cal_raw_to_voltage(adc1_get_raw(ADC1_CHANNEL_0), &adc1_chars);
    ESP_LOGI(TAG, "ADC1_CHANNEL_0: %d mV", voltage);

    // vTaskDelay(1000/portTICK_PERIOD_MS);
    xSemaphoreGive(xMutexMpu);
    return (float)voltage - 1300.f;
}

Solver solver;

void task_display(void*){
	while(1){
        xSemaphoreTake(xBinarySemaphoreMpuInterrupt, portMAX_DELAY);
        // int64_t time_since_boot = esp_timer_get_time();
        // ESP_LOGI("Sensor changed", "One-shot timer called, time since boot: %lld us", time_since_boot);
        solver.changeEngineAcc();
    }
	vTaskDelete(NULL);
}

esp_timer_handle_t oneshot_timer;

static void oneshot_timer_callback(void* arg)
{
    // int64_t time_since_boot = esp_timer_get_time();
    // ESP_LOGI("oneshot_timer_callback", "One-shot timer called, time since boot: %lld us", time_since_boot);
    // ESP_LOGI(TAG, "Starting moveEngine");
    solver.moveEngine();
}

int64_t Dynamics::getTime(){
    return esp_timer_get_time();
}

void DynamicWithTimer::setTimerPeriod(float timerPeriod){
    if (timerPeriod != FLT_MAX)
    {
        // ESP_LOGI("setTimerPeriod", "Started setTimerPeriod: %f", timerPeriod);
        ESP_ERROR_CHECK(esp_timer_start_once(oneshot_timer, timerPeriod * 1000000));
    }
}

void DynamicWithTimer::stopTimer(){
    // ESP_LOGI(TAG, "Started stopTimer");
    if (esp_timer_is_active(oneshot_timer))
        // ESP_LOGI(TAG, "Stopping Timer");
        esp_timer_stop(oneshot_timer);
}

void DynamicWithTimer::initTimer(){
    // ESP_LOGI(TAG, "Started initTimer: ");
    const esp_timer_create_args_t oneshot_timer_args = {
            .callback = &oneshot_timer_callback,
            .name = "one-shot"
    };
    ESP_ERROR_CHECK(esp_timer_create(&oneshot_timer_args, &oneshot_timer));
    _prevTime = getTime();
}

static void periodic_timer_callback(void *arg)
{
    // int64_t time_since_boot = esp_timer_get_time();
    // ESP_LOGI(TAG, "Periodic timer called, time since boot: %lld us", time_since_boot);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(xBinarySemaphoreMpuInterrupt, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void app_main(void)
{
    auto adc_width = (adc_bits_width_t)ADC_WIDTH_BIT_DEFAULT;
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, adc_width, 0, &adc1_chars);
    ESP_ERROR_CHECK(adc1_config_width(adc_width));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11));

    const esp_timer_create_args_t periodic_timer_args = {
            .callback = &periodic_timer_callback,
            .name = "periodic"
    };

    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));

    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 10000));

    // ESP_LOGI(TAG, "Started app_main");
    // while (true)
    // {
    //     ESP_ERROR_CHECK(iot_servo_write_angle(LEDC_LOW_SPEED_MODE, 0, 0)); usleep(5000000);
    //     ESP_ERROR_CHECK(iot_servo_write_angle(LEDC_LOW_SPEED_MODE, 0, 90)); usleep(5000000);
    //     ESP_ERROR_CHECK(iot_servo_write_angle(LEDC_LOW_SPEED_MODE, 0, 180)); usleep(5000000);
    //     ESP_ERROR_CHECK(iot_servo_write_angle(LEDC_LOW_SPEED_MODE, 0, 90)); usleep(5000000);
    // }

    // for(int posDegrees = 0; posDegrees <= 180; posDegrees++) {
    //     printf("angle: %i\n", posDegrees);
    //     ESP_ERROR_CHECK(iot_servo_write_angle(LEDC_LOW_SPEED_MODE, 0, posDegrees));
    //     usleep(20000);
    // }

    // engine.initTimer();
    // engine.setAcc(1);
    solver.initEngineTimer();
    
    // usleep(5000000);
    // ESP_LOGI(TAG, "Reverse");
    // solver.setEngineAcc(-1);
    // usleep(10000000);
    // esp_timer_stop(oneshot_timer);

    // for (auto &a: {1, -1}){
    //     engine.setAcc(a);
    //     usleep(20000);
    // }

    // solver.test();
    xTaskCreatePinnedToCore(&task_display, "disp_task", 8192, NULL, 1, NULL, 0);
    // xTaskCreatePinnedToCore(&task_display, "disp_task", 8192, NULL, 1, NULL, 1);

    // xTaskCreate(&task_display, "disp_task", 8192, NULL, 1, NULL);
    // xTaskCreate(&task_display, "disp_task", 8192, NULL, 1, NULL);
}
