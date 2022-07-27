#include "freertos/FreeRTOS.h"
#include "sdkconfig.h"
#include "freertos/task.h"

#include <driver/i2c.h>
#include <esp_log.h>
#include <esp_err.h>
#include "MPU6050.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "iot_servo.h"
#include "robot.hpp"
#include "esp_timer.h"

#define PIN_SDA 21
#define PIN_CLK 22

#define GPIO_MPU_INTERRUPT GPIO_NUM_33
#define ESP_INTR_FLAG_DEFAULT 0
#define GPIO_SERVO GPIO_NUM_32
#define TAG "example"

#include <iostream>
#include <Eigen/Dense>
 
using Eigen::MatrixXd;

MPU6050 mpu = MPU6050();
SemaphoreHandle_t xBinarySemaphoreMpuInterrupt = xSemaphoreCreateBinary();
SemaphoreHandle_t xMutexMpu = xSemaphoreCreateMutex();
 
extern "C" {
	void app_main(void);
}

void IRAM_ATTR mpu_isr_handler(void* arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(xBinarySemaphoreMpuInterrupt, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void Engine::initServo(){
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
    iot_servo_init(LEDC_LOW_SPEED_MODE, &servo_cfg);
}

void Engine::writeServo(int pos){
    iot_servo_write_angle(LEDC_LOW_SPEED_MODE, 0, pos);
}

float Sensor::angle(){
    xSemaphoreTake(xMutexMpu, portMAX_DELAY);
    // printf("core is %i ", xPortGetCoreID());
    uint8_t mpuIntStatus = mpu.getIntStatus();
    uint16_t fifoCount = mpu.getFIFOCount();
    uint8_t fifoBuffer[64]; // FIFO storage buffer
    float angle = 1000;

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // printf("resetting FIFO on the core %i \n", xPortGetCoreID());
        mpu.resetFIFO();
    } else if (mpuIntStatus & 0x02) {
        printf("calc angles on the core %i ", xPortGetCoreID());
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < PACKETSIZE) fifoCount = mpu.getFIFOCount();

        Quaternion q;
        VectorFloat gravity;
        float ypr[3];
        mpu.getFIFOBytes(fifoBuffer, PACKETSIZE);
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        angle = 180 * (1 - ypr[2] / M_PI) - 90;
        // printf("angles on the core %i ", xPortGetCoreID());
        printf("YAW: %3.1f, ", ypr[0] * 180/M_PI);
        printf("PITCH: %3.1f, ", ypr[1] * 180/M_PI);
        printf("ROLL: %3.1f \n", ypr[2] * 180/M_PI);
    }
    // vTaskDelay(1000/portTICK_PERIOD_MS);
    xSemaphoreGive(xMutexMpu);
    return angle;
}

Sensor sensor;
// MyServo servo;

void task_display(void*){
	while(1){
    xSemaphoreTake(xBinarySemaphoreMpuInterrupt, portMAX_DELAY);
    sensor.angle();
    // vTaskDelay(5/portTICK_PERIOD_MS);
	}
	vTaskDelete(NULL);
}

Engine engine(DynamicWithTimer(0, 0, 0), sensor);
esp_timer_handle_t oneshot_timer;

static void oneshot_timer_callback(void* arg)
{
    // int64_t time_since_boot = esp_timer_get_time();
    // ESP_LOGI(TAG, "One-shot timer called, time since boot: %lld us", time_since_boot);
    // ESP_ERROR_CHECK(esp_timer_start_once(oneshot_timer, 1000000));
    // time_since_boot = esp_timer_get_time();
    // ESP_LOGI(TAG, "Restarted periodic timer with 1s period, time since boot: %lld us",
    //         time_since_boot);
    engine.moveEngine();
}

int64_t DynamicWithTimer::getTime(){
    return esp_timer_get_time();
}

void DynamicWithTimer::setTimerPeriod(float timerPeriod){
    ESP_LOGI(TAG, "Started setTimerPeriod: %f", timerPeriod);
    ESP_ERROR_CHECK(esp_timer_start_once(oneshot_timer, timerPeriod * 1000000));
}

void DynamicWithTimer::initTimer(){
    ESP_LOGI(TAG, "Started initTimer: ");
    const esp_timer_create_args_t oneshot_timer_args = {
            .callback = &oneshot_timer_callback,
            .name = "one-shot"
    };
    ESP_ERROR_CHECK(esp_timer_create(&oneshot_timer_args, &oneshot_timer));
}

void app_main(void)
{
    ESP_LOGI(TAG, "Started app_main");
    engine.initTimer();
    engine.setAcc(1);
    // for(int posDegrees = 0; posDegrees <= 180; posDegrees++) {
    //     printf("angle: %i\n", posDegrees);
    //     // engine.servo.write(posDegrees);
    //     // iot_servo_write_angle(LEDC_LOW_SPEED_MODE, 0, posDegrees);
    //     usleep(20000);
    //     //   std::this_thread::sleep_for(20);
    //     // Serial.println(posDegrees);
    //     // delay(20);
    // }

    // MatrixXd m(2,2);
    // m(0,0) = 3;
    // m(1,0) = 2.5;
    // m(0,1) = -1;
    // m(1,1) = m(1,0) + m(0,1);
    // std::cout << m << std::endl;

	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = (gpio_num_t)PIN_SDA;
	conf.scl_io_num = (gpio_num_t)PIN_CLK;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = 400000;
    conf.clk_flags = 0;
	ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
	mpu.initialize();
	mpu.dmpInitialize();

    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);

	mpu.setDMPEnabled(true);

    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.pin_bit_mask = 1ULL<<GPIO_MPU_INTERRUPT;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);
    gpio_set_intr_type(GPIO_MPU_INTERRUPT, GPIO_INTR_ANYEDGE);
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(GPIO_MPU_INTERRUPT, mpu_isr_handler, (void*) GPIO_MPU_INTERRUPT);
    vTaskDelay(500/portTICK_PERIOD_MS);
    xTaskCreatePinnedToCore(&task_display, "disp_task", 8192, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(&task_display, "disp_task", 8192, NULL, 1, NULL, 1);
    // xTaskCreate(&task_display, "disp_task", 8192, NULL, 1, NULL);
    // xTaskCreate(&task_display, "disp_task", 8192, NULL, 1, NULL);
}
