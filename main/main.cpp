#include "freertos/FreeRTOS.h"
#include "sdkconfig.h"
#include "freertos/task.h"

#include <driver/i2c.h>
#include <esp_log.h>
#include <esp_err.h>
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
    writeServo(90);
}

void Engine::writeServo(int pos){
    // ESP_LOGI("writeServo", "Servo angle: %i", pos);
    ESP_ERROR_CHECK(iot_servo_write_angle(LEDC_LOW_SPEED_MODE, 0, pos));
}

#define I2C_ADDRESS 0x1e
static char tag[] = "hmc5883l";

std::array<float, SENSOR_OUTPUT_DIM> Sensor::angles(){
    xSemaphoreTake(xMutexMpu, portMAX_DELAY);
    // printf("core is %i ", xPortGetCoreID());
	uint8_t data[6];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(cmd, 0x03, 1); // Data registers
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_READ, 1);
    i2c_master_read_byte(cmd, data,   (i2c_ack_type_t) 0);
    i2c_master_read_byte(cmd, data+1, (i2c_ack_type_t) 0);
    i2c_master_read_byte(cmd, data+2, (i2c_ack_type_t) 0);
    i2c_master_read_byte(cmd, data+3, (i2c_ack_type_t) 0);
    i2c_master_read_byte(cmd, data+4, (i2c_ack_type_t) 0);
    i2c_master_read_byte(cmd, data+5, (i2c_ack_type_t) 1);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    short x = data[0] << 8 | data[1];
    short z = (data[2] << 8 | data[3]) + 650;
    short y = data[4] << 8 | data[5];
    // int angle = atan2((double)z,(double)x) * (180 / 3.14159265) + 180; // angle in degrees
    // ESP_LOGI(tag, "x: %d, y: %d, z: %d", x, y, z);
    // vTaskDelay(1000/portTICK_PERIOD_MS);

    xSemaphoreGive(xMutexMpu);
    return std::array<float, SENSOR_OUTPUT_DIM>{(float)x, (float)y, (float)z};
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
    // ESP_ERROR_CHECK(esp_timer_start_once(oneshot_timer, 1000000));
    // time_since_boot = esp_timer_get_time();
    // ESP_LOGI(TAG, "Restarted periodic timer with 1s period, time since boot: %lld us",
    //         time_since_boot);
    // ESP_LOGI(TAG, "Starting moveEngine");
    solver.moveEngine();
}

int64_t getTime(){
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

void init_i2c()
{
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

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, 1);
	i2c_master_write_byte(cmd, 0x02, 1); // Mode register
	i2c_master_write_byte(cmd, 0x00, 1); // value 0
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, 1);
	i2c_master_write_byte(cmd, 0x01, 1); // Mode register
	i2c_master_write_byte(cmd, 0x20, 1); // value 0
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
}

void app_main(void)
{
    // Eigen::PartialPivLU<Matrix<MAT_SIZE, MAT_SIZE>> _partialSolver;
    // Matrix<MAT_SIZE, MAT_SIZE> _observations = Matrix<MAT_SIZE, MAT_SIZE>::Random();
    // _partialSolver.compute(_observations);
    // for (int i = 0; i < 1000; i++)
    // {
    //     std::cout << i << "\n";
    //     Vector<MAT_SIZE> sensData = Vector<MAT_SIZE>::Random();
    //     auto ratios = _partialSolver.solve(sensData);
    // }

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

    solver.initEngineTimer();
    
    // for (auto &a: {1, -1}){
    //     engine.setAcc(a);
    //     usleep(20000);
    // }

    init_i2c();
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.pin_bit_mask = 1ULL<<GPIO_MPU_INTERRUPT;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);
    gpio_set_intr_type(GPIO_MPU_INTERRUPT, GPIO_INTR_NEGEDGE);
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(GPIO_MPU_INTERRUPT, mpu_isr_handler, (void*) GPIO_MPU_INTERRUPT);
    vTaskDelay(500/portTICK_PERIOD_MS);
    // solver.test();
    xTaskCreatePinnedToCore(&task_display, "disp_task", 8192, NULL, tskIDLE_PRIORITY, NULL, 0);
    // xTaskCreatePinnedToCore(&task_display, "disp_task", 8192, NULL, tskIDLE_PRIORITY, NULL, 1);

    // xTaskCreate(&task_display, "disp_task", 8192, NULL, 1, NULL);
    // xTaskCreate(&task_display, "disp_task", 8192, NULL, 1, NULL);
}
