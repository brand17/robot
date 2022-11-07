#include "freertos/FreeRTOS.h"
#include "sdkconfig.h"
#include "freertos/task.h"

#include <driver/i2c.h>
#include <esp_log.h>
#include <esp_err.h>
#include "robot.hpp"
#include "esp_timer.h"
#include "driver/mcpwm.h"

#define PIN_SDA_GEO 21
#define PIN_CLK_GEO 22
// #define GPIO_GY271_INTERRUPT GPIO_NUM_36
#define ESP_INTR_FLAG_DEFAULT 0

#include <iostream>

// SemaphoreHandle_t xBinarySemaphoreGY271Interrupt = xSemaphoreCreateBinary();
SemaphoreHandle_t xMutexMpu = xSemaphoreCreateMutex();
 
extern "C" {
	void app_main(void);
}

// void IRAM_ATTR gy271_isr_handler(void* arg)
// {
//     BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//     xSemaphoreGiveFromISR(xBinarySemaphoreGY271Interrupt, &xHigherPriorityTaskWoken);
//     portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
// }

#define MOTOR_CTRL_MCPWM_UNIT   MCPWM_UNIT_0
#define MOTOR_CTRL_MCPWM_TIMER  MCPWM_TIMER_0
#define GPIO_PWM0A_OUT 19
#define GPIO_PWM0B_OUT 23

void brushed_motor_set_duty(float duty_cycle)
{
    if (duty_cycle > 0) {
        mcpwm_set_signal_low(MOTOR_CTRL_MCPWM_UNIT, MOTOR_CTRL_MCPWM_TIMER, MCPWM_OPR_A);
        mcpwm_set_duty(MOTOR_CTRL_MCPWM_UNIT, MOTOR_CTRL_MCPWM_TIMER, MCPWM_OPR_B, duty_cycle);
        mcpwm_set_duty_type(MOTOR_CTRL_MCPWM_UNIT, MOTOR_CTRL_MCPWM_TIMER, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);  //call this each time, if operator was previously in low/high state
    }
    else {
        mcpwm_set_signal_low(MOTOR_CTRL_MCPWM_UNIT, MOTOR_CTRL_MCPWM_TIMER, MCPWM_OPR_B);
        mcpwm_set_duty(MOTOR_CTRL_MCPWM_UNIT, MOTOR_CTRL_MCPWM_TIMER, MCPWM_OPR_A, -duty_cycle);
        mcpwm_set_duty_type(MOTOR_CTRL_MCPWM_UNIT, MOTOR_CTRL_MCPWM_TIMER, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
    }
}

Engine::Engine(){
    ESP_ERROR_CHECK(mcpwm_gpio_init(MOTOR_CTRL_MCPWM_UNIT, MCPWM0A, GPIO_PWM0A_OUT));
    ESP_ERROR_CHECK(mcpwm_gpio_init(MOTOR_CTRL_MCPWM_UNIT, MCPWM0B, GPIO_PWM0B_OUT));
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 1000;     //frequency = 1kHz,
    pwm_config.cmpr_a = 0;                              //initial duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;                              //initial duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;         //up counting mode
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    ESP_ERROR_CHECK(mcpwm_init(MOTOR_CTRL_MCPWM_UNIT, MOTOR_CTRL_MCPWM_TIMER, &pwm_config));    //Configure PWM0A & PWM0B with above settings
    setDuty(100);
}

void Engine::setDuty(float duty){
    // ESP_LOGI("writeServo", "Servo angle: %i", pos);
    brushed_motor_set_duty(duty); 
}

#define I2C_ADDRESS_GEO 0x1e

void write_GY271_register(const uint8_t reg, const uint8_t data){
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ADDRESS_GEO << 1) | I2C_MASTER_WRITE, 1));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, reg, 1)); // Data registers
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, data, 1)); 
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    esp_err_t err;
    do err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
    while (err != ESP_OK);
    i2c_cmd_link_delete(cmd);
}

void read_GY271_registers(const uint8_t reg, uint8_t* data, const uint8_t bytes){
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ADDRESS_GEO << 1) | I2C_MASTER_WRITE, 1));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, reg, 1)); // Data registers
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ADDRESS_GEO << 1) | I2C_MASTER_READ, 1));
    ESP_ERROR_CHECK(i2c_master_read(cmd, data, bytes - 1, (i2c_ack_type_t) 0));
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data + bytes - 1, (i2c_ack_type_t) 1));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    esp_err_t err;
    do err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
    while (err != ESP_OK);
    i2c_cmd_link_delete(cmd);
}

// Multisample msf = Multisample(16);

float angles_GY271(){
    // printf("core is %i ", xPortGetCoreID());
	uint8_t data[6];
    write_GY271_register(2, 1); // single measurement mode
    // uint8_t status[1];
    // read_GY271_registers(9, status, 1);
    // std::cout << int(status[0]) << "\n";
    read_GY271_registers(3, data, 6);
    float x = (data[0] << 8 | data[1]) - 475 + 990 - 963 - 109;
    // short z = (data[2] << 8 | data[3]);
    // short y = data[4] << 8 | data[5];
    // int angle = atan2((double)z,(double)x) * (180 / 3.14159265) + 180; // angle in degrees
    // ESP_LOGI("angles", "x: %d", x);
    // vTaskDelay(1000/portTICK_PERIOD_MS);

    return x;
}

float Sensor::angle(){
    xSemaphoreTake(xMutexMpu, portMAX_DELAY);
    auto x = angles_GY271();
    // std::cout << getTime() << "\n";
    xSemaphoreGive(xMutexMpu);
    return x;
}

Solver solver;

void task_display(void*){
	while(1){
        // xSemaphoreTake(xBinarySemaphoreGY271Interrupt, portMAX_DELAY);
        // int64_t time_since_boot = esp_timer_get_time();
        // ESP_LOGI("Sensor changed", "One-shot timer called, time since boot: %lld us", time_since_boot);
        solver.changeEngineAcc();
    }
	vTaskDelete(NULL);
}

int64_t getTime(){
    return esp_timer_get_time();
}

#define PIN_SDA_AS5600 32
#define PIN_CLK_AS5600 33
#define AS5600_SLAVE_ADDR 0x36
#define AS5600_ANGLE_REGISTER_H 0x0E

IRAM_ATTR static float read_angle_AS5600() 
{
    uint8_t write_buffer = AS5600_ANGLE_REGISTER_H;
    uint8_t read_buffer[2] = {0,0};
    uint16_t raw;
    esp_err_t err;

    do {
        err =  i2c_master_write_read_device(I2C_NUM_1,
                                        AS5600_SLAVE_ADDR,
                                        &write_buffer,
                                        1,
                                        read_buffer,
                                        2,
                                        portMAX_DELAY);
 
    } while (err != ESP_OK);

    raw = read_buffer[0];
    raw <<= 8;
    raw |= read_buffer[1];

    float a = float((raw + 4096 + 2048 - 3338) % 4096 - 2048); 
    a /= 2048; 
    a *= M_PI; 
    a = sinf(a); 
    // std::cout << a << " ";
    return a;
}

float Engine::angle()
{
    float x = read_angle_AS5600();
    return x;
}

void init_i2c()
{
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = (gpio_num_t)PIN_SDA_GEO;
	conf.scl_io_num = (gpio_num_t)PIN_CLK_GEO;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = 1000000;
    conf.clk_flags = 0;
	ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));

    // write_GY271_register(2, 0x00); // continuous mode 
    // write_GY271_register(0, 0x54); // set 30Hz, oversampling 4

    write_GY271_register(1, 0x00);

	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = (gpio_num_t)PIN_SDA_AS5600;
	conf.scl_io_num = (gpio_num_t)PIN_CLK_AS5600;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = 1000000;
    conf.clk_flags = 0;
	ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_1, &conf));
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_1, I2C_MODE_MASTER, 0, 0, 0));
    ESP_ERROR_CHECK(i2c_filter_enable(I2C_NUM_1, 7));
}

void app_main(void)
{
    init_i2c();

    // int del = 3000;
    // int div = 1000000 / del / 2;
    // for (float d = 30; d > 0; d--)
    // {
    //     brushed_motor_set_duty(d); 
    //     std::cout << d << "\n";
    //     usleep(1000000);
    // }
    // float duty = 100;
    // brushed_motor_set_duty(duty); 
    // for (int i = 0; i < 10000000 / del; i++)
    // while (true)
    // {
    //     // if (i % div == 0) 
    //     // {
    //     //     if (duty > 0) duty = -100;
    //     //     else duty = 100;
    //     //     brushed_motor_set_duty(duty); 
    //     // }
    //     auto r = angles_GY271();
    //     std::cout << r[0] << " " << "\n";
    //     usleep(del);
    // }

    // solver.test();
    xTaskCreatePinnedToCore(&task_display, "disp_task", 8192, NULL, tskIDLE_PRIORITY, NULL, 0);
    // xTaskCreatePinnedToCore(&task_display, "disp_task", 8192, NULL, tskIDLE_PRIORITY, NULL, 1);

    // xTaskCreate(&task_display, "disp_task", 8192, NULL, 1, NULL);
    // xTaskCreate(&task_display, "disp_task", 8192, NULL, 1, NULL);
}
