#include "freertos/FreeRTOS.h"
#include "sdkconfig.h"
#include "freertos/task.h"

#include <driver/i2c.h>
#include <esp_log.h>
#include <esp_err.h>
#include "robot.hpp"
#include "esp_timer.h"
#include "driver/mcpwm.h"
// #include "lis3mdl.h"

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

void write_i2c_register(const uint8_t reg, const uint8_t data){
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

void write_i2c_register(const uint8_t reg, const uint8_t* data, const uint8_t bytes){
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ADDRESS_GEO << 1) | I2C_MASTER_WRITE, 1));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, reg, 1)); // Data registers
    // ESP_ERROR_CHECK(i2c_master_write(cmd, data, bytes, (i2c_ack_type_t) 1));
    for (uint8_t i = 0; i < bytes; i++)
        ESP_ERROR_CHECK(i2c_master_write_byte(cmd, data[i], 1));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    esp_err_t err;
    do err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
    while (err != ESP_OK);
    i2c_cmd_link_delete(cmd);
}

void read_i2c_registers(uint8_t reg, uint8_t* data, const uint8_t bytes){
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ADDRESS_GEO << 1) | I2C_MASTER_WRITE, 1));
    if (bytes > 1)
        reg |= 0x80;
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, reg, 1)); // Data registers
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ADDRESS_GEO << 1) | I2C_MASTER_READ, 1));
    if (bytes > 1)
        ESP_ERROR_CHECK(i2c_master_read(cmd, data, bytes - 1, (i2c_ack_type_t) 0));
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data + bytes - 1, (i2c_ack_type_t) 1));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    auto err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1/portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (err != ESP_OK)
    {
        std::cout << "err=" << int(err) << "\n";
        // delete sensor;
        // sensor = lis3mdl_init_sensor (I2C_NUM_0, LIS3MDL_I2C_ADDRESS_2, 0);
        // lis3mdl_set_scale(sensor, lis3mdl_scale_4_Gs);
        // lis3mdl_set_mode (sensor, lis3mdl_lpm_1000);
        // write_i2c_register(0x22, 0); // continuous mode
        // ESP_ERROR_CHECK(i2c_master_clear_bus(I2C_NUM_0));
    }
}

float angles_GY271(){
	uint8_t data[6] = {1};
    write_i2c_register(2, data, 1); // single measurement mode
    read_i2c_registers(3, data, 6);
    float x = (data[0] << 8 | data[1]) - 475 + 990 - 963 - 109;
    return x;
}

int64_t getTime(){
    return esp_timer_get_time();
}

// static lis3mdl_sensor_t* sensor;

float angles_LIS3MDL(){
    // printf("core is %i ", xPortGetCoreID());
    // std::cout << "angles_LIS3MDL called\n";
	uint8_t data[6];
    do {
        read_i2c_registers(0x27, data, 1);
        if (data[0] == 0)
            write_i2c_register(0x22, 0); // continuous mode
    } while (data[0] == 0);
    // if (data[0] == 0)
    // {
    //     // std::cout << "status=" << int(data[0]) << "\n";
    // }
    read_i2c_registers(0x28, data, 6);
    int16_t x = ((uint16_t)data[1] << 8) | data[0];
    // std::cout << int(data[0]) << " " << int(data[1]) << x << "\n";
    // int16_t y = ((uint16_t)data[3] << 8) | data[2];
    // int16_t z = ((uint16_t)data[5] << 8) | data[4];
    // std::cout << x << " " << y << " " << z << "\n";
    return x + 585;

    // lis3mdl_float_data_t  data2;

    // if (//lis3mdl_new_data (sensor) &&
    //     lis3mdl_get_float_data (sensor, &data2))
    //     // max. full scale is +-16 g and best resolution is 1 mg, i.e. 5 digits
    //     printf("%+7.3f %+7.3f %+7.3f\n",
    //             data2.mx, data2.my, data2.mz);

    // return data2.mx;
}

float Sensor::angle(){
    xSemaphoreTake(xMutexMpu, portMAX_DELAY);
    auto x = angles_LIS3MDL();
    // std::cout << getTime() << "\n";
    xSemaphoreGive(xMutexMpu);
    return x;
}

Solver solver;

void task_display(void*){
    vector<double> x_init{-2, 1, 1};
	while(1){
        // xSemaphoreTake(xBinarySemaphoreGY271Interrupt, portMAX_DELAY);
        // int64_t time_since_boot = esp_timer_get_time();
        // ESP_LOGI("Sensor changed", "One-shot timer called, time since boot: %lld us", time_since_boot);
        x_init = solver.changeEngineAcc(x_init);
    }
	vTaskDelete(NULL);
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

    // write_i2c_register(2, 0x00); // continuous mode 
    // write_i2c_register(0, 0x54); // set 30Hz, oversampling 4

    // write_i2c_register(1, 0x00);

    // sensor = lis3mdl_init_sensor (I2C_NUM_0, LIS3MDL_I2C_ADDRESS_2, 0);
    // lis3mdl_set_scale(sensor, lis3mdl_scale_16_Gs);
    // lis3mdl_set_mode (sensor, lis3mdl_lpm_1000);

    uint8_t ctrl_regs[5] = { 0x02, 0x60, 0x00, 0x00, 0x00 };
    uint8_t int_cfg = 0x00;
    write_i2c_register(0x20, ctrl_regs, 5);
    write_i2c_register(0x30, &int_cfg, 1);
    std::cout << "geomagnit sensor initialized\n";

    // uint8_t reg;
    // read_i2c_registers(0x0f, &reg, 1);
    // std::cout << "LIS3MDL address: " << int(reg) << "\n";
    // write_i2c_register(0x20, 0x02); // 1000 Hz ODR
    // write_i2c_register(0x22, 0); // continuous mode
    // write_i2c_register(0x24, 0x40); // set BDU

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
    struct duty2pos_params p = {{1, 11.4380671505738, 37.3006540085341, 73.5050061255704}, // positions
                        {1, 1.738686685, 2.133604423, 2.558234859}, // time
                        {-0.288339009, -0.05, -0.996708909},// duty
                        {1, 0, 0, 0}, // velocity
                        {0, 0, 0} // acceleration
                        }; 

    double x_init[] = {-2, 1, 1};
    auto d = get_duty(p, x_init, -39.0958356);
    return;

    init_i2c();

    // for (float d = 30; d > 0; d--)
    // {
    //     brushed_motor_set_duty(d); 
    //     std::cout << d << "\n";
    //     usleep(1000000);
    // }
    // float duty = 100;
    // brushed_motor_set_duty(duty); 
    // for (int i = 0; i < 10000000 / del; i++)

    // int del = 300000;
    // while (true)
    // {
    //     // if (i % div == 0) 
    //     // {
    //     //     if (duty > 0) duty = -100;
    //     //     else duty = 100;
    //     //     brushed_motor_set_duty(duty); 
    //     // }
    //     auto r = angles_LIS3MDL();
    //     std::cout << r << " " << "\n";
    //     usleep(del);
    // }

    // solver.test();
    xTaskCreatePinnedToCore(&task_display, "disp_task", 8192, NULL, tskIDLE_PRIORITY, NULL, 0);
    // xTaskCreatePinnedToCore(&task_display, "disp_task", 8192, NULL, tskIDLE_PRIORITY, NULL, 1);

    // xTaskCreate(&task_display, "disp_task", 8192, NULL, 1, NULL);
    // xTaskCreate(&task_display, "disp_task", 8192, NULL, 1, NULL);
}
