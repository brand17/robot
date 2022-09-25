#include "freertos/FreeRTOS.h"
#include "sdkconfig.h"
#include "freertos/task.h"

#include <driver/i2c.h>
#include <esp_log.h>
#include <esp_err.h>
#include "robot.hpp"
#include "esp_timer.h"
#include "driver/mcpwm.h"

#define PIN_SDA_GY271 21
#define PIN_CLK_GY271 22

#define GPIO_GY271_INTERRUPT GPIO_NUM_36
#define ESP_INTR_FLAG_DEFAULT 0

#include <iostream>

SemaphoreHandle_t xBinarySemaphoreMpuInterrupt = xSemaphoreCreateBinary();
SemaphoreHandle_t xMutexMpu = xSemaphoreCreateMutex();
 
extern "C" {
	void app_main(void);
}

void IRAM_ATTR gy271_isr_handler(void* arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(xBinarySemaphoreMpuInterrupt, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

#define MOTOR_CTRL_MCPWM_UNIT   MCPWM_UNIT_0
#define MOTOR_CTRL_MCPWM_TIMER  MCPWM_TIMER_0
#define GPIO_PWM0A_OUT 19
#define GPIO_PWM0B_OUT 23

void brushed_motor_set_duty(float duty_cycle)
{
    /* motor moves in forward direction, with duty cycle = duty % */
    if (duty_cycle > 0) {
        mcpwm_set_signal_low(MOTOR_CTRL_MCPWM_UNIT, MOTOR_CTRL_MCPWM_TIMER, MCPWM_OPR_A);
        mcpwm_set_duty(MOTOR_CTRL_MCPWM_UNIT, MOTOR_CTRL_MCPWM_TIMER, MCPWM_OPR_B, duty_cycle);
        mcpwm_set_duty_type(MOTOR_CTRL_MCPWM_UNIT, MOTOR_CTRL_MCPWM_TIMER, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);  //call this each time, if operator was previously in low/high state
    }
    /* motor moves in backward direction, with duty cycle = -duty % */
    else {
        mcpwm_set_signal_low(MOTOR_CTRL_MCPWM_UNIT, MOTOR_CTRL_MCPWM_TIMER, MCPWM_OPR_B);
        mcpwm_set_duty(MOTOR_CTRL_MCPWM_UNIT, MOTOR_CTRL_MCPWM_TIMER, MCPWM_OPR_A, -duty_cycle);
        mcpwm_set_duty_type(MOTOR_CTRL_MCPWM_UNIT, MOTOR_CTRL_MCPWM_TIMER, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
    }
}

void Engine::initEngine(){
    ESP_ERROR_CHECK(mcpwm_gpio_init(MOTOR_CTRL_MCPWM_UNIT, MCPWM0A, GPIO_PWM0A_OUT));
    ESP_ERROR_CHECK(mcpwm_gpio_init(MOTOR_CTRL_MCPWM_UNIT, MCPWM0B, GPIO_PWM0B_OUT));
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 1000;     //frequency = 1kHz,
    pwm_config.cmpr_a = 0;                              //initial duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;                              //initial duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;         //up counting mode
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    ESP_ERROR_CHECK(mcpwm_init(MOTOR_CTRL_MCPWM_UNIT, MOTOR_CTRL_MCPWM_TIMER, &pwm_config));    //Configure PWM0A & PWM0B with above settings
}

void Engine::writeServo(int pos){
    // ESP_LOGI("writeServo", "Servo angle: %i", pos);
    // brushed_motor_set_duty(100); 
}

#define I2C_ADDRESS_GY271 0x1e
// static char tag[] = "hmc5883l";

std::array<float, SENSOR_OUTPUT_DIM> Sensor::angles(){
    xSemaphoreTake(xMutexMpu, portMAX_DELAY);
    // printf("core is %i ", xPortGetCoreID());
	uint8_t data[6];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (I2C_ADDRESS_GY271 << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(cmd, 0x03, 1); // Data registers
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (I2C_ADDRESS_GY271 << 1) | I2C_MASTER_READ, 1);
    i2c_master_read_byte(cmd, data,   (i2c_ack_type_t) 0);
    i2c_master_read_byte(cmd, data+1, (i2c_ack_type_t) 0);
    i2c_master_read_byte(cmd, data+2, (i2c_ack_type_t) 0);
    i2c_master_read_byte(cmd, data+3, (i2c_ack_type_t) 0);
    i2c_master_read_byte(cmd, data+4, (i2c_ack_type_t) 0);
    i2c_master_read_byte(cmd, data+5, (i2c_ack_type_t) 1);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    short x = (data[0] << 8 | data[1]) - 475;
    // short z = (data[2] << 8 | data[3]);
    // short y = data[4] << 8 | data[5];
    // int angle = atan2((double)z,(double)x) * (180 / 3.14159265) + 180; // angle in degrees
    // ESP_LOGI("angles", "x: %d, y: %d, z: %d", x, y, z);
    // vTaskDelay(1000/portTICK_PERIOD_MS);

    xSemaphoreGive(xMutexMpu);
    return std::array<float, SENSOR_OUTPUT_DIM>{(float)x};
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

#define PIN_SDA_AS5600 32
#define PIN_CLK_AS5600 33
#define AS5600_SLAVE_ADDR 0x36
#define AS5600_ANGLE_REGISTER_H 0x0E

IRAM_ATTR static uint16_t read_angle_AS5600() 
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

    // if(raw > AS5600_PULSES_PER_REVOLUTION - 1) {
    //     raw = AS5600_PULSES_PER_REVOLUTION - 1 ;
    // }

    return raw;
}

void init_i2c()
{
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = (gpio_num_t)PIN_SDA_GY271;
	conf.scl_io_num = (gpio_num_t)PIN_CLK_GY271;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = 400000;
    conf.clk_flags = 0;
	ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (I2C_ADDRESS_GY271 << 1) | I2C_MASTER_WRITE, 1);
	i2c_master_write_byte(cmd, 0x02, 1); // Mode register
	i2c_master_write_byte(cmd, 0x00, 1); // continuous mode 
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (I2C_ADDRESS_GY271 << 1) | I2C_MASTER_WRITE, 1);
	i2c_master_write_byte(cmd, 0x00, 1); // A register
	// i2c_master_write_byte(cmd, 0x70, 1); // set 15Hz, oversampling 8
	// i2c_master_write_byte(cmd, 0x34, 1); // set 30Hz, oversampling 2
	i2c_master_write_byte(cmd, 0x54, 1); // set 30Hz, oversampling 4
	// i2c_master_write_byte(cmd, 0x74, 1); // set 30Hz, oversampling 8
	// i2c_master_write_byte(cmd, 0x18, 1); // set 75Hz, no oversampling (1)
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (I2C_ADDRESS_GY271 << 1) | I2C_MASTER_WRITE, 1);
	i2c_master_write_byte(cmd, 0x01, 1); // B register
	i2c_master_write_byte(cmd, 0x20, 1); // set gain
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = (gpio_num_t)PIN_SDA_AS5600;
	conf.scl_io_num = (gpio_num_t)PIN_CLK_AS5600;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = 1000000;
    conf.clk_flags = 0;
	ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_1, &conf));
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_1, I2C_MODE_MASTER, 0, 0, 0));
    i2c_filter_enable(I2C_NUM_1, 7);
}

void app_main(void)
{
    brushed_motor_set_duty(100); 
    // ESP_LOGI("initEngine", "Starting the engine");
    // usleep(10000000);
    // ESP_LOGI("initEngine", "Stopping the engine");
    // usleep(10000000);
    // ESP_ERROR_CHECK(iot_servo_write_angle(LEDC_LOW_SPEED_MODE, 0, 90)); usleep(50000000);
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

    // solver.initEngineTimer();
    
    // for (auto &a: {1, -1}){
    //     engine.setAcc(a);
    //     usleep(20000);
    // }

    init_i2c();
    while (true)
    {
        auto r = read_angle_AS5600();
        std::cout << r << "\n";
        usleep(100000);
    }
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.pin_bit_mask = 1ULL<<GPIO_GY271_INTERRUPT;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);
    gpio_set_intr_type(GPIO_GY271_INTERRUPT, GPIO_INTR_NEGEDGE);
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(GPIO_GY271_INTERRUPT, gy271_isr_handler, (void*) GPIO_GY271_INTERRUPT);
    vTaskDelay(500/portTICK_PERIOD_MS);
    // solver.test();
    xTaskCreatePinnedToCore(&task_display, "disp_task", 8192, NULL, tskIDLE_PRIORITY, NULL, 0);
    // xTaskCreatePinnedToCore(&task_display, "disp_task", 8192, NULL, tskIDLE_PRIORITY, NULL, 1);

    // xTaskCreate(&task_display, "disp_task", 8192, NULL, 1, NULL);
    // xTaskCreate(&task_display, "disp_task", 8192, NULL, 1, NULL);
}
