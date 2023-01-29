//#include "C:\SysGCC\prj\test02\wiringPi\wiringPi.h"
#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <stdint.h>
#include <unistd.h>
#include <math.h>

// #define NUM 20   //for Orange Pi Zero 2
#define I2C_ADDRESS_GEO 0x20 // rm3100
#define AS5600_SLAVE_ADDR 0x36

float angles_rm3100(int fd_rm3100){
    // printf("core is %i ", xPortGetCoreID());
	uint8_t data[9];
    do
    {
        // read_i2c_registers(0x34, data, 1);
        data[0] = wiringPiI2CReadReg8(fd_rm3100, 0x34);
        if ((data[0] & 0x80) != 0x80)
        {
            // ESP_LOGI("", "waiting status");
            // write_i2c_register(0x01, 0x41); // continuous mode
            wiringPiI2CWriteReg8(fd_rm3100, 0x01, 0x41); // initialize continous 
        }
    } while ((data[0] & 0x80) != 0x80);

    data[0] = wiringPiI2CReadReg8(fd_rm3100, 0x2a);
    data[1] = wiringPiI2CReadReg8(fd_rm3100, 0x2b);
    data[2] = wiringPiI2CReadReg8(fd_rm3100, 0x2c);
    // read_i2c_registers(0x2a, data, 3);

    int x = 0;
    if (data[0] & 0x80){
        x = 0xFF;
    }

    x = (x * 256 * 256 * 256) | (uint32_t)(data[0]) * 256 * 256 | (uint16_t)(data[1]) * 256 | data[2];
    x -= 300;

    // for (int i = 0; i < 3; i++)
    //     printf("%3i ", data[i]);
    // printf("%i ", x);
    // printf("%f\n", float(x));

    return x;
}

#define AS5600_ANGLE_REGISTER_H 0x0E

static float read_angle_AS5600(int fd_rm3100) 
{
    uint16_t raw;
    raw = wiringPiI2CReadReg16(fd_rm3100, AS5600_ANGLE_REGISTER_H);
    raw = __builtin_bswap16(raw);

    float a = float((raw + 4096 + 2048 - 88) % 4096 - 2048); 
    // printf("%f\n", a);
    a /= 2048; 
    a *= M_PI; 
    a = sinf(a); 
    // std::cout << a << " ";
    return a;
}

#define L298N_IN2 6
#define PWM_PIN 4

#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>

auto fd_mem = open ("/dev/mem", O_RDWR | O_SYNC | O_CLOEXEC);
auto _0x0300a000 = (uint32_t *)mmap(0, 4*1024, PROT_READ | PROT_WRITE, MAP_SHARED, fd_mem, 0x0300a000);

void brushed_motor_set_duty(float duty_cycle)
{
    if (duty_cycle > 0) {
        digitalWrite (L298N_IN2, HIGH);
        *(_0x0300a000 + (0x84 >> 2)) = 0xffff0000 + duty_cycle;
    }
    else {
        digitalWrite (L298N_IN2, LOW);
        *(_0x0300a000 + (0x84 >> 2)) = 0xffffffff + duty_cycle;
    }
}

// extern "C"
// {
//     #include <OrangePi.h>
// }

int main()
    {
        auto fd_rm3100 = wiringPiI2CSetupInterface("/dev/i2c-3", I2C_ADDRESS_GEO);
        // auto fd_rm3100 = wiringPiI2CSetup(I2C_ADDRESS_GEO);
        wiringPiI2CWriteReg8(fd_rm3100, 0x01, 0x41); // initialize continous
        wiringPiI2CWriteReg8(fd_rm3100, 0x0b, 0x96); // TMRC 600Hz
        // write_i2c_register(0x01, 0x41); // initialize continous
        // write_i2c_register(0x04, 0x00); // CCX
        // write_i2c_register(0x05, 0x00); // CCX
        // write_i2c_register(0x06, 0x00); // CCY
        // write_i2c_register(0x07, 0x00); // CCY
        // write_i2c_register(0x08, 0x00); // CCZ
        // write_i2c_register(0x09, 0x64); // CCZ 100 cycles = 850Hz
        // write_i2c_register(0x0B, 0x96); // TMRC 600Hz

        auto fd_as5600 = wiringPiI2CSetupInterface("/dev/i2c-3", AS5600_SLAVE_ADDR);
        // auto fd5 = wiringPiI2CSetupInterface("/dev/i2c-5", I2C_ADDRESS_GEO);

        // while (true)
        // {
        //     auto a = read_angle_AS5600(fd_as5600);
        //     // auto a = angles_rm3100(fd_rm3100);
        //     printf("%f\n", a);
        //     usleep(100000);
        // }

        if (wiringPiSetup() == -1)
            exit(1);
        pinMode(L298N_IN2, OUTPUT);

	    auto _0x0300b000 = (uint32_t *)mmap(0, 4*1024, PROT_READ | PROT_WRITE, MAP_SHARED, fd_mem, 0x0300b000);

        // printf("0x300b0fc=%x\n", *(_0x0300b000 + (0xfc >> 2))); // PH3 Port controller
        // printf("0x300a020=%x\n", *(_0x0300a000 + (0x20 >> 2))); // clock
        // printf("0x300a084=%x\n", *(_0x0300a000 + (0x84 >> 2))); // period
        // // printf("0x300a088=%x\n", *(_0x0300a000 + (0x88 >> 2))); 
        // printf("0x300a080=%x\n", *(_0x0300a000 + (0x80 >> 2))); // mode, prescal_k
        // printf("0x300a040=%x\n", *(_0x0300a000 + (0x40 >> 2))); // enable

        *(_0x0300b000 + (0xfc >> 2)) = 0x44554422; // PH3 Port controller
        *(_0x0300a000 + (0x20 >> 2)) = 0x00000010; // set gating_clk
        *(_0x0300a000 + (0x80 >> 2)) = 0x00000001; // prescale, mode
        // *(_0x0300a000 + (0x84 >> 2)) = 0x04000400; // period
        *(_0x0300a000 + (0x40 >> 2)) = 0x00000002; // enable

        for (float d = 1000; d > -1000; d--)
        {
            brushed_motor_set_duty(0xffff * d / 1000);
            printf("%f\n", d);
            usleep(10000);
        }
        return 0;
    }