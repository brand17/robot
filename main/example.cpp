/*
 * Display.c
 *
 *  Created on: 14.08.2017
 *      Author: darek
 */
#include <driver/i2c.h>
#include <esp_log.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "MPU6050.h"
// #include "MPU6050_6Axis_MotionApps20.h"
#include "sdkconfig.h"
#include "iot_servo.h"

#define PIN_SDA 21
#define PIN_CLK 22

Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
uint16_t packetSize = 42;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
extern MPU6050 mpu;
bool mpuInterrupt = false;

void printAngles(){
    mpuIntStatus = mpu.getIntStatus();
    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();

    // otherwise, check for DMP data ready interrupt frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO

        mpu.getFIFOBytes(fifoBuffer, packetSize);
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        printf("YAW: %3.1f, ", ypr[0] * 180/M_PI);
        printf("PITCH: %3.1f, ", ypr[1] * 180/M_PI);
        printf("ROLL: %3.1f \n", ypr[2] * 180/M_PI);
    }
}

void task_display(void*){
	while(1){
        if (mpuInterrupt){
            mpuInterrupt = false;
            printAngles();
        }
	    //Best result is to match with DMP refresh rate
	    // Its last value in components/MPU6050/MPU6050_6Axis_MotionApps20.h file line 310
	    // Now its 0x13, which means DMP is refreshed with 10Hz rate
		vTaskDelay(5/portTICK_PERIOD_MS);
	}

	vTaskDelete(NULL);
}
