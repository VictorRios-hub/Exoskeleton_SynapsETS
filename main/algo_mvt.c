#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_pthread.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/mcpwm.h"
#include "math.h"
#include "string.h"

// 8 bit uint8_t leg status logic
// msb to bit 4 left leg 0xb0000XXXX
#define STATUS_RAISED_RIGHT_LEG		0b10000000
#define STATUS_LOWERED_RIGHT_LEG	0b01000000
#define STATUS_RAISED_LEFT_LEG		0b00100000
#define STATUS_LOWERED_LEFT_LEG	    0b00010000
// 3 bit to lsb right leg 0bXXXX0000
#define STATUS_LEFT_LEG_FORWARD		0b00001000
#define STATUS_LEFT_LEG_BACKWARD	0b00000100
#define STATUS_RIGHT_LEG_FORWARD	0b00000010
#define STATUS_RIGHT_LEG_BACKWARD	0b00000001

// Values to store the baseline on the IMU reading
float baseline_Gxyz_high_left[3] = {0};
float baseline_Gxyz_high_right[3] = {0};
float baseline_Gxyz_low_left[3] = {0};
float baseline_Gxyz_low_right[3] = {0};
float baseline_Axyz_high_left[3] = {0};
float baseline_Axyz_high_right[3] = {0};
float baseline_Axyz_low_left[3] = {0};
float baseline_Axyz_low_right[3] = {0};

// Values to store the highest error on our IMU Accelerometer value and Gyroscope
float lowest_baseline_value_acc = 0;
float highest_baseline_value_acc = 0;
float lowest_baseline_value_gyro = 0;
float highest_baseline_value_gyro = 0;

// Global variable to control IMU averaging logic
float averaging_counter = 0;

void set_baseline_extr_value_gyro(float value){
        if(value > highest_baseline_value_gyro){
        highest_baseline_value_gyro = value;
    }
    else if(value < lowest_baseline_value_gyro){
        lowest_baseline_value_gyro = value;
    }
}

void set_baseline_extr_value_acc(float value){
        if(value > highest_baseline_value_acc){
        highest_baseline_value_acc = value;
    }
    else if(value < lowest_baseline_value_acc){
        lowest_baseline_value_acc = value;
    }
}

void calculate_imu_error(float Gxyz_high_left[3], float Gxyz_high_right[3], float Gxyz_low_left[3], float Gxyz_low_right[3], float Axyz_high_left[3], float Axyz_high_right[3], float Axyz_low_left[3], float Axyz_low_right[3]){

    // Calculating baseline value and highest possible derivation
    if(averaging_counter < 100){
        averaging_counter++;
        for(int i = 0; i<3;i++){
            baseline_Gxyz_high_left[i] = (baseline_Gxyz_high_left[i] + Gxyz_high_left[i]);
            set_baseline_extr_value_gyro(Gxyz_high_left[i]);
            baseline_Gxyz_high_right[i] = (baseline_Gxyz_high_right[i] + Gxyz_high_right[i]);
            set_baseline_extr_value_gyro(Gxyz_high_right[i]);
            baseline_Gxyz_low_left[i] = (baseline_Gxyz_low_left[i] + Gxyz_low_left[i]);
            set_baseline_extr_value_gyro(Gxyz_low_left[i]);
            baseline_Gxyz_low_right[i] = (baseline_Gxyz_low_right[i] + Gxyz_low_right[i]);
            set_baseline_extr_value_gyro(Gxyz_low_right[i]);
            baseline_Axyz_high_left[i] = (baseline_Axyz_high_left[i] + Axyz_high_left[i]);
            set_baseline_extr_value_acc(Axyz_high_left[i]);
            baseline_Axyz_high_right[i] = (baseline_Axyz_high_right[i] + Axyz_high_right[i]);
            set_baseline_extr_value_acc(Axyz_high_right[i]);
            baseline_Axyz_low_left[i] = (baseline_Axyz_low_left[i] + Axyz_low_left[i]);
            set_baseline_extr_value_acc(Axyz_low_left[i]);
            baseline_Axyz_low_right[i] = (baseline_Axyz_low_right[i] + Axyz_low_right[i]);
            set_baseline_extr_value_acc(Axyz_low_right[i]);
        }
    }

    // calculate baseline value
    if(averaging_counter == 100){
        for(int i = 0; i < 3; i++){
            baseline_Gxyz_high_left[i] = (baseline_Gxyz_high_left[i]/averaging_counter);
            baseline_Gxyz_high_right[i] = (baseline_Gxyz_high_right[i]/averaging_counter);
            baseline_Gxyz_low_left[i] = (baseline_Gxyz_low_left[i]/averaging_counter);
            baseline_Gxyz_low_right[i] = (baseline_Gxyz_low_right[i]/averaging_counter);
            baseline_Axyz_high_left[i] = (baseline_Axyz_high_left[i]/averaging_counter);
            baseline_Axyz_high_right[i] = (baseline_Axyz_high_right[i]/averaging_counter);
            baseline_Axyz_low_left[i] = (baseline_Axyz_low_left[i]/averaging_counter);
            baseline_Axyz_low_right[i] = (baseline_Axyz_low_right[i]/averaging_counter);
        }
    }
}

uint8_t algo_mvt(float Axyz_high_left[3], float Axyz_high_right[3], float Axyz_low_left[3], float Axyz_low_right[3]){

    // Output variable use to carry state values
    uint8_t output = 0;

    // Logic for state activation 
    // Because of the way the IMU are positioned, X is left/right, Y is up/down and Z is forward/backward

    // Left leg raised and in front
    if((Axyz_high_left[1] > (Axyz_high_right[1]+highest_baseline_value_acc)) && (Axyz_low_left[1] > (Axyz_low_right[1]+highest_baseline_value_acc))){
        output = (STATUS_RAISED_LEFT_LEG & STATUS_LEFT_LEG_FORWARD & STATUS_LOWERED_RIGHT_LEG & STATUS_RIGHT_LEG_BACKWARD);
    }
    // Right leg raised and in front
    else if((Axyz_high_right[1] > (Axyz_high_left[1]+highest_baseline_value_acc)) && (Axyz_low_right[1] > (Axyz_low_left[1]+highest_baseline_value_acc))){
        output = (STATUS_LOWERED_LEFT_LEG & STATUS_LEFT_LEG_BACKWARD & STATUS_RAISED_RIGHT_LEG & STATUS_RIGHT_LEG_FORWARD);
    }
    // Left Leg leg forward but not raised
    else if((Axyz_high_left[1] < (Axyz_high_right[1]+highest_baseline_value_acc)) && (Axyz_low_left[2] > (Axyz_low_right[2]+highest_baseline_value_acc))){
        output = (STATUS_LOWERED_LEFT_LEG & STATUS_LEFT_LEG_FORWARD & STATUS_LOWERED_RIGHT_LEG & STATUS_RIGHT_LEG_BACKWARD);
    }
    // Right leg forward but not raised
    else if((Axyz_high_right[1] < (Axyz_high_left[1]+highest_baseline_value_acc)) && (Axyz_low_right[2] > (Axyz_low_left[2]+highest_baseline_value_acc))){
        output = (STATUS_LOWERED_LEFT_LEG & STATUS_LEFT_LEG_BACKWARD & STATUS_LOWERED_RIGHT_LEG & STATUS_RIGHT_LEG_FORWARD);
    }
    // Left leg and right leg lowered (squat)
    // Right leg forward but not raised
    else if((Axyz_high_right[1] < (baseline_Axyz_high_right[1]-highest_baseline_value_acc)) && (Axyz_high_left[1] < (baseline_Axyz_high_left[1]-highest_baseline_value_acc))){
        output = (STATUS_LOWERED_LEFT_LEG & STATUS_LOWERED_RIGHT_LEG);
    }
    return output;
}