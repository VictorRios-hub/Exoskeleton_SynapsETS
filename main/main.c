/* Exo code

   This code at this time handle IMU sensor captation and command motor driven by ESC

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
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
#include "driver/pcnt.h"
#include "rotary_encoder.h"
#include "driver/uart.h"
#include "driver/gpio.h"


static const char *TAG = "EXO_code";

// I2C defines

#define I2C_MASTER_SCL_IO           17                         /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           18                         /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_0                0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define I2C_MASTER_SCL_I1           15                         /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_I1           16                         /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_1                1                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

// IMU defines

#define MPU9250_SENSOR_ADDR69               0x69        /*!< Slave address of the MPU9250 sensor */
#define MPU9250_SENSOR_ADDR68               0x68        /*!< Slave address of the MPU9250 sensor */
#define MPU9250_WHO_AM_I_REG_ADDR           0x75        /*!< Register addresses of the "who am I" register */
#define MPU9250_RA_ACCEL_XOUT_H             0x3B         

#define MPU9250_CLOCK_PLL_XGYRO             0x01
#define MPU9250_GYRO_FS_250                 0x00
#define MPU9250_ACCEL_FS_2                  0x00
#define MPU9250_PWR1_CLKSEL_BIT             2
#define MPU9250_GCONFIG_FS_SEL_BIT          4
#define MPU9250_ACONFIG_AFS_SEL_BIT         4
#define MPU9250_PWR1_SLEEP_BIT              6

#define NUM_SAMPLES 1000 // Nombre d'échantillons à collecter pendant la calibration

// Structure pour stocker les valeurs min/max des capteurs
typedef struct {
  float min;
  float max;
} CalibrationData;

// PWM defines
#define SERVO_MIN_PULSEWIDTH_US (500) // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH_US (2500) // Maximum pulse width in microsecond

#define SERVO_PULSE_GPIO_L        (14)   // GPIO connects to the PWM signal line : GPIO14
#define SERVO_PULSE_GPIO_R        (13)   // GPIO connects to the PWM signal line : GPIO13

// Detection defines
#define ANGLE_MAX 60
#define ANGLE_MIN 0
uint8_t output = 0;     // WATCH OUT TIMER BETWEEN THREADS

// Encoder init
#define ENCODER_RIGHT_A_PIN GPIO_NUM_4
#define ENCODER_RIGHT_B_PIN GPIO_NUM_5

#define ENCODER_LEFT_A_PIN GPIO_NUM_6
#define ENCODER_LEFT_B_PIN GPIO_NUM_7

// Kalman filter defines 
#define Kk      0.6

// Variables globales
uint8_t buffer[14];

int16_t ax, ay, az;
int16_t gx, gy, gz;

// Calcul des valeurs de correction (offset) pour chaque IMU
float accelerometerOffset_low_left[3] = {0};
float accelerometerOffset_high_left[3] = {0};
float accelerometerOffset_low_right[3] = {0};
float accelerometerOffset_high_right[3] = {0};

float gyroscopeOffset_low_left[3] = {0};
float gyroscopeOffset_high_left[3] = {0};
float gyroscopeOffset_low_right[3] = {0};
float gyroscopeOffset_high_right[3] = {0};

// IMU low left
float Axyz_low_left[3];
float Gxyz_low_left[3];
float previous_mesure_accel_LL[3];
float previous_mesure_gyro_LL[3];

// IMU low right
float Axyz_low_right[3];
float Gxyz_low_right[3];
float previous_mesure_accel_LR[3];
float previous_mesure_gyro_LR[3];

// IMU high left
float Axyz_high_left[3];
float Gxyz_high_left[3];
float previous_mesure_accel_HL[3];
float previous_mesure_gyro_HL[3];

// IMU high right
float Axyz_high_right[3];
float Gxyz_high_right[3];
float previous_mesure_accel_HR[3];
float previous_mesure_gyro_HR[3];

// Initialisation des previous à 0
float previous_mesure_accel_LL[3] = {0};
float previous_mesure_gyro_LL[3] = {0};
float previous_mesure_accel_LR[3] = {0};
float previous_mesure_gyro_LR[3] = {0};
float previous_mesure_accel_HL[3] = {0};
float previous_mesure_gyro_HL[3] = {0};
float previous_mesure_accel_HR[3] = {0};
float previous_mesure_gyro_HR[3] = {0};

// Declarations for mvt algorithm
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

//************************************** I2C functions ******************************************//

/**
 * @brief Read a sequence of bytes from a MPU9250 sensor registers
 */
static esp_err_t mpu9250_register_read(uint8_t I2c_master, uint8_t sensor_addr, uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2c_master, sensor_addr, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/**
 * @brief Write a byte to a MPU9250 sensor register
 */
static esp_err_t mpu9250_register_write_byte(uint8_t I2c_master, uint8_t sensor_addr, uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(I2c_master, sensor_addr, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    return ret;
}

/**
 * @brief Write localised byte to a MPU9250 sensor register
 */
static esp_err_t mpu9250_register_write_localized_byte(uint8_t I2c_master, uint8_t sensor_addr, uint8_t reg_addr, uint8_t data, uint8_t decalage, uint8_t length)
{
    uint8_t b;
    mpu9250_register_read(I2c_master, sensor_addr, reg_addr, &b, length);

    uint8_t mask = ((1 << length) - 1) << (decalage - length + 1);
    data <<= (decalage - length + 1); // shift data into correct position
    data &= mask; // zero all non-important bits in data
    b &= ~(mask); // zero all important bits in existing byte
    b |= data; // combine data with existing byte

    int ret;
    uint8_t write_buf[2] = {reg_addr, b};

    ret = i2c_master_write_to_device(I2c_master, sensor_addr, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    return ret;
}

//************************************** IMU functions ******************************************//

void __setClockSource(uint8_t I2c_master, uint8_t sensor_addr, uint8_t source) {
    // writeBits(devAddr, MPU9250_RA_PWR_MGMT_1, MPU9250_PWR1_CLKSEL_BIT, MPU9250_PWR1_CLKSEL_LENGTH, source);
    mpu9250_register_write_localized_byte(I2c_master, sensor_addr, 0x6B, source, MPU9250_PWR1_CLKSEL_BIT, 3);
}

void setFullScaleGyroRange(uint8_t I2c_master, uint8_t sensor_addr, uint8_t range) {
    // writeBits(devAddr, MPU9250_RA_GYRO_CONFIG, MPU9250_GCONFIG_FS_SEL_BIT, MPU9250_GCONFIG_FS_SEL_LENGTH, range);
    mpu9250_register_write_localized_byte(I2c_master, sensor_addr, 0x1B, range, MPU9250_GCONFIG_FS_SEL_BIT, 2);
}

void setFullScaleAccelRange(uint8_t I2c_master, uint8_t sensor_addr, uint8_t range) {
    // writeBits(devAddr, MPU9250_RA_ACCEL_CONFIG, MPU9250_ACONFIG_AFS_SEL_BIT, MPU9250_ACONFIG_AFS_SEL_LENGTH, range);
    mpu9250_register_write_localized_byte(I2c_master, sensor_addr, 0x1C, range, MPU9250_ACONFIG_AFS_SEL_BIT, 2);
}

void setSleepEnabled(uint8_t I2c_master, uint8_t sensor_addr, bool enabled) {
    // writeBit(devAddr, MPU9250_RA_PWR_MGMT_1, MPU9250_PWR1_SLEEP_BIT, enabled);
    mpu9250_register_write_localized_byte(I2c_master, sensor_addr, 0x6B, enabled, MPU9250_PWR1_SLEEP_BIT, 1);
}

/** Power on and prepare for general usage.
 * This will activate the device and take it out of sleep mode (which must be done
 * after start-up). This function also sets both the accelerometer and the gyroscope
 * to their most sensitive settings, namely +/- 2g and +/- 250 degrees/sec, and sets
 * the clock source to use the X Gyro for reference, which is slightly better than
 * the default internal clock source.
 */
void IMU_initialize(uint8_t I2c_master, uint8_t sensor_addr ) {
    __setClockSource(I2c_master, sensor_addr,MPU9250_CLOCK_PLL_XGYRO);
    setFullScaleGyroRange(I2c_master, sensor_addr,MPU9250_GYRO_FS_250);
    setFullScaleAccelRange(I2c_master, sensor_addr,MPU9250_ACCEL_FS_2);
    setSleepEnabled(I2c_master, sensor_addr,false); // thanks to Jack Elston for pointing this one out!
}

/** Get raw 6-axis motion sensor readings (accel/gyro).
 * Retrieves all currently available motion sensor values.
 * @param ax 16-bit signed integer container for accelerometer X-axis value
 * @param ay 16-bit signed integer container for accelerometer Y-axis value
 * @param az 16-bit signed integer container for accelerometer Z-axis value
 * @param gx 16-bit signed integer container for gyroscope X-axis value
 * @param gy 16-bit signed integer container for gyroscope Y-axis value
 * @param gz 16-bit signed integer container for gyroscope Z-axis value
 * @see getAcceleration()
 * @see getRotation()
 * @see MPU9250_RA_ACCEL_XOUT_H
 */
void getMotion6(uint8_t I2c_master, uint8_t sensor_addr, int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) {
    mpu9250_register_read(I2c_master, sensor_addr, MPU9250_RA_ACCEL_XOUT_H,buffer, 14);
    *ax = (((int16_t)buffer[0]) << 8) | buffer[1];
    *ay = (((int16_t)buffer[2]) << 8) | buffer[3];
    *az = (((int16_t)buffer[4]) << 8) | buffer[5];
    *gx = (((int16_t)buffer[8]) << 8) | buffer[9];
    *gy = (((int16_t)buffer[10]) << 8) | buffer[11];
    *gz = (((int16_t)buffer[12]) << 8) | buffer[13];
}

void get_Data(uint8_t I2c_master, uint8_t sensor_addr, float* Axyz, float* Gxyz, float* A_offset, float* G_offset) {
    getMotion6(I2c_master, sensor_addr, &ax, &ay, &az, &gx, &gy, &gz);
    Axyz[0] = (double) ax / 16384 + A_offset[0];
    Axyz[1] = (double) ay / 16384 + A_offset[1];
    Axyz[2] = (double) az / 16384 + A_offset[2];
    Gxyz[0] = (double) gx * 250 / 32768 + G_offset[0];
    Gxyz[1] = (double) gy * 250 / 32768 + G_offset[1];
    Gxyz[2] = (double) gz * 250 / 32768 + G_offset[2];
    memset(buffer, 0, sizeof(buffer));

}

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port0 = I2C_MASTER_0;
    int i2c_master_port1 = I2C_MASTER_1;

    i2c_config_t conf0 = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_config_t conf1 = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = I2C_MASTER_SDA_I1,
    .scl_io_num = I2C_MASTER_SCL_I1,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port0, &conf0);
    i2c_param_config(i2c_master_port1, &conf1);

    i2c_driver_install(i2c_master_port0, conf0.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);

    return i2c_driver_install(i2c_master_port1, conf1.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

// Fonction de calibration
void calibrateIMUs() {

    // Tableaux pour stocker les valeurs min/max des capteurs pour chaque IMU
    CalibrationData accelerometer_low_left[3] = {0};
    CalibrationData accelerometer_high_left[3] = {0};
    CalibrationData accelerometer_low_right[3]= {0};
    CalibrationData accelerometer_high_right[3]= {0};

    CalibrationData gyroscope_low_left[3] = {0};
    CalibrationData gyroscope_high_left[3] = {0};
    CalibrationData gyroscope_low_right[3]= {0};
    CalibrationData gyroscope_high_right[3]= {0};

    // Collecte des données brutes pour la calibration
    get_Data(I2C_MASTER_0, MPU9250_SENSOR_ADDR68, Axyz_low_left, Gxyz_low_left, accelerometerOffset_low_left, gyroscopeOffset_low_left );
    get_Data(I2C_MASTER_0, MPU9250_SENSOR_ADDR69, Axyz_high_left, Gxyz_high_left, accelerometerOffset_high_left, gyroscopeOffset_high_left );
    get_Data(I2C_MASTER_1, MPU9250_SENSOR_ADDR68, Axyz_low_right, Gxyz_low_right, accelerometerOffset_low_right, gyroscopeOffset_low_right );
    get_Data(I2C_MASTER_1, MPU9250_SENSOR_ADDR69, Axyz_high_right, Gxyz_high_right, accelerometerOffset_high_right, gyroscopeOffset_high_right );

    ESP_LOGI(TAG, "IMU Calibration en cours");

  for (int i = 0; i < NUM_SAMPLES; i++) {
    
    // Mise à jour des valeurs min/max des capteurs pour chaque IMU pour chaque axe

    for (int j = 0; j < 3; j ++)
    {
        accelerometer_low_left[j].min = fminf(accelerometer_low_left[j].min, Axyz_low_left[j]);
        accelerometer_low_left[j].max = fmaxf(accelerometer_low_left[j].max, Axyz_low_left[j]);
        accelerometer_high_left[j].min = fminf(accelerometer_high_left[j].min, Axyz_high_left[j]);
        accelerometer_high_left[j].max = fmaxf(accelerometer_high_left[j].max, Axyz_high_left[j]);
        accelerometer_low_right[j].min = fminf(accelerometer_low_right[j].min, Axyz_low_right[j]);
        accelerometer_low_right[j].max = fmaxf(accelerometer_low_right[j].max, Axyz_low_right[j]);
        accelerometer_high_right[j].min = fminf(accelerometer_high_right[j].min, Axyz_high_right[j]);
        accelerometer_high_right[j].max = fmaxf(accelerometer_high_right[j].max, Axyz_high_right[j]);

        gyroscope_low_left[j].min = fminf(gyroscope_low_left[j].min, Gxyz_low_left[j]);
        gyroscope_low_left[j].max = fmaxf(gyroscope_low_left[j].max, Gxyz_low_left[j]);
        gyroscope_high_left[j].min = fminf(gyroscope_high_left[j].min, Gxyz_high_left[j]);
        gyroscope_high_left[j].max = fmaxf(gyroscope_high_left[j].max, Gxyz_high_left[j]);
        gyroscope_low_right[j].min = fminf(gyroscope_low_right[j].min, Gxyz_low_right[j]);
        gyroscope_low_right[j].max = fmaxf(gyroscope_low_right[j].max, Gxyz_low_right[j]);
        gyroscope_high_right[j].min = fminf(gyroscope_high_right[j].min, Gxyz_high_right[j]);
        gyroscope_high_right[j].max = fmaxf(gyroscope_high_right[j].max, Gxyz_high_right[j]);

        usleep(1000);
    }
  }

  for (int imuaxe = 0; imuaxe < 3; imuaxe++) {
    accelerometerOffset_low_left[imuaxe] = (accelerometer_low_left[imuaxe].max + accelerometer_low_left[imuaxe].min) / 2.0f;
    accelerometerOffset_high_left[imuaxe] = (accelerometer_high_left[imuaxe].max + accelerometer_high_left[imuaxe].min) / 2.0f;
    accelerometerOffset_low_right[imuaxe] = (accelerometer_low_right[imuaxe].max + accelerometer_low_right[imuaxe].min) / 2.0f;
    accelerometerOffset_high_right[imuaxe] = (accelerometer_high_right[imuaxe].max + accelerometer_high_right[imuaxe].min) / 2.0f;

    gyroscopeOffset_low_left[imuaxe] = (gyroscope_low_left[imuaxe].max + gyroscope_low_left[imuaxe].min) / 2.0f;
    gyroscopeOffset_high_left[imuaxe] = (gyroscope_high_left[imuaxe].max + gyroscope_high_left[imuaxe].min) / 2.0f;
    gyroscopeOffset_low_right[imuaxe] = (gyroscope_low_right[imuaxe].max + gyroscope_low_right[imuaxe].min) / 2.0f;
    gyroscopeOffset_high_right[imuaxe] = (gyroscope_high_right[imuaxe].max + gyroscope_high_right[imuaxe].min) / 2.0f;

  }

    ESP_LOGI(TAG, "IMU Calibration done");
}

//************************************** Filter functions ******************************************//

void filter(float* accel_LL, float* gyro_LL, float* accel_LR, float* gyro_LR, float* accel_HR, float* gyro_HR, float* accel_HL, float* gyro_HL )
{
    // Kalman filter for IMU data

    // Xk = current position
    // Kk = Kalman gain
    // Zk = measured value
    // Xk-1 = previous estimation

    float measured_value_accel[3];
    float measured_value_gyro[3];

    // Low left
    for (uint8_t i = 0; i < 3; i++)
    {
        if((accel_LL[i]!=0.0) || (gyro_LL[i]!=0.0)){
        measured_value_accel[i] = Kk * accel_LL[i] + (1 - Kk) * previous_mesure_accel_LL[i];
        measured_value_gyro[i] = Kk * gyro_LL[i] + (1 - Kk) * previous_mesure_gyro_LL[i];

        previous_mesure_accel_LL[i] = measured_value_accel[i];
        previous_mesure_gyro_LL[i] = measured_value_gyro[i];

        accel_LL[i] = measured_value_accel[i];
        gyro_LL[i] = measured_value_gyro[i];
        }
        else{
            accel_LL[i] = previous_mesure_accel_LL[i];
            gyro_LL[i] = previous_mesure_gyro_LL[i];
        }
    }

    // Low right
    for (uint8_t i = 0; i < 3; i++)
    {
        if((accel_LL[i]!=0.0) || (gyro_LR[i]!=0.0)){
        measured_value_accel[i] = Kk * accel_LR[i] + (1 - Kk) * previous_mesure_accel_LR[i];
        measured_value_gyro[i] = Kk * gyro_LR[i] + (1 - Kk) * previous_mesure_gyro_LR[i];

        previous_mesure_accel_LR[i] = measured_value_accel[i];
        previous_mesure_gyro_LR[i] = measured_value_gyro[i];

        accel_LR[i] = measured_value_accel[i];
        gyro_LR[i] = measured_value_gyro[i];
        }
        else{
            accel_LR[i] = previous_mesure_accel_LR[i];
            gyro_LR[i] = previous_mesure_gyro_LR[i];
        }
    }

    // High right
    for (uint8_t i = 0; i < 3; i++)
    {
        if((accel_HR[i]!=0.0) || (gyro_HR[i]!=0.0)){
        measured_value_accel[i] = Kk * accel_HR[i] + (1 - Kk) * previous_mesure_accel_HR[i];
        measured_value_gyro[i] = Kk * gyro_HR[i] + (1 - Kk) * previous_mesure_gyro_HR[i];

        previous_mesure_accel_HR[i] = measured_value_accel[i];
        previous_mesure_gyro_HR[i] = measured_value_gyro[i];

        accel_HR[i] = measured_value_accel[i];
        gyro_HR[i] = measured_value_gyro[i];
        }
        else{
            accel_HR[i] = previous_mesure_accel_HR[i];
            gyro_HR[i] = previous_mesure_gyro_HR[i];
        }
    }

    // High left
    for (uint8_t i = 0; i < 3; i++)
    {
        if((accel_HL[i]!=0.0) || (gyro_HL[i]!=0.0)){
        measured_value_accel[i] = Kk * accel_HL[i] + (1 - Kk) * previous_mesure_accel_HL[i];
        measured_value_gyro[i] = Kk * gyro_HL[i] + (1 - Kk) * previous_mesure_gyro_HL[i];

        previous_mesure_accel_HL[i] = measured_value_accel[i];
        previous_mesure_gyro_HL[i] = measured_value_gyro[i];

        accel_HL[i] = measured_value_accel[i];
        gyro_HL[i] = measured_value_gyro[i];
        }
        else{
            accel_HL[i] = previous_mesure_accel_HL[i];
            gyro_HL[i] = previous_mesure_gyro_HL[i];
        }
    }
}

//************************************** PWM functions ******************************************//

void PWM_initialize()
{
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, SERVO_PULSE_GPIO_L); // To drive a RC servo, one MCPWM generator is enough
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, SERVO_PULSE_GPIO_R); // To drive a RC servo, one MCPWM generator is enough


    mcpwm_config_t pwm_config = {
        .frequency = 50, // frequency = 50Hz, i.e. for every servo motor time period should be 20ms
        .cmpr_a = 0,     // duty cycle of PWMxA = 0
        .counter_mode = MCPWM_UP_COUNTER,
        .duty_mode = MCPWM_DUTY_MODE_0,
    };
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    // ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, SERVO_MAX_PULSEWIDTH_US));
    // vTaskDelay(pdMS_TO_TICKS(1000)); //Add delay to check max range
    // ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A,SERVO_MIN_PULSEWIDTH_US));
    // vTaskDelay(pdMS_TO_TICKS(1000)); //Add delay to check min range

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);
    // ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, SERVO_MAX_PULSEWIDTH_US));
    // vTaskDelay(pdMS_TO_TICKS(1000)); //Add delay to check max range
    // ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B,SERVO_MIN_PULSEWIDTH_US));
    // vTaskDelay(pdMS_TO_TICKS(1000)); //Add delay to check min range
}

//************************************** Data functions ******************************************//

float angleToPWM(float angle) {
  // Plage de l'angle : 0 degré à 180 degrés
  // Plage de la commande PWM : 500 µs à 2500 µs

  // Convertir l'angle en commande PWM
  float pwm = ((angle - 0) * (2500 - 500) / (180 - 0)) + 500;

  return pwm;
}

uint8_t calculate_angle (float* A_low, float* G_low, float* A_high, float* G_high)
{
    // Calculer l'angle d'Euler à partir des données de l'IMU low
    // float roll1 = atan2(A_low[1], A_low[2]) * 180 / M_PI;
    // float pitch1 = atan2(-A_low[0], sqrt(A_low[1] * A_low[1] + A_low[2] * A_low[2])) * 180 / M_PI;
    float yaw1 = atan2(G_low[0], sqrt(G_low[1] * G_low[1] + G_low[2] * G_low[2])) * 180 / M_PI;

    // Calculer l'angle d'Euler à partir des données de l'IMU high
    // float roll2 = atan2(A_high[1], A_high[2]) * 180 / M_PI;
    // float pitch2 = atan2(-A_high[0], sqrt(A_high[1] * A_high[1] + A_high[2] * A_high[2])) * 180 / M_PI;
    float yaw2 = atan2(G_high[0], sqrt(G_high[1] * G_high[1] + G_high[2] * G_high[2])) * 180 / M_PI;

    // Calculer l'angle entre les deux IMU
    float angle = atan2(sin(yaw2-yaw1), cos(yaw2-yaw1)) * 180 / M_PI;

    return angle;
}

//************************************** Mouvement algorithm functions ******************************************//

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
    if(averaging_counter < 1000){
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
    if(averaging_counter >= 1000){
        for(int i = 0; i < 3; i++){
            baseline_Gxyz_high_left[i] = (baseline_Gxyz_high_left[i]/averaging_counter);
            baseline_Gxyz_high_right[i] = (baseline_Gxyz_high_right[i]/averaging_counter);
            baseline_Gxyz_low_left[i] = (baseline_Gxyz_low_left[i]/averaging_counter);
            baseline_Gxyz_low_right[i] = (baseline_Gxyz_low_right[i]/averaging_counter);
            baseline_Axyz_high_left[i] = (baseline_Axyz_high_left[i]/averaging_counter);
            baseline_Axyz_high_right[i] = (baseline_Axyz_high_right[i]/averaging_counter);
            baseline_Axyz_low_left[i] = (baseline_Axyz_low_left[i]/averaging_counter);
            baseline_Axyz_low_right[i] = (baseline_Axyz_low_right[i]/averaging_counter);
            set_baseline_extr_value_gyro(baseline_Gxyz_high_left[i]);
            set_baseline_extr_value_gyro(baseline_Gxyz_high_right[i]);
            set_baseline_extr_value_gyro(baseline_Gxyz_low_left[i]);
            set_baseline_extr_value_gyro(baseline_Gxyz_low_right[i]);
            set_baseline_extr_value_acc(baseline_Axyz_high_left[i]);
            set_baseline_extr_value_acc(baseline_Axyz_high_right[i]);
            set_baseline_extr_value_acc(baseline_Axyz_low_left[i]);
            set_baseline_extr_value_acc(baseline_Axyz_low_right[i]);
        }
    }
}

uint8_t algo_mvt(float Axyz_high_left[3], float Axyz_high_right[3], float Axyz_low_left[3], float Axyz_low_right[3]){

    // Output variable use to carry state values
    output = 0;

    // Logic for state activation 
    // Because of the way the IMU are positioned, X is left/right, Y is up/down and Z is forward/backward

    // Left leg raised and in front
    if((Axyz_high_left[1] > (Axyz_high_right[1]+highest_baseline_value_acc)) && (Axyz_low_left[1] > (Axyz_low_right[1]+highest_baseline_value_acc))){
        output = (STATUS_RAISED_LEFT_LEG | STATUS_LEFT_LEG_FORWARD | STATUS_LOWERED_RIGHT_LEG | STATUS_RIGHT_LEG_BACKWARD);
    }
    // Right leg raised and in front
    else if((Axyz_high_right[1] > (Axyz_high_left[1]+highest_baseline_value_acc)) && (Axyz_low_right[1] > (Axyz_low_left[1]+highest_baseline_value_acc))){
        output = (STATUS_LOWERED_LEFT_LEG | STATUS_LEFT_LEG_BACKWARD | STATUS_RAISED_RIGHT_LEG | STATUS_RIGHT_LEG_FORWARD);
    }
    // Left Leg leg forward but not raised
    else if((Axyz_high_left[1] < (Axyz_high_right[1]+highest_baseline_value_acc)) && (Axyz_low_left[2] > (Axyz_low_right[2]+highest_baseline_value_acc))){
        output = (STATUS_LOWERED_LEFT_LEG | STATUS_LEFT_LEG_FORWARD | STATUS_LOWERED_RIGHT_LEG | STATUS_RIGHT_LEG_BACKWARD);
    }
    // Right leg forward but not raised
    else if((Axyz_high_right[1] < (Axyz_high_left[1]+highest_baseline_value_acc)) && (Axyz_low_right[2] > (Axyz_low_left[2]+highest_baseline_value_acc))){
        output = (STATUS_LOWERED_LEFT_LEG | STATUS_LEFT_LEG_BACKWARD | STATUS_LOWERED_RIGHT_LEG | STATUS_RIGHT_LEG_FORWARD);
    }
    // Left leg and right leg lowered (squat)
    // Right leg forward but not raised
    else if((Axyz_high_right[1] < (baseline_Axyz_high_right[1]-highest_baseline_value_acc)) && (Axyz_high_left[1] < (baseline_Axyz_high_left[1]-highest_baseline_value_acc))){
        output = (STATUS_LOWERED_LEFT_LEG | STATUS_LOWERED_RIGHT_LEG);  // Squat 
    }

    // // Left leg raised and in front
    // if((Axyz_high_left[1] > (Axyz_high_right[1]+highest_baseline_value_acc))){
    //     output = (STATUS_RAISED_LEFT_LEG | STATUS_LEFT_LEG_FORWARD | STATUS_LOWERED_RIGHT_LEG | STATUS_RIGHT_LEG_BACKWARD);
    // }
    // // Right leg raised and in front
    // else if((Axyz_high_right[1] > (Axyz_high_left[1]+highest_baseline_value_acc))){
    //     output = (STATUS_LOWERED_LEFT_LEG | STATUS_LEFT_LEG_BACKWARD | STATUS_RAISED_RIGHT_LEG | STATUS_RIGHT_LEG_FORWARD);
    // }
    // // Left Leg leg forward but not raised
    // else if((Axyz_high_left[1] < (Axyz_high_right[1]+highest_baseline_value_acc))){
    //     output = (STATUS_LOWERED_LEFT_LEG | STATUS_LEFT_LEG_FORWARD | STATUS_LOWERED_RIGHT_LEG | STATUS_RIGHT_LEG_BACKWARD);
    // }
    // // Right leg forward but not raised
    // else if((Axyz_high_right[1] < (Axyz_high_left[1]+highest_baseline_value_acc))){
    //     output = (STATUS_LOWERED_LEFT_LEG | STATUS_LEFT_LEG_BACKWARD | STATUS_LOWERED_RIGHT_LEG | STATUS_RIGHT_LEG_FORWARD);
    // }

    return output;
}

void get_desired_angle(uint8_t output, float* Dangle_left, float* Dangle_right) {

    if (output & STATUS_RAISED_LEFT_LEG){
        *Dangle_left = (float)ANGLE_MAX;
        printf("Status: STATUS_RAISED_LEFT_LEG \n");
    }
    else if (output & STATUS_LOWERED_RIGHT_LEG){
        *Dangle_right += (float)ANGLE_MIN;
        printf("Status: STATUS_LOWERED_RIGHT_LEG \n");
    }
    else if (output & STATUS_LOWERED_LEFT_LEG){
        *Dangle_left += (float)ANGLE_MIN;
        printf("Status: STATUS_LOWERED_LEFT_LEG \n");
    }
    else if (output & STATUS_RAISED_RIGHT_LEG){
        *Dangle_right += (float)ANGLE_MAX;
        printf("Status: STATUS_RAISED_RIGHT_LEG \n");
    }
    // printf("The value of Output: %d\n", output);
}

//************************************** Main loop ******************************************//

static void *imu_thread(void * arg);
static void *pwm_thread(void * arg);
// static void *uart_thread(void * arg);


void app_main(void)
{
    pthread_t thread_imu;
    pthread_t thread_pwm;
    int res;

    // I2C initialization
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    // Sensor initialization
    IMU_initialize(I2C_MASTER_0, MPU9250_SENSOR_ADDR68);    // Low left 17 18
    IMU_initialize(I2C_MASTER_0, MPU9250_SENSOR_ADDR69);    // High left
    IMU_initialize(I2C_MASTER_1, MPU9250_SENSOR_ADDR68);    // Low right 15 16
    IMU_initialize(I2C_MASTER_1, MPU9250_SENSOR_ADDR69);    // High right

    // calibrateIMUs();
    ESP_LOGI(TAG, "IMU initialized successfully");

    // PWM initialization
    PWM_initialize();
    ESP_LOGI(TAG, "PWM initialized successfully");

    // Create a pthread for IMU reception
    res = pthread_create(&thread_imu, NULL, imu_thread, NULL);
    assert(res == 0);
    printf("Created thread 0x%lx\n", thread_imu);

    // Create a pthread for PWM sending
    res = pthread_create(&thread_pwm, NULL, pwm_thread, NULL);
    assert(res == 0);
    printf("Created thread 0x%lx\n", thread_pwm);

    // DO THREADs STUFF


    // End of threads
    res = pthread_join(thread_imu, NULL);
    assert(res == 0);
    res = pthread_join(thread_pwm, NULL);
    assert(res == 0);

    printf("Thread has exited\n\n");
}

static void *imu_thread(void * arg)
{
    usleep(250 * 1000);
    printf("This thread has ID 0x%lx and %u bytes free stack\n", pthread_self(), uxTaskGetStackHighWaterMark(NULL));

    /* Read the MPU9250 WHO_AM_I register, on power up the register should have the value 0x71 */
    while(1)
    {
        // Data recuperation
        get_Data(I2C_MASTER_0, MPU9250_SENSOR_ADDR68, Axyz_low_left, Gxyz_low_left, accelerometerOffset_low_left, gyroscopeOffset_low_left );
        get_Data(I2C_MASTER_0, MPU9250_SENSOR_ADDR69, Axyz_high_left, Gxyz_high_left, accelerometerOffset_high_left, gyroscopeOffset_high_left );
        get_Data(I2C_MASTER_1, MPU9250_SENSOR_ADDR68, Axyz_low_right, Gxyz_low_right, accelerometerOffset_low_right, gyroscopeOffset_low_right );
        get_Data(I2C_MASTER_1, MPU9250_SENSOR_ADDR69, Axyz_high_right, Gxyz_high_right, accelerometerOffset_high_right, gyroscopeOffset_high_right );

        // Kalman Filtering 
        filter(Axyz_low_left, Gxyz_low_left, Axyz_low_right, Gxyz_low_right, Axyz_high_right, Gxyz_high_right, Axyz_high_left, Gxyz_high_left);

        // Data print 

        // printf("Acceleration(g) of X,Y,Z: \n");
        // printf("AccLL%f, %f, %f \n",Axyz_low_left[0],Axyz_low_left[1],Axyz_low_left[2]);
        // printf("AccHL: %f, %f, %f \n",Axyz_high_left[0],Axyz_high_left[1],Axyz_high_left[2]);
        // printf("AccLR: %f, %f, %f \n",Axyz_low_right[0],Axyz_low_right[1],Axyz_low_right[2]);
        // printf("%f, %f, %f \n",Axyz_high_right[0],Axyz_high_right[1],Axyz_high_right[2]);

        // printf("Gyroscope(g) of X,Y,Z: \n");
        // printf("GyroLL : %f, %f, %f \n",Gxyz_low_left[0],Gxyz_low_left[1],Gxyz_low_left[2]);
        // printf("GyroHL : %f, %f, %f \n",Gxyz_high_left[0],Gxyz_high_left[1],Gxyz_high_left[2]);
        // printf("GyroLR : %f, %f, %f \n",Gxyz_low_right[0],Gxyz_low_right[1],Gxyz_low_right[2]);
        // printf("GyroHR : %f, %f, %f \n",Gxyz_high_right[0],Gxyz_high_right[1],Gxyz_high_right[2]);
        
        if(averaging_counter>=1000){            // Checker si détection pas trop lente
            algo_mvt(Axyz_high_left, Axyz_high_right, Axyz_low_left, Axyz_low_right);
            // printf("highest baseline value : %f \n",highest_baseline_value_acc);
            // printf("lowest baseline value : %f \n",lowest_baseline_value_acc);

            // Delay 500ms
            usleep(1000 * 100);
        }
        else  calculate_imu_error(Gxyz_high_left, Gxyz_high_right, Gxyz_low_left, Gxyz_low_right,Axyz_high_left, Axyz_high_right, Axyz_low_left, Axyz_low_right);


        // Delay 50ms
        // usleep(50*1000);

    }

    return NULL;
}

static void *pwm_thread(void * arg)
{
    usleep(250 * 1000);
    printf("This thread has ID 0x%lx and %u bytes free stack\n", pthread_self(), uxTaskGetStackHighWaterMark(NULL));
    
    // Variables de commande
    float position_cible_left = 90;
    float position_cible_right = 90;
    uint16_t commandePWM_left = 0;
    uint16_t commandePWM_right = 0;

    while(1)
    {
        // Boucle de régulation de position (PID)
        
        // Récupération des angles cibles
        if(averaging_counter>=1000){ 
            // get_desired_angle(output, &position_cible_left, &position_cible_right);
        }

        // filter(Axyz_low_left, Gxyz_low_left, Axyz_low_right, Gxyz_low_right, Axyz_high_right, Gxyz_high_right, Axyz_high_left, Gxyz_high_left);

        // if(Axyz_low_left[0] - previous_mesure_accel_LL[0] > 0.2 && Axyz_low_left[2] - previous_mesure_accel_LL[2] > 0.2)
        
        // uint8_t angle = calculate_angle(Axyz_low_right, Gxyz_low_right, Axyz_high_right, Gxyz_high_right);
        // printf("Position cible :%d\n", angle);
        
        // position_cible_left = calculate_angle(Axyz_low_right, Gxyz_low_right, Axyz_high_left, Gxyz_high_left);
        // printf("Position cible :%f\n", position_cible_left);

        // Conversion de l'angle en PWM
        commandePWM_left = angleToPWM(position_cible_left);
        printf("Commande PWM :%d",commandePWM_left);
        commandePWM_right = angleToPWM(position_cible_right);
        printf("Commande PWM :%d",commandePWM_right);

        // if ((output & STATUS_RAISED_LEFT_LEG) || (output & STATUS_RAISED_RIGHT_LEG)){usleep(1000*1000);}
        // Send command Left
        ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, commandePWM_left));
        vTaskDelay(pdMS_TO_TICKS(100)); //Add delay 

        // Send command Right
        ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, commandePWM_right));
        vTaskDelay(pdMS_TO_TICKS(100)); //Add delay 

        // Delay 500ms in total
        // usleep(1000 * 300);
    }

    return NULL;
}

