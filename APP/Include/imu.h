#ifndef __APP_IMU_H
#define __APP_IMU_H

#include <stdbool.h>
#include <stdint.h>
#include "imu_lsm6dsv.h"
#include "user_config.h"

// 陀螺仪采样范围±4g,16位采样(-32768~32767), 
// 陀螺仪数据°/s的单位化系数为2000/32768 = 1000/16384, 

#define GRAVITY_MS2    (9.8)

#define GYRO_RANGE_2000DPS
//#define GYRO_RANGE_1000DPS
//#define GYRO_RANGE_500DPS

// 单位g的采样值为8192(2^13)
#define IMU_ACC_G_VALUE (8192)
#define IMU_ACC_G_SQUARE_VALUE (IMU_ACC_G_VALUE*IMU_ACC_G_VALUE)
#define IMU_ACC_G_SHIFT_BIT_NUM (0x0D)

// 加速度值归一化系数，8192*1024，保证求平方根导数的精度
#define IMU_ACC_G_NORM_VALUE (8388608)
#define IMU_ACC_G_NORM_SHIFT_BIT_NUM (0x0A)  // 1024 = 2^10 
#define IMU_ACC_TO_MS2  (9.8/8192)

// 由加速度采样值数据格式转换为q14格式
#define IMU_G_TO_Q14_SHIFT_BIT_NUM  (Q14_SHIFT_BIT_NUM - IMU_ACC_G_SHIFT_BIT_NUM)

#define  IMU_DATA_STORAGE_IN_FLASH

typedef enum {
    IMU_LSM6DSV = 0,
}imu_type_t;

extern imu_type_t imu_type;
extern float imu_gyro_to_deg;

void remo_imu_init(void);
void remo_imu_update(void);

const int16_t *remo_imu_get_gyro_smp(void);
const float *remo_imu_get_gyro_corr(void);
const float *remo_imu_get_gyro_fine_corr(void);

const int16_t *remo_imu_get_accel_smp(void);
const float *remo_imu_get_accel_corr(void);

int16_t remo_imu_get_temperature_smp(void);
const float *remo_imu_get_gyro_tempreture_offset(void);

uint8_t remo_imu_get_i2c_wr_state(void);


bool remo_imu_data_refreshed(void);
void remo_imu_clear_data_refresh_flag(void);

int16_t remo_imu_get_gyro_bias(uint8_t index);
const int16_t *remo_imu_get_accel_bias(void);

void remo_imu_set_gyro_bias(int16_t bias_xaxis, int16_t bias_yaxis, int16_t bias_zaxis);
void remo_imu_set_accel_bias(int16_t bias_xaxis, int16_t bias_yaxis, int16_t bias_zaxis);
void remo_imu_set_accel_trans_matrix(const float *trans_matrix);
void remo_imu_set_gyro_trans_matrix(const float *trans_matrix);

void remo_imu_set_stop_sampling_flag(bool flag);

void remo_imu_reset_register_filter(void);

void remo_imu_set_tempreture_offset(float *temperature_offset, uint8_t state);
void remo_imu_set_fine_offset(float *fine_offset);

float remo_imu_get_gyro_deg(uint8_t index);

int16_t remo_imu_get_xaixs_gyro_ddps(void);
int16_t remo_imu_get_yaixs_gyro_ddps(void);
int16_t remo_imu_get_zaixs_gyro_ddps(void);
int16_t remo_imu_get_xaxis_accel_cms2(void);
int16_t remo_imu_get_yaxis_accel_cms2(void);
int16_t remo_imu_get_zaxis_accel_cms2(void);

#endif   // __APP_IMU_H
