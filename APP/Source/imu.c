#include "imu.h"
#include "bsp_spi.h"
#include "user_config.h"

struct{
    lsm6dsvd_params_t lsm6dsvd_params;
}imu_params = {
    .lsm6dsvd_params = {
        .who_am_i = 0xAA,
        .temp_output = 0,
        .gyro_output = {0},
        .accel_output = {0},
        .ctrl1_accel = 0x00,
        .ctrl2_gyro = 0x00,
        .ctrl3 = 0x00, //0x04,
        .ctrl4 = 0x00,
        .ctrl5 = 0x00,
        .ctrl6_gyro = 0x00,
        .ctrl7 = 0x00,
        .ctrl8_accel = 0x00,
        .ctrl9_accel = 0x00,
        .ctrl10 = 0x00 //0xE0
    }
};


struct{
    uint8_t spi_wr_state;    // I2C读写状态，0:正常，>1: 不同错误

    int16_t accel_smp[3];
    int16_t gyro_smp[3];
    int16_t temp_smp;

    float accel_trans_matrix[9];
    float gyro_trans_matrix[9];
    float gyro_fine_offset[3];
      
    float accel_correct[3];
    float gyro_fine_correct[3];
    float gyro_correct[3];

    int16_t accel_offset[3];
    int16_t gyro_offset[3];
    float gyro_temp_offset[3];

	
    int16_t temp_correct;
    
    bool run_normal_flag;

    bool data_reflash_flag;

    bool stop_sampling_flag;

}imu_info = {
    .spi_wr_state = 1,
    .run_normal_flag = true,
    .data_reflash_flag = false,
    .stop_sampling_flag = true,
    .accel_trans_matrix = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f},
    .gyro_trans_matrix = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f},
    .accel_offset = {0, 0, 0},
    .gyro_offset = {0, 0, 0},
    .gyro_temp_offset = {0, 0 ,0}
};

static uint8_t remo_imu_spi_send(uint8_t cmd, uint8_t *data, uint8_t len);
static uint8_t remo_imu_spi_receive(uint8_t cmd, uint8_t *data, uint8_t len);
static uint8_t remo_imu_spi_receive_simple(uint8_t cmd, uint8_t *data, uint8_t len);

static void remo_imu_params_init(void);
static void remo_imu_register_init(void);

imu_type_t imu_type = 0;
float imu_gyro_to_deg = 0.0f;

uint8_t imu_spi_div = 1;

/************************************************************************************************
 * 函数名称: remo_imu_params_init
 * 输入参数: void
 * 返回结果: void
 * 功能描述: 初始化imu的参数
**************************************************************************************************/
static void remo_imu_params_init(void)
{
    bool flash_data_invalid_flag = false;


    // 综合配置
    imu_params.lsm6dsvd_params.ctrl3.types.sw_reset = 1;   // 软件reset
    imu_params.lsm6dsvd_params.ctrl3.types.if_inc = 1;     // 多字节读取时寄存器地址自动增加
    imu_params.lsm6dsvd_params.ctrl3.types.bdu = 1;
    imu_params.lsm6dsvd_params.ctrl4.types.drdy_mask = 0;
        
    // 陀螺仪输出设置
    imu_params.lsm6dsvd_params.ctrl2_gyro.types.op_mode_g = 0;  // 默认陀螺仪高性能模式使能
#if (IMU_ODR__VEL_CTRL__FREQ == 7680)
    imu_params.lsm6dsvd_params.ctrl2_gyro.types.odr_g = 12;     // 9:960Hz, 10:1.92KHz, 11:3.84KHz, 12:7.68KHz
#elif (IMU_ODR__VEL_CTRL__FREQ == 3840)
    imu_params.lsm6dsvd_params.ctrl2_gyro.types.odr_g = 11;     // 9:960Hz, 10:1.92KHz, 11:3.84KHz, 12:7.68KHz
#elif (IMU_ODR__VEL_CTRL__FREQ == 1920)
    imu_params.lsm6dsvd_params.ctrl2_gyro.types.odr_g = 10;     // 9:960Hz, 10:1.92KHz, 11:3.84KHz, 12:7.68KHz
#elif (IMU_ODR__VEL_CTRL__FREQ == 960)
    imu_params.lsm6dsvd_params.ctrl2_gyro.types.odr_g = 9;     // 9:960Hz, 10:1.92KHz, 11:3.84KHz, 12:7.68KHz
#else
    imu_params.lsm6dsvd_params.ctrl2_gyro.types.odr_g = 10;     // 9:960Hz, 10:1.92KHz, 11:3.84KHz, 12:7.68KHz
#endif
#ifdef GYRO_RANGE_2000DPS 
    imu_params.lsm6dsvd_params.ctrl6_gyro.types.fs_g = 4;  // 4:2000dps, 3:1000dps, 2:500dps
#elif defined(GYRO_RANGE_1000DPS)
    imu_params.lsm6dsvd_params.ctrl6_gyro.types.fs_g = 3;  // 4:2000dps, 3:1000dps, 2:500dps
#elif defined(GYRO_RANGE_500DPS)
    imu_params.lsm6dsvd_params.ctrl6_gyro.types.fs_g = 2;  // 4:2000dps, 3:1000dps, 2:500dps
#endif

    // 陀螺仪滤波器设置
    imu_params.lsm6dsvd_params.ctrl7.types.lpf1_g_en = 1;      // 辅助spi不使能时使能陀螺仪lpf1
    imu_params.lsm6dsvd_params.ctrl6_gyro.types.lpf1_g_bw = 1; // odr在1.92KHz下，陀螺仪lpf1带宽选择0:272Hz, 1:209Hz, 2:155Hz
                                                               // odr在3.84KHz下，陀螺仪lpf1带宽选择0:279Hz, 1:212Hz, 2:156Hz


    // 加速度计输出设置
    imu_params.lsm6dsvd_params.ctrl1_accel.types.odr_xl = imu_params.lsm6dsvd_params.ctrl2_gyro.types.odr_g-1;     // 9:960Hz, 10:1.92KHz, 11:3.84KHz, 12:7.68KHz
    imu_params.lsm6dsvd_params.ctrl1_accel.types.op_mode_xl = 0;  // 默认加速度计高性能模式使能
    imu_params.lsm6dsvd_params.ctrl8_accel.types.fs_xl = 1;       // 1:±4g
        
    // 加速度计滤波器设置
    imu_params.lsm6dsvd_params.ctrl9_accel.types.lpf2_xl_en = 1;  // 加速度计低通滤波器2使能
    imu_params.lsm6dsvd_params.ctrl9_accel.types.hp_slope_xl_en = 0;   // 加速度计斜率滤波不使能
    imu_params.lsm6dsvd_params.ctrl9_accel.types.hp_ref_mode_xl = 0;   // 加速度计高通滤波不使能
    imu_params.lsm6dsvd_params.ctrl8_accel.types.hp_lpf2_xl_bw = 3; // 低通滤波器2带宽，2:odr/20, 3:odr/45
 

//    remo_flash_read_halfwords(FLASH_ADDR_ACC_MATRIX, 9, (uint16_t *)imu_info.accel_trans_matrix);
//    remo_flash_read_halfwords(FLASH_ADDR_GYRO_MATRIX, 9, (uint16_t *)imu_info.gyro_trans_matrix);
//    remo_flash_read_halfwords(FLASH_ADDR_ACC_OFFSET, 3, (uint16_t *)imu_info.accel_offset);
//    remo_flash_read_halfwords(FLASH_ADDR_GYRO_RST_OFFSET, 3, (uint16_t *)imu_info.gyro_offset);

//    flash_data_invalid_flag = (imu_info.accel_trans_matrix[0] == (int16_t)0x0000 || imu_info.accel_trans_matrix[4] == (int16_t)0x0000 
//                            || imu_info.accel_trans_matrix[8] ==(int16_t)0x0000) || (imu_info.accel_trans_matrix[0] == (int16_t)0xffff
//                            || imu_info.accel_trans_matrix[4] == (int16_t)0xffff || imu_info.accel_trans_matrix[8] == (int16_t)0xffff);
//    if (isnan(imu_info.accel_trans_matrix[0]) || isnan(imu_info.accel_trans_matrix[0]) ||  flash_data_invalid_flag)
//    {
        imu_info.accel_trans_matrix[0] = IMU_ACC_TO_MS2;
        imu_info.accel_trans_matrix[1] = 0;
        imu_info.accel_trans_matrix[2] = 0;
        imu_info.accel_trans_matrix[3] = 0;
        imu_info.accel_trans_matrix[4] = IMU_ACC_TO_MS2;
        imu_info.accel_trans_matrix[5] = 0;
        imu_info.accel_trans_matrix[6] = 0;
        imu_info.accel_trans_matrix[7] = 0;
        imu_info.accel_trans_matrix[8] = IMU_ACC_TO_MS2;
//    }

//    flash_data_invalid_flag = (imu_info.gyro_trans_matrix[0] == (int16_t)0x0000 || imu_info.gyro_trans_matrix[4] == (int16_t)0x0000 
//                            || imu_info.gyro_trans_matrix[8] ==(int16_t)0x0000) || (imu_info.gyro_trans_matrix[0] == (int16_t)0xffff
//                            || imu_info.gyro_trans_matrix[4] == (int16_t)0xffff || imu_info.gyro_trans_matrix[8] == (int16_t)0xffff);
//    if (isnan(imu_info.gyro_trans_matrix[0]) || isnan(imu_info.gyro_trans_matrix[8]) ||  flash_data_invalid_flag || \
//            (abs(imu_info.gyro_trans_matrix[0]) << (CONST_1_Q14 >> 1)) || \
//            (abs(imu_info.gyro_trans_matrix[4]) << (CONST_1_Q14 >> 1)) || \
//            (abs(imu_info.gyro_trans_matrix[8]) << (CONST_1_Q14 >> 1)))
//    {
        imu_info.gyro_trans_matrix[0] = IMU_GYRO_TO_DEG;
        imu_info.gyro_trans_matrix[1] = 0;
        imu_info.gyro_trans_matrix[2] = 0;
        imu_info.gyro_trans_matrix[3] = 0;
        imu_info.gyro_trans_matrix[4] = IMU_GYRO_TO_DEG;
        imu_info.gyro_trans_matrix[5] = 0;
        imu_info.gyro_trans_matrix[6] = 0;
        imu_info.gyro_trans_matrix[7] = 0;
        imu_info.gyro_trans_matrix[8] = IMU_GYRO_TO_DEG;
//    }

//    flash_data_invalid_flag = (imu_info.accel_offset[0] == (int16_t)0xffff || imu_info.accel_offset[1] == (int16_t)0xffff 
//                            || imu_info.accel_offset[2] == (int16_t)0xffff);
//    if (isnan(imu_info.accel_offset[0]) || isnan(imu_info.accel_offset[2]) ||  flash_data_invalid_flag)
//    {
        imu_info.accel_offset[0] = 0x0000;
        imu_info.accel_offset[1] = 0x0000;
        imu_info.accel_offset[2] = 0x0000;
//    }

//    flash_data_invalid_flag = (imu_info.gyro_offset[0] == (int16_t)0xffff || imu_info.gyro_offset[1] == (int16_t)0xffff 
//                            || imu_info.gyro_offset[2] == (int16_t)0xffff);
//    if (isnan(imu_info.gyro_offset[0]) || isnan(imu_info.gyro_offset[2]) ||  flash_data_invalid_flag)
//    {
        imu_info.gyro_offset[0] = 0x0000;
        imu_info.gyro_offset[1] = 0x0000;
        imu_info.gyro_offset[2] = 0x0000;
//    }

    imu_info.gyro_temp_offset[0] = 0x0000;
    imu_info.gyro_temp_offset[1] = 0x0000;
    imu_info.gyro_temp_offset[2] = 0x0000;
    
//    imu_info.stop_sampling_flag = false;
}


/************************************************************************************************
 * 函数名称: remo_imu_register_init
 * 输入参数: void
 * 返回结果: void
 * 功能描述: imu芯片寄存器配置初始化
**************************************************************************************************/
static void remo_imu_register_init(void)
{
      // 软件reset
      imu_info.spi_wr_state = remo_imu_spi_send(LSM6DSVD_CTRL3_C_12, 
                          (uint8_t *)&imu_params.lsm6dsvd_params.ctrl3, 1);
      clock_cpu_delay_ms(20);
      imu_params.lsm6dsvd_params.ctrl3.types.sw_reset = 0;
      imu_info.spi_wr_state = remo_imu_spi_send(LSM6DSVD_CTRL3_C_12, 
                          (uint8_t *)&imu_params.lsm6dsvd_params.ctrl3, 1);
      clock_cpu_delay_ms(20);
      imu_info.spi_wr_state = remo_imu_spi_receive(LSM6DSVD_CTRL3_C_12, 
                          (uint8_t *)&imu_params.lsm6dsvd_params.ctrl3, 1);
      clock_cpu_delay_ms(10);
      
      imu_info.spi_wr_state = remo_imu_spi_send(LSM6DSVD_CTRL1_XL_10, 
                          (uint8_t *)&imu_params.lsm6dsvd_params.ctrl1_accel, 1);
      clock_cpu_delay_ms(10);
                          
      imu_info.spi_wr_state = remo_imu_spi_send(LSM6DSVD_CTRL2_G_11, 
                          (uint8_t *)&imu_params.lsm6dsvd_params.ctrl2_gyro, 1);
      clock_cpu_delay_ms(10);
      
      imu_info.spi_wr_state = remo_imu_spi_send(LSM6DSVD_CTRL4_C_13, 
                          (uint8_t *)&imu_params.lsm6dsvd_params.ctrl4, 1);
      clock_cpu_delay_ms(10);
      
      imu_info.spi_wr_state = remo_imu_spi_send(LSM6DSVD_CTRL6_G_15, 
                          (uint8_t *)&imu_params.lsm6dsvd_params.ctrl6_gyro, 1);
      clock_cpu_delay_ms(10);
      
      imu_info.spi_wr_state = remo_imu_spi_send(LSM6DSVD_CTRL7_C_16, 
                          (uint8_t *)&imu_params.lsm6dsvd_params.ctrl7, 1);
      clock_cpu_delay_ms(10);
      
      imu_info.spi_wr_state = remo_imu_spi_send(LSM6DSVD_CTRL8_XL_17, 
                          (uint8_t *)&imu_params.lsm6dsvd_params.ctrl8_accel, 1);
      clock_cpu_delay_ms(10);
                  
      imu_info.spi_wr_state = remo_imu_spi_send(LSM6DSVD_CTRL9_XL_18, 
              (uint8_t *)&imu_params.lsm6dsvd_params.ctrl9_accel, 1);
      clock_cpu_delay_ms(20);
}

/************************************************************************************************
 * 函数名称: remo_imu_init
 * 输入参数: void
 * 返回结果: void
 * 功能描述: 初始化imu
**************************************************************************************************/
void remo_imu_init(void)
{
    // imu参数配置初始化
    remo_imu_params_init();
    
    //判断能否正确读取数据
    while (1)
    {
        imu_info.spi_wr_state = remo_imu_spi_receive(LSM6DSVD_WHO_AM_I_0F, 
               (uint8_t *)&imu_params.lsm6dsvd_params.who_am_i, 1);
        if (imu_info.spi_wr_state != 0 || imu_params.lsm6dsvd_params.who_am_i != LSM6DSVD_WHO_AM_I_VALUE)
        {
            imu_info.run_normal_flag = false;
        }
        else
        {
            imu_info.run_normal_flag = true;
            break;
        }

        clock_cpu_delay_ms(20);
    }

    // 设置寄存器
    remo_imu_register_init();

    if (imu_info.run_normal_flag)  
    {
        imu_info.stop_sampling_flag = false;
    }
    else
    {
        imu_info.stop_sampling_flag = true;
    }

    imu_spi_div = magencoder_imu_spi_clcok / 10000000;
}

/************************************************************************************************
 * 函数名称: remo_imu_update
 * 输入参数: void
 * 返回结果: void
 * 功能描述: 初始化imu
**************************************************************************************************/
uint32_t gyro_temp_cnt = 0;
struct {
    uint8_t flag;
    float deg;
    float dt;
}gyro_test = {
    .flag = 0,
    .deg = 0.0f,
    .dt = 0.0008344};
    float gyro_correct[3] = {0};
void remo_imu_update(void)
{
    static int16_t accel_temp[3] = {0}, gyro_temp[3] = {0};
    static int16_t gyro_median[3][7] = {0};
    static uint8_t gyro_median_cnt = 0;
    //float gyro_correct[3] = {0};

    static uint16_t smp_state = 0;
    
    if (imu_info.stop_sampling_flag)
    {
        return;
    }

    bsp_magencoder_imu_spi_set_clock_div(imu_spi_div);
    

    //imu_info.spi_wr_state = remo_imu_spi_receive_simple(LSM6DSVD_STATUS_REG_1E, 
    //            (uint8_t *)&imu_params.lsm6dsvd_params.status_reg.which_type, 1);
    //if (!imu_params.lsm6dsvd_params.status_reg.types.accel_da || !imu_params.lsm6dsvd_params.status_reg.types.gyro_da)
    //{
    //    return;
    //}
    
    if (schedule_ptmr_state % 2 == IMU_ACCEL_SAMPLE_SCHD_INDEX)
    {
        imu_info.spi_wr_state = remo_imu_spi_receive_simple(LSM6DSVD_TEMP_OUTL_20, 
                    (uint8_t *)&imu_params.lsm6dsvd_params.temp_output.types.output_l, 14);

        accel_temp[0] = imu_params.lsm6dsvd_params.accel_output[0].which_type;
        accel_temp[1] = imu_params.lsm6dsvd_params.accel_output[1].which_type;
        accel_temp[2] = imu_params.lsm6dsvd_params.accel_output[2].which_type;
        imu_info.temp_smp = (imu_params.lsm6dsvd_params.temp_output.which_type*25 >> 6) + 2500;
    }
    else 
    {
        imu_info.spi_wr_state = remo_imu_spi_receive_simple(LSM6DSVD_GYRO_XOUTL_22, 
                    (uint8_t *)&imu_params.lsm6dsvd_params.gyro_output[0].which_type, 6);
    }
  
    gyro_temp[0] = imu_params.lsm6dsvd_params.gyro_output[0].which_type;
    gyro_temp[1] = imu_params.lsm6dsvd_params.gyro_output[1].which_type;
    gyro_temp[2] = imu_params.lsm6dsvd_params.gyro_output[2].which_type;
  
    imu_info.accel_smp[0] = -accel_temp[0] - imu_info.accel_offset[0];
    imu_info.accel_smp[1] = accel_temp[1] - imu_info.accel_offset[1];
    imu_info.accel_smp[2] = accel_temp[2] - imu_info.accel_offset[2];

    imu_info.gyro_smp[0] = -gyro_temp[2] - imu_info.gyro_offset[0];
    imu_info.gyro_smp[1] = gyro_temp[1] - imu_info.gyro_offset[1];
    imu_info.gyro_smp[2] = gyro_temp[0] - imu_info.gyro_offset[2];

    

    imu_info.accel_correct[0] = imu_info.accel_smp[0] * imu_info.accel_trans_matrix[0] +
                                imu_info.accel_smp[1] * imu_info.accel_trans_matrix[1] +
                                imu_info.accel_smp[2] * imu_info.accel_trans_matrix[2];
    imu_info.accel_correct[1] = imu_info.accel_smp[0] * imu_info.accel_trans_matrix[3] +
                                imu_info.accel_smp[1] * imu_info.accel_trans_matrix[4] +
                                imu_info.accel_smp[2] * imu_info.accel_trans_matrix[5];
    imu_info.accel_correct[2] = imu_info.accel_smp[0] * imu_info.accel_trans_matrix[6] +
                                imu_info.accel_smp[1] * imu_info.accel_trans_matrix[7] +
                                imu_info.accel_smp[2] * imu_info.accel_trans_matrix[8];

    imu_info.temp_correct = imu_info.temp_smp;// - TEMPERATURE_OFFSET;

    gyro_correct[0] = imu_info.gyro_smp[0] * imu_info.gyro_trans_matrix[0] +
                      imu_info.gyro_smp[1] * imu_info.gyro_trans_matrix[1] +
                      imu_info.gyro_smp[2] * imu_info.gyro_trans_matrix[2];
    gyro_correct[1] = imu_info.gyro_smp[0] * imu_info.gyro_trans_matrix[3] +
                      imu_info.gyro_smp[1] * imu_info.gyro_trans_matrix[4] +
                      imu_info.gyro_smp[2] * imu_info.gyro_trans_matrix[5];
    gyro_correct[2] = imu_info.gyro_smp[0] * imu_info.gyro_trans_matrix[6] +
                      imu_info.gyro_smp[1] * imu_info.gyro_trans_matrix[7] +
                      imu_info.gyro_smp[2] * imu_info.gyro_trans_matrix[8];
						
    imu_info.gyro_fine_correct[0] = gyro_correct[0] - imu_info.gyro_fine_offset[0];
    imu_info.gyro_fine_correct[1] = gyro_correct[1] - imu_info.gyro_fine_offset[1];
    imu_info.gyro_fine_correct[2] = gyro_correct[2] - imu_info.gyro_fine_offset[2];
	
    imu_info.gyro_correct[0] = imu_info.gyro_fine_correct[0] - imu_info.gyro_temp_offset[0];
    imu_info.gyro_correct[1] = imu_info.gyro_fine_correct[1] - imu_info.gyro_temp_offset[1];
    imu_info.gyro_correct[2] = imu_info.gyro_fine_correct[2] - imu_info.gyro_temp_offset[2];                   
	


    //imu_info.data_reflash_flag = true;
    

}

/************************************************************************************************
 * 函数名称: remo_imu_spi_send
 * 输入参数: void
 * 返回结果: void
 * 功能描述: imu的i2c发送
**************************************************************************************************/
static uint8_t remo_imu_spi_send(uint8_t cmd, uint8_t *data, uint8_t len)
{
    return bsp_imu_spi_send(cmd&0x7F, 0, data, len);
}

/************************************************************************************************
 * 函数名称: remo_imu_spi_receive
 * 输入参数: void
 * 返回结果: void
 * 功能描述: imu的i2c接收
**************************************************************************************************/
static uint8_t remo_imu_spi_receive(uint8_t cmd, uint8_t *data, uint8_t len)
{
    return bsp_imu_spi_receive(cmd|0x80, 0, data, len);
}

static uint8_t remo_imu_spi_receive_simple(uint8_t cmd, uint8_t *data, uint8_t len)
{
    return bsp_imu_spi_receive_simple(cmd|0x80, 0, data, len);
}

/************************************************************************************************
 * 函数名称: remo_imu_get_gyro_smp
 * 输入参数: void
 * 返回结果: int16_t *
 * 功能描述: 获取imu的采样值
**************************************************************************************************/
const int16_t *remo_imu_get_gyro_smp(void)
{
    return imu_info.gyro_smp;
}

/************************************************************************************************
 * 函数名称: remo_imu_get_accel_smp
 * 输入参数: void
 * 返回结果: int16_t *
 * 功能描述: 获取imu的采样值
**************************************************************************************************/
const int16_t *remo_imu_get_accel_smp(void)
{
    return imu_info.accel_smp;
}

/************************************************************************************************
 * 函数名称: remo_imu_get_temperature_smp
 * 输入参数: void
 * 返回结果: int16_t *
 * 功能描述: 获取imu的采样值
**************************************************************************************************/
int16_t remo_imu_get_temperature_smp(void)
{
    return imu_info.temp_smp;
}

/************************************************************************************************
 * 函数名称: remo_imu_get_gyro_smp
 * 输入参数: void
 * 返回结果: int16_t *
 * 功能描述: 获取imu的采样值
**************************************************************************************************/
const float *remo_imu_get_gyro_corr(void)
{
    return imu_info.gyro_correct;
}

const float *remo_imu_get_gyro_fine_corr(void)
{
	return imu_info.gyro_fine_correct;
}
/************************************************************************************************
 * 函数名称: remo_imu_get_accel_corr
 * 输入参数: void
 * 返回结果: int16_t *
 * 功能描述: 获取imu的采样值
**************************************************************************************************/
const float *remo_imu_get_accel_corr(void)
{
    return imu_info.accel_correct;
}

/************************************************************************************************
 * 函数名称: remo_imu_get_accel_corr
 * 输入参数: void
 * 返回结果: int16_t *
 * 功能描述: 获取imu的加速度计的偏置
**************************************************************************************************/
const int16_t *remo_imu_get_accel_bias(void)
{
    return &imu_info.accel_offset[0];
}

/************************************************************************************************
 * 函数名称: remo_imu_get_gyro_bias
 * 输入参数: void
 * 返回结果: int16_t *
 * 功能描述: 获取imu的陀螺仪的偏置
**************************************************************************************************/
int16_t remo_imu_get_gyro_bias(uint8_t index)
{
    if (index <= 2)
    {
        return imu_info.gyro_offset[index];
    }
    else 
    {
        return 0;
    }
}

const float *remo_imu_get_gyro_tempreture_offset(void)
{
    return imu_info.gyro_temp_offset;
}

/************************************************************************************************
 * 函数名称: remo_imu_get_spi_wr_state
 * 输入参数: void
 * 返回结果: uint8_t
 * 功能描述: 获取imu的i2c状态
**************************************************************************************************/
uint8_t remo_imu_get_spi_wr_state(void)
{
    return imu_info.spi_wr_state;
}

/************************************************************************************************
 * 函数名称: remo_imu_data_refreshed
 * 输入参数: void
 * 返回结果: bool
 * 功能描述: 获取imu的数据更新标志
**************************************************************************************************/
bool remo_imu_data_refreshed(void)
{
    return imu_info.data_reflash_flag;
}

/************************************************************************************************
 * 函数名称: remo_imu_clear_data_refresh_flag
 * 输入参数: void
 * 返回结果: void
 * 功能描述: 重置imu数据更新标志
**************************************************************************************************/
void remo_imu_clear_data_refresh_flag(void)
{
    imu_info.data_reflash_flag = false;
}

/************************************************************************************************
 * 函数名称: remo_imu_set_gyro_bias
 * 输入参数: void
 * 返回结果: void
 * 功能描述: 设置陀螺仪零偏值
**************************************************************************************************/
void remo_imu_set_gyro_bias(int16_t bias_xaxis, int16_t bias_yaxis, int16_t bias_zaxis)
{
    imu_info.gyro_offset[XAXIS] = bias_xaxis;
    imu_info.gyro_offset[YAXIS] = bias_yaxis;
    imu_info.gyro_offset[ZAXIS] = bias_zaxis;
}

/************************************************************************************************
 * 函数名称: remo_imu_set_accel_bias
 * 输入参数: int16_t
 * 返回结果: void
 * 功能描述: 设置加速度零偏值
**************************************************************************************************/
void remo_imu_set_accel_bias(int16_t bias_xaxis, int16_t bias_yaxis, int16_t bias_zaxis)
{
    imu_info.accel_offset[XAXIS] = bias_xaxis;
    imu_info.accel_offset[YAXIS] = bias_yaxis;
    imu_info.accel_offset[ZAXIS] = bias_zaxis;
}

/************************************************************************************************
 * 函数名称: remo_imu_set_accel_trans_matrix
 * 输入参数: const int16_t *
 * 返回结果: void
 * 功能描述: 设置加速度转换矩阵
**************************************************************************************************/
void remo_imu_set_accel_trans_matrix(const float *trans_matrix)
{
    imu_info.accel_trans_matrix[0] = trans_matrix[0];
    imu_info.accel_trans_matrix[1] = trans_matrix[1];
    imu_info.accel_trans_matrix[2] = trans_matrix[2];
    imu_info.accel_trans_matrix[3] = trans_matrix[3];
    imu_info.accel_trans_matrix[4] = trans_matrix[4];
    imu_info.accel_trans_matrix[5] = trans_matrix[5];
    imu_info.accel_trans_matrix[6] = trans_matrix[6];
    imu_info.accel_trans_matrix[7] = trans_matrix[7];
    imu_info.accel_trans_matrix[8] = trans_matrix[8];
}

void remo_imu_set_gyro_trans_matrix(const float *trans_matrix)
{
    imu_info.gyro_trans_matrix[0] = trans_matrix[0];
    imu_info.gyro_trans_matrix[1] = trans_matrix[1];
    imu_info.gyro_trans_matrix[2] = trans_matrix[2];
    imu_info.gyro_trans_matrix[3] = trans_matrix[3];
    imu_info.gyro_trans_matrix[4] = trans_matrix[4];
    imu_info.gyro_trans_matrix[5] = trans_matrix[5];
    imu_info.gyro_trans_matrix[6] = trans_matrix[6];
    imu_info.gyro_trans_matrix[7] = trans_matrix[7];
    imu_info.gyro_trans_matrix[8] = trans_matrix[8];
}
/************************************************************************************************
 * 函数名称: remo_imu_set_stop_sampling_flag
 * 输入参数: bool
 * 返回结果: void
 * 功能描述: 设置imu停止采样标志
**************************************************************************************************/
void remo_imu_set_stop_sampling_flag(bool flag)
{
    imu_info.stop_sampling_flag = flag;
}

//uint32_t gyro_temp_offset_update_cnt = 0;
void remo_imu_set_tempreture_offset(float *temperature_offset, uint8_t state)
{
 //   static float gyro_temp_offset_median[3][5] = {0};
 //   static uint8_t gyro_temp_offset_cnt = 0;
 //   static float gyro_temp_offset_last[3] = {0};
	//float temp_f = 0.0f;
	//float filter_alpha = 0.0f;
    
 //   gyro_temp_offset_median[0][gyro_temp_offset_cnt] = temperature_offset[0];
 //   gyro_temp_offset_median[1][gyro_temp_offset_cnt] = temperature_offset[1];
 //   gyro_temp_offset_median[2][gyro_temp_offset_cnt] = temperature_offset[2];
    
 //   if(++gyro_temp_offset_cnt >= 5) gyro_temp_offset_cnt = 0;
	//gyro_temp_offset_update_cnt++;
	//filter_alpha = (float)gyro_temp_offset_update_cnt *0.05f;
	//if (filter_alpha >= 0.95f)
	//{
	//	filter_alpha = 0.98f;
	//}
	
	//if (state == 0)
	//{
	//	filter_alpha = 0.5f;
	//}
    
	//for (uint8_t i = 0; i < 3; i++)
	//{
	//	temp_f = ((float)imu_info.gyro_temp_offset[i])*filter_alpha + ((float)filter_median_fw5(&gyro_temp_offset_median[i][0]))*(1.0f - filter_alpha);
	//	imu_info.gyro_temp_offset[i] = temp_f ;
	//}
}

void remo_imu_set_fine_offset(float *fine_offset)
{
    imu_info.gyro_fine_offset[0] = fine_offset[0]*imu_gyro_to_deg;
    imu_info.gyro_fine_offset[1] = fine_offset[1]*imu_gyro_to_deg;
    imu_info.gyro_fine_offset[2] = fine_offset[2]*imu_gyro_to_deg;

}

bool remo_imu_get_run_normal_flag(void)
{
    return imu_info.run_normal_flag;
}

float remo_imu_get_gyro_deg(uint8_t index)
{
    return imu_info.gyro_smp[index]*imu_gyro_to_deg;
}

int16_t remo_imu_get_xaixs_gyro_ddps(void)
{   
    return 100*imu_info.gyro_correct[XAXIS];
}
int16_t remo_imu_get_yaixs_gyro_ddps(void)
{   
    return 100*imu_info.gyro_correct[YAXIS];
}
int16_t remo_imu_get_zaixs_gyro_ddps(void)
{   
    return 100*imu_info.gyro_correct[ZAXIS];
}

int16_t remo_imu_get_xaxis_accel_cms2(void)
{   
    return 100*imu_info.accel_correct[XAXIS];
}
int16_t remo_imu_get_yaxis_accel_cms2(void)
{   
    return 100*imu_info.accel_correct[YAXIS];
}
int16_t remo_imu_get_zaxis_accel_cms2(void)
{   
    return 100*imu_info.accel_correct[ZAXIS];
}
