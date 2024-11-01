#ifndef __IMU_LSM6DSVD_H
#define __IMU_LSM6DSVD_H

#include "stdint.h"

#define LSM6DSVD_I2C_RW_ADDR   ((uint8_t)(0x6A << 1))   // LSM6DSVD的I2C从地址、

// 寄存器

// USER BANK 0 
#define LSM6DSVD_FUNC_CFG_ACCESS_01      ((uint8_t)0x01)  // R/W
#define LSM6DSVD_PIN_CTRL_02             ((uint8_t)0x02)  // R/W
#define LSM6DSVD_IF_CFG_03               ((uint8_t)0x03)  // R/W

#define LSM6DSVD_FIFO_CTRL1_07           ((uint8_t)0x07)  // R/W
#define LSM6DSVD_FIFO_CTRL2_08           ((uint8_t)0x08)  // R/W
#define LSM6DSVD_FIFO_CTRL3_09           ((uint8_t)0x09)  // R/W
#define LSM6DSVD_FIFO_CTRL4_0A           ((uint8_t)0x0A)  // R/W
#define LSM6DSVD_COUNTER_BDR_REG1_0B     ((uint8_t)0x0B)  // R/W
#define LSM6DSVD_COUNTER_BDR_REG2_0C     ((uint8_t)0x0C)  // R/W
#define LSM6DSVD_INT1_CTRL_0D            ((uint8_t)0x0D)  // R/W
#define LSM6DSVD_INT2_CTRL_0E            ((uint8_t)0x0E)  // R/W

#define LSM6DSVD_CTRL1_XL_10     ((uint8_t)0x10)  // R/W
#define LSM6DSVD_CTRL2_G_11      ((uint8_t)0x11)  // R/W
#define LSM6DSVD_CTRL3_C_12      ((uint8_t)0x12)  // R/W
#define LSM6DSVD_CTRL4_C_13      ((uint8_t)0x13)  // R/W
#define LSM6DSVD_CTRL5_C_14      ((uint8_t)0x14)  // R/W
#define LSM6DSVD_CTRL6_G_15      ((uint8_t)0x15)  // R/W
#define LSM6DSVD_CTRL7_C_16      ((uint8_t)0x16)  // R/W
#define LSM6DSVD_CTRL8_XL_17     ((uint8_t)0x17)  // R/W
#define LSM6DSVD_CTRL9_XL_18     ((uint8_t)0x18)  // R/W
#define LSM6DSVD_CTRL10_C_19     ((uint8_t)0x19)  // R/W

#define LSM6DSVD_FIFO_STATUS1_1B      ((uint8_t)0x1B)  // R/W
#define LSM6DSVD_FIFO_STATUS1_1C      ((uint8_t)0x1C)  // R/W
#define LSM6DSVD_ALL_INT_SRC_1D       ((uint8_t)0x1D)  // R/W
#define LSM6DSVD_STATUS_REG_1E        ((uint8_t)0x1E)  // R/W

#define LSM6DSVD_TEMP_OUTL_20             ((uint8_t)0x20)  // R
#define LSM6DSVD_TEMP_OUTH_21             ((uint8_t)0x21)  // R
#define LSM6DSVD_GYRO_XOUTL_22            ((uint8_t)0x22)  // R
#define LSM6DSVD_GYRO_XOUTH_23            ((uint8_t)0x23)  // R
#define LSM6DSVD_GYRO_YOUTL_24            ((uint8_t)0x24)  // R
#define LSM6DSVD_GYRO_YOUTH_25            ((uint8_t)0x25)  // R
#define LSM6DSVD_GYRO_ZOUTL_26            ((uint8_t)0x26)  // R
#define LSM6DSVD_GYRO_ZOUTH_27            ((uint8_t)0x27)  // R
#define LSM6DSVD_ACCEL_XOUTL_28           ((uint8_t)0x28)  // R
#define LSM6DSVD_ACCEL_XOUTH_29           ((uint8_t)0x29)  // R
#define LSM6DSVD_ACCEL_YOUTL_2A           ((uint8_t)0x2A)  // R
#define LSM6DSVD_ACCEL_YOUTH_2B           ((uint8_t)0x2B)  // R
#define LSM6DSVD_ACCEL_ZOUTL_2C           ((uint8_t)0x2C)  // R
#define LSM6DSVD_ACCEL_ZOUTH_2D           ((uint8_t)0x2D)  // R


#define LSM6DSVD_UI_OUTX_L_A_OIS_EIS_DUALC_34      ((uint8_t)0x34)  // R
#define LSM6DSVD_UI_OUTX_H_A_OIS_EIS_DUALC_35      ((uint8_t)0x35)  // R
#define LSM6DSVD_UI_OUTY_L_A_OIS_EIS_DUALC_36      ((uint8_t)0x36)  // R
#define LSM6DSVD_UI_OUTY_H_A_OIS_EIS_DUALC_37      ((uint8_t)0x37)  // R
#define LSM6DSVD_UI_OUTZ_L_A_OIS_EIS_DUALC_38      ((uint8_t)0x38)  // R
#define LSM6DSVD_UI_OUTZ_H_A_OIS_EIS_DUALC_39      ((uint8_t)0x39)  // R

#define LSM6DSVD_TIMESTAMP0_40                   ((uint8_t)0x40)   // R
#define LSM6DSVD_TIMESTAMP1_41                   ((uint8_t)0x41)   // R
#define LSM6DSVD_TIMESTAMP2_42                   ((uint8_t)0x42)   // R
#define LSM6DSVD_TIMESTAMP3_43                   ((uint8_t)0x43)   // R

#define LSM6DSVD_WAKE_UP_SRC_45                ((uint8_t)0x45)   // R
#define LSM6DSVD_TAP_SRC_46                    ((uint8_t)0x46)   // R
#define LSM6DSVD_D6D_SRC_47                    ((uint8_t)0x47)   // R
#define LSM6DSVD_STATUS_MASTER_MAINPAGE_48     ((uint8_t)0x48)   // R
#define LSM6DSVD_EMB_FUNC_STATUS_MAINPAGE_49   ((uint8_t)0x49)   // R
#define LSM6DSVD_FSM_STATUS_MAINPAGE_4A        ((uint8_t)0x4A)   // R
#define LSM6DSVD_MLC_STATUS_MAINPAGE_4B        ((uint8_t)0x4B)   // R

#define LSM6DSVD_INTERNAL_FREQ_FINE_4F      ((uint8_t)0x4F)   // R
#define LSM6DSVD_FUNCTIONS_ENABLE_50        ((uint8_t)0x50)   // R/W

#define LSM6DSVD_INACTIVITY_DUR_54          ((uint8_t)0x54)   // R/W
#define LSM6DSVD_INACTIVITY_THS_55          ((uint8_t)0x55)   // R/W


#define LSM6DSVD_TAP_CFG0_56                     ((uint8_t)0x56)   // R/W
#define LSM6DSVD_TAP_CFG1_57                     ((uint8_t)0x57)   // R/W
#define LSM6DSVD_TAP_CFG2_58                     ((uint8_t)0x58)   // R/W
#define LSM6DSVD_TAP_THS_6D_59                   ((uint8_t)0x59)   // R/W
#define LSM6DSVD_INT_DUR2_5A                     ((uint8_t)0x5A)   // R/W
#define LSM6DSVD_WAKE_UP_THS_5B                  ((uint8_t)0x5B)   // R/W
#define LSM6DSVD_WAKE_UP_DUR_5C                  ((uint8_t)0x5C)   // R/W
#define LSM6DSVD_FREE_FALL_5D                    ((uint8_t)0x5D)   // R/W
#define LSM6DSVD_MD1_CFG_5E                      ((uint8_t)0x5E)   // R/W
#define LSM6DSVD_MD2_CFG_5F                      ((uint8_t)0x5F)   // R/W


#define LSM6DSVD_EMB_FUNC_CFG_63           ((uint8_t)0x63)    // R/W
#define LSM6DSVD_UI_HANDSHAKE_CTRL_64      ((uint8_t)0x64)    // R/W

#define LSM6DSVD_X_OFS_USR_73                    ((uint8_t)0x73)
#define LSM6DSVD_Y_OFS_USR_74                    ((uint8_t)0x74)
#define LSM6DSVD_Z_OFS_USR_75                    ((uint8_t)0x75)

#define LSM6DSVD_FIFO_DATA_OUT_TAG_78            ((uint8_t)0x78)    // R
#define LSM6DSVD_FIFO_DATA_OUT_X_L_79            ((uint8_t)0x79)    // R
#define LSM6DSVD_FIFO_DATA_OUT_X_H_7A            ((uint8_t)0x7A)    // R
#define LSM6DSVD_FIFO_DATA_OUT_Y_L_7B            ((uint8_t)0x7B)    // R
#define LSM6DSVD_FIFO_DATA_OUT_Y_H_7C            ((uint8_t)0x7C)    // R
#define LSM6DSVD_FIFO_DATA_OUT_Z_L_7D            ((uint8_t)0x7D)    // R
#define LSM6DSVD_FIFO_DATA_OUT_Z_H_7E            ((uint8_t)0x7E)    // R

#define LSM6DSVD_WHO_AM_I_0F         ((uint8_t)0x0F)  // R

// 复位后的非零默认值
#define LSM6DSVD_WHO_AM_I_VALUE        (0x71)

typedef struct {

    // 我是谁
    uint8_t who_am_i;

    // 温度输出
    union{
    struct
        {
            uint8_t output_l;     
            uint8_t output_h;
        }types;
        int16_t which_type;
    }temp_output;

    // 陀螺仪输出
    union{
    struct
        {
            uint8_t output_l;     
            uint8_t output_h;
        }types;
        int16_t which_type;
    }gyro_output[3];

    // 加速度计输出
    union{
    struct
        {
            uint8_t output_l;     
            uint8_t output_h;
        }types;
        int16_t which_type;
    }accel_output[3];


    // 设备配置
    union{
        struct
        {
            uint8_t reserved0_def_0: 2;
            uint8_t sw_por: 1;
            uint8_t reserved1_def_0: 4;
            uint8_t func_cfg: 1;     // 是否使能内嵌函数寄存器
        }types;
        uint8_t which_type;
    }func_cfg_access;
    // pin脚配置
    union{
        struct
        {
            uint8_t reserved0_def_1: 2;    // 默认值为1
            uint8_t reserved1_def_0: 3;    // 默认值为0
            uint8_t ibhr_por_en: 1;    // 复位的行为
            uint8_t sdo_pu_en: 1;      // 使能SDO上拉
            uint8_t reserved2_def_0: 1; 
        }types;
        uint8_t which_type;
    }pin_ctrl;
	// 接口配置
    union{
        struct
        {
            uint8_t i2c_i3c_disable: 1;
            uint8_t reserved0_def_0: 1;  // 默认值为0
            uint8_t sim: 1;              // spi模式
            uint8_t pp_od: 1;            // 中断引脚是推挽还是开漏
            uint8_t h_lactivel: 1;       // 中断电平
            uint8_t asf_ctrl: 1;         // 
            uint8_t reserved1_def_0: 1;
            uint8_t sda_pu_en: 1;        //  使能sda上拉
        }types;
        uint8_t which_type;
    }if_ctrl;

    // odr触发配置
    uint8_t odr_trig_nord;
	
	

    // 加速度计配置
    union{
    struct
        {
            uint8_t odr_xl: 4;         // 输出速率选择
            uint8_t op_mode_xl: 3;     // 工作模式选择 0: 高性能模式，1: 高精度odr模式，...
            uint8_t reserved1_def_0: 1; 
        }types;
        uint8_t which_type;
    }ctrl1_accel;
    // 陀螺仪配置
    union{
    struct
        {
            uint8_t odr_g: 4;        // 输出速率选择
            uint8_t op_mode_g: 3;    // 工作模式选择 0: 高性能模式，1: 高精度odr模式，...
            uint8_t reserved1_def_0: 1; 
        }types;
        uint8_t which_type;
    }ctrl2_gyro;

    union{
    struct
        {
            uint8_t sw_reset: 1;    // 软件重置
            uint8_t reserved0_def_0: 1;
            uint8_t if_inc: 1;            // 使能寄存器地址读取数据时自动增加
            uint8_t reserved1_def_0: 3;   // spi接口选择，0:4线，1:3线
            uint8_t bdu: 1;         // 测量块数据更新，0:连续更新，1:数据读取后才更新
            uint8_t boot: 1;        // reboot内存内容，1:reboot
        }types;
        uint8_t which_type;
    }ctrl3;

    union{
    struct
        {
            uint8_t reserved0_def_0: 1;
            uint8_t drdy_pulsed: 1;     // 使能冲击数据使能模式
            uint8_t int2_drdy_temp: 1; 
            uint8_t drdy_mask: 1;       // 使能数据准备io口
            uint8_t int2_on_int1: 1;   // 是否将所有中断放在int1的io口
            uint8_t reserved1_def_0: 3;
        }types;
        uint8_t which_type;
    }ctrl4;

    union{
    struct
        {
            uint8_t int_en_i3c: 1;       // 当i3c使能时是否使能int引脚
            uint8_t bus_act_sel: 2;    
            uint8_t reserved0_def_0: 5;
        }types;
        uint8_t which_type;
    }ctrl5;

    union{
    struct
        {
            uint8_t fs_g: 4;            // 陀螺仪量程选择。 0:±125dps, 1:±250dps, 2:±500dps, 3:±1000dps, 4:±2000dps, 5:±4000dps
            uint8_t lpf1_g_bw: 3;       // 陀螺仪低通滤波1带宽。
            uint8_t reserved0_def_0: 1;    
        }types;
        uint8_t which_type;
    }ctrl6_gyro;

    union{
    struct
        {
            uint8_t lpf1_g_en: 1;        // 是否使能陀螺仪低通滤波器1
            uint8_t reserved0_def_0: 7; 
        }types;
        uint8_t which_type;
    }ctrl7;

    union{
    struct
        {
            uint8_t fs_xl: 2;            // 加速度计量程。0:±2g, 1:±4g, 2:±8g, 3:±16g,
            uint8_t reserved0_def_0: 1; 
            uint8_t xl_dualc_en: 1;
            uint8_t reserved1_def_0: 1; 
            uint8_t hp_lpf2_xl_bw: 3;   // 配置加速度计低通滤波器2和高通滤波器的截止频率
        }types;
        uint8_t which_type;
    }ctrl8_accel;

    union{
    struct
        {
            uint8_t usr_off_on_out: 1;   // 使能用户加速度计offset块
            uint8_t usr_off_w: 1;        // 用户加速度计offset块单位
            uint8_t reserved0_def_0: 1; 
            uint8_t lpf2_xl_en: 1;       // 加速度计高精度选择
            uint8_t hp_slope_xl_en: 1;   // 加速度计斜率滤波和高通滤波选择
            uint8_t xl_fastsettl_mode: 1;   // 使能加速度计低通滤波器2和高通滤波器的设置模式
            uint8_t hp_ref_mode_xl: 1;      // 使能加速度计高通滤波参考模式
            uint8_t reserved1_def_0: 1; 
		}types;
        uint8_t which_type;
    }ctrl9_accel;

    union{
    struct
        {
            uint8_t st_xl: 2;    // 加速度计自测选择
            uint8_t st_g: 2;     // 陀螺仪自测选择
            uint8_t reserved0_def_0: 2; 
            uint8_t emb_func_debug: 1;   // 使能嵌入函数的debug模式
            uint8_t reserved1_def_0: 1; 
        }types;
        uint8_t which_type;
    }ctrl10;

    // 中断源配置
    union{
    struct
        {
            uint8_t ff_ia: 1;             // 自由落体事件状态
            uint8_t wu_ia: 1;             // 唤醒事件状态
            uint8_t tap_ia: 1;            // 单击或双击事件状态
            uint8_t reserved0: 1;
            uint8_t d6d_ia: 1;            // 位置变换(纵向、横向、面朝上、面朝下)事件状态
            uint8_t sleep_change_ia: 1;   // 激活和不激活事件的改变
            uint8_t reserved1: 1;      
            uint8_t emb_func_ia: 1;       // 嵌入函数中断状态
        }types;
        uint8_t which_type;
    }all_int_src;
	
	// 状态寄存器
    union{
    struct
        {
            uint8_t accel_da: 1;    // 加速度数据准备好
            uint8_t gyro_da: 1;     // 陀螺仪数据准备好
            uint8_t temp_da: 1;     // 温度数据准备好
            uint8_t reserved0: 4;
            uint8_t timestamp_endcount: 1; 
        }types;
        uint8_t which_type;
    }status_reg;

    // 唤醒源配置
    union{
    struct
        {
            uint8_t z_wu: 1;             // Z轴检测唤醒事件状态
            uint8_t y_wu: 1;             // Y轴检测唤醒事件状态
            uint8_t x_wu: 1;             // X轴检测唤醒事件状态
            uint8_t wu_ia: 1;            // 检测唤醒事件状态
            uint8_t sleep_state: 1;      // 睡眠事件状态
            uint8_t ff_ia: 1;            // 检测自由落体事件状态
            uint8_t sleep_change_ia: 1;  // 检测激活和不激活改变事件状态
            uint8_t reserved0: 1; 
        }types;
        uint8_t which_type;
    }wake_up_src;

    // 敲击源配置
    union{
    struct
        {
            uint8_t z_tap: 1;           // Z轴检测敲击事件状态
            uint8_t y_tap: 1;           // Y轴检测敲击事件状态
            uint8_t x_tap: 1;           // X轴检测敲击事件状态
            uint8_t tap_sign: 1;        // 敲击事件符号
            uint8_t reserved0: 1; 
            uint8_t double_tap: 1;      // 双击事件状态
            uint8_t single_tap: 1;      // 单击事件状态
            uint8_t tap_ia: 1;          // 敲击事件状态
        }types;
        uint8_t which_type;
    }tap_src;

    // 6维方位变化源配置
    union{
    struct
        {
            uint8_t zl: 1;        // Z轴数据小
            uint8_t zh: 1;        // Z轴数据大
            uint8_t yl: 1;        // Y轴数据小
            uint8_t yh: 1;        // Y轴数据大
            uint8_t xl: 1;        // X轴数据小
            uint8_t xh: 1;        // X轴数据大
            uint8_t d6d_ia: 1;    // 检测位置变换(纵向、横向、面朝上、面朝下)事件状态
            uint8_t reserved0: 1; 
        }types;
        uint8_t which_type;
    }d6d_src;



    // 内嵌函数状态
    union{
    struct
        {
            uint8_t reserved0: 3;   
            uint8_t is_step_det: 1;   // 阶跃探测
            uint8_t is_tilt: 1;       // 倾斜探测
            uint8_t is_sigmot: 1;     // 重要运动探测
            uint8_t reserved1: 1;   
            uint8_t is_fsm_lc: 1;     // 状态机超时中断
        }types;
        uint8_t which_type;
    }emb_func_status;

    // status_master_mainpage

    // fifo_status1
    // fifo_status2

    //union{
    //    uint8_t timestamp_u8[4];
    //    uint32_t timestamp_u32;
    //}timesstamp;

    // tap_cfg0
    // tag_cfg1
    // tag_cfg2
    // tag_ths_6d
    // int_dur2
    // wake_up_ths
    // wake_up_dur
    // free_fall
    // md1_cfg
    // md2_cfg

    // s4s_st_cmd_code
    // s4s_dt_reg
    // i3c_bus_avg

    // odr差别
    uint8_t internal_freq_fine;


    // x_ofs_usr
    // y_ofs_usr
    // z_ofs_usr
    // fifo_data_out_tag
    // fifo_data_out_x_l
    // fifo_data_out_x_h
    // fifo_data_out_y_l
    // fifo_data_out_y_h
    // fifo_data_out_z_l
    // fifo_data_out_z_h
}lsm6dsvd_params_t;


#endif   // __IMU_LSM6DSVD_H
