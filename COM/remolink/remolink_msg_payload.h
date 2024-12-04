#ifndef __REMOLINK_MSG_PAYLOAD_H__
#define __REMOLINK_MSG_PAYLOAD_H__
#include "hpm_soc.h"

#include "remolink.h"
#include "Flightcontrol_meg.h"

#include "ahrs.h"
#include "angle_encoder.h"
#include "calibrate.h"
#include "imu.h"




 


/*ahrs姿态相关信息*/        //用不到？
#define REMOLINK_MSG_ID_AHRS_STATUS_LEN         181
#define REMOLINK_MSG_ID_AHRS_STATUS_CRC_LEN     (REMOLINK_MSG_ID_AHRS_STATUS_LEN+12)     //181+7

#define REMOLINK_MSG_ID_FTC_STATUS              0x00
#define REMOLINK_MSG_ID_CTF_STATUS              0x00
#define REMOLINK_MSG_ID_AHRS_STATUS             0x01
// typedef enum
// {
    
//     AHRS_BASE_VERT_UP = 0,  // 正挂。 
//     AHRS_BASE_VERT_DOWN,    // 倒挂。
//     AHRS_BASE_HOR_CAM_HOR,  // 竖拍。
//     AHRS_BASE_HOR_CAM_VERT, // 俯拍。
//     AHRS_BASE_ROLL_SPECIAL = 10,
//     AHRS_BASE_PITCH_SPECIAL = 11,
//     AHRS_BASE_YAW_SPECIAL = 12,
//     AHRS_BASE_HOR_CAM_HOR01 = 21,    // 竖拍，镜头左倾朝前
//     AHRS_BASE_HOR_CAM_HOR_V11 = 22,  // 竖拍，镜头左倾朝下
//     AHRS_BASE_HOR_CAM_HOR_V21 = 23,  // 竖拍，镜头左倾朝上
//     AHRS_BASE_HOR_CAM_HOR02 = 24,    // 竖拍，镜头右倾朝前
//     AHRS_BASE_HOR_CAM_HOR_V12 = 25,  // 竖拍，镜头右倾朝下
//     AHRS_BASE_HOR_CAM_HOR_V22 = 26,  // 竖拍，镜头右倾朝上
    
//     AHRS_BASE_HOR_CAM_VERT1 = 31,   // 俯拍
//     AHRS_BASE_HOR_CAM_VERT2 = 32,   // 仰拍
// } ahrs_base_ori_t; // 云台底座姿态
// typedef struct
// {
//     float sin;
//     float cos;
// } trig_func_f_t;
// typedef struct
// {
//     bool init_finished_flag;             //1

//     float euler_rad[3];     // 欧拉角    //4
//     trig_func_f_t euler_trigf[3];        //8*3=24
//     float euler_deg[3];     // 欧拉角    //12
//     float euler_deg_yaw_offset;          //4
//     float quaternion[4];    // 四元素    //16
//     float dcm_b2n[3][3]; // 机体坐标系到地球坐标系的转换矩阵 //9*4=36
  
//     // 根据MPU6500测量的原始数据，经过PI修正后得到的四元素和欧拉角，内部使用
//     float euler_raw_rad[3];              //12
    
//     float dcm_ba2b[3][3];                //9*4=36
//     struct 
//     {
//         float base_dcmz[3];              //12
//         ahrs_base_ori_t base_ori;      // 底座当前姿态       //4
//         ahrs_base_ori_t base_ori_last; // 底座上一次的姿态   //4
//         bool base_init_done;             //1
//         bool base_change_flag;           //1
		
//         float base_dcmz_var[3];          //12
//     }base_state;
//     bool base_ori_fixed;                 //1

//     bool gimbal_lock_flag;   // 万向节标志，正常运行时不会出现此类情况。但是在校准时会出现，特殊处理     //1
// } ahrs_status_t;
// ahrs_status_t


/****************hall_info_t霍尔相关信息****************/  //用不到？

#define REMOLINK_MSG_ID_HALL_INFO_LEN         72
#define REMOLINK_MSG_ID_HALL_INFO_CRC_LEN     (REMOLINK_MSG_ID_HALL_INFO_LEN+12)     //181+7

#define REMOLINK_MSG_ID_HALL_INFO             0x02
//hall_info_t hall_info[3];
// typedef struct
// {
//     uint16_t sample_freq; // 采样频率
//     uint16_t cutoff_freq; // 截止频率

//     float a[2];  // 分母：1+a[0]*z^(-1)+a[1]*z^(-2)
//     float b[3];  // 分子：b[0]+b[1]*z^(-1)+b[2]*z^(-2)
//     float de[3]; // 延时元素，delay_element
// } low_pass_filter_2nd_t;    //32+4
// typedef struct
// {
//     bool data_update;            //1

//     float raw_data;              //4
//     float raw_data_last;         //4
//     float fp_alpha;              //4
//     float raw_data0_offset;      //4
//     float raw_data_new;          //4
//     uint16_t hall_offset[2];     //2*2=4
    
//     bool linear_flag;            //1
//     uint16_t linear_num;         //2
//     float *linear_pbuf[2];       //4


//     int32_q7_t rotor_angle; //原始数据转化为度       //4

//     low_pass_filter_2nd_t lp_2nd_filter;             //36
//     // int32_t speed_dps_filter;
//     //low_pass_filter_2nd_fp_t speed_dps_lp_2nd;
// }hall_info_t;


/*joint_state云台的旋转轴 相关信息*/     // 可能会用到

#define REMOLINK_MSG_ID_JOINT_STATE_LEN         72
#define REMOLINK_MSG_ID_JOINT_STATE_CRC_LEN     (REMOLINK_MSG_ID_JOINT_STATE_LEN+12)     //181+7

#define REMOLINK_MSG_ID_JOINT_STATE             0x03

// typedef struct   
// {
//     float joint_deg[3];             // 关节的角度值
//     int32_t joint_deg_raw[3];       // 原始的关节角度值
//     int32_t joint_deg_offset[3];    // 关节角度的偏移量
//     trig_func_f_t joint_trig[3];    // 三角函数值 
// }joint_state_t;


/*joint_data_fit_info_t机械关节相关的数据拟合或校正信息*/       //用不到？
#define REMOLINK_MSG_ID_JOINT_FIT_INFO_LEN         37
#define REMOLINK_MSG_ID_JOINT_FIT_INFO_CRC_LEN     (REMOLINK_MSG_ID_JOINT_FIT_INFO_LEN+12)     //181+7

#define REMOLINK_MSG_ID_JOINT_FIT_INFO             0x04
//joint_data_fit_info_t joint_data_fit_info[3];
// typedef enum{
//     JFIT_IDLE = 0,
//     JFIT_EL_ALIGN,
//     JFIT_INIT_POS,
//     JFIT_LOWER_LIMIT,
//     JFIT_UPPER_LIMIT,
//     JFIT_SAVE_DATA,
//     JFIT_FINISH,
//     JFIT_TEST,
//     JFIT_ERR,
//     JFIT_ERR_EL_ALIGN,
//     JFIT_ERR_INIT_POS,
//     JFIT_ERR_LOWER_LIMIT,
//     JFIT_ERR_UPPER_LIMIT,
//     JFIT_ERR_SAVE_DATA,
// }joint_data_fit_state_t;

//  typedef struct {
//     float *data_pbuf[2];
//     bool init_flag;

//     joint_data_fit_state_t jfit_state;
//     uint32_t align_clock_0p1us[2];
//     uint32_t cnt;
    
//     float angle_th;
//     float delta_angle_th;
//     float angle_last;
    
//     float params[4];
// }joint_data_fit_info_t;




/*gimbal_status_t相关*/     //提取有用的数据
#define REMOLINK_MSG_ID_GIMBAL_STATUS_LEN         36
#define REMOLINK_MSG_ID_GIMBAL_STATUS_CRC_LEN     (REMOLINK_MSG_ID_GIMBAL_STATUS_LEN+12)     //181+7

#define REMOLINK_MSG_ID_GIMBAL_STATUS             0x05
typedef struct
{
    float euler_rad[3];             // 欧拉角   //欧拉角（单位：弧度）  ROLL(0)  PITCH(1) YAW(2)
    //trig_func_f_t euler_trigf[3];   //欧拉角的三角函数值    
    //float euler_deg[3];             // 欧拉角   //欧拉角（单位：度）    
    //float accel_filter[3];          // 滤波后的 3 轴加速度数据      
    //float gyro_rad_comp[3];         //补偿后的陀螺仪角速度（单位：弧度每秒）
    float accel_correct[3];         // 加速度计的修正值             
    float gyro_correct[3];          // 最终修正后的陀螺仪数据       
    //int16_t temp_correct;           // 温度传感器的修正值           
}gimbal_status_t;




//发给摄像头命令
uint16_t remolink_pack_get_camera_msg_s(uint8_t chan, uint8_t insender, uint8_t inreceiver, uint8_t camera_msg_id, uint8_t* buffer);

//打包飞控发给摄像机的消息
uint16_t remolink_pack_flightc_s(uint8_t chan, uint8_t insender, uint8_t inreceiver, mavlink_message_t *ftc_msg, uint8_t* buffer, uint16_t length);


//ahrs相关信息
uint16_t remolink_pack_ahrs_status_s(uint8_t chan, uint8_t insender, uint8_t inreceiver, const ahrs_status_t* ahrs_status, uint8_t* buffer); //把要发送的消息打包到buffer,对云台而言此串口的接收者只能是摄像头，摄像头得标注接收方看要不要转发
void remolink_ahrs_status_decode_r(const remolink_message_t* re_msg, ahrs_status_t* re_ahrs_status);                                                     


//hall_info相关
uint16_t remolink_pack_hall_info_s(uint8_t chan, uint8_t insender, uint8_t inreceiver, const hall_info_t* hall_info_status, uint8_t* buffer);
void remolink_hall_info_decode_r(const remolink_message_t* re_msg, hall_info_t* hall_info_status);

//joint_state_t相关
uint16_t remolink_pack_joint_state_s(uint8_t chan, uint8_t insender, uint8_t inreceiver, const joint_state_t* joint_state, uint8_t* buffer) ;
void remolink_joint_state_decode_r(const remolink_message_t* re_msg, joint_state_t* joint_state);

//joint_data_fit_info_t相关
uint16_t remolink_pack_jjoint_data_fit_info_s(uint8_t chan, uint8_t insender, uint8_t inreceiver, const joint_data_fit_info_t* joint_data_fit_info, uint8_t* buffer);
void remolink_joint_data_fit_info_decode_r(const remolink_message_t* re_msg, joint_data_fit_info_t* joint_data_fit_info);

//gimbal_status_t相关
uint16_t remolink_pack_gimbal_s(uint8_t chan, uint8_t insender, uint8_t inreceiver,  gimbal_status_t* gimbal_status, uint8_t* buffer);
void remolink_gimbal_status_decode_r(const remolink_message_t* re_msg, gimbal_status_t* gimbal_status);
#endif







