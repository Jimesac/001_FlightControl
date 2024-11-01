#include "angle_encoder.h"
#include "bsp_spi.h"
#include "bsp_adc.h"
#include "user_config.h"
#include "ahrs.h"
#include "esc_ctrl.h"
#include <math.h>

#define DEG360_TO_I16   (182.044444f)
#define I16_TO_DEG360   (1.0f/DEG360_TO_I16)
//读写寄存器
#define KTH5701_READ_REGISTER   (0x50U)
#define KTH5701_WRITE_REGISTER  (0x60U)

#define MAG_ENCODER_MEDIAN_SIZE   (5U)

hall_info_t hall_info[3];

joint_state_t joint_state = {
    .joint_deg = {0.0f},
    .joint_deg_raw = {0},
    .joint_deg_offset = {0, 0, 0},
    .joint_trig = {{0.0f, 1.0f},{0.0f, 1.0f}, {0.0f, 1.0f} }
};

struct {
    uint8_t pole_num[3];
    int16_q7_t el_angle[3];
    int16_q7_t *el_angle_offset_ptr[3];

    bool el_align_flag[3];
}el_angle_info = {
    .pole_num = {4, 4, 4},
    .el_align_flag = {false, false, false},
};

int16_t mag_encoder_median_buf[MAG_ENCODER_MEDIAN_SIZE] = {0};
uint8_t mag_encoder_median_cnt;

bool joint_use_euler_flag[3] = {true, true, true};
uint16_t* hall_smp_ptr[2] = {NULL, NULL};

uint8_t magencoder_spi_div = 1;

static void remo_encoder_hall_linear_update(uint8_t index);

static uint8_t remo_spi_register_KTH5701_write(uint16_t writeData,uint8_t Register);
static uint8_t remo_spi_register_KTH5701_init(void);

void angle_encoder_init(void)
{
    remo_spi_register_KTH5701_init();
    //bsp_magencoder_spi_send(0x60, uint8_t addr, uint8_t *wbuff, uint16_t len);


    //bsp_magencoder_spi_receive(uint8_t cmd, uint8_t addr, uint8_t *rbuff, uint16_t len);
    //bsp_magencoder_spi_receive(magencoder_mt6835_write_cmd, magencoder_mt6835_angle_addr, (uint8_t *)&magencoder_mt6835_raw_data, 4);
  
    clock_cpu_delay_ms(200);
    magencoder_spi_div = magencoder_imu_spi_clcok / 5000000;

    hall_smp_ptr[PITCH] = bsp_adc_get_hall_pitch_filter_ptr();
    hall_smp_ptr[ROLL] = bsp_adc_get_hall_roll_filter_ptr();
    
    for (uint8_t i = 0; i < 3; i++)
    {
      remo_esc_set_el_angle_ptr(&el_angle_info.el_angle[i], i);
      el_angle_info.el_angle_offset_ptr[i] = remo_esc_get_el_angle_offset_ptr(i);
    }
}

void angle_encoder_update(void)
{
    trig_func_q14_t trig_q14 = {0};
    int32_t angle_temp = 0;
    int16_q7_t angle_temp_q7;
    uint8_t i = 0;

    uint8_t mag_data[9];
    int16_t mag_angle_temp;
    
    int16_t hall_adc_temp[2] = {0};
    float hall_deg_temp;
    const float *euler_angle = ahrs_get_euler_deg_ptr();

    bsp_magencoder_imu_spi_set_clock_div(magencoder_spi_div);
    // 第一个字节是状态，2、3是16位温度，4、5是16位角度，6、7是16位平面磁场强度，8、9是Z轴磁场强度
    bsp_magencoder_spi_receive_simple(0x4F, 0, mag_data, 5);
    mag_angle_temp =  (int16_t)((mag_data[3] << 8) + mag_data[4]);
    //bsp_magencoder_spi_receive_simple(0x52, 0, angle1, 6);
    //joint_state.joint_deg_raw[YAW] =  (int16_t)((angle1[0] << 8) + angle1[1]);

    mag_encoder_median_buf[mag_encoder_median_cnt] = mag_angle_temp;
    if (++mag_encoder_median_cnt >= MAG_ENCODER_MEDIAN_SIZE) mag_encoder_median_cnt = 0;
    mag_angle_temp = filter_median_i16w5(mag_encoder_median_buf);
    
    // hall 
    for (i = 0; i < 3; i++)
    {
        if (i < 2)
        {
          hall_adc_temp[0] = hall_smp_ptr[i][0] - 32768;  
          hall_adc_temp[1] = hall_smp_ptr[i][1] - 32768;  
        
          //  角度范围为0~360°
          hall_deg_temp = atan2f(hall_adc_temp[0], hall_adc_temp[1]) * RAD_TO_DEG;
          hall_info[i].raw_data = hall_deg_temp;
        }
        else 
        {
            hall_info[i].raw_data = (float)mag_angle_temp * I16_TO_DEG360;
        }

        if (joint_use_euler_flag[i])
        {
            hall_info[i].raw_data_new = euler_angle[i];
        }
        else
        {
            remo_encoder_hall_linear_update(i);
            //hall_info[i].raw_data_new = hall_info[i].raw_data; 
            //if (hall_info[i].raw_data_new >= 180.0f) hall_info[i].raw_data_new -= 360.0f;
            //else if (hall_info[i].raw_data_new < -180.0f) hall_info[i].raw_data_new += 360.0f;
        }

        joint_state.joint_deg_raw[i] = hall_info[i].raw_data_new * DEG360_TO_I16;
    }
  
    for (i = 0; i < 3; i++)
    {
        angle_temp = joint_state.joint_deg_raw[i] - joint_state.joint_deg_offset[i];

        if (angle_temp >= 32768)
        {
            angle_temp -= 65536;
        }
        else if (angle_temp < -32768)
        {
            angle_temp += 65536;
        }
        joint_state.joint_deg[i] = (float)angle_temp*I16_TO_DEG360;
        angle_temp_q7 = joint_state.joint_deg[i]*128;


        remo_trig_func(angle_temp_q7, &trig_q14);
        joint_state.joint_trig[i].cos = (float)trig_q14.cos/CONST_1_Q14;
        joint_state.joint_trig[i].sin = (float)trig_q14.sin/CONST_1_Q14;

        //电角度目前是2^7定标
        angle_temp = joint_state.joint_deg_raw[i] * el_angle_info.pole_num[i] - *el_angle_info.el_angle_offset_ptr[i];
        // 将电角度调整到-180~180度之间
        while (angle_temp >= 32768)
        {
            angle_temp -= 65536;
        }
        while (angle_temp < -32768)
        {
            angle_temp += 65536;
        }
        el_angle_info.el_angle[i] = (angle_temp*45) >> 6;   // 转成Q7格式

        if (el_angle_info.el_align_flag[i])
        {
            remo_esc_set_esc_status(ESC_ALIGNMENT, i);
            el_angle_info.el_align_flag[i] = false;
        }
    }
    
    

    //bsp_magencoder_spi_receive(magencoder_mt6835_write_cmd, magencoder_mt6835_angle_addr, (uint8_t *)&magencoder_mt6835_raw_data, 4);
}

static void remo_encoder_hall_linear_update(uint8_t index)
{
    float angle_temp;
    int16_t data_index;
    uint16_t max_index = 0, index0 = 0, index1 = 0;
    uint16_t i = 0;
    float k = 0.0f, tempf = 0.0f, tempf_trig[2] = {0.0f};

    angle_temp = hall_info[index].raw_data - hall_info[index].raw_data0_offset;
    if (angle_temp < 0.0f) angle_temp += 360.0f;
    else if (angle_temp > 360.0f) angle_temp -= 360.0f;
    max_index = hall_info[index].linear_num-1;

    // 初始化，找到hall原始角度所在的位置索引，0°~360°，[lower, upper)为区间进行确定
    if (angle_temp < hall_info[index].linear_pbuf[1][0] || angle_temp > 355.0f) 
    {
        data_index = -1;
        if (angle_temp > 355.0f) angle_temp -= 360.0f;   // 0°在极限位置可能出现359°这种情况
    }
    else if (angle_temp >= hall_info[index].linear_pbuf[1][max_index])
    {
        data_index = max_index+1; 
    }
    else
    {
        index0 = 0;
        index1 = max_index;
        while(1)
        {
            i = index0 + ((index1 - index0) >> 1);
            if (angle_temp > hall_info[index].linear_pbuf[1][i])
            {
                index0 = i;
            }
            else
            {
                index1 = i;
            }
            if (index1 - index0 <= 1) break;
        }
        data_index = index0;
    }

    // 进行线性插值更新
    if (data_index == -1 || data_index == 0)
    {
        tempf = (hall_info[index].linear_pbuf[1][1] - hall_info[index].linear_pbuf[1][0]);
        if (tempf == 0) tempf = 0.01f;
        k = (hall_info[index].linear_pbuf[0][1] - hall_info[index].linear_pbuf[0][0]) / tempf;
        tempf = (angle_temp - hall_info[index].linear_pbuf[1][0]);
        if (tempf > 180.0f) tempf -= 360.0f;   // 解决hall_angle[0] = 0, raw_data = 359°这种情况
        hall_info[index].raw_data_new = k * tempf + hall_info[index].linear_pbuf[0][0];
    }
    else if (data_index == max_index+1 || data_index == max_index)
    {
        tempf = (hall_info[index].linear_pbuf[1][max_index] - hall_info[index].linear_pbuf[1][max_index-1]);
        if (tempf == 0) tempf = 0.01f;
        k = (hall_info[index].linear_pbuf[0][max_index] - hall_info[index].linear_pbuf[0][max_index-1]) / tempf;
        tempf = (angle_temp - hall_info[index].linear_pbuf[1][max_index]);
        if (tempf < -180.0f) tempf += 360.0f;   // 解决hall_angle[0] = 1, raw_data = 359°这种情况
        hall_info[index].raw_data_new = k * tempf + hall_info[index].linear_pbuf[0][max_index-1];
    }
    else
    {
        i = data_index;
        tempf = (hall_info[index].linear_pbuf[1][i+1] - hall_info[index].linear_pbuf[1][i]);
        if (tempf == 0) tempf = 0.01f;
        k = (hall_info[index].linear_pbuf[0][i+1] - hall_info[index].linear_pbuf[0][i]) / tempf;
        hall_info[index].raw_data_new = k * (angle_temp - hall_info[index].linear_pbuf[1][i]) + hall_info[index].linear_pbuf[0][i];
    }

}


static uint8_t remo_spi_register_KTH5701_write(uint16_t writeData, uint8_t Register)
{
    uint8_t i;	
    uint8_t registerName;
    uint8_t sta = 0;
    uint8_t comWR[3]; 

    registerName = Register <<2 ;
    comWR[0] = writeData >> 8; 
    comWR[1] = writeData & 0xff;
    comWR[2] = registerName;

    //remo_spi_register_write_spi(0x00, 4, comWR, ROLL_ANGLE_SPI_TYPE);
    bsp_magencoder_spi_send(KTH5701_WRITE_REGISTER, 0, comWR, 3);

    return sta;
}

static uint8_t remo_spi_register_KTH5701_init(void)
{
    uint8_t temp_KTH5701[3]; 
    uint8_t KTH5701_ID[3];

    //remo_spi_register_read_bytes(0x80, 1, temp_KTH5701, ROLL_ANGLE_SPI_TYPE);
    //remo_spi_register_read_bytes(0xF0, 0, temp_KTH5701, ROLL_ANGLE_SPI_TYPE);
    bsp_magencoder_spi_receive(0x80, 0, temp_KTH5701, 1);

    clock_cpu_delay_ms(10);
    remo_spi_register_KTH5701_write(0xc000, 27);
    clock_cpu_delay_ms(10);
    remo_spi_register_KTH5701_write(0x0727, 28);
    clock_cpu_delay_ms(10);	
    remo_spi_register_KTH5701_write(0x00C3, 29);
    clock_cpu_delay_ms(10);	
    remo_spi_register_KTH5701_write(0x8000, 30);

    clock_cpu_delay_ms(10);
    //remo_spi_register_read_bytes(0x1F,1,temp_KTH5701,ROLL_ANGLE_SPI_TYPE);
    bsp_magencoder_spi_receive(0x1F, 0, temp_KTH5701, 1);
    clock_cpu_delay_ms(10);
}

void remo_encoder_set_joint_use_euler_flag(bool flag, uint8_t index)
{
    if (index > 2) return;
    joint_use_euler_flag[index] = flag;
}

void remo_encoder_set_hall_linear_pbuf(float *pbuf1, float *pbuf2, uint8_t index)
{
    if (index > 2) return;
    hall_info[index].linear_pbuf[0] = pbuf1;
    hall_info[index].linear_pbuf[1] = pbuf2;
}

void remo_encoder_set_hall_linear_flag(bool flag, uint8_t index)
{
    if (index > 2) return;
    hall_info[index].linear_flag = flag;
    joint_use_euler_flag[index] = !flag;
}

void remo_encoder_set_hall_linear_num(uint16_t num, uint8_t index)
{
    if (index > 2) return;
    hall_info[index].linear_num = num;
}

void remo_encoder_set_hall_raw_data0_offset(float offset, uint8_t index)
{
    if (index > 2) return;
    hall_info[index].raw_data0_offset = offset;
}

void remo_encoder_set_joint_deg_offset(float offset, uint8_t index)
{
    if (index > 2) return;
    joint_state.joint_deg_offset[index] = offset;
}



/**************-***********************/
float remo_encoder_get_hall_raw_data(uint8_t index)
{
    if (index > 2) return 0.0f;
    return hall_info[index].raw_data;
}


const float *remo_encoder_get_joint_deg_ptr(void)
{
	return &joint_state.joint_deg[0];
}

const trig_func_f_t *remo_encoder_get_joint_trig(uint8_t type)
{
	return (trig_func_f_t *)&joint_state.joint_trig[type];
}