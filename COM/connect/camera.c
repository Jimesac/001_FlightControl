#include "camera.h"



extern ahrs_status_t ahrs_status;
extern hall_info_t hall_info[3];
extern joint_state_t joint_state;
extern joint_data_fit_info_t joint_data_fit_info[3];
uint8_t Flag_sending_direction;



/****************************发送**********************/
void remolink_get_camera_send(uint8_t chan, uint8_t insender, uint8_t camera_msg_id)    //让摄像头发送指定信息
{
    uint8_t* buffer;
    uint8_t* spbuf;
    uint16_t length;
    length=remolink_pack_get_camera_msg_s(chan, insender,CAMERA_COMPID, camera_msg_id, buffer);
    spbuf = bsp_uart_com_ma_get_tx_pbuf();
    memcpy(spbuf,buffer,length);
    bsp_uart_com_ma_set_tx_size(length);
    bsp_uart_com_ma_schedule_transmit();

}


void remolink_flight_to_camera_send(uint8_t chan, uint8_t insender, mavlink_message_t* ftc_msg,uint16_t length)
{
    uint8_t* buffer;
    uint8_t* spbuf;
    uint16_t remolink_length;
    remolink_length=remolink_pack_flightc_s( chan,  insender, CAMERA_COMPID, ftc_msg, buffer , length);
    spbuf = bsp_uart_com_ma_get_tx_pbuf();
    memcpy(spbuf,buffer,remolink_length);
    bsp_uart_com_ma_set_tx_size(remolink_length);
    bsp_uart_com_ma_schedule_transmit();

}


void remolink_ahrs_status_send(uint8_t chan, uint8_t insender, const ahrs_status_t* ahrs_status)
{
    uint8_t* buffer;
    uint8_t* spbuf;
    uint16_t length;
    length=remolink_pack_ahrs_status_s(chan, insender, CAMERA_COMPID, ahrs_status, buffer);
    spbuf = bsp_uart_com_ma_get_tx_pbuf();
    memcpy(spbuf,buffer,length);
    bsp_uart_com_ma_set_tx_size(length);
    bsp_uart_com_ma_schedule_transmit();

}

void remolink_hall_info_send(uint8_t chan, uint8_t insender, const hall_info_t* hall_info)    //hall_info发送
{
    uint8_t* buffer;
    uint8_t* spbuf;
    uint16_t length;
    length=remolink_pack_hall_info_s(chan, insender, CAMERA_COMPID, hall_info, buffer);
    spbuf = bsp_uart_com_ma_get_tx_pbuf();
    memcpy(spbuf,buffer,length);
    bsp_uart_com_ma_set_tx_size(length);
    bsp_uart_com_ma_schedule_transmit();

}

void remolink_joint_state_send(uint8_t chan, uint8_t insender, const joint_state_t* joint_state)    //joint_state_t 发送
{
    uint8_t* buffer;
    uint8_t* spbuf;
    uint16_t length;
    length=remolink_pack_joint_state_s(chan, insender, CAMERA_COMPID, joint_state, buffer);
    spbuf = bsp_uart_com_ma_get_tx_pbuf();
    memcpy(spbuf,buffer,length);
    bsp_uart_com_ma_set_tx_size(length);
    bsp_uart_com_ma_schedule_transmit();

}

void remolink_joint_data_fit_info_send(uint8_t chan, uint8_t insender, const joint_data_fit_info_t* joint_data_fit_info)    //joint_data_fit_info_t 发送
{
    uint8_t* buffer;
    uint8_t* spbuf;
    uint16_t length;
    length=remolink_pack_jjoint_data_fit_info_s(chan, insender, CAMERA_COMPID, joint_data_fit_info, buffer);
    spbuf = bsp_uart_com_ma_get_tx_pbuf();
    memcpy(spbuf,buffer,length);
    bsp_uart_com_ma_set_tx_size(length);
    bsp_uart_com_ma_schedule_transmit();

}

void remolink_gimbal_status_send(uint8_t chan, uint8_t insender, const gimbal_status_t* gimbal_status)    //gimbal_status_t 发送
{
    uint8_t* buffer;
    uint8_t* spbuf;
    uint16_t length;
    length=remolink_pack_gimbal_s(chan, insender, CAMERA_COMPID, gimbal_status, buffer);
    spbuf = bsp_uart_com_ma_get_tx_pbuf();
    memcpy(spbuf,buffer,length);
    bsp_uart_com_ma_set_tx_size(length);
    bsp_uart_com_ma_schedule_transmit();

}






/************************接收消息*********************/

static void remolink_handle_message_r(remolink_message_t* r_msg);


void remolink_receive_r(uint8_t channel, uint8_t Rx_data) //一个一个接收的情况
{
    remolink_message_t return_rx_msg;
    remolink_state_t return_state;
    if( remolink_receive_to_messige_chan_r(channel, Rx_data, &return_rx_msg, &return_state) == REMOLINK_RE_OK )
    {
        remolink_handle_message_r(&return_rx_msg);
    }
}

static void remolink_handle_message_r(remolink_message_t* r_msg)
{
    uint8_t* spbuf;
    uint8_t i;
    if(r_msg->receiver == GIMBAL_COMPID)    //sender为摄像头  接收者为云台
    {
        if(r_msg->command == REMOLINK_COMMAND_RECEIVE)      //接收消息
        {
            switch (r_msg->msgid)
            {
            case REMOLINK_MSG_ID_AHRS_STATUS:
                 ahrs_status_t re_ahrs_status;
                 remolink_ahrs_status_decode_r(r_msg,  &re_ahrs_status);
                break;
            case REMOLINK_MSG_ID_HALL_INFO:
                 hall_info_t hall_info;
                 remolink_hall_info_decode_r(r_msg, &hall_info);
                 break;
            case REMOLINK_MSG_ID_JOINT_STATE:
                 joint_state_t joint_state;
                 remolink_joint_state_decode_r(r_msg, &joint_state);
                 break;
            case REMOLINK_MSG_ID_JOINT_FIT_INFO:
                 joint_data_fit_info_t joint_data_fit_info;
                 remolink_joint_data_fit_info_decode_r(r_msg, &joint_data_fit_info);
                 break; 
            case REMOLINK_MSG_ID_GIMBAL_STATUS:
                 gimbal_status_t gimbal_status;
                 remolink_gimbal_status_decode_r(r_msg, &gimbal_status);             
                break;

            default:
                break;
            }
        }
        else if(r_msg->command == REMOLINK_COMMAND_SEND)   //给摄像头发送指定消息 
        {
            
            switch (r_msg->msgid)
            {
            case REMOLINK_MSG_ID_AHRS_STATUS:
                 //发送AHRS的消息
                 remolink_ahrs_status_send(0, GIMBAL_COMPID, &ahrs_status);
                break;
            case REMOLINK_MSG_ID_HALL_INFO:
                 for(i=0;i<3;i++)
                    remolink_hall_info_send(0, GIMBAL_COMPID, &hall_info[i]);
                 break;
            case REMOLINK_MSG_ID_JOINT_STATE:
                 remolink_joint_state_send(0, GIMBAL_COMPID, &joint_state);
                 break;
            case REMOLINK_MSG_ID_JOINT_FIT_INFO:
                 for(i=0;i<3;i++)
                    remolink_joint_data_fit_info_send(0, GIMBAL_COMPID, &joint_data_fit_info[i]);
                 break;  
            case REMOLINK_MSG_ID_GIMBAL_STATUS:
                 gimbal_status_t gimbal_status;
                 remolink_gimbal_status_send(0, GIMBAL_COMPID, &gimbal_status);
                 break;  
            
            default:
                break;
            }
        }
       
    }
    else if(r_msg->receiver == GIMBAL_COMPID)//sender为摄像头  接收者为飞控
    {
        //给飞控发送r_msg->payload8里面的消息
        spbuf = bsp_uart_com_fc_get_tx_pbuf();
        memcpy(spbuf,r_msg->payload8,r_msg->len);
        bsp_uart_com_fc_set_tx_size(r_msg->len);
        bsp_uart_com_fc_schedule_transmit();
        //flag
        Flag_sending_direction = FLIGHT_TO_CAMERA;

    }

    
}



