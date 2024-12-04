#include "Flightcontrol_meg.h"
#include "camera.h"

extern uint8_t Flag_sending_direction;




//mavlink协议发送函数
void mavlink_send_message(uint8_t chan, uint8_t system_id, uint8_t component_id, uint8_t min_length, uint8_t message_id, uint8_t* payload, uint8_t payload_len , uint8_t crc_extra) {
    mavlink_message_t *msg;
    mavlink_status_t *status;
    msg = mavlink_get_channel_buffer(chan);
    status = mavlink_get_channel_status(chan);
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len;

    // 打包消息，将系统ID、组件ID和有效载荷打包到消息中
    msg->msgid = message_id;
    mavlink_finalize_message_buffer(msg,system_id, component_id, status, min_length, payload_len ,crc_extra);

    // 序列化消息，将消息转为字节流存入缓冲区
    len = mavlink_msg_to_send_buffer(buffer, msg);

    // 发送字节流
    for (uint16_t i = 0; i < len; i++) {
        //serial_write(buffer[i]); // 将数据写入发送接口，例如串口
    }
}

//mavlink协议发送函数,通过自带的通道(通道0)
void mavlink_send_message_chan(uint8_t system_id, uint8_t component_id, uint8_t min_length, uint8_t message_id, uint8_t* payload, uint8_t payload_len , uint8_t crc_extra) {
    mavlink_message_t msg;
    //mavlink_status_t status;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len;

    // 打包消息，将系统ID、组件ID和有效载荷打包到消息中
    msg.msgid = message_id;
    mavlink_finalize_message(&msg , system_id, component_id, min_length , payload_len ,crc_extra);      //自动分配 mavlink_status_t status

    // 序列化消息，将消息转为字节流存入缓冲区
    len = mavlink_msg_to_send_buffer(buffer, &msg);

    // 发送字节流
    for (uint16_t i = 0; i < len; i++) {
        //serial_write(buffer[i]); // 将数据写入发送接口，例如串口
    }
}


//发送姿态信息
void mavlink_send_message_attitude(uint8_t system_id, uint8_t component_id)
{
    mavlink_message_t msg;          //消息信息  外部自己定义
    mavlink_attitude_t attitude;    //姿态信息
    uint16_t len;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint8_t mykey[32]={0x01,0x02};
    mavlink_status_t status1;

    mavlink_set_proto_version(MAVLINK_COMM_0,2);        //status 通道0 设置版本V2
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
    status->signing->flags = 1;             //设为V2
    status->signing->link_id=1;
    status->signing->timestamp=0;   //时间戳
    memcpy(status->signing->secret_key, mykey, sizeof(mykey));

    mavlink_msg_attitude_encode_chan( system_id,  component_id,  MAVLINK_COMM_0, &msg, &attitude);  //status 通道0
    mavlink_msg_attitude_encode_status(system_id,  component_id, &status1 , &msg, &attitude);
    // 序列化消息，将消息转为字节流存入缓冲区
    len = mavlink_msg_to_send_buffer(buffer, &msg);
    // 发送字节流
    for (uint16_t i = 0; i < len; i++) {
        //serial_write(buffer[i]); // 将数据写入发送接口，例如串口
    }

}


//请求发送数据 
//start_stop    1:开始发送；0:停止发送
void mavlink_send_message_request_data(uint8_t system_id,uint8_t component_id,uint8_t R_system_id,uint8_t R_component_id,uint8_t R_message_id,uint8_t start_stop_m)
{
    mavlink_message_t msg;          //消息信息  外部自己定义
    mavlink_request_data_stream_t request_data_stream;
    uint16_t len;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint8_t mykey[32]={0x01,0x02};

    mavlink_set_proto_version(MAVLINK_COMM_0,2);        //status 通道0 设置版本V2
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
    status->signing->flags = 1;             //设为V2
    status->signing->link_id=1;
    status->signing->timestamp=0;   //时间戳
    //status->signing->secret_key={};     //uint8_t secret_key[32]; 
    memcpy(status->signing->secret_key, mykey, sizeof(mykey));

    request_data_stream.req_message_rate    = 11520;//.....
    request_data_stream.target_system       = R_system_id;
    request_data_stream.target_component    = R_component_id;   
    request_data_stream.req_stream_id       = R_message_id;
    request_data_stream.start_stop          = start_stop_m;   

    mavlink_msg_request_data_stream_encode_chan( system_id,  component_id,  MAVLINK_COMM_0,  &msg, &request_data_stream);
    // 序列化消息，将消息转为字节流存入缓冲区
    len = mavlink_msg_to_send_buffer(buffer, &msg);
    // 发送字节流
    for (uint16_t i = 0; i < len; i++) {
        //serial_write(buffer[i]); // 将数据写入发送接口，例如串口
    }

}
















//mavlink协议 接收消息函数   接收通道和发送通道不能一样
void mavlink_receive_message(uint8_t chan,uint8_t *u8_char,uint16_t len) {
    mavlink_message_t msg_r;
    mavlink_status_t status_r;
    uint8_t mykey[32]={0x01,0x02};
    uint16_t i;

    mavlink_message_t *msg = mavlink_get_channel_buffer(chan);
    mavlink_status_t *status = mavlink_get_channel_status(chan);
    //status->signing->flags = 1;
    //status->signing->link_id=1;
    //status->signing->timestamp=0;   //时间戳  自动更新
    //status->signing->secret_key={};     //uint8_t secret_key[32];
    memcpy(status->signing->secret_key, mykey, sizeof(mykey));

    // 从接口逐字节读取数据（例如串口或网络流）
    for(i=0;i<len;i++)
    {
        //uint8_t c ;//= serial_read();

        // 解析数据
        if (mavlink_parse_char(chan, u8_char[i], &msg_r, &status_r)==MAVLINK_FRAMING_OK) {     //MAVLINK_COMM_0   ==MAVLINK_FRAMING_OK
            // 成功解析出一条消息，处理该消息
            //handle_mavlink_message(msg_r);
            //break
            
        }
    }
}

//mavlink协议 接收消息函数,有默认通道（通道0）
void mavlink_receive_message_chan(void) {
    mavlink_message_t msg;
    mavlink_status_t status;

    // 从接口逐字节读取数据（例如串口或网络流）
    //while (serial_available()) 
    {
        uint8_t c ;//= serial_read();

        // 解析数据
        if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {     //MAVLINK_COMM_0
            // 成功解析出一条消息，处理该消息
            //handle_mavlink_message(msg);
        }
    }
}



void handle_mavlink_message(mavlink_message_t* msg) {

    if(Flag_sending_direction == FLIGHT_TO_CAMERA)
    {
        remolink_flight_to_camera_send(0, FLIGHTC_COMPID, msg, msg->len + MAVLINK_NUM_NON_PAYLOAD_BYTES);
        Flag_sending_direction=S_DIRECTION_INIT;
    }
    else switch (msg->msgid) 
    {
        case MAVLINK_MSG_ID_HEARTBEAT:
            // 处理心跳消息
            mavlink_heartbeat_t heartbeat;
            mavlink_msg_heartbeat_decode(msg, &heartbeat);
            break;

        case MAVLINK_MSG_ID_ATTITUDE:
            // 处理姿态消息
            mavlink_attitude_t attitude;
            mavlink_msg_attitude_decode(msg, &attitude);
            break;

        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
            //经纬度
            mavlink_global_position_int_t global_position;
            mavlink_msg_global_position_int_decode(msg,&global_position);
            break;

        case MAVLINK_MSG_ID_VFR_HUD:
            //目视飞行平视显示器
            mavlink_vfr_hud_t vfr_hud;
            mavlink_msg_vfr_hud_decode(msg,&vfr_hud);
            break;

        case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
            //RC通道数据
            mavlink_rc_channels_raw_t rc_channels_raw;
            mavlink_msg_rc_channels_raw_decode(msg,&rc_channels_raw);
            break;

        case MAVLINK_MSG_ID_LANDING_TARGET:
            //着陆目标
            mavlink_landing_target_t landing_target;
            mavlink_msg_landing_target_decode(msg, &landing_target);
            break;

        case MAVLINK_MSG_ID_HOME_POSITION:
            //原位置
            mavlink_home_position_t home_position;
            mavlink_msg_home_position_decode(msg, &home_position);
            break;

        case MAVLINK_MSG_ID_SYS_STATUS: 
            //系统状态
            mavlink_sys_status_t sys_status;
            mavlink_msg_sys_status_decode(msg, &sys_status);
            break;

        case MAVLINK_MSG_ID_BATTERY_STATUS:
            //电池状态
            mavlink_battery_status_t battery_status;
            mavlink_msg_battery_status_decode(msg, &battery_status);
            break;

        case MAVLINK_MSG_ID_GPS_STATUS:
            //GPS状态
            mavlink_gps_status_t gps_status;
            mavlink_msg_gps_status_decode(msg, &gps_status);
            break;

        case MAVLINK_MSG_ID_RADIO_STATUS:
            //收音机状态
            mavlink_radio_status_t radio_status;
            mavlink_msg_radio_status_decode(msg, &radio_status);
            break;




        // 处理其他消息类型
        default:
            break;
    }
}




