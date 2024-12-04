#include "remolink_msg_payload.h"
extern ahrs_status_t ahrs_status;


/*给摄像头发送命令返回摄像头信息打包*/
uint16_t remolink_pack_get_camera_msg_s(uint8_t chan, uint8_t insender, uint8_t inreceiver, uint8_t camera_msg_id, uint8_t* buffer)
{
    uint16_t length;
    remolink_pack_to_message_s(remolink_get_chan_message(chan) , insender , inreceiver, camera_msg_id, 0 ,remolink_get_chan_state(chan));
    length = remolink_message_to_send_buffer_s(remolink_get_chan_send_buffer(chan),remolink_get_chan_message(chan));
    memcpy(buffer, remolink_get_chan_send_buffer(chan), length);
    return length;
}

/*飞控信息*/
uint16_t remolink_pack_flightc_s(uint8_t chan, uint8_t insender, uint8_t inreceiver, mavlink_message_t *ftc_msg, uint8_t* buffer, uint16_t length)
{
    uint16_t length2;
    memcpy(MOVE_PAYLOAD_NON_CONST(remolink_get_chan_message(chan)), ftc_msg, length);
    remolink_pack_to_message_s(remolink_get_chan_message(chan) , insender , inreceiver, REMOLINK_MSG_ID_FTC_STATUS, length ,remolink_get_chan_state(chan));
    length2 = remolink_message_to_send_buffer_s(remolink_get_chan_send_buffer(chan),remolink_get_chan_message(chan));
    memcpy(buffer, remolink_get_chan_send_buffer(chan), length2);
    return length2;

}

/*ahrs相关信息*/

static inline uint16_t remolink_msg_ahrs_status_pack_encode_s(uint8_t insender, uint8_t inreceiver, remolink_state_t *state, remolink_message_t* msg,
                                                     const ahrs_status_t* ahrs_status)
{
    memcpy(MOVE_PAYLOAD_NON_CONST(msg), ahrs_status, REMOLINK_MSG_ID_AHRS_STATUS_LEN);
    //msg->msgid=REMOLINK_MSG_ID_AHRS_STATUS;
    remolink_pack_to_message_s(msg , insender , inreceiver, REMOLINK_MSG_ID_AHRS_STATUS, REMOLINK_MSG_ID_AHRS_STATUS_LEN ,state);
    return REMOLINK_MSG_ID_AHRS_STATUS_CRC_LEN;
}

uint16_t remolink_pack_ahrs_status_s(uint8_t chan, uint8_t insender,uint8_t inreceiver, const ahrs_status_t* ahrs_status, uint8_t* buffer)         
{
    uint16_t length;
    remolink_msg_ahrs_status_pack_encode_s(insender, inreceiver, remolink_get_chan_state(chan), remolink_get_chan_message(chan), ahrs_status);
    length = remolink_message_to_send_buffer_s(remolink_get_chan_send_buffer(chan),remolink_get_chan_message(chan));
    memcpy(buffer, remolink_get_chan_send_buffer(chan), length);
    //buffer = remolink_get_chan_send_buffer(chan);
    return length;
}



void remolink_ahrs_status_decode_r(const remolink_message_t* re_msg, ahrs_status_t* re_ahrs_status)
{
    uint16_t length;
    length=REMOLINK_MSG_ID_AHRS_STATUS_LEN;
    memcpy(re_ahrs_status, MOVE_PAYLOAD_NON_CONST(re_msg), length);
}


/*hall_info相关信息*/

static inline uint16_t remolink_msg_hall_info_pack_encode_s(uint8_t insender, uint8_t inreceiver, remolink_state_t *state, remolink_message_t* msg,
                                                     const hall_info_t* hall_info_status)
{
    memcpy(MOVE_PAYLOAD_NON_CONST(msg), hall_info_status, REMOLINK_MSG_ID_HALL_INFO_LEN);
    //msg->msgid=REMOLINK_MSG_ID_AHRS_STATUS;
    remolink_pack_to_message_s(msg , insender , inreceiver, REMOLINK_MSG_ID_HALL_INFO, REMOLINK_MSG_ID_HALL_INFO_LEN ,state);
    return REMOLINK_MSG_ID_HALL_INFO_CRC_LEN;
}

uint16_t remolink_pack_hall_info_s(uint8_t chan, uint8_t insender, uint8_t inreceiver, const hall_info_t* hall_info_status, uint8_t* buffer)           
{
    uint16_t length;
    remolink_msg_hall_info_pack_encode_s(insender, inreceiver, remolink_get_chan_state(chan), remolink_get_chan_message(chan), hall_info_status);
    length = remolink_message_to_send_buffer_s(remolink_get_chan_send_buffer(chan),remolink_get_chan_message(chan));
    memcpy(buffer, remolink_get_chan_send_buffer(chan), length);
    //buffer = remolink_get_chan_send_buffer(chan);
    return length;
}



void remolink_hall_info_decode_r(const remolink_message_t* re_msg, hall_info_t* hall_info_status)
{
    uint16_t length;
    length=REMOLINK_MSG_ID_HALL_INFO_LEN;
    memcpy(hall_info_status, MOVE_PAYLOAD_NON_CONST(re_msg), length);
}

/*joint_state相关信息*/

static inline uint16_t remolink_msg_joint_state_pack_encode_s(uint8_t insender, uint8_t inreceiver, remolink_state_t *state, remolink_message_t* msg,
                                                     const joint_state_t* joint_state)
{
    memcpy(MOVE_PAYLOAD_NON_CONST(msg), joint_state, REMOLINK_MSG_ID_JOINT_STATE_LEN);
    //msg->msgid=REMOLINK_MSG_ID_AHRS_STATUS;
    remolink_pack_to_message_s(msg , insender , inreceiver, REMOLINK_MSG_ID_JOINT_STATE, REMOLINK_MSG_ID_JOINT_STATE_LEN ,state);
    return REMOLINK_MSG_ID_JOINT_STATE_CRC_LEN;
}

uint16_t remolink_pack_joint_state_s(uint8_t chan, uint8_t insender, uint8_t inreceiver, const joint_state_t* joint_state, uint8_t* buffer)          
{
    uint16_t length;
    remolink_msg_joint_state_pack_encode_s(insender, inreceiver, remolink_get_chan_state(chan), remolink_get_chan_message(chan), joint_state);
    length = remolink_message_to_send_buffer_s(remolink_get_chan_send_buffer(chan),remolink_get_chan_message(chan));
    memcpy(buffer, remolink_get_chan_send_buffer(chan), length);
    //buffer = remolink_get_chan_send_buffer(chan);
    return length;
}



void remolink_joint_state_decode_r(const remolink_message_t* re_msg, joint_state_t* joint_state)
{
    uint16_t length;
    length=REMOLINK_MSG_ID_JOINT_STATE_LEN;
    memcpy(joint_state, MOVE_PAYLOAD_NON_CONST(re_msg), length);
}


/***********************joint_data_fit_info_t相关信息****************************/
static inline uint16_t remolink_msg_joint_data_fit_info_pack_encode_s(uint8_t insender, uint8_t inreceiver, remolink_state_t* state, remolink_message_t* msg,
                                                     const joint_data_fit_info_t* joint_data_fit_info)
{
    memcpy(MOVE_PAYLOAD_NON_CONST(msg), joint_data_fit_info, REMOLINK_MSG_ID_JOINT_FIT_INFO_LEN);
    //msg->msgid=REMOLINK_MSG_ID_AHRS_STATUS;
    remolink_pack_to_message_s(msg , insender , inreceiver, REMOLINK_MSG_ID_JOINT_FIT_INFO, REMOLINK_MSG_ID_JOINT_FIT_INFO_LEN ,state);
    return REMOLINK_MSG_ID_JOINT_FIT_INFO_CRC_LEN;
}

uint16_t remolink_pack_jjoint_data_fit_info_s(uint8_t chan, uint8_t insender, uint8_t inreceiver, const joint_data_fit_info_t* joint_data_fit_info, uint8_t* buffer)         
{
    uint16_t length;
    remolink_msg_joint_data_fit_info_pack_encode_s(insender, inreceiver, remolink_get_chan_state(chan), remolink_get_chan_message(chan), joint_data_fit_info);
    length = remolink_message_to_send_buffer_s(remolink_get_chan_send_buffer(chan),remolink_get_chan_message(chan));
    memcpy(buffer, remolink_get_chan_send_buffer(chan), length);
    //buffer = remolink_get_chan_send_buffer(chan);
    return length;
}



void remolink_joint_data_fit_info_decode_r(const remolink_message_t* re_msg, joint_data_fit_info_t* joint_data_fit_info)
{
    uint16_t length;
    length=REMOLINK_MSG_ID_JOINT_FIT_INFO_LEN;
    memcpy(joint_data_fit_info, MOVE_PAYLOAD_NON_CONST(re_msg), length);
}



/******************gimbal_status_t相关信息*************/

uint16_t remolink_pack_gimbal_s(uint8_t chan, uint8_t insender, uint8_t inreceiver,  gimbal_status_t* gimbal_status, uint8_t* buffer)
{
    uint16_t length;
    const float *accel_correct_p;
    const float *gyro_correct_p;
    gimbal_status->euler_rad[0] = ahrs_status.euler_rad[0];
    gimbal_status->euler_rad[1] = ahrs_status.euler_rad[1];
    gimbal_status->euler_rad[2] = ahrs_status.euler_rad[2];
    accel_correct_p =  remo_imu_get_accel_corr();
    gimbal_status->accel_correct[0] =accel_correct_p[0];
    gimbal_status->accel_correct[1] =accel_correct_p[1];
    gimbal_status->accel_correct[2] =accel_correct_p[2];
    gyro_correct_p = remo_imu_get_gyro_corr();
    gimbal_status->gyro_correct[0] = gyro_correct_p[0];
    gimbal_status->gyro_correct[1] = gyro_correct_p[1];
    gimbal_status->gyro_correct[2] = gyro_correct_p[2];
    memcpy(MOVE_PAYLOAD_NON_CONST(remolink_get_chan_message(chan)), gimbal_status, REMOLINK_MSG_ID_GIMBAL_STATUS_LEN);
    remolink_pack_to_message_s(remolink_get_chan_message(chan) , insender , inreceiver, REMOLINK_MSG_ID_GIMBAL_STATUS, REMOLINK_MSG_ID_GIMBAL_STATUS_LEN ,remolink_get_chan_state(chan));
    length = remolink_message_to_send_buffer_s(remolink_get_chan_send_buffer(chan),remolink_get_chan_message(chan));
    memcpy(buffer, remolink_get_chan_send_buffer(chan), length);
    return length;

}

void remolink_gimbal_status_decode_r(const remolink_message_t* re_msg, gimbal_status_t* gimbal_status)
{
    uint16_t length;
    length=REMOLINK_MSG_ID_JOINT_FIT_INFO_LEN;
    memcpy(gimbal_status, MOVE_PAYLOAD_NON_CONST(re_msg), length);
}
