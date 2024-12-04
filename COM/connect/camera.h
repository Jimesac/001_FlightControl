#ifndef __CAMERA_H__
#define __CAMERA_H__

#include "remolink.h"
#include "remolink_msg_payload.h"
#include "Flightcontrol_meg.h"

#include "bsp_uart.h"
#include "bsp_dma.h"
#include "ahrs.h"
#include "angle_encoder.h"
#include "calibrate.h"


typedef enum 
{
    S_DIRECTION_INIT=0,
    FLIGHT_TO_CAMERA=1
}sending_direction;


void remolink_flight_to_camera_send(uint8_t chan, uint8_t insender, mavlink_message_t* ftc_msg,uint16_t length);

void remolink_ahrs_status_send(uint8_t chan, uint8_t insender, const ahrs_status_t* ahrs_status);
void remolink_hall_info_send(uint8_t chan, uint8_t insender, const hall_info_t* hall_info);
void remolink_joint_state_send(uint8_t chan, uint8_t insender, const joint_state_t* joint_state);
void remolink_joint_data_fit_info_send(uint8_t chan, uint8_t insender, const joint_data_fit_info_t* joint_data_fit_info);
void remolink_gimbal_send(uint8_t chan, uint8_t insender, const gimbal_status_t* gimbal_status);

void remolink_receive_r(uint8_t channel, uint8_t Rx_data);



#endif




