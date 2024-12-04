#ifndef __FIGHTCONTROL_H__
#define __FIGHTCONTROL_H__
#include "common/mavlink.h"
//#include "./mavlink_helpers.h"

void mavlink_send_message_attitude(uint8_t system_id, uint8_t component_id);
void mavlink_receive_message(uint8_t chan,uint8_t *u8_char,uint16_t len);

#endif



