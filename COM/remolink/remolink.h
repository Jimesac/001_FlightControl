#ifndef __REMO_MA_H__
#define __REMO_MA_H__

#include "hpm_soc.h"
#include "checkcrc.h"
//#include "remolink_msg_payload.h"
//#include "remolink_msg_payload.h"



#define GIMBAL  //云台
//#define CAMERA  //眼镜


#define REMOLINK_HEAD_LEN           1
#define REMOLINK_HEADOTHER_LEN      9
#define REMOLINK_MAX_PAYLOAD_LEN    300
#define REMOLINK_CHECKSUN_LEN       2

#define REMOLINK_HEAD_ALL_LEN       (REMOLINK_HEAD_LEN+REMOLINK_HEADOTHER_LEN)  //10
#define REMOLINK_NO_PAYLOAD_LEN     (REMOLINK_HEAD_LEN+REMOLINK_HEADOTHER_LEN+REMOLINK_CHECKSUN_LEN)    //12
#define REMOLINK_PACK_LEN           (REMOLINK_NO_PAYLOAD_LEN+REMOLINK_MAX_PAYLOAD_LEN)

/***/
#define REMOLINK_HEAD   0xAA
#define GIMBAL_COMPID   0x01
#define CAMERA_COMPID   0x02
#define FLIGHTC_COMPID  0x03

#ifdef GIMBAL
#define MY_COMPID   GIMBAL_COMPID
#else   
#define MY_COMPID   CAMERA_COMPID
#endif

/**/



#define MOVE_PAYLOAD(msg) ((const char *)(&((msg)->payload8[0])))          //返回消息 msg 的有效载荷部分的常量和非常量指针。
#define MOVE_PAYLOAD_NON_CONST(msg) ((char *)(&((msg)->payload8[0])))
#ifdef __GNUC__
  #define MAVPACKED( __Declaration__ ) __Declaration__ __attribute__((packed))
#else
  #define MAVPACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )    //对结构体或联合体进行内存对齐设置
#endif


typedef enum 
{
    REMOLINK_RECEIVE_STATE_UNINIT=0,
    REMOLINK_RECEIVE_STATE_HEAD,
    REMOLINK_RECEIVE_STATE_SENDER,
    REMOLINK_RECEIVE_STATE_RECEIVER,
    REMOLINK_RECEIVE_STATE_MSGID,
    REMOLINK_RECEIVE_STATE_LEN1,
    REMOLINK_RECEIVE_STATE_LEN2,
    REMOLINK_RECEIVE_STATE_SEQ,
    REMOLINK_RECEIVE_STATE_RESERVE1,     //保留
    REMOLINK_RECEIVE_STATE_RESERVE2,
    REMOLINK_RECEIVE_STATE_PAYLOAD,
    REMOLINK_RECEIVE_STATE_CRC1,
    REMOLINK_RECEIVE_STATE_CRC2
}remolink_receive_state_t;

typedef enum 
{
    REMOLINK_RE_INCOMPLETE=0,       //没有解析完
    REMOLINK_RE_OK=1,
    REMOLINK_RE_BAD_CRC=2
}remolink_framing_t;

typedef enum 
{
    REMOLINK_COMMAND_RECEIVE=0,
    REMOLINK_COMMAND_SEND=1
}remolink_commamd_t;

MAVPACKED(
typedef struct __remolink_message
{
    uint16_t    checksum;
    uint8_t     head;
    uint8_t     sender;
    uint8_t     receiver;
    uint8_t     msgid;
    uint16_t    len;
    uint8_t     seq;
    uint8_t     reserve[2];
    uint8_t     command;
    uint8_t     payload8[REMOLINK_MAX_PAYLOAD_LEN];
    uint8_t     sendchecksum[2];
})remolink_message_t;

MAVPACKED(
typedef struct __remolink_state
{
    uint8_t     tx_seq;
    uint8_t     rx_seq;
    uint16_t    packet_idx;
    uint8_t     received_result;
    uint8_t     rx_success_count;
    uint8_t     rx_fail_count;
//    uint8_t     rx_error_count;
    remolink_receive_state_t  receive_state;
})remolink_state_t;



/*发送相关打包*/

uint16_t remolink_pack_to_message_s(remolink_message_t *msg , uint8_t insender , uint8_t inreceiver, uint8_t msg_id, 
                                                uint16_t length ,remolink_state_t *state);
uint16_t remolink_message_to_send_buffer_s(uint8_t *buffer, const remolink_message_t *msg);



uint8_t remolink_receive_to_messige_chan_r(uint8_t chan, uint8_t Rx_data, remolink_message_t* return_rx_msg, remolink_state_t* return_state);

remolink_state_t* remolink_get_chan_state(uint8_t chan);
remolink_message_t* remolink_get_chan_message(uint8_t chan);
uint8_t* remolink_get_chan_send_buffer(uint8_t chan);


#endif


