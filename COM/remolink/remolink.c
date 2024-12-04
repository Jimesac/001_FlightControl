#include "remolink.h"



remolink_state_t    m_remolink_state[10];
remolink_message_t  m_remolink_message[10];
uint8_t             m_remolink_send_buffer[10][REMOLINK_PACK_LEN];


static inline uint8_t remolink_receive_to_messige_r( remolink_message_t* rx_msg,
                                                    remolink_state_t* state,
                                                    uint8_t Rx_data,
                                                    remolink_message_t* return_rx_msg,
                                                    remolink_state_t* return_state);






/*****************************send *****************************/


uint16_t remolink_pack_to_message_s(remolink_message_t *msg , uint8_t insender , uint8_t inreceiver, uint8_t msg_id, 
                                                uint16_t length ,remolink_state_t *state)
{
    msg->head   = REMOLINK_HEAD;
    msg->sender = insender;
    msg->receiver= inreceiver;
    msg->msgid  = msg_id;
    msg->len    = length;
    msg->seq    = state->tx_seq;
    msg->reserve[0]=0; 
    msg->reserve[1]=0;
    state->tx_seq       = (state->tx_seq+1)%256;
    msg->checksum       = crc16_ccitt( ((const uint8_t*)(msg)) + 3 ,REMOLINK_HEADOTHER_LEN + length );
    msg->sendchecksum[0]= msg->checksum & 0xFF; //低八位
    msg->sendchecksum[1]= (msg->checksum>>8) & 0xFF; //高八位
    return length + REMOLINK_NO_PAYLOAD_LEN;
}

uint16_t remolink_message_to_send_buffer_s(uint8_t *buffer, const remolink_message_t *msg)
{
    memcpy(buffer, (const uint8_t *)&msg->head, REMOLINK_HEAD_ALL_LEN + (uint16_t)msg->len );
    uint8_t* ck = buffer + (REMOLINK_HEAD_ALL_LEN + (uint16_t)msg->len);
    ck[0] = msg->sendchecksum[0];
    ck[1] = msg->sendchecksum[1];

    return (uint16_t)msg->len + REMOLINK_NO_PAYLOAD_LEN;
}





/**************************receive*****************/

static inline uint8_t remolink_receive_to_messige_r( remolink_message_t* rx_msg,
                                                    remolink_state_t* state,
                                                    uint8_t Rx_data,
                                                    remolink_message_t* return_rx_msg,
                                                    remolink_state_t* return_state)
{
    static uint8_t last_redata=0, current_redata=0;
    last_redata    = current_redata;
    current_redata = Rx_data;

    if( (current_redata==GIMBAL_COMPID || current_redata== CAMERA_COMPID) && last_redata==REMOLINK_HEAD )
    {
           if( state->receive_state > REMOLINK_RECEIVE_STATE_SENDER)   //后面数据出错
           {
                state->receive_state=REMOLINK_RECEIVE_STATE_RECEIVER;
                state->received_result=REMOLINK_RE_INCOMPLETE;
                rx_msg->len=0;
                rx_msg->head=REMOLINK_HEAD;
                rx_msg->sender=Rx_data;
                state->rx_fail_count++;
           }
    }
 

    switch (state->receive_state)
    {
    case REMOLINK_RECEIVE_STATE_UNINIT:         //空闲状态
         state->received_result=REMOLINK_RE_INCOMPLETE;

    case REMOLINK_RECEIVE_STATE_HEAD:          //处理帧头
         if(Rx_data==REMOLINK_HEAD)
         {
            state->receive_state=REMOLINK_RECEIVE_STATE_SENDER;
            rx_msg->len=0;
            rx_msg->head=Rx_data;
         }
        break;
    case REMOLINK_RECEIVE_STATE_SENDER:
         state->receive_state=REMOLINK_RECEIVE_STATE_RECEIVER;
         rx_msg->sender=Rx_data;
         break;
    case REMOLINK_RECEIVE_STATE_RECEIVER:
         state->receive_state=REMOLINK_RECEIVE_STATE_MSGID;
         rx_msg->receiver=Rx_data;                //
         break;
    case REMOLINK_RECEIVE_STATE_MSGID:
         state->receive_state=REMOLINK_RECEIVE_STATE_LEN1;
         rx_msg->msgid=Rx_data;
         break;
    case REMOLINK_RECEIVE_STATE_LEN1:
         state->receive_state=REMOLINK_RECEIVE_STATE_LEN2;
         state->packet_idx=0;
         rx_msg->len=Rx_data&0xFF;
         break;
    case REMOLINK_RECEIVE_STATE_LEN2:
         state->receive_state=REMOLINK_RECEIVE_STATE_SEQ;
         rx_msg->len=(Rx_data<<8) | rx_msg->len;
    case REMOLINK_RECEIVE_STATE_SEQ:
         state->receive_state=REMOLINK_RECEIVE_STATE_RESERVE1;
         rx_msg->seq=Rx_data;
         break;
    case REMOLINK_RECEIVE_STATE_RESERVE1:
         state->receive_state=REMOLINK_RECEIVE_STATE_RESERVE2;
    case REMOLINK_RECEIVE_STATE_RESERVE2:
         state->receive_state=REMOLINK_RECEIVE_STATE_PAYLOAD;
    case REMOLINK_RECEIVE_STATE_PAYLOAD:
         rx_msg->payload8[state->packet_idx++]=Rx_data;
         if(state->packet_idx==rx_msg->len)
         {
            state->receive_state=REMOLINK_RECEIVE_STATE_CRC1;
         }
         break;
    case REMOLINK_RECEIVE_STATE_CRC1:
         state->receive_state=REMOLINK_RECEIVE_STATE_CRC2;
         rx_msg->sendchecksum[0]=Rx_data;
         break;
    case REMOLINK_RECEIVE_STATE_CRC2:
         rx_msg->sendchecksum[1]=Rx_data;
         rx_msg->checksum = crc16_ccitt( ((const uint8_t*)(rx_msg)) + 3 ,REMOLINK_HEADOTHER_LEN + rx_msg->len );
         if(  rx_msg->checksum == ((rx_msg->sendchecksum[1]<<8) | (rx_msg->sendchecksum[0]))  )   //正确
         {
            state->received_result=REMOLINK_RE_OK;

         }
         else                       //错误
         {
            state->received_result=REMOLINK_RE_BAD_CRC;
         }
         state->receive_state=REMOLINK_RECEIVE_STATE_UNINIT;
    default:
        break;
    }


    if(state->received_result==REMOLINK_RE_OK)      //接收正确
    {
        state->rx_seq = rx_msg->seq;
        state->rx_success_count++;
        memcpy(return_rx_msg, rx_msg,sizeof(remolink_message_t));
        memcpy(return_state, state,sizeof(remolink_state_t));

    }
    else if(state->received_result==REMOLINK_RE_BAD_CRC)    //错误
    {
        state->rx_fail_count++;
        memcpy(return_rx_msg, rx_msg,sizeof(remolink_message_t));
        memcpy(return_state, state,sizeof(remolink_state_t));
    }
    return state->received_result;

}



uint8_t remolink_receive_to_messige_chan_r(uint8_t chan, uint8_t Rx_data, remolink_message_t* return_rx_msg, remolink_state_t* return_state)
{
    return remolink_receive_to_messige_r( &m_remolink_message[chan], &m_remolink_state[chan],Rx_data, return_rx_msg, return_state);
}


remolink_state_t* remolink_get_chan_state(uint8_t chan)
{
    return &m_remolink_state[chan];
}
remolink_message_t* remolink_get_chan_message(uint8_t chan)
{
    return &m_remolink_message[chan];
}
uint8_t* remolink_get_chan_send_buffer(uint8_t chan)
{
    return m_remolink_send_buffer[chan];
}



