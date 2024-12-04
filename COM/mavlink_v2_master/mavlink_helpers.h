#pragma once

#include "string.h"
#include "checksum.h"
#include "mavlink_types.h"
#include "mavlink_conversions.h"
#include <stdio.h>


////引入
//#include "common/mavlink.h"
//#define MAVLINK_STX 253
//#define MAVLINK_MESSAGE_CRCS {{0, 50, 9, 9, 0, 0, 0}, {1, 124, 31, 43, 0, 0, 0}, {2, 137, 12, 12, 0, 0, 0}, {4, 237, 14, 14, 3, 12, 13}, {5, 217, 28, 28, 1, 0, 0}, {6, 104, 3, 3, 0, 0, 0}, {7, 119, 32, 32, 0, 0, 0}, {8, 117, 36, 36, 0, 0, 0}, {11, 89, 6, 6, 1, 4, 0}, {20, 214, 20, 20, 3, 2, 3}, {21, 159, 2, 2, 3, 0, 1}, {22, 220, 25, 25, 0, 0, 0}, {23, 168, 23, 23, 3, 4, 5}, {24, 24, 30, 52, 0, 0, 0}, {25, 23, 101, 101, 0, 0, 0}, {26, 170, 22, 24, 0, 0, 0}, {27, 144, 26, 29, 0, 0, 0}, {28, 67, 16, 16, 0, 0, 0}, {29, 115, 14, 16, 0, 0, 0}, {30, 39, 28, 28, 0, 0, 0}, {31, 246, 32, 48, 0, 0, 0}, {32, 185, 28, 28, 0, 0, 0}, {33, 104, 28, 28, 0, 0, 0}, {34, 237, 22, 22, 0, 0, 0}, {35, 244, 22, 22, 0, 0, 0}, {36, 222, 21, 37, 0, 0, 0}, {37, 212, 6, 7, 3, 4, 5}, {38, 9, 6, 7, 3, 4, 5}, {39, 254, 37, 38, 3, 32, 33}, {40, 230, 4, 5, 3, 2, 3}, {41, 28, 4, 4, 3, 2, 3}, {42, 28, 2, 18, 0, 0, 0}, {43, 132, 2, 3, 3, 0, 1}, {44, 221, 4, 9, 3, 2, 3}, {45, 232, 2, 3, 3, 0, 1}, {46, 11, 2, 2, 0, 0, 0}, {47, 153, 3, 8, 3, 0, 1}, {48, 41, 13, 21, 1, 12, 0}, {49, 39, 12, 20, 0, 0, 0}, {50, 78, 37, 37, 3, 18, 19}, {51, 196, 4, 5, 3, 2, 3}, {54, 15, 27, 27, 3, 24, 25}, {55, 3, 25, 25, 0, 0, 0}, {61, 167, 72, 72, 0, 0, 0}, {62, 183, 26, 26, 0, 0, 0}, {63, 119, 181, 181, 0, 0, 0}, {64, 191, 225, 225, 0, 0, 0}, {65, 118, 42, 42, 0, 0, 0}, {66, 148, 6, 6, 3, 2, 3}, {67, 21, 4, 4, 0, 0, 0}, {69, 243, 11, 30, 1, 10, 0}, {70, 124, 18, 38, 3, 16, 17}, {73, 38, 37, 38, 3, 32, 33}, {74, 20, 20, 20, 0, 0, 0}, {75, 158, 35, 35, 3, 30, 31}, {76, 152, 33, 33, 3, 30, 31}, {77, 143, 3, 10, 3, 8, 9}, {80, 14, 4, 4, 3, 2, 3}, {81, 106, 22, 22, 0, 0, 0}, {82, 49, 39, 51, 3, 36, 37}, {83, 22, 37, 37, 0, 0, 0}, {84, 143, 53, 53, 3, 50, 51}, {85, 140, 51, 51, 0, 0, 0}, {86, 5, 53, 53, 3, 50, 51}, {87, 150, 51, 51, 0, 0, 0}, {89, 231, 28, 28, 0, 0, 0}, {90, 183, 56, 56, 0, 0, 0}, {91, 63, 42, 42, 0, 0, 0}, {92, 54, 33, 33, 0, 0, 0}, {93, 47, 81, 81, 0, 0, 0}, {100, 175, 26, 34, 0, 0, 0}, {101, 102, 32, 117, 0, 0, 0}, {102, 158, 32, 117, 0, 0, 0}, {103, 208, 20, 57, 0, 0, 0}, {104, 56, 32, 116, 0, 0, 0}, {105, 93, 62, 63, 0, 0, 0}, {106, 138, 44, 44, 0, 0, 0}, {107, 108, 64, 65, 0, 0, 0}, {108, 32, 84, 92, 0, 0, 0}, {109, 185, 9, 9, 0, 0, 0}, {110, 84, 254, 254, 3, 1, 2}, {111, 34, 16, 18, 3, 16, 17}, {112, 174, 12, 12, 0, 0, 0}, {113, 124, 36, 39, 0, 0, 0}, {114, 237, 44, 44, 0, 0, 0}, {115, 4, 64, 64, 0, 0, 0}, {116, 76, 22, 24, 0, 0, 0}, {117, 128, 6, 6, 3, 4, 5}, {118, 56, 14, 14, 0, 0, 0}, {119, 116, 12, 12, 3, 10, 11}, {120, 134, 97, 97, 0, 0, 0}, {121, 237, 2, 2, 3, 0, 1}, {122, 203, 2, 2, 3, 0, 1}, {123, 250, 113, 113, 3, 0, 1}, {124, 87, 35, 57, 0, 0, 0}, {125, 203, 6, 6, 0, 0, 0}, {126, 220, 79, 81, 3, 79, 80}, {127, 25, 35, 35, 0, 0, 0}, {128, 226, 35, 35, 0, 0, 0}, {129, 46, 22, 24, 0, 0, 0}, {130, 29, 13, 13, 0, 0, 0}, {131, 223, 255, 255, 0, 0, 0}, {132, 85, 14, 39, 0, 0, 0}, {133, 6, 18, 18, 0, 0, 0}, {134, 229, 43, 43, 0, 0, 0}, {135, 203, 8, 8, 0, 0, 0}, {136, 1, 22, 22, 0, 0, 0}, {137, 195, 14, 16, 0, 0, 0}, {138, 109, 36, 120, 0, 0, 0}, {139, 168, 43, 43, 3, 41, 42}, {140, 181, 41, 41, 0, 0, 0}, {141, 47, 32, 32, 0, 0, 0}, {142, 72, 243, 243, 0, 0, 0}, {143, 131, 14, 16, 0, 0, 0}, {144, 127, 93, 93, 0, 0, 0}, {146, 103, 100, 100, 0, 0, 0}, {147, 154, 36, 54, 0, 0, 0}, {148, 178, 60, 78, 0, 0, 0}, {149, 200, 30, 60, 0, 0, 0}, {162, 189, 8, 9, 0, 0, 0}, {192, 36, 44, 54, 0, 0, 0}, {225, 208, 65, 73, 0, 0, 0}, {230, 163, 42, 42, 0, 0, 0}, {231, 105, 40, 40, 0, 0, 0}, {232, 151, 63, 65, 0, 0, 0}, {233, 35, 182, 182, 0, 0, 0}, {234, 150, 40, 40, 0, 0, 0}, {235, 179, 42, 42, 0, 0, 0}, {241, 90, 32, 32, 0, 0, 0}, {242, 104, 52, 60, 0, 0, 0}, {243, 85, 53, 61, 1, 52, 0}, {244, 95, 6, 6, 0, 0, 0}, {245, 130, 2, 2, 0, 0, 0}, {246, 184, 38, 38, 0, 0, 0}, {247, 81, 19, 19, 0, 0, 0}, {248, 8, 254, 254, 3, 3, 4}, {249, 204, 36, 36, 0, 0, 0}, {250, 49, 30, 30, 0, 0, 0}, {251, 170, 18, 18, 0, 0, 0}, {252, 44, 18, 18, 0, 0, 0}, {253, 83, 51, 54, 0, 0, 0}, {254, 46, 9, 9, 0, 0, 0}, {256, 71, 42, 42, 3, 8, 9}, {257, 131, 9, 9, 0, 0, 0}, {258, 187, 32, 232, 3, 0, 1}, {259, 92, 235, 237, 0, 0, 0}, {260, 146, 5, 14, 0, 0, 0}, {261, 179, 27, 61, 0, 0, 0}, {262, 12, 18, 23, 0, 0, 0}, {263, 133, 255, 255, 0, 0, 0}, {264, 49, 28, 32, 0, 0, 0}, {265, 26, 16, 20, 0, 0, 0}, {266, 193, 255, 255, 3, 2, 3}, {267, 35, 255, 255, 3, 2, 3}, {268, 14, 4, 4, 3, 2, 3}, {269, 109, 213, 215, 0, 0, 0}, {270, 59, 19, 20, 0, 0, 0}, {271, 22, 52, 53, 0, 0, 0}, {275, 126, 31, 32, 0, 0, 0}, {276, 18, 49, 50, 0, 0, 0}, {277, 62, 30, 30, 0, 0, 0}, {280, 70, 33, 33, 0, 0, 0}, {281, 48, 13, 13, 0, 0, 0}, {282, 123, 35, 35, 3, 32, 33}, {283, 74, 144, 145, 0, 0, 0}, {284, 99, 32, 32, 3, 30, 31}, {285, 137, 40, 49, 3, 38, 39}, {286, 210, 53, 57, 3, 50, 51}, {287, 1, 23, 23, 3, 20, 21}, {288, 20, 23, 23, 3, 20, 21}, {290, 251, 46, 46, 0, 0, 0}, {291, 10, 57, 57, 0, 0, 0}, {299, 19, 96, 98, 0, 0, 0}, {300, 217, 22, 22, 0, 0, 0}, {301, 243, 58, 58, 0, 0, 0}, {310, 28, 17, 17, 0, 0, 0}, {311, 95, 116, 116, 0, 0, 0}, {320, 243, 20, 20, 3, 2, 3}, {321, 88, 2, 2, 3, 0, 1}, {322, 243, 149, 149, 0, 0, 0}, {323, 78, 147, 147, 3, 0, 1}, {324, 132, 146, 146, 0, 0, 0}, {330, 23, 158, 167, 0, 0, 0}, {331, 91, 230, 233, 0, 0, 0}, {332, 236, 239, 239, 0, 0, 0}, {333, 231, 109, 109, 0, 0, 0}, {334, 72, 10, 10, 0, 0, 0}, {335, 225, 24, 24, 0, 0, 0}, {336, 245, 84, 84, 0, 0, 0}, {339, 199, 5, 5, 0, 0, 0}, {340, 99, 70, 70, 0, 0, 0}, {350, 232, 20, 252, 0, 0, 0}, {360, 11, 25, 25, 0, 0, 0}, {370, 26, 140, 140, 0, 0, 0}, {373, 117, 42, 42, 0, 0, 0}, {375, 251, 140, 140, 0, 0, 0}, {380, 232, 20, 20, 0, 0, 0}, {385, 147, 133, 133, 3, 2, 3}, {386, 132, 16, 16, 3, 4, 5}, {387, 4, 72, 72, 3, 4, 5}, {388, 8, 37, 37, 3, 32, 33}, {390, 156, 238, 238, 0, 0, 0}, {395, 0, 212, 212, 0, 0, 0}, {396, 50, 160, 160, 0, 0, 0}, {397, 182, 108, 108, 0, 0, 0}, {400, 110, 254, 254, 3, 4, 5}, {401, 183, 6, 6, 3, 4, 5}, {410, 160, 53, 53, 0, 0, 0}, {411, 106, 3, 3, 0, 0, 0}, {412, 33, 6, 6, 3, 4, 5}, {413, 77, 7, 7, 3, 4, 5}, {440, 66, 35, 35, 0, 0, 0}, {9000, 113, 137, 137, 0, 0, 0}, {9005, 117, 34, 34, 0, 0, 0}, {12900, 114, 44, 44, 3, 0, 1}, {12901, 254, 59, 59, 3, 30, 31}, {12902, 140, 53, 53, 3, 4, 5}, {12903, 249, 46, 46, 3, 0, 1}, {12904, 77, 54, 54, 3, 28, 29}, {12905, 49, 43, 43, 3, 0, 1}, {12915, 94, 249, 249, 3, 0, 1}, {12918, 139, 51, 51, 0, 0, 0}, {12919, 7, 18, 18, 3, 16, 17}, {12920, 20, 5, 5, 0, 0, 0}}


#ifndef MAVLINK_HELPER
#define MAVLINK_HELPER
#endif

#include "mavlink_sha256.h"

#ifdef MAVLINK_USE_CXX_NAMESPACE
namespace mavlink {
#endif

/*
 * Internal function to give access to the channel status for each channel			//用于访问每个通道的通道状态的内部函数
 */
#ifndef MAVLINK_GET_CHANNEL_STATUS
MAVLINK_HELPER mavlink_status_t* mavlink_get_channel_status(uint8_t chan)
{
#ifdef MAVLINK_EXTERNAL_RX_STATUS
	// No m_mavlink_status array defined in function,
	// has to be defined externally
#else
	static mavlink_status_t m_mavlink_status[MAVLINK_COMM_NUM_BUFFERS];		//16
#endif
	return &m_mavlink_status[chan];
}
#endif

/*
 * Internal function to give access to the channel buffer for each channel			//用于访问每个通道的通道缓冲区的内部函数
 */
#ifndef MAVLINK_GET_CHANNEL_BUFFER
MAVLINK_HELPER mavlink_message_t* mavlink_get_channel_buffer(uint8_t chan)
{
	
#ifdef MAVLINK_EXTERNAL_RX_BUFFER
	// No m_mavlink_buffer array defined in function,
	// has to be defined externally
#else
	static mavlink_message_t m_mavlink_buffer[MAVLINK_COMM_NUM_BUFFERS];
#endif
	return &m_mavlink_buffer[chan];
}
#endif // MAVLINK_GET_CHANNEL_BUFFER

/* Enable this option to check the length of each message.
    This allows invalid messages to be caught much sooner. Use if the transmission
    medium is prone to missing (or extra) characters (e.g. a radio that fades in
    and out). Only use if the channel will only contain messages types listed in
    the headers.
	启用此选项可检查每封邮件的长度。这允许更快地捕获无效消息。如果传输媒体易于丢失(或额外)
	字符(例如，收音机渐强渐弱)，则使用此选项。仅当通道仅包含标头中列出的消息类型时使用。
*/
//#define MAVLINK_CHECK_MESSAGE_LENGTH

/**
 * @brief Reset the status of a channel.					//重置通道的状态。
 */
MAVLINK_HELPER void mavlink_reset_channel_status(uint8_t chan)
{
	mavlink_status_t *status = mavlink_get_channel_status(chan);
	status->parse_state = MAVLINK_PARSE_STATE_IDLE;					//空闲状态，准备接收新消息
}

#ifndef MAVLINK_NO_SIGN_PACKET
/**
 * @brief create a signature block for a packet		//为数据包创建签名块
 */
MAVLINK_HELPER uint8_t mavlink_sign_packet(mavlink_signing_t *signing,		//包含签名所需的状态信息（如密钥、时间戳、标志等）
					   uint8_t signature[MAVLINK_SIGNATURE_BLOCK_LEN],		//一个数组，用于保存生成的签名块	MAVLINK_SIGNATURE_BLOCK_LEN=13
					   const uint8_t *header, uint8_t header_len,			//指向数据包头的指针。  		数据包头的长度。
					   const uint8_t *packet, uint8_t packet_len,			//指向数据包数据部分的指针。	数据包数据部分的长度
					   const uint8_t crc[2])								//一个包含 2 字节 CRC 校验值的数组
{
	mavlink_sha256_ctx ctx;		//用于 SHA-256 哈希计算的上下文结构体
	union {
	    uint64_t t64;			//存储 64 位的时间戳
	    uint8_t t8[8];			//其对应的字节数组形式，用于存储在签名中
	} tstamp;			//存储时间戳
	if (signing == NULL || !(signing->flags & MAVLINK_SIGNING_FLAG_SIGN_OUTGOING)) {
	    return 0;
	}
	signature[0] = signing->link_id;		//设置签名块的第一个字节为 link_id，该值标识通信链路	
	tstamp.t64 = signing->timestamp;		//将当前的时间戳存储到 tstamp.t64
	memcpy(&signature[1], tstamp.t8, 6);	//将时间戳的前 6 个字节复制到签名块的 signature[1]-[6] 中
	signing->timestamp++;					//递增时间戳，为下一次签名准备一个新的时间戳
	
	mavlink_sha256_init(&ctx);				//初始化哈希上下文
	mavlink_sha256_update(&ctx, signing->secret_key, sizeof(signing->secret_key));	//将私密密钥添加到哈希计算中
	mavlink_sha256_update(&ctx, header, header_len);		//将消息头添加到哈希计算中
	mavlink_sha256_update(&ctx, packet, packet_len);		//将数据包内容添加到哈希计算中
	mavlink_sha256_update(&ctx, crc, 2);					//将 CRC 校验值添加到哈希计算中
	mavlink_sha256_update(&ctx, signature, 7);				//将当前生成的部分签名（包括链路 ID 和时间戳）添加到哈希计算中。
	mavlink_sha256_final_48(&ctx, &signature[7]);			//最终生成 48 位的 SHA-256 签名，并将其存储到签名块的后续部分（从 signature[7] 开始）
	
	return MAVLINK_SIGNATURE_BLOCK_LEN;
}
#endif

/**
 * @brief Trim payload of any trailing zero-populated bytes (MAVLink 2 only).	//修剪任何尾随零填充字节的有效负载(仅限MAVLink 2)。
 *
 * @param payload Serialised payload buffer.
 * @param length Length of full-width payload buffer.
 * @return Length of payload after zero-filled bytes are trimmed.
 * 找到payload数据末尾不是0的地址
 */
MAVLINK_HELPER uint8_t _mav_trim_payload(const char *payload, uint8_t length)
{
	while (length > 1 && payload[length-1] == 0) {
		length--;
	}
	return length;
}

#ifndef MAVLINK_NO_SIGNATURE_CHECK
/**
 * @brief check a signature block for a packet		检查数据包的签名块，验证传入的 MAVLink 消息的签名  !!!!!!!!!!
 */
MAVLINK_HELPER bool mavlink_signature_check(mavlink_signing_t *signing,		//指向 mavlink_signing_t 结构体的指针，包含签名状态（如密钥、时间戳等）
					    mavlink_signing_streams_t *signing_streams,			//指向 mavlink_signing_streams_t 结构体的指针，跟踪活动的签名流和它们的时间戳
					    const mavlink_message_t *msg)						//指向传入的 MAVLink 消息的指针，需要进行签名验证
{
	if (signing == NULL) {
		return true;
	}
        const uint8_t *p = (const uint8_t *)&msg->magic;					//指向msg的头消息当中的magic
	const uint8_t *psig = msg->signature;									//指向消息中的签名字段
        const uint8_t *incoming_signature = psig+7;						//signature[7]开始的后8位存储着 最终生成 48 位的 SHA-256 签名
	mavlink_sha256_ctx ctx;						//
	uint8_t signature[6];						//签名字段
	uint16_t i;
        
	mavlink_sha256_init(&ctx);					//初始化一个 SHA-256 上下文（ctx），用于计算哈希值
	mavlink_sha256_update(&ctx, signing->secret_key, sizeof(signing->secret_key));	//加入签名使用的 密钥 进行哈希计算
	mavlink_sha256_update(&ctx, p, MAVLINK_NUM_HEADER_BYTES);						//消息头部 10个字节
	mavlink_sha256_update(&ctx, _MAV_PAYLOAD(msg), msg->len);						//消息载荷
	mavlink_sha256_update(&ctx, msg->ck, 2);										//校验和
	mavlink_sha256_update(&ctx, psig, 1+6);											//签名的前 7 个字节
	mavlink_sha256_final_48(&ctx, signature);										//计算得到的哈希存储在 signature 中
        if (memcmp(signature, incoming_signature, 6) != 0) {					//比较计算出的签名和传入的签名sha256校验，签名的后6位
                signing->last_status = MAVLINK_SIGNING_STATUS_BAD_SIGNATURE;	//记录验签结果，签名验证失败，签名无效
		return false;
	}

	// now check timestamp
	union tstamp {
	    uint64_t t64;
	    uint8_t t8[8];
	} tstamp;							//使用联合体 tstamp 存储时间戳，支持 64 位和字节数组两种形式
	uint8_t link_id = psig[0];			//并提取 link_id 作为签名的第一个字节
	tstamp.t64 = 0;
	memcpy(tstamp.t8, psig+1, 6);		//从签名中提取时间戳（psig[1] 到 psig[6]）

	if (signing_streams == NULL) {		//没有可用的签名流
                signing->last_status = MAVLINK_SIGNING_STATUS_NO_STREAMS;		//记录验签结果，签名验证失败，没有可用的消息流记录
                return false;
	}
	
	// find stream
	for (i=0; i<signing_streams->num_signing_streams; i++) {					//遍历 signing_streams 中的所有签名流
		if (msg->sysid == signing_streams->stream[i].sysid &&					//消息的系统 ID 必须与流中的 sysid 匹配
		    msg->compid == signing_streams->stream[i].compid &&					//组件 ID 必须与流中的 compid
		    link_id == signing_streams->stream[i].link_id) {					//链接 ID 必须与流中的 link_id 匹配
			break;
		}
	}
	if (i == signing_streams->num_signing_streams) {									//现有签名流中未找到匹配的流，需要添加一个新的流
		if (signing_streams->num_signing_streams >= MAVLINK_MAX_SIGNING_STREAMS) {		//如果当前签名流数量超过 MAVLINK_MAX_SIGNING_STREAMS 的最大限制
			// over max number of streams
                        signing->last_status = MAVLINK_SIGNING_STATUS_TOO_MANY_STREAMS;	//标志 消息流记录过多，可能存在资源限制
                        return false;
		}
		// new stream. Only accept if timestamp is not more than 1 minute old			//如果新流的时间戳比当前签名的 timestamp 落后超过1分钟(6秒？)
		if (tstamp.t64 + 6000*1000UL < signing->timestamp) {							
                        signing->last_status = MAVLINK_SIGNING_STATUS_OLD_TIMESTAMP;	//标志 消息的时间戳过旧
                        return false;
		}
		// add new stream	//如果时间戳有效，将消息的 sysid、compid 和 link_id 添加到新的签名流中，并增加 num_signing_streams 以记录流的数量
		signing_streams->stream[i].sysid = msg->sysid;
		signing_streams->stream[i].compid = msg->compid;
		signing_streams->stream[i].link_id = link_id;
		signing_streams->num_signing_streams++;
	} else {										//如果找到匹配的流
		union tstamp last_tstamp;						//创建 last_tstamp 来存储签名流中的上一个时间戳
		last_tstamp.t64 = 0;
		memcpy(last_tstamp.t8, signing_streams->stream[i].timestamp_bytes, 6);		//通过 memcpy 从流中复制 6 字节的时间戳到 last_tstamp
		if (tstamp.t64 <= last_tstamp.t64) {										//如果新时间戳小于或等于上一个时间戳
			// repeating old timestamp
                        signing->last_status = MAVLINK_SIGNING_STATUS_REPLAY;		//测到重放攻击，即相同消息被重复发送
                        return false;
		}
	}

	// remember last timestamp
	memcpy(signing_streams->stream[i].timestamp_bytes, psig+1, 6);					//将当前签名中的时间戳（位于 psig+1）复制到签名流的 timestamp_bytes，更新记录

	// our next timestamp must be at least this timestamp
	if (tstamp.t64 > signing->timestamp) {			//确保签名的 timestamp 不会小于流中的时间戳
		signing->timestamp = tstamp.t64;			//更新 signing 的 timestamp，以确保在当前时间戳之后生成的消息不会低于此时间戳
	}
        signing->last_status = MAVLINK_SIGNING_STATUS_OK;	//如果所有检查都通过，设置签名状态为 OK 表示验证成功
        return true;
}
#endif


/**
 * @brief Finalize a MAVLink message with channel assignment		//使用通道分配完成MAVLink消息
 *
 * This function calculates the checksum and sets length and aircraft id correctly.
 * It assumes that the message id and the payload are already correctly set. This function
 * can also be used if the message header has already been written before (as in mavlink_msg_xxx_pack
 * instead of mavlink_msg_xxx_pack_headerless), it just introduces little extra overhead.
 * 该函数计算校验和，并正确设置长度和飞机id。它假设已经正确设置了消息id和有效负载。如果之前已经编写了消息头，
 * 也可以使用该函数(如在mavlink_msg_xxx_pack中，而不是在MAV link _ msg _ XXX _ pack _ header less中)，它只是引入了很少的额外开销。
 *
 * @param msg Message to finalize
 * @param system_id Id of the sending (this) system, 1-127
 * @param length Message length
 * msg：			待封装的消息。
 * system_id：		系统 ID。
 * component_id：	组件 ID。
 * status：			通信状态，包含标志位和签名信息。
 * min_length：		最小消息长度。
 * length：			消息内容长度。
 * crc_extra：		用于校验的额外字节
 */
MAVLINK_HELPER uint16_t mavlink_finalize_message_buffer(mavlink_message_t* msg, uint8_t system_id, uint8_t component_id,
						      mavlink_status_t* status, uint8_t min_length, uint8_t length, uint8_t crc_extra)
{
	bool mavlink1 = (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) != 0;		//判断是否为 MAVLink 1 协议  默认flags=0,V2
#ifndef MAVLINK_NO_SIGN_PACKET
	bool signing = 	(!mavlink1) && status->signing && (status->signing->flags & MAVLINK_SIGNING_FLAG_SIGN_OUTGOING);	//断是否需要为消息添加签名，仅在 MAVLink 2 中有效。
#else
	bool signing = false;
#endif
	uint8_t signature_len = signing? MAVLINK_SIGNATURE_BLOCK_LEN : 0;			//如果需要签名，签名长度为 13
        uint8_t header_len = MAVLINK_CORE_HEADER_LEN+1;							//初始化为默认的 MAVLink 2 消息头长度9+1(10)
	uint8_t buf[MAVLINK_CORE_HEADER_LEN+1];										//存储消息头
	if (mavlink1) {			//如果是mavlink v1	
		msg->magic = MAVLINK_STX_MAVLINK1;					//帧头 0xFE
		header_len = MAVLINK_CORE_HEADER_MAVLINK1_LEN+1;	//帧头长度 6
	} else {				//如果是mavlink v2	
		msg->magic = MAVLINK_STX;							//帧头 0xFD
	}
	msg->len = mavlink1?min_length:_mav_trim_payload(_MAV_PAYLOAD(msg), length);//如果是mavlink v1，消息长度为min_length，否则v2为指到末尾不为0地方
	msg->sysid = system_id;									//设置系统 ID
	msg->compid = component_id;								//设置组件 ID
	msg->incompat_flags = 0;
	if (signing) {
		msg->incompat_flags |= MAVLINK_IFLAG_SIGNED;		//必须理解的标志设置为0x01，表示消息是已签名的
	}
	msg->compat_flags = 0;									//兼容的标志设为0
	msg->seq = status->current_tx_seq;						//更新消息的序列号 seq
	status->current_tx_seq = status->current_tx_seq + 1;	//通信状态 的发送序列号+1

	// form the header as a byte array for the crc	//将报头形成为crc的字节数组
	buf[0] = msg->magic;
	buf[1] = msg->len;
	if (mavlink1) {
		buf[2] = msg->seq;
		buf[3] = msg->sysid;
		buf[4] = msg->compid;
		buf[5] = msg->msgid & 0xFF;
	} else {
		buf[2] = msg->incompat_flags;						//不兼容标志 0x01(表示用数据签名) 0x00(不用数据签名)
		buf[3] = msg->compat_flags;							//兼容标志   0x00
		buf[4] = msg->seq;
		buf[5] = msg->sysid;
		buf[6] = msg->compid;
		buf[7] = msg->msgid & 0xFF;							//低8位	消息ID
		buf[8] = (msg->msgid >> 8) & 0xFF;					//中8位
		buf[9] = (msg->msgid >> 16) & 0xFF;					//高8位
	}
	
	uint16_t checksum = crc_calculate(&buf[1], header_len-1);		//LEN--MSG 计算校验和
	crc_accumulate_buffer(&checksum, _MAV_PAYLOAD(msg), msg->len);	//计算数据的校验和
	crc_accumulate(crc_extra, &checksum);							//计算额外校验和 存在checksum
	mavlink_ck_a(msg) = (uint8_t)(checksum & 0xFF);					//校验和低字节
	mavlink_ck_b(msg) = (uint8_t)(checksum >> 8);					//校验和高字节

	msg->checksum = checksum;

#ifndef MAVLINK_NO_SIGN_PACKET										//MAVLINK_NO_SIGN_PACKET （不用签名）   //如果用签名
	if (signing) {
		mavlink_sign_packet(status->signing,						//调用 mavlink_sign_packet 为消息添加签名，并将签名写入 msg->signature
				    msg->signature,
				    (const uint8_t *)buf, header_len,
				    (const uint8_t *)_MAV_PAYLOAD(msg), msg->len,
				    (const uint8_t *)_MAV_PAYLOAD(msg)+(uint16_t)msg->len);
	}
#endif

	return msg->len + header_len + 2 + signature_len;		//msg->len  +10+2+13
}



//完成带有通道分配的MAVLink消息
MAVLINK_HELPER uint16_t mavlink_finalize_message_chan(mavlink_message_t* msg, uint8_t system_id, uint8_t component_id,
						      uint8_t chan, uint8_t min_length, uint8_t length, uint8_t crc_extra)
{
	mavlink_status_t *status = mavlink_get_channel_status(chan);
	return mavlink_finalize_message_buffer(msg, system_id, component_id, status, min_length, length, crc_extra);
}



/**
 * @brief Finalize a MAVLink message with MAVLINK_COMM_0 as default channel		//以MAVLINK_COMM_0作为默认通道完成MAVLink消息
 */
MAVLINK_HELPER uint16_t mavlink_finalize_message(mavlink_message_t* msg, uint8_t system_id, uint8_t component_id, 
						 uint8_t min_length, uint8_t length, uint8_t crc_extra)
{
    return mavlink_finalize_message_chan(msg, system_id, component_id, MAVLINK_COMM_0, min_length, length, crc_extra);
}



//记录解析过程中发生错误的次数 +1
static inline void _mav_parse_error(mavlink_status_t *status)
{
    status->parse_error++;
}

#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

MAVLINK_HELPER void _mavlink_send_uart(mavlink_channel_t chan, const char *buf, uint16_t len);

/**
 * @brief Finalize a MAVLink message with channel assignment and send
 */
MAVLINK_HELPER void _mav_finalize_message_chan_send(mavlink_channel_t chan, uint32_t msgid,
                                                    const char *packet, 
						    uint8_t min_length, uint8_t length, uint8_t crc_extra)
{
	uint16_t checksum;
	uint8_t buf[MAVLINK_NUM_HEADER_BYTES];
	uint8_t ck[2];
	mavlink_status_t *status = mavlink_get_channel_status(chan);
        uint8_t header_len = MAVLINK_CORE_HEADER_LEN;
	uint8_t signature_len = 0;
	uint8_t signature[MAVLINK_SIGNATURE_BLOCK_LEN];
	bool mavlink1 = (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) != 0;
	bool signing = 	(!mavlink1) && status->signing && (status->signing->flags & MAVLINK_SIGNING_FLAG_SIGN_OUTGOING);

        if (mavlink1) {
            length = min_length;
            if (msgid > 255) {
                // can't send 16 bit messages
                _mav_parse_error(status);
                return;
            }
            header_len = MAVLINK_CORE_HEADER_MAVLINK1_LEN;
            buf[0] = MAVLINK_STX_MAVLINK1;
            buf[1] = length;
            buf[2] = status->current_tx_seq;
            buf[3] = mavlink_system.sysid;
            buf[4] = mavlink_system.compid;
            buf[5] = msgid & 0xFF;
        } else {
	    uint8_t incompat_flags = 0;
	    if (signing) {
		incompat_flags |= MAVLINK_IFLAG_SIGNED;
	    }
            length = _mav_trim_payload(packet, length);
            buf[0] = MAVLINK_STX;
            buf[1] = length;
            buf[2] = incompat_flags;
            buf[3] = 0; // compat_flags
            buf[4] = status->current_tx_seq;
            buf[5] = mavlink_system.sysid;
            buf[6] = mavlink_system.compid;
            buf[7] = msgid & 0xFF;
            buf[8] = (msgid >> 8) & 0xFF;
            buf[9] = (msgid >> 16) & 0xFF;
        }
	status->current_tx_seq++;
	checksum = crc_calculate((const uint8_t*)&buf[1], header_len);
	crc_accumulate_buffer(&checksum, packet, length);
	crc_accumulate(crc_extra, &checksum);
	ck[0] = (uint8_t)(checksum & 0xFF);
	ck[1] = (uint8_t)(checksum >> 8);

#ifndef MAVLINK_NO_SIGN_PACKET
	if (signing) {
		// possibly add a signature
		signature_len = mavlink_sign_packet(status->signing, signature, buf, header_len+1,
						    (const uint8_t *)packet, length, ck);
	}
#endif

	MAVLINK_START_UART_SEND(chan, header_len + 3 + (uint16_t)length + (uint16_t)signature_len);
	_mavlink_send_uart(chan, (const char *)buf, header_len+1);
	_mavlink_send_uart(chan, packet, length);
	_mavlink_send_uart(chan, (const char *)ck, 2);
	if (signature_len != 0) {
		_mavlink_send_uart(chan, (const char *)signature, signature_len);
	}
	MAVLINK_END_UART_SEND(chan, header_len + 3 + (uint16_t)length + (uint16_t)signature_len);
}

/**
 * @brief re-send a message over a uart channel
 * this is more stack efficient than re-marshalling the message
 * If the message is signed then the original signature is also sent
 */
MAVLINK_HELPER void _mavlink_resend_uart(mavlink_channel_t chan, const mavlink_message_t *msg)
{
	uint8_t ck[2];

	ck[0] = (uint8_t)(msg->checksum & 0xFF);
	ck[1] = (uint8_t)(msg->checksum >> 8);
	// XXX use the right sequence here

        uint8_t header_len;
        uint8_t signature_len;
        
        if (msg->magic == MAVLINK_STX_MAVLINK1) {
            header_len = MAVLINK_CORE_HEADER_MAVLINK1_LEN + 1;
            signature_len = 0;
            MAVLINK_START_UART_SEND(chan, header_len + msg->len + 2 + signature_len);
            // we can't send the structure directly as it has extra mavlink2 elements in it
            uint8_t buf[MAVLINK_CORE_HEADER_MAVLINK1_LEN + 1];
            buf[0] = msg->magic;
            buf[1] = msg->len;
            buf[2] = msg->seq;
            buf[3] = msg->sysid;
            buf[4] = msg->compid;
            buf[5] = msg->msgid & 0xFF;
            _mavlink_send_uart(chan, (const char*)buf, header_len);
        } else {
            header_len = MAVLINK_CORE_HEADER_LEN + 1;
            signature_len = (msg->incompat_flags & MAVLINK_IFLAG_SIGNED)?MAVLINK_SIGNATURE_BLOCK_LEN:0;
            MAVLINK_START_UART_SEND(chan, header_len + msg->len + 2 + signature_len);
            uint8_t buf[MAVLINK_CORE_HEADER_LEN + 1];
            buf[0] = msg->magic;
            buf[1] = msg->len;
            buf[2] = msg->incompat_flags;
            buf[3] = msg->compat_flags;
            buf[4] = msg->seq;
            buf[5] = msg->sysid;
            buf[6] = msg->compid;
            buf[7] = msg->msgid & 0xFF;
            buf[8] = (msg->msgid >> 8) & 0xFF;
            buf[9] = (msg->msgid >> 16) & 0xFF;
            _mavlink_send_uart(chan, (const char *)buf, header_len);
        }
	_mavlink_send_uart(chan, _MAV_PAYLOAD(msg), msg->len);
	_mavlink_send_uart(chan, (const char *)ck, 2);
        if (signature_len != 0) {
	    _mavlink_send_uart(chan, (const char *)msg->signature, MAVLINK_SIGNATURE_BLOCK_LEN);
        }
        MAVLINK_END_UART_SEND(chan, header_len + msg->len + 2 + signature_len);
}
#endif // MAVLINK_USE_CONVENIENCE_FUNCTIONS





/**
 * @brief Pack a message to send it over a serial byte stream		//打包消息，通过串行字节流发送
 */
MAVLINK_HELPER uint16_t mavlink_msg_to_send_buffer(uint8_t *buf, const mavlink_message_t *msg)
{
	uint8_t signature_len, header_len;		//签名长度和头部长度。
	uint8_t *ck;
        uint8_t length = msg->len;
        
	if (msg->magic == MAVLINK_STX_MAVLINK1) {		//MAVLINK V1协议
		signature_len = 0;							
		header_len = MAVLINK_CORE_HEADER_MAVLINK1_LEN;	//5
		buf[0] = msg->magic;
		buf[1] = length;
		buf[2] = msg->seq;
		buf[3] = msg->sysid;
		buf[4] = msg->compid;
		buf[5] = msg->msgid & 0xFF;
		memcpy(&buf[6], _MAV_PAYLOAD(msg), msg->len);	//存储数据 
		ck = buf + header_len + 1 + (uint16_t)msg->len;	//定位到校验和位置上
	} else {
		length = _mav_trim_payload(_MAV_PAYLOAD(msg), length);		//计算实际长度，除去末尾为0的
		header_len = MAVLINK_CORE_HEADER_LEN;						//9
		buf[0] = msg->magic;
		buf[1] = length;
		buf[2] = msg->incompat_flags;								//不兼容标志
		buf[3] = msg->compat_flags;									//兼容标志
		buf[4] = msg->seq;
		buf[5] = msg->sysid;
		buf[6] = msg->compid;
		buf[7] = msg->msgid & 0xFF;									//消息ID
		buf[8] = (msg->msgid >> 8) & 0xFF;
		buf[9] = (msg->msgid >> 16) & 0xFF;
		memcpy(&buf[10], _MAV_PAYLOAD(msg), length);				//存储数据
		ck = buf + header_len + 1 + (uint16_t)length;				//校验和位置上
		signature_len = (msg->incompat_flags & MAVLINK_IFLAG_SIGNED)?MAVLINK_SIGNATURE_BLOCK_LEN:0;	//13
	}
	ck[0] = (uint8_t)(msg->checksum & 0xFF);
	ck[1] = (uint8_t)(msg->checksum >> 8);
	if (signature_len > 0) {
		memcpy(&ck[2], msg->signature, signature_len);				//存放 签名
	}

	return header_len + 1 + 2 + (uint16_t)length + (uint16_t)signature_len;			//数据的总长度
}

union __mavlink_bitfield {
	uint8_t uint8;
	int8_t int8;
	uint16_t uint16;
	int16_t int16;
	uint32_t uint32;
	int32_t int32;
};


//初始化校验和的初值
MAVLINK_HELPER void mavlink_start_checksum(mavlink_message_t* msg)
{
	uint16_t crcTmp = 0;
	crc_init(&crcTmp);		//0xffff
	msg->checksum = crcTmp;
}

//加入C更新校验和的值
MAVLINK_HELPER void mavlink_update_checksum(mavlink_message_t* msg, uint8_t c)
{
	uint16_t checksum = msg->checksum;
	crc_accumulate(c, &checksum);
	msg->checksum = checksum;
}

/*
  return the crc_entry value for a msgid		
  根据消息 ID (msgid) 查找 MAVLink 消息表中的对应条目 mavlink_msg_entry_t
*/
#ifndef MAVLINK_GET_MSG_ENTRY
MAVLINK_HELPER const mavlink_msg_entry_t *mavlink_get_msg_entry(uint32_t msgid)
{
	//存储 MAVLink 消息条目。它使用 MAVLINK_MESSAGE_CRCS 初始化，假设 MAVLINK_MESSAGE_CRCS 是一个消息条目数组，包含消息 ID 和 CRC 校验等信息
	static const mavlink_msg_entry_t mavlink_message_crcs[] = MAVLINK_MESSAGE_CRCS;
        /*
	  use a bisection search to find the right entry. A perfect hash may be better
	  Note that this assumes the table is sorted by msgid
	  使用二分法查找正确的条目。完美的散列可能更好。注意，这假设表是按msgid排序的
	*/
        uint32_t low=0, high=sizeof(mavlink_message_crcs)/sizeof(mavlink_message_crcs[0]) - 1;		//low 和 high 分别初始化为 0 和数组的最后一个索引，以表示查找范围
        while (low < high) {
            uint32_t mid = (low+1+high)/2;
            if (msgid < mavlink_message_crcs[mid].msgid) {
                high = mid-1;
                continue;
            }
            if (msgid > mavlink_message_crcs[mid].msgid) {
                low = mid;
                continue;
            }
            low = mid;
            break;
        }
        if (mavlink_message_crcs[low].msgid != msgid) {
            // msgid is not in the table
            return NULL;
        }
        return &mavlink_message_crcs[low];
}
#endif // MAVLINK_GET_MSG_ENTRY

/*
  return the crc_extra value for a message		//返回消息的crc_extra值
*/
MAVLINK_HELPER uint8_t mavlink_get_crc_extra(const mavlink_message_t *msg)
{
	const mavlink_msg_entry_t *e = mavlink_get_msg_entry(msg->msgid);
	return e?e->crc_extra:0;
}

/*
  return the min message length					//返回最小消息长度
*/
#define MAVLINK_HAVE_MIN_MESSAGE_LENGTH
MAVLINK_HELPER uint8_t mavlink_min_message_length(const mavlink_message_t *msg)
{
	const mavlink_msg_entry_t *e = mavlink_get_msg_entry(msg->msgid);
        return e?e->min_msg_len:0;
}

/*
  return the max message length (including extensions)		//返回最大消息长度(包括扩展名)
*/
#define MAVLINK_HAVE_MAX_MESSAGE_LENGTH
MAVLINK_HELPER uint8_t mavlink_max_message_length(const mavlink_message_t *msg)
{
	const mavlink_msg_entry_t *e = mavlink_get_msg_entry(msg->msgid);
        return e?e->max_msg_len:0;
}

/**
 * This is a variant of mavlink_frame_char() but with caller supplied
 * parsing buffers. It is useful when you want to create a MAVLink
 * parser in a library that doesn't use any global variables
 * 这是mavlink_frame_char()的一个变体，但是带有调用者提供的解析缓冲区。
 * 当您想在一个不使用任何全局变量的库中创建一个MAVLink解析器时，这很有用
 *
 * @param rxmsg    parsing message buffer		解析消息缓冲区
 * @param status   parsing status buffer		解析状态缓冲区
 * @param c        The char to parse			要分析的字符
 *
 * @param r_message NULL if no message could be decoded, otherwise the message data				如果没有可以解码的消息，则为空，否则为消息数据
 * @param r_mavlink_status if a message was decoded, this is filled with the channel's stats	如果一条消息被解码，它将被填充通道的统计信息
 * @return 0 if no message could be decoded, 1 on good message and CRC, 2 on bad CRC			如果没有消息可以被解码，则为0，对于好的消息和CRC为1，对于坏的CRC为2
 *
 * 一个字符缓冲解析函数，用于按字节解析接收的 MAVLink 消息
 */
MAVLINK_HELPER uint8_t mavlink_frame_char_buffer(mavlink_message_t* rxmsg, 				// 用于存储解析中的 MAVLink 消息
                                                 mavlink_status_t* status,				// 解析过程中更新的状态缓冲区
                                                 uint8_t c, 							// 待解析的单字节数据
                                                 mavlink_message_t* r_message, 			// 成功解析出的消息（若解析成功）
                                                 mavlink_status_t* r_mavlink_status)	// 通道状态信息（若解析成功）
{

	status->msg_received = MAVLINK_FRAMING_INCOMPLETE;				//帧接收未完成， 等待更多数据

	switch (status->parse_state)	////当前的解析状态机状态
	{
	case MAVLINK_PARSE_STATE_UNINIT:				//未初始化状态，解析尚未开始
	case MAVLINK_PARSE_STATE_IDLE:					//空闲状态，准备接收新消息
		if (c == MAVLINK_STX)										//检查是否为帧头字节 MAVLINK_STX 0xFD(MAVLINK v2)
		{
			status->parse_state = MAVLINK_PARSE_STATE_GOT_STX;		//已接收到起始字节（STX 字节）， 表示消息的开始
			rxmsg->len = 0;											//重置消息长度
			rxmsg->magic = c;										//初始化 magic 字段
                        status->flags &= ~MAVLINK_STATUS_FLAG_IN_MAVLINK1;		//1111 1110  flags=0 清除 MAVLink 1 的标志位，标记当前消息使用 MAVLink 2
			mavlink_start_checksum(rxmsg);							//初始化 校验和计算
		} else if (c == MAVLINK_STX_MAVLINK1)
		{
			status->parse_state = MAVLINK_PARSE_STATE_GOT_STX;
			rxmsg->len = 0;
			rxmsg->magic = c;
                        status->flags |= MAVLINK_STATUS_FLAG_IN_MAVLINK1;		//flags设为1；flags(=0) |  0000 00001 =0000 0001  标记为 MAVLink 1 
			mavlink_start_checksum(rxmsg);
		}
		break;

	case MAVLINK_PARSE_STATE_GOT_STX:				//已接收到数据包的起始字节（STX）。		LEN
			if (status->msg_received 								//如果 msg_received 不为 0，表示有接收过数据包
/* Support shorter buffers than the
   default maximum packet size */
#if (MAVLINK_MAX_PAYLOAD_LEN < 255)
				|| c > MAVLINK_MAX_PAYLOAD_LEN
#endif
				)
		{
			status->buffer_overrun++;								//接收缓冲区溢出的次数+1
			_mav_parse_error(status);								//记录解析过程中发生错误的次数 +1
			status->msg_received = 0;								//已成功接收的消息数量清零
			status->parse_state = MAVLINK_PARSE_STATE_IDLE;			//空闲状态，等待新数据包的开始
		}
		else
		{
			// NOT counting STX, LENGTH, SEQ, SYSID, COMPID, MSGID, CRC1 and CRC2
			rxmsg->len = c;											//将收到的字节 c 作为消息的负载长度
			status->packet_idx = 0;									//初始化负载的当前索引为 0
			mavlink_update_checksum(rxmsg, c);						//将当前字节 c 加入校验和计算
                        if (status->flags & MAVLINK_STATUS_FLAG_IN_MAVLINK1) {		//如果当前消息是 MAVLink 1
                            rxmsg->incompat_flags = 0;				//不兼容标志和兼容标志清零
                            rxmsg->compat_flags = 0;
                            status->parse_state = MAVLINK_PARSE_STATE_GOT_COMPAT_FLAGS;		//已接收到兼容标志字段，用于指示协议扩展兼容性
                        } else {
                            status->parse_state = MAVLINK_PARSE_STATE_GOT_LENGTH;			//已接收到消息长度信息
                        }
		}
		break;

	case MAVLINK_PARSE_STATE_GOT_LENGTH:			//已接收到消息长度信息    处理incompat_flags 不兼容标志信息
		rxmsg->incompat_flags = c;									
		if ((rxmsg->incompat_flags & ~MAVLINK_IFLAG_MASK) != 0) {						//!!!!!!!!!!!!!
			// message includes an incompatible feature flag
			_mav_parse_error(status);										//记录解析过程中发生错误的次数 +1
			status->msg_received = 0;										//帧接收未完成，等待更多数据
			status->parse_state = MAVLINK_PARSE_STATE_IDLE;					//空闲状态，准备接收新消息
			break;
		}
		mavlink_update_checksum(rxmsg, c);									//将incompat_flags加入到crc计算
		status->parse_state = MAVLINK_PARSE_STATE_GOT_INCOMPAT_FLAGS;		//已接收到不兼容标志字段
		break;

	case MAVLINK_PARSE_STATE_GOT_INCOMPAT_FLAGS:	//已接收到不兼容标志字段   处理compat_flags
		rxmsg->compat_flags = c;											//该消息的兼容标志字段
		mavlink_update_checksum(rxmsg, c);									//将 c 加入校验和计算
		status->parse_state = MAVLINK_PARSE_STATE_GOT_COMPAT_FLAGS;			//状态切换  已接收到兼容标志字段
		break;

	case MAVLINK_PARSE_STATE_GOT_COMPAT_FLAGS:		//已接收到兼容标志字段		处理SEQ
		rxmsg->seq = c;
		mavlink_update_checksum(rxmsg, c);									//将 c 加入校验和计算
		status->parse_state = MAVLINK_PARSE_STATE_GOT_SEQ;					//已接收到消息序列号
		break;
                
	case MAVLINK_PARSE_STATE_GOT_SEQ:				//已接收到消息序列号    开始处理SYSID 系统 ID
		rxmsg->sysid = c;
		mavlink_update_checksum(rxmsg, c);
		status->parse_state = MAVLINK_PARSE_STATE_GOT_SYSID;				 //已接收到系统 ID
		break;

	case MAVLINK_PARSE_STATE_GOT_SYSID:				 //已接收到系统 ID		开始处理 组件ID 
		rxmsg->compid = c;
		mavlink_update_checksum(rxmsg, c);
                status->parse_state = MAVLINK_PARSE_STATE_GOT_COMPID;		//已接收到组件 ID
		break;

	case MAVLINK_PARSE_STATE_GOT_COMPID:			 //已接收到组件 ID		开始处理消息ID1 (或者消息ID第一字节)
		rxmsg->msgid = c;
		mavlink_update_checksum(rxmsg, c);
		if (status->flags & MAVLINK_STATUS_FLAG_IN_MAVLINK1) {				//MAVLINK V1 版本
			if(rxmsg->len > 0) {
				status->parse_state = MAVLINK_PARSE_STATE_GOT_MSGID3;		//已接收到消息 ID 的第三字节
			} else {
				status->parse_state = MAVLINK_PARSE_STATE_GOT_PAYLOAD;		//已接收到载荷数据  表示有效数据为空
			}
#ifdef MAVLINK_CHECK_MESSAGE_LENGTH
			if (rxmsg->len < mavlink_min_message_length(rxmsg) ||
				rxmsg->len > mavlink_max_message_length(rxmsg)) {
				_mav_parse_error(status);
				status->parse_state = MAVLINK_PARSE_STATE_IDLE;
				break;
			}
#endif
		} else {
			status->parse_state = MAVLINK_PARSE_STATE_GOT_MSGID1;			//已接收到消息 ID 的第一字节
		}
		break;

	case MAVLINK_PARSE_STATE_GOT_MSGID1:			//已接收到消息 ID 的第一字节    开始处理第二字节
		rxmsg->msgid |= ((uint32_t)c)<<8;
		mavlink_update_checksum(rxmsg, c);
		status->parse_state = MAVLINK_PARSE_STATE_GOT_MSGID2;				//已接收到消息 ID 的第2字节
		break;

	case MAVLINK_PARSE_STATE_GOT_MSGID2:			//已接收到消息 ID 的第2字节
		rxmsg->msgid |= ((uint32_t)c)<<16;
		mavlink_update_checksum(rxmsg, c);
		if(rxmsg->len > 0){
			status->parse_state = MAVLINK_PARSE_STATE_GOT_MSGID3;			//已接收到消息 ID 的第3字节
		} else {
			status->parse_state = MAVLINK_PARSE_STATE_GOT_PAYLOAD;			//跳到已接收到载荷数据  表示有效数据为空
		}
#ifdef MAVLINK_CHECK_MESSAGE_LENGTH
        if (rxmsg->len < mavlink_min_message_length(rxmsg) ||
            rxmsg->len > mavlink_max_message_length(rxmsg))
        {
			_mav_parse_error(status);
			status->parse_state = MAVLINK_PARSE_STATE_IDLE;
			break;
        }
#endif
		break;
                
	case MAVLINK_PARSE_STATE_GOT_MSGID3:			//已接收到消息 ID 的第3字节			开始处理有效数据					
		_MAV_PAYLOAD_NON_CONST(rxmsg)[status->packet_idx++] = (char)c;		//插入有效数据
		mavlink_update_checksum(rxmsg, c);
		if (status->packet_idx == rxmsg->len)								//接收完毕
		{
			status->parse_state = MAVLINK_PARSE_STATE_GOT_PAYLOAD;			//已接收到载荷数据
		}
		break;

	case MAVLINK_PARSE_STATE_GOT_PAYLOAD: {			//已接收到载荷数据		处理CRC低字节
		const mavlink_msg_entry_t *e = mavlink_get_msg_entry(rxmsg->msgid);	//获取额外CRC值
		uint8_t crc_extra = e?e->crc_extra:0;
		mavlink_update_checksum(rxmsg, crc_extra);							//加入额外CRC之后更新CRC
		if (c != (rxmsg->checksum & 0xFF)) {
			status->parse_state = MAVLINK_PARSE_STATE_GOT_BAD_CRC1;			//接收到的CRC低字节有问题
		} else {
			status->parse_state = MAVLINK_PARSE_STATE_GOT_CRC1;				//已接收到第一个 CRC 字节
		}
                rxmsg->ck[0] = c;											//存储crc低字节

		// zero-fill the packet to cope with short incoming packets			//零填充数据包，以处理短的传入数据包
                if (e && status->packet_idx < e->max_msg_len) {
                        memset(&_MAV_PAYLOAD_NON_CONST(rxmsg)[status->packet_idx], 0, e->max_msg_len - status->packet_idx);
		}
		break;
        }

	case MAVLINK_PARSE_STATE_GOT_CRC1:				//已接收到第一个 CRC 字节
	case MAVLINK_PARSE_STATE_GOT_BAD_CRC1:			//第一个CRC有问题
		if (status->parse_state == MAVLINK_PARSE_STATE_GOT_BAD_CRC1 || c != (rxmsg->checksum >> 8)) {	//CRC第一个货第二个有问题
			// got a bad CRC message
			status->msg_received = MAVLINK_FRAMING_BAD_CRC;					//接收的消息的状态标记   校验和错误，帧无效。
		} else {
			// Successfully got message
			status->msg_received = MAVLINK_FRAMING_OK;						//帧完整且校验通过。
		}
		rxmsg->ck[1] = c;													//存储crc高字节

		if (rxmsg->incompat_flags & MAVLINK_IFLAG_SIGNED) {					//必须理解的标志 为 消息是已签名的
			status->parse_state = MAVLINK_PARSE_STATE_SIGNATURE_WAIT;		//等待消息签名
			status->signature_wait = MAVLINK_SIGNATURE_BLOCK_LEN;			//剩余待接收的签名字节数，仅在启用签名验证时使用

			// If the CRC is already wrong, don't overwrite msg_received,
			// otherwise we can end up with garbage flagged as valid.
			if (status->msg_received != MAVLINK_FRAMING_BAD_CRC) {			//如果CRC验证成功
				status->msg_received = MAVLINK_FRAMING_INCOMPLETE;			//帧接收未完成，等待更多数据
			}
		} else {
			if (status->signing &&											//检查是否启用了签名处理
			   	(status->signing->accept_unsigned_callback == NULL ||		//需要消息签名验证
			   	 !status->signing->accept_unsigned_callback(status, rxmsg->msgid))) {		//验证结果不接受无签名消息

				// If the CRC is already wrong, don't overwrite msg_received.  如果CRC已经错误，不要覆盖msg_received。
				if (status->msg_received != MAVLINK_FRAMING_BAD_CRC) {
					status->msg_received = MAVLINK_FRAMING_BAD_SIGNATURE;	//签名验证失败（用于支持签名的消息），帧无效
				}
			}
			status->parse_state = MAVLINK_PARSE_STATE_IDLE;					//空闲状态，准备接收新消息
			if (r_message != NULL) {										//果 r_message 不为空，
				memcpy(r_message, rxmsg, sizeof(mavlink_message_t));		//则将当前消息的内容复制到 r_message 中供外部访问。
			}
		}
		break;
	case MAVLINK_PARSE_STATE_SIGNATURE_WAIT:		//等待消息签名（仅在使用签名时出现），用于安全验证     处理数据签名
		rxmsg->signature[MAVLINK_SIGNATURE_BLOCK_LEN-status->signature_wait] = c;		//signature_wait= 13  将接收到的签名字节存储到 rxmsg->signature 中
		status->signature_wait--;											//剩余待接收的签名字节数
		if (status->signature_wait == 0) {									//签名数据已全部接收
			// we have the whole signature, check it is OK
#ifndef MAVLINK_NO_SIGNATURE_CHECK
			bool sig_ok = mavlink_signature_check(status->signing, status->signing_streams, rxmsg);		//进行签名验证  !!!!!!!!!!!!!!!!!
#else
			bool sig_ok = true;
#endif
			if (!sig_ok &&				//如果签名验证失败，但通过应用级回调允许无签名消息，则强制设 sig_ok 为 true，即接受该消息
			   	(status->signing->accept_unsigned_callback &&
			   	 status->signing->accept_unsigned_callback(status, rxmsg->msgid))) {
				// accepted via application level override
				sig_ok = true;
			}
			if (sig_ok) {
				status->msg_received = MAVLINK_FRAMING_OK;					//帧完整且校验通过。
			} else {
				status->msg_received = MAVLINK_FRAMING_BAD_SIGNATURE;		//签名验证失败（用于支持签名的消息），帧无效
			}
			status->parse_state = MAVLINK_PARSE_STATE_IDLE;					//空闲状态，准备接收新消息
			if (r_message !=NULL) {
				memcpy(r_message, rxmsg, sizeof(mavlink_message_t));		//并将消息内容复制到 r_message
			}
		}
		break;
	}

	// If a message has been successfully decoded, check index
	if (status->msg_received == MAVLINK_FRAMING_OK)							//如果 成功接收到并正确解析的消息帧
	{
		//while(status->current_seq != rxmsg->seq)
		//{
		//	status->packet_rx_drop_count++;
		//               status->current_seq++;
		//}
		status->current_rx_seq = rxmsg->seq;								//将当前消息的序列号 rxmsg->seq 赋值给 status->current_rx_seq，用于跟踪当前接收的序列号
		// Initial condition: If no packet has been received so far, drop count is undefined
		if (status->packet_rx_success_count == 0) status->packet_rx_drop_count = 0;		//接收方检测到的丢失数据包总数 置0
		// Count this packet as received
		status->packet_rx_success_count++;									//成功接收到的数据包总数 +1	
	}

       if (r_message != NULL) {
           r_message->len = rxmsg->len; // Provide visibility on how far we are into current msg	提供我们对当前消息的了解程度
       }
       if (r_mavlink_status != NULL) {	
           r_mavlink_status->parse_state = status->parse_state;				//示当前的解析状态。
           r_mavlink_status->packet_idx = status->packet_idx;				//指示当前处理的字节索引
           r_mavlink_status->current_rx_seq = status->current_rx_seq+1;		//下一条消息的序列号准备
           r_mavlink_status->packet_rx_success_count = status->packet_rx_success_count;	//记录成功接收的包数量
           r_mavlink_status->packet_rx_drop_count = status->parse_error;	//记录解析错误的数量。
           r_mavlink_status->flags = status->flags;							//存储各种状态标志
       }
       status->parse_error = 0;												//解析过程错误 清零

	if (status->msg_received == MAVLINK_FRAMING_BAD_CRC) {					//接收到的消息帧出现 CRC 校验错误
		/*
		  the CRC came out wrong. We now need to overwrite the
		  msg CRC with the one on the wire so that if the
		  caller decides to forward the message anyway that
		  mavlink_msg_to_send_buffer() won't overwrite the
		  checksum
		 */
            if (r_message != NULL) {
                r_message->checksum = rxmsg->ck[0] | (rxmsg->ck[1]<<8);		//存储CRC校验
            }
	}

	return status->msg_received;
}

/**
 * This is a convenience function which handles the complete MAVLink parsing.
 * the function will parse one byte at a time and return the complete packet once
 * it could be successfully decoded. This function will return 0, 1 or
 * 2 (MAVLINK_FRAMING_INCOMPLETE, MAVLINK_FRAMING_OK or MAVLINK_FRAMING_BAD_CRC)
 * 这是一个方便的函数，处理完整的MAVLink解析。该函数将一次解析一个字节，并在成功解码后返回完整的数据包。
 * 此函数将返回0、1或2 (MAVLINK_FRAMING_INCOMPLETE、MAVLINK_FRAMING_OK或MAVLINK_FRAMING_BAD_CRC)
 *
 * Messages are parsed into an internal buffer (one for each channel). When a complete
 * message is received it is copies into *r_message and the channel's status is
 * copied into *r_mavlink_status.
 * 消息被解析到内部缓冲区(每个通道一个)。当接收到完整的消息时，它被复制到*r_message中，并且信道的状态被复制到*r_mavlink_status中。
 *
 * @param chan     ID of the channel to be parsed.
 *                 A channel is not a physical message channel like a serial port, but a logical partition of
 *                 the communication streams. COMM_NB is the limit for the number of channels
 *                 on MCU (e.g. ARM7), while COMM_NB_HIGH is the limit for the number of channels in Linux/Windows
 * 					要解析的通道的ID。通道不是像串行端口那样的物理消息通道，而是通信流的逻辑分区。
 * 					COMM_NB是MCU(例如ARM7)上通道数量的限制，而COMM_NB_HIGH是Linux/Windows中通道数量的限制
 * @param c        The char to parse
 *
 * @param r_message NULL if no message could be decoded, otherwise the message data				如果没有可以解码的消息，则为空，否则为消息数据
 * @param r_mavlink_status if a message was decoded, this is filled with the channel's stats	如果一条消息被解码，它将被填充通道的统计信息
 * @return 0 if no message could be decoded, 1 on good message and CRC, 2 on bad CRC			如果没有消息可以被解码，则为0，对于好的消息和CRC为1
 *
 * A typical use scenario of this function call is:
 *
 * @code
 * #include <mavlink.h>
 *
 * mavlink_status_t status;
 * mavlink_message_t msg;
 * int chan = 0;
 *
 *
 * while(serial.bytesAvailable > 0)
 * {
 *   uint8_t byte = serial.getNextByte();
 *   if (mavlink_frame_char(chan, byte, &msg, &status) != MAVLINK_FRAMING_INCOMPLETE)
 *     {
 *     printf("Received message with ID %d, sequence: %d from component %d of system %d", msg.msgid, msg.seq, msg.compid, msg.sysid);
 *     }
 * }
 *
 *
 * @endcode
 */
MAVLINK_HELPER uint8_t mavlink_frame_char(uint8_t chan, uint8_t c, mavlink_message_t* r_message, mavlink_status_t* r_mavlink_status)
{
	return mavlink_frame_char_buffer(mavlink_get_channel_buffer(chan),
					 mavlink_get_channel_status(chan),
					 c,
					 r_message,
					 r_mavlink_status);
}

/**
 * Set the protocol version		设置协议版本
 */
MAVLINK_HELPER void mavlink_set_proto_version(uint8_t chan, unsigned int version)
{
	mavlink_status_t *status = mavlink_get_channel_status(chan);
	if (version > 1) {
		status->flags &= ~(MAVLINK_STATUS_FLAG_OUT_MAVLINK1);		//flags = flags & 1111 1101  =  xxxx xx0x	(v2)
	} else {
		status->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;			//flags = flags | 0000 0010  =  xxxx xx1x	(v1)
	}
}

/**
 * Get the protocol version   获取版本号
 *
 * @return 1 for v1, 2 for v2
 */
MAVLINK_HELPER unsigned int mavlink_get_proto_version(uint8_t chan)
{
	mavlink_status_t *status = mavlink_get_channel_status(chan);
	if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) > 0) {				// flags(0000 0001) & 0000 0010 
		return 1;
	} else {		//flags(0) & 0000 0010 
		return 2;
	}
}

/**
 * This is a convenience function which handles the complete MAVLink parsing.
 * the function will parse one byte at a time and return the complete packet once
 * it could be successfully decoded. This function will return 0 or 1.
 * 这是一个方便的函数，处理完整的MAVLink解析。该函数将一次解析一个字节，并在成功解码后返回完整的数据包。该函数将返回0或1。
 *
 * Messages are parsed into an internal buffer (one for each channel). When a complete
 * message is received it is copies into *r_message and the channel's status is
 * copied into *r_mavlink_status.
 *
 * @param chan     ID of the channel to be parsed.
 *                 A channel is not a physical message channel like a serial port, but a logical partition of
 *                 the communication streams. COMM_NB is the limit for the number of channels
 *                 on MCU (e.g. ARM7), while COMM_NB_HIGH is the limit for the number of channels in Linux/Windows
 * 				    要解析的通道的ID。通道不是像串行端口那样的物理消息通道，而是通信流的逻辑分区。 COMM_NB 是MCU(例如ARM7)上通道数量的限制，
 * 				   而COMM_NB_HIGH是Linux/Windows中通道数量的限制
 * @param c        The char to parse		要分析的字符
 *
 * @param r_message NULL if no message could be decoded, otherwise the message data				如果没有可以解码的消息，则为NULL，否则为消息数据。
 * @param r_mavlink_status if a message was decoded, this is filled with the channel's stats	如果一条消息被解码，它将被填充通道的统计信息
 * @return 0 if no message could be decoded or bad CRC, 1 on good message and CRC				如果没有消息可以解码或CRC错误，则为0；如果消息和CRC正确，则为1
 *
 * A typical use scenario of this function call is:
 *
 * @code
 * #include <mavlink.h>
 *
 * mavlink_status_t status;
 * mavlink_message_t msg;
 * int chan = 0;
 *
 *
 * while(serial.bytesAvailable > 0)
 * {
 *   uint8_t byte = serial.getNextByte();
 *   if (mavlink_parse_char(chan, byte, &msg, &status))
 *     {
 *     printf("Received message with ID %d, sequence: %d from component %d of system %d", msg.msgid, msg.seq, msg.compid, msg.sysid);
 *     }
 * }
 *
 *
 * @endcode
 */
//chan 				表示逻辑通道ID，用于分隔不同的数据流
//c 				传入的字节数据
//r_message  		用于存储成功解析出的 MAVLink 消息
//r_mavlink_status	存储解析过程中获取的状态信息
MAVLINK_HELPER uint8_t mavlink_parse_char(uint8_t chan, uint8_t c, mavlink_message_t* r_message, mavlink_status_t* r_mavlink_status)
{
    uint8_t msg_received = mavlink_frame_char(chan, c, r_message, r_mavlink_status);	//解析传入的单个字节 c，并决定当前解析的状态
    if (msg_received == MAVLINK_FRAMING_BAD_CRC ||						//CRC 校验失败或者签名验证失败，处理为解析失败
	msg_received == MAVLINK_FRAMING_BAD_SIGNATURE) {
	    // we got a bad CRC. Treat as a parse failure
	    mavlink_message_t* rxmsg = mavlink_get_channel_buffer(chan);	//获取当前通道的消息缓冲区 rxmsg 
	    mavlink_status_t* status = mavlink_get_channel_status(chan);	//获取当前通道的状态信息 status
	    _mav_parse_error(status);										//将解析错误数 parse_error 加 1
	    status->msg_received = MAVLINK_FRAMING_INCOMPLETE;				//帧接收未完成，等待更多数据
	    status->parse_state = MAVLINK_PARSE_STATE_IDLE;					//将解析状态 parse_state 重置为 MAVLINK_PARSE_STATE_IDLE（空闲状态）
	    if (c == MAVLINK_STX)						//若当前字节为 MAVLINK_STX（帧头字节）
	    {
		    status->parse_state = MAVLINK_PARSE_STATE_GOT_STX;			//新消息的开始 设置状态为接收到帧头
		    rxmsg->len = 0;												// 重置消息长度
		    mavlink_start_checksum(rxmsg);								// 重新开始校验和计算  初始化msg->checksum为0xFF
	    }
	    return 0;
    }
    return msg_received;
}

/**
 * @brief Put a bitfield of length 1-32 bit into the buffer 		//将长度为1-32位的位字段放入缓冲区
 *
 * @param b the value to add, will be encoded in the bitfield		//要添加的值将被编码到位域中
 * @param bits number of bits to use to encode b, e.g. 1 for boolean, 2, 3, etc.		//用于对b进行编码的位数，例如，布尔型为1，布尔型为2，3等。
 * @param packet_index the position in the packet (the index of the first byte to use)	//数据包中的位置(要使用的第一个字节的索引)
 * @param bit_index the position in the byte (the index of the first bit to use)		//字节中的位置(要使用的第一位的索引)
 * @param buffer packet buffer to write into											//要写入的数据包缓冲区
 * @return new position of the last used byte in the buffer								//缓冲区中最后使用的字节的新位置
 * 
 * 将一个整数 b 的低 bits 位（bit field）插入到字节缓冲区 buffer 的指定位置，使用给定的 packet_index 和 bit_index 作为起始位置，
 * 并将插入后的位置更新到 r_bit_index。这是一个处理跨字节插入位段（bitfield）的辅助函数
 */
MAVLINK_HELPER uint8_t put_bitfield_n_by_index(int32_t b, uint8_t bits, uint8_t packet_index, uint8_t bit_index, uint8_t* r_bit_index, uint8_t* buffer)
{
	uint16_t bits_remain = bits;				//剩余未插入的位数 15
	// Transform number into network order
	int32_t v;									//将存储可能需要进行字节序转换后的 b 值
	uint8_t i_bit_index, i_byte_index, curr_bits_n;	//i_bit_index 和 i_byte_index 分别是当前位和字节位置，curr_bits_n 表示在当前字节中插入的位数
#if MAVLINK_NEED_BYTE_SWAP
	union {
		int32_t i;
		uint8_t b[4];
	} bin, bout;
	bin.i = b;
	bout.b[0] = bin.b[3];
	bout.b[1] = bin.b[2];
	bout.b[2] = bin.b[1];
	bout.b[3] = bin.b[0];
	v = bout.i;
#else
	v = b;
#endif

	// buffer in
	// 01100000 01000000 00000000 11110001
	// buffer out
	// 11110001 00000000 01000000 01100000

	// Existing partly filled byte (four free slots)
	// 0111xxxx

	// Mask n free bits
	// 00001111 = 2^0 + 2^1 + 2^2 + 2^3 = 2^n - 1
	// = ((uint32_t)(1 << n)) - 1; // = 2^n - 1

	// Shift n bits into the right position
	// out = in >> n;

	// Mask and shift bytes
	i_bit_index = bit_index;	//5
	i_byte_index = packet_index;
	if (bit_index > 0)
	{
		// If bits were available at start, they were available
		// in the byte before the current index
		i_byte_index--;
	}

	// While bits have not been packed yet
	while (bits_remain > 0)		//15
	{
		// Bits still have to be packed
		// there can be more than 8 bits, so
		// we might have to pack them into more than one byte

		// First pack everything we can into the current 'open' byte
		//curr_bits_n = bits_remain << 3; // Equals  bits_remain mod 8
		//FIXME
		if (bits_remain <= (uint8_t)(8 - i_bit_index))
		{
			// Enough space
			curr_bits_n = (uint8_t)bits_remain;	//这一次插入的位数 
		}
		else
		{
			curr_bits_n = (8 - i_bit_index);		//3
		}
		
		// Pack these n bits into the current byte
		// Mask out whatever was at that position with ones (xxx11111)  xxxx x111
		buffer[i_byte_index] &= (0xFF >> (8 - curr_bits_n));  //0000 0111    buffer[i_byte_index] &= ~(0xFF >> (8 - curr_bits_n));
		// Put content to this position, by masking out the non-used part	通过屏蔽掉不使用的部分，将内容放在这个位置
		buffer[i_byte_index] |= ((0x00 << curr_bits_n) & v);  //0000 0000  		buffer[i_byte_index] |= ((0xFF >> (8 - curr_bits_n)) & v)
		
		// Increment the bit index
		i_bit_index += curr_bits_n;		//8

		// Now proceed to the next byte, if necessary
		bits_remain -= curr_bits_n;		//12
		if (bits_remain > 0)
		{
			// Offer another 8 bits / one byte
			i_byte_index++;
			i_bit_index = 0;
		}
	}
	
	*r_bit_index = i_bit_index;
	// If a partly filled byte is present, mark this as consumed
	if (i_bit_index != 7) i_byte_index++;
	return i_byte_index - packet_index;
}

#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

// To make MAVLink work on your MCU, define comm_send_ch() if you wish
// to send 1 byte at a time, or MAVLINK_SEND_UART_BYTES() to send a
// whole packet at a time

/*

#include "mavlink_types.h"

void comm_send_ch(mavlink_channel_t chan, uint8_t ch)
{
    if (chan == MAVLINK_COMM_0)
    {
        uart0_transmit(ch);
    }
    if (chan == MAVLINK_COMM_1)
    {
    	uart1_transmit(ch);
    }
}
 */

MAVLINK_HELPER void _mavlink_send_uart(mavlink_channel_t chan, const char *buf, uint16_t len)
{
#ifdef MAVLINK_SEND_UART_BYTES
	/* this is the more efficient approach, if the platform
	   defines it */
	MAVLINK_SEND_UART_BYTES(chan, (const uint8_t *)buf, len);
#else
	/* fallback to one byte at a time */
	uint16_t i;
	for (i = 0; i < len; i++) {
		comm_send_ch(chan, (uint8_t)buf[i]);
	}
#endif
}
#endif // MAVLINK_USE_CONVENIENCE_FUNCTIONS

#ifdef MAVLINK_USE_CXX_NAMESPACE
} // namespace mavlink
#endif
