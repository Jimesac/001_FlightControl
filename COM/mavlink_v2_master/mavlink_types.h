#pragma once

// Visual Studio versions before 2010 don't have stdint.h, so we just error out.
#if (defined _MSC_VER) && (_MSC_VER < 1600)
#error "The C-MAVLink implementation requires Visual Studio 2010 or greater"
#endif

#include <stdbool.h>
#include <stdint.h>

#ifdef MAVLINK_USE_CXX_NAMESPACE
namespace mavlink {
#endif

// Macro to define packed structures
#ifdef __GNUC__
  #define MAVPACKED( __Declaration__ ) __Declaration__ __attribute__((packed))
#else
  #define MAVPACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )      //对结构体或联合体进行内存对齐设置
#endif

#ifndef MAVLINK_MAX_PAYLOAD_LEN
// it is possible to override this, but be careful!
#define MAVLINK_MAX_PAYLOAD_LEN 255 ///< Maximum payload length
#endif

#define MAVLINK_CORE_HEADER_LEN 9 ///< Length of core header (of the comm. layer)       //(通信层的)核心报头长度（LEN--MSGID）
#define MAVLINK_CORE_HEADER_MAVLINK1_LEN 5 ///< Length of MAVLink1 core header (of the comm. layer) // MAVLink1核心报头的长度(通信层)
#define MAVLINK_NUM_HEADER_BYTES (MAVLINK_CORE_HEADER_LEN + 1) ///< Length of all header bytes, including core and stx      //10 (STX-MSGID)
#define MAVLINK_NUM_CHECKSUM_BYTES 2        //校验和长度
#define MAVLINK_NUM_NON_PAYLOAD_BYTES (MAVLINK_NUM_HEADER_BYTES + MAVLINK_NUM_CHECKSUM_BYTES)       //12 (STX-MSGID)+CHECKSUM

#define MAVLINK_SIGNATURE_BLOCK_LEN 13      //数据签名长度

#define MAVLINK_MAX_PACKET_LEN (MAVLINK_MAX_PAYLOAD_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES + MAVLINK_SIGNATURE_BLOCK_LEN) ///< Maximum packet length 总长度 255+12+13

/**
 * Old-style 4 byte param union
 *
 * This struct is the data format to be used when sending
 * parameters. The parameter should be copied to the native
 * type (without type conversion)
 * and re-instanted on the receiving side using the
 * native type as well.
 * 该结构是发送参数时使用的数据格式。应该将参数复制到本机类型(不进行类型转换)，
 * 并在接收端使用本机类型重新实例化
 */
MAVPACKED(
typedef struct param_union {
	union {
		float param_float;
		int32_t param_int32;
		uint32_t param_uint32;
		int16_t param_int16;
		uint16_t param_uint16;
		int8_t param_int8;
		uint8_t param_uint8;
		uint8_t bytes[4];
	};
	uint8_t type;
}) mavlink_param_union_t;


/**
 * New-style 8 byte param union
 * mavlink_param_union_double_t will be 8 bytes long, and treated as needing 8 byte alignment for the purposes of MAVLink 1.0 field ordering.
 * The mavlink_param_union_double_t will be treated as a little-endian structure.
 *
 * If is_double is 1 then the type is a double, and the remaining 63 bits are the double, with the lowest bit of the mantissa zero.
 * The intention is that by replacing the is_double bit with 0 the type can be directly used as a double (as the is_double bit corresponds to the
 * lowest mantissa bit of a double). If is_double is 0 then mavlink_type gives the type in the union.
 * The mavlink_types.h header will also need to have shifts/masks to define the bit boundaries in the above,
 * as bitfield ordering isn't consistent between platforms. The above is intended to be for gcc on x86,
 * which should be the same as gcc on little-endian arm. When using shifts/masks the value will be treated as a 64 bit unsigned number,
 * and the bits pulled out using the shifts/masks.
*/
MAVPACKED(
typedef struct param_union_extended {
    union {
    struct {
        uint8_t is_double:1;
        uint8_t mavlink_type:7;
        union {
            char c;
            uint8_t uint8;
            int8_t int8;
            uint16_t uint16;
            int16_t int16;
            uint32_t uint32;
            int32_t int32;
            float f;
            uint8_t align[7];
        };
    };
    uint8_t data[8];
    };
}) mavlink_param_union_double_t;

/**
 * This structure is required to make the mavlink_send_xxx convenience functions
 * work, as it tells the library what the current system and component ID are.
 */
MAVPACKED(
typedef struct __mavlink_system {
    uint8_t sysid;   ///< Used by the MAVLink message_xx_send() convenience function
    uint8_t compid;  ///< Used by the MAVLink message_xx_send() convenience function
}) mavlink_system_t;

MAVPACKED(
typedef struct __mavlink_message {
	uint16_t checksum;      ///< sent at end of packet                          //校验和
	uint8_t magic;          ///< protocol magic marker                          //数据包启动标记 帧头
	uint8_t len;            ///< Length of payload                              //载荷长度  字节
	uint8_t incompat_flags; ///< flags that must be understood                  //必须理解的标志    0x01(表示用数据签名 必须为) 0x00(不用数据签名)
	uint8_t compat_flags;   ///< flags that can be ignored if not understood    //如果不理解，可以忽略的标志
	uint8_t seq;            ///< Sequence of packet                             //数据包序列号
	uint8_t sysid;          ///< ID of message sender system/aircraft           //系统 ID
	uint8_t compid;         ///< ID of the message sender component             //组件ID
	uint32_t msgid:24;      ///< ID of message in payload                       //消息 ID
	uint64_t payload64[(MAVLINK_MAX_PAYLOAD_LEN+MAVLINK_NUM_CHECKSUM_BYTES+7)/8];   //数组是64位的，发送的数据    字节数=(255+2+7)/8 *8=264/8 *8=33*8=264
	uint8_t ck[2];          ///< incoming checksum bytes                        //传入校验和字节
	uint8_t signature[MAVLINK_SIGNATURE_BLOCK_LEN];                             //签名 第[0]字节为 mavlink_signing_t->link_id；1-6为mavlink_signing_t->timestamp，7-12为sha256校验码
}) mavlink_message_t;

typedef enum {
	MAVLINK_TYPE_CHAR     = 0,
	MAVLINK_TYPE_UINT8_T  = 1,
	MAVLINK_TYPE_INT8_T   = 2,
	MAVLINK_TYPE_UINT16_T = 3,
	MAVLINK_TYPE_INT16_T  = 4,
	MAVLINK_TYPE_UINT32_T = 5,
	MAVLINK_TYPE_INT32_T  = 6,
	MAVLINK_TYPE_UINT64_T = 7,
	MAVLINK_TYPE_INT64_T  = 8,
	MAVLINK_TYPE_FLOAT    = 9,
	MAVLINK_TYPE_DOUBLE   = 10
} mavlink_message_type_t;

#define MAVLINK_MAX_FIELDS 64

typedef struct __mavlink_field_info {
	const char *name;                 // name of this field
        const char *print_format;         // printing format hint, or NULL
        mavlink_message_type_t type;      // type of this field
        unsigned int array_length;        // if non-zero, field is an array
        unsigned int wire_offset;         // offset of each field in the payload
        unsigned int structure_offset;    // offset in a C structure
} mavlink_field_info_t;

// note that in this structure the order of fields is the order
// in the XML file, not necessary the wire order
typedef struct __mavlink_message_info {
	uint32_t msgid;                                        // message ID
	const char *name;                                      // name of the message
	unsigned num_fields;                                   // how many fields in this message
	mavlink_field_info_t fields[MAVLINK_MAX_FIELDS];       // field information
} mavlink_message_info_t;

#define _MAV_PAYLOAD(msg) ((const char *)(&((msg)->payload64[0])))          //返回消息 msg 的有效载荷部分的常量和非常量指针。
#define _MAV_PAYLOAD_NON_CONST(msg) ((char *)(&((msg)->payload64[0])))

// checksum is immediately after the payload bytes      //校验和紧跟在有效负载字节之后
#define mavlink_ck_a(msg) *((msg)->len + (uint8_t *)_MAV_PAYLOAD_NON_CONST(msg))                //分别定位校验和的第一个字节 CK_A 和第二个字节 CK_B。
#define mavlink_ck_b(msg) *(((msg)->len+(uint16_t)1) + (uint8_t *)_MAV_PAYLOAD_NON_CONST(msg))  //发送的数据后面两字节

#ifndef HAVE_MAVLINK_CHANNEL_T
typedef enum {
    MAVLINK_COMM_0,
    MAVLINK_COMM_1,
    MAVLINK_COMM_2,
    MAVLINK_COMM_3
} mavlink_channel_t;
#endif

/*
 * applications can set MAVLINK_COMM_NUM_BUFFERS to the maximum number
 * of buffers they will use. If more are used, then the result will be
 * a stack overrun
 */
#ifndef MAVLINK_COMM_NUM_BUFFERS
#if (defined linux) | (defined __linux) | (defined  __MACH__) | (defined _WIN32)
# define MAVLINK_COMM_NUM_BUFFERS 16
#else
# define MAVLINK_COMM_NUM_BUFFERS 4
#endif
#endif

typedef enum {
    MAVLINK_PARSE_STATE_UNINIT=0,                   //未初始化状态，解析尚未开始
    MAVLINK_PARSE_STATE_IDLE,                       //空闲状态，准备接收新消息
    MAVLINK_PARSE_STATE_GOT_STX,                    //已接收到起始字节（STX 字节），表示消息的开始
    MAVLINK_PARSE_STATE_GOT_LENGTH,                 //已接收到消息长度信息
    MAVLINK_PARSE_STATE_GOT_INCOMPAT_FLAGS,         //已接收到不兼容标志字段，用于指示协议版本兼容性
    MAVLINK_PARSE_STATE_GOT_COMPAT_FLAGS,           //已接收到兼容标志字段，用于指示协议扩展兼容性
    MAVLINK_PARSE_STATE_GOT_SEQ,                    //已接收到消息序列号，确保消息顺序传递
    MAVLINK_PARSE_STATE_GOT_SYSID,                  //已接收到系统 ID，标识消息的发送源系统
    MAVLINK_PARSE_STATE_GOT_COMPID,                 //已接收到组件 ID，标识消息的发送源组件
    MAVLINK_PARSE_STATE_GOT_MSGID1,                 //已接收到消息 ID 的不同部分（V2 中分为 3 个字节），标识消息类型。
    MAVLINK_PARSE_STATE_GOT_MSGID2,                 //已接收到消息有效载荷部分
    MAVLINK_PARSE_STATE_GOT_MSGID3,
    MAVLINK_PARSE_STATE_GOT_PAYLOAD,                //已接收到载荷数据
    MAVLINK_PARSE_STATE_GOT_CRC1,                   //已接收到第一个 CRC 字节
    MAVLINK_PARSE_STATE_GOT_BAD_CRC1,               //接收到的CRC字节有问题
    MAVLINK_PARSE_STATE_SIGNATURE_WAIT              //等待消息签名（仅在使用签名时出现），用于安全验证
} mavlink_parse_state_t; ///< The state machine for the comm parser

typedef enum {
    MAVLINK_FRAMING_INCOMPLETE=0,                   //帧接收未完成，等待更多数据
    MAVLINK_FRAMING_OK=1,                           //帧完整且校验通过。
    MAVLINK_FRAMING_BAD_CRC=2,                      //校验和错误，帧无效。
    MAVLINK_FRAMING_BAD_SIGNATURE=3                 //签名验证失败（用于支持签名的消息），帧无效
} mavlink_framing_t;

#define MAVLINK_STATUS_FLAG_IN_MAVLINK1  1 // last incoming packet was MAVLink1     //最后一个传入的数据包是MAVLink1
#define MAVLINK_STATUS_FLAG_OUT_MAVLINK1 2 // generate MAVLink1 by default          //默认情况下生成MAVLink1
#define MAVLINK_STATUS_FLAG_IN_SIGNED    4 // last incoming packet was signed and validated     //最后一个传入的数据包已经过签名和验证
#define MAVLINK_STATUS_FLAG_IN_BADSIG    8 // last incoming packet had a bad signature          //最后一个传入数据包的签名不正确

#define MAVLINK_STX_MAVLINK1 0xFE          // marker for old protocol               //旧协议的标记

typedef struct __mavlink_status {
    uint8_t msg_received;               ///< Number of received messages    //记录已成功接收的消息的状态。
    uint8_t buffer_overrun;             ///< Number of buffer overruns      //记录接收缓冲区溢出的次数。当缓冲区已满但仍有数据要写入时，会导致缓冲区溢出，并增加该计数器
    uint8_t parse_error;                ///< Number of parse errors         //记录解析过程中发生错误的次数
    mavlink_parse_state_t parse_state;  ///< Parsing state machine          //当前的解析状态机状态
    uint8_t packet_idx;                 ///< Index in current packet        //当前数据包中的索引位置，指示解析过程中正处理的字节位置，便于跟踪当前解析到的数据位置。
    uint8_t current_rx_seq;             ///< Sequence number of last packet received    //最近接收到的数据包的序列号，用于检测丢包情况
    uint8_t current_tx_seq;             ///< Sequence number of last packet sent        //最近发送的数据包的序列号。
    uint16_t packet_rx_success_count;   ///< Received packets                           //记录成功接收到的数据包总数
    uint16_t packet_rx_drop_count;      ///< Number of packet drops                     //记录接收方检测到的丢失数据包总数
    uint8_t flags;                      ///< MAVLINK_STATUS_FLAG_*                      //存储各种状态标志，例如是否启用签名验证
    uint8_t signature_wait;             ///< number of signature bytes left to receive  //剩余待接收的签名字节数，仅在启用签名验证时使用
    struct __mavlink_signing *signing;  ///< optional signing state                     //向用于签名验证的状态结构体，存储签名相关的上下文信息
    struct __mavlink_signing_streams *signing_streams; ///< global record of stream timestamps  //指向全局流签名时间戳记录结构体，用于存储消息流的签名时间戳记录，用于防止重放攻击等
} mavlink_status_t;

/*
  a callback function to allow for accepting unsigned packets       定义了一个回调函数类型，用于处理“无签名消息”的接受逻辑
 */
typedef bool (*mavlink_accept_unsigned_t)(const mavlink_status_t *status, uint32_t msgid);

/*
  flags controlling signing
 */
#define MAVLINK_SIGNING_FLAG_SIGN_OUTGOING 1    ///< Enable outgoing signing启用传出签名        //设置此标志位时，所有发送的消息都会附带数字签名，以增强安全性

typedef enum {
    MAVLINK_SIGNING_STATUS_NONE=0,                  //当前没有签名状态。
    MAVLINK_SIGNING_STATUS_OK=1,                    //签名验证成功
    MAVLINK_SIGNING_STATUS_BAD_SIGNATURE=2,         //签名验证失败，签名无效
    MAVLINK_SIGNING_STATUS_NO_STREAMS=3,            //签名验证失败，没有可用的消息流记录
    MAVLINK_SIGNING_STATUS_TOO_MANY_STREAMS=4,      //消息流记录过多，可能存在资源限制
    MAVLINK_SIGNING_STATUS_OLD_TIMESTAMP=5,         //消息的时间戳过旧，可能是重放攻击的标志
    MAVLINK_SIGNING_STATUS_REPLAY=6,                //测到重放攻击，即相同消息被重复发送
} mavlink_signing_status_t;
    
/*
  state of MAVLink signing for this channel         此通道的MAVLink签名状态
 */
typedef struct __mavlink_signing {
    uint8_t flags;                     ///< MAVLINK_SIGNING_FLAG_*      //控制签名行为的标志位,设置了该标志位时，表示启用消息签名
    uint8_t link_id;                   ///< Same as MAVLINK_CHANNEL     //标识通信链接的 ID
    uint64_t timestamp;                ///< Timestamp, in microseconds since UNIX epoch GMT     //记录消息的时间戳，以 UNIX 时间格式表示，单位是微秒
    uint8_t secret_key[32];             //32 字节的私钥，用于对消息进行签名或验证签名
    mavlink_accept_unsigned_t accept_unsigned_callback;     //回调函数指针，用于判断是否接受无签名消息
    mavlink_signing_status_t last_status;                   //记录上一次签名验证的状态
} mavlink_signing_t;

/*
  timestamp state of each logical signing stream. This needs to be the same structure for all
  connections in order to be secure
  每个逻辑签名流的时间戳状态。所有人都需要相同的结构连接，以确保安全
 */
#ifndef MAVLINK_MAX_SIGNING_STREAMS
#define MAVLINK_MAX_SIGNING_STREAMS 16      //在 MAVLink 协议中最多支持 16 个签名流
#endif
typedef struct __mavlink_signing_streams {      //管理 MAVLink 通信中的每个签名流的时间戳状态
    uint16_t num_signing_streams;           //当前使用的签名流数量。
    struct __mavlink_signing_stream {
        uint8_t link_id;              ///< ID of the link (MAVLINK_CHANNEL)     //链接的标识符
        uint8_t sysid;                ///< Remote system ID                     //远程系统的 ID
        uint8_t compid;               ///< Remote component ID                  //远程组件的 ID
        uint8_t timestamp_bytes[6];   ///< Timestamp, in microseconds since UNIX epoch GMT //6 字节的时间戳，从 UNIX 纪元（1970 年 1 月 1 日）起以微秒为单位表示
    } stream[MAVLINK_MAX_SIGNING_STREAMS];      //个结构体数组，每个元素表示一个签名流的状态
} mavlink_signing_streams_t;


#define MAVLINK_BIG_ENDIAN 0        //大端字节序
#define MAVLINK_LITTLE_ENDIAN 1     //小端字节序

#define MAV_MSG_ENTRY_FLAG_HAVE_TARGET_SYSTEM    1      //标记消息是否包含目标系统 ID。
#define MAV_MSG_ENTRY_FLAG_HAVE_TARGET_COMPONENT 2      //用于标记消息是否包含目标组件 ID。

/*
  entry in table of information about each message type     关于每种消息类型的信息表中的条目
 */
typedef struct __mavlink_msg_entry {                    
	uint32_t msgid;                     //每个消息都有一个唯一的 msgid，用于标识消息类型
	uint8_t crc_extra;                  //附加 CRC 校验字节，用于增强消息的完整性检查
        uint8_t min_msg_len;       // minimum message length        //该消息的最小长度，确保解析时不会少于此值
        uint8_t max_msg_len;       // maximum message length (e.g. including mavlink2 extensions)   //该消息的最大长度，包含可能的扩展
        uint8_t flags;             // MAV_MSG_ENTRY_FLAG_*          //标志位，用于指定特定消息的属性
	uint8_t target_system_ofs; // payload offset to target_system, or 0     //指示消息中 target_system 字段的位置偏移量。若消息中不存在该字段，则为 0
	uint8_t target_component_ofs; // payload offset to target_component, or 0   //指示消息中 target_component 字段的位置偏移量，若该字段不存在，则为 0
} mavlink_msg_entry_t;

/*
  incompat_flags bits
 */
#define MAVLINK_IFLAG_SIGNED  0x01      //表示消息是已签名的
#define MAVLINK_IFLAG_MASK    0x01 // mask of all understood bits   //所有已知标志位的掩码

#ifdef MAVLINK_USE_CXX_NAMESPACE
} // namespace mavlink
#endif
