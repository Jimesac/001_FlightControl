#pragma once

#if defined(MAVLINK_USE_CXX_NAMESPACE)
namespace mavlink {
#elif defined(__cplusplus)
extern "C" {
#endif

// Visual Studio versions before 2010 don't have stdint.h, so we just error out.
//2010年之前的Visual Studio版本没有stdint.h，所以我们只是报错。
#if (defined _MSC_VER) && (_MSC_VER < 1600)
#error "The C-MAVLink implementation requires Visual Studio 2010 or greater"
#endif

#include <stdint.h>

/**
 *
 *  CALCULATE THE CHECKSUM
 *
 */

#define X25_INIT_CRC 0xffff
#define X25_VALIDATE_CRC 0xf0b8

#ifndef HAVE_CRC_ACCUMULATE
/**
 * @brief Accumulate the CRC16_MCRF4XX checksum by adding one char at a time.   //通过一次添加一个字符来累积MCRF4XX CRC16。
 *
 * The checksum function adds the hash of one char at a time to the             //校验和函数将一个字符的散列值一次添加到16位校验和(uint16_t)中。
 * 16 bit checksum (uint16_t).
 *
 * @param data new char to hash                         //要哈希的新字符
 * @param crcAccum the already accumulated checksum     //已经累计的校验和
 **/
static inline void crc_accumulate(uint8_t data, uint16_t *crcAccum)
{
        /*Accumulate one byte of data into the CRC*/
        uint8_t tmp;

        tmp = data ^ (uint8_t)(*crcAccum &0xff);
        tmp ^= (tmp<<4);
        *crcAccum = (*crcAccum>>8) ^ (tmp<<8) ^ (tmp <<3) ^ (tmp>>4);
}
#endif


/**
 * @brief Initialize the buffer for the MCRF4XX CRC16           //初始化MCRF4XX CRC16的缓冲器
 *
 * @param crcAccum the 16 bit MCRF4XX CRC16
 */
static inline void crc_init(uint16_t* crcAccum)
{
        *crcAccum = X25_INIT_CRC;                               //初始化为 0xffff
}


/**
 * @brief Calculates the CRC16_MCRF4XX checksum on a byte buffer                //计算字节缓冲区中的MCRF4XX CRC16校验和
 *
 * @param  pBuffer buffer containing the byte array to hash                     //包含要哈希的字节数组的缓冲区
 * @param  length  length of the byte array
 * @return the checksum over the buffer bytes
 **/
static inline uint16_t crc_calculate(const uint8_t* pBuffer, uint16_t length)
{
        uint16_t crcTmp;
        crc_init(&crcTmp);
	while (length--) {
                crc_accumulate(*pBuffer++, &crcTmp);
        }
        return crcTmp;
}


/**
 * @brief Accumulate the MCRF4XX CRC16 by adding an array of bytes              //通过添加一个字节数组来累加MCRF4XX CRC16 CRC
 *
 * The checksum function adds the hash of one char at a time to the
 * 16 bit checksum (uint16_t).
 *
 * @param data new bytes to hash
 * @param crcAccum the already accumulated checksum
 **/
static inline void crc_accumulate_buffer(uint16_t *crcAccum, const char *pBuffer, uint16_t length)
{
	const uint8_t *p = (const uint8_t *)pBuffer;
	while (length--) {
                crc_accumulate(*p++, crcAccum);
        }
}

#if defined(MAVLINK_USE_CXX_NAMESPACE) || defined(__cplusplus)
}
#endif
