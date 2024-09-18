#include "magencoder.h"
#include "bsp_spi.h"

uint8_t magencoder_mt6835_write_cmd = 0xA0; 
uint8_t magencoder_mt6835_angle_addr = 0x03;

struct {
    uint8_t angle_bits20_13;
    uint8_t angle_bits12_5;
    struct {
      uint8_t status:3;
      uint8_t angle_bits4_0: 5;
    }angle_bits4_0_staus;
    uint8_t crc;
}magencoder_mt6835_raw_data;

void magencoder_init(void)
{
    bsp_magencoder_spi_receive(magencoder_mt6835_write_cmd, magencoder_mt6835_angle_addr, (uint8_t *)&magencoder_mt6835_raw_data, 4);
}

void magencoder_update(void)
{
    bsp_magencoder_spi_receive(magencoder_mt6835_write_cmd, magencoder_mt6835_angle_addr, (uint8_t *)&magencoder_mt6835_raw_data, 4);
}