#include "bsp_flash.h"
#include <stdio.h>

struct {
    xpi_nor_config_option_t option;     //储配置选项，将在初始化过程中传递给 NOR Flash 驱动函数。
    xpi_nor_config_t xpi_nor_config;    //存储 NOR Flash 配置信息，将在初始化过程中传递给 NOR Flash 驱动函数。

    uint32_t flash_size;                // 存储 NOR Flash 的总大小，以字节为单位。
    uint32_t sector_size;               // 存储 NOR Flash 的扇区大小，以字节为单位。
    uint32_t page_size;                 // 存储 NOR Flash 的页大小，以字节为单位。
}nor_flash_info;

void bsp_flash_init(void)
{
    hpm_stat_t status;      // 存储函数的返回状态，用于检查函数执行的结果。

    nor_flash_info.option.header.U = BOARD_XPI_NOR_CFG_OPT_HDR;     // 配置选项的头部，用于标识配置选项的类型。
    nor_flash_info.option.option0.U = BOARD_XPI_NOR_CFG_OPT_OPT0;   // 配置选项的第一个选项，用于指定 NOR Flash 的工作模式。
    nor_flash_info.option.option1.U = BOARD_XPI_NOR_CFG_OPT_OPT1;   // 配置选项的第二个选项，用于指定 NOR Flash 的工作模式。

    status = rom_xpi_nor_auto_config(BOARD_XPI_NOR_XPI_BASE, &nor_flash_info.xpi_nor_config, &nor_flash_info.option);   // 自动配置 NOR Flash。
    if (status != status_success) {
        printf("nor flash configure failed\n");
    }
    rom_xpi_nor_get_property(BOARD_XPI_NOR_XPI_BASE, &nor_flash_info.xpi_nor_config, xpi_nor_property_total_size, &nor_flash_info.flash_size);          // 获取 NOR Flash 的总大小。
    rom_xpi_nor_get_property(BOARD_XPI_NOR_XPI_BASE, &nor_flash_info.xpi_nor_config, xpi_nor_property_sector_size, &nor_flash_info.sector_size);        // 获取 NOR Flash 的扇区大小。
    rom_xpi_nor_get_property(BOARD_XPI_NOR_XPI_BASE, &nor_flash_info.xpi_nor_config, xpi_nor_property_page_size, &nor_flash_info.page_size);            // 获取 NOR Flash 的页大小。

    if (nor_flash_info.sector_size < USER_PARAMS_FLASH_WORDS_SIZE)  // 检查用户参数的闪存大小是否小于扇区大小。820U
    {
        printf("ERROR: USER FLASH SIZE AND FLASH SECTOR SIZE ERR \n");
    }

}

//addr：要读取的 Flash 起始地址。
//pbuf：指向存储读取数据的缓冲区指针。
//len：读取的长度，以字（32 位）为单位
hpm_stat_t bsp_flash_read_words(uint32_t addr, uint32_t *pbuf, uint16_t len)    
{
    hpm_stat_t status;   

    status = rom_xpi_nor_read(BOARD_XPI_NOR_XPI_BASE, xpi_xfer_channel_auto, &nor_flash_info.xpi_nor_config, pbuf, addr, len*4);    //自动选择通道的方式读取len是字节数，所以要乘4，这里是uint32_t，所以要乘4
    return status;
}

//功能：将指定长度的数据写入 Flash。
//addr：要写入的 Flash 起始地址。必须是扇区的开始地址。
hpm_stat_t bsp_flash_write_words(uint32_t addr, const uint32_t *pbuf, uint16_t len)
{
    hpm_stat_t status;
    
    if (addr % nor_flash_info.sector_size != 0)  return status_invalid_argument;    // 检查地址是否对齐到扇区边界。否则返回无效参数的状态。

    status = rom_xpi_nor_program(BOARD_XPI_NOR_XPI_BASE, xpi_xfer_channel_auto, &nor_flash_info.xpi_nor_config, pbuf, addr, len*4);//
    return status;
}

//读取倒数第二扇区开始的len个数据数据
hpm_stat_t bsp_flash_user_params_read_words(uint32_t addr_words_offset, uint32_t *pbuf, uint16_t len)
{
    hpm_stat_t status;  // 存储函数的返回状态，用于检查函数执行的结果。
    uint32_t addr;

    addr = nor_flash_info.flash_size - nor_flash_info.sector_size*USER_PARAMS_FLASH_INDEX_LAST; // 计算用户参数的起始地址。

    status = rom_xpi_nor_read(BOARD_XPI_NOR_XPI_BASE, xpi_xfer_channel_auto, &nor_flash_info.xpi_nor_config, pbuf, addr, len*4);
    return status;
}

//功能：将指定长度的数据写入用户参数扇区。 
//将用户参数数据写入到 Flash 的指定区域，同时保留原有数据的完整性。该方法通过读取、修改、擦除和重新写入实现对 Flash 的非对齐写操作。
hpm_stat_t bsp_flash_user_params_write_words(uint32_t addr_words_offset, const uint32_t *pbuf, uint16_t len)
{
    hpm_stat_t status;                                      // 存储函数的返回状态，用于检查函数执行的结果。
    uint32_t s_read_buf[USER_PARAMS_FLASH_WORDS_SIZE];      //820*4 = 3280  3.2KB   一扇区=4KB
    uint32_t addr;                                          // 存储用户参数的起始地址。
    
    addr = nor_flash_info.flash_size - nor_flash_info.sector_size*USER_PARAMS_FLASH_INDEX_LAST;     //倒数第二扇区的地址开始
    status = bsp_flash_read_words(addr, s_read_buf, USER_PARAMS_FLASH_WORDS_SIZE);                  //读取倒数第二扇区的内容

    if (status == status_success)                                       //读取成功
    {
        for (uint16_t i = 0, j = addr_words_offset; i < len; i++, j++)  //将pbuf中的数据写入到s_read_buf中
        {
            s_read_buf[j] = pbuf[i];                                    //j是s_read_buf的索引，i是pbuf的索引
            if (j >= (nor_flash_info.sector_size>>2)) return 0;         //如果j大于等于扇区大小的一半，就返回0
        }
        disable_irq_from_intc();                                        //关闭中断
        status = rom_xpi_nor_erase(BOARD_XPI_NOR_XPI_BASE, xpi_xfer_channel_auto, &nor_flash_info.xpi_nor_config, addr, nor_flash_info.sector_size);    //擦除扇区
        status = rom_xpi_nor_program(BOARD_XPI_NOR_XPI_BASE, xpi_xfer_channel_auto, &nor_flash_info.xpi_nor_config, s_read_buf, \
                addr, USER_PARAMS_FLASH_WORDS_SIZE*4);                  //写入扇区 大小为3280字节
        enable_irq_from_intc();                                         //打开中断
        return status;
    }
    else 
    {
        return status;
    }
}