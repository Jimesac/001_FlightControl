#include "bsp_uart.h"
#include "bsp_dma.h"
#include <stdio.h>

//定义了一个 UART 信息结构体，管理 UART 发送缓冲区和相关的索引
typedef struct{         
    uint8_t *tx_pbuf[UART_TX_MESSAGE_QUEUES_NUM];       //3 指向发送缓冲区的指针数组，用于管理多个消息队列的缓冲区指针
    uint16_t tx_buf_len[UART_TX_MESSAGE_QUEUES_NUM];    //3 每个缓冲区的数据长度数组，对应 tx_pbuf
    uint8_t tx_index;                                   //0-2 当前使用的缓冲区索引，用于追踪当前正在发送的数据位置
    uint8_t tx_queue_index;                             //0-2 队列索引，用于管理多条消息发送的队列位置
}uart_info_t; 

//非缓存 BSS 区域：这通常用于 DMA（直接内存访问）或与硬件直接交互的内存区域，确保内存与外设一致性，避免 CPU 缓存问题
ATTR_PLACE_AT_NONCACHEABLE_BSS_WITH_ALIGNMENT(4) uint8_t uart_dbg_tx_buf[UART_TX_MESSAGE_QUEUES_NUM][UART_TX_BUF_SIZE];     //[3][256]uart调试接口的缓冲区
ATTR_PLACE_AT_NONCACHEABLE_BSS_WITH_ALIGNMENT(4) uint8_t uart_com_ma_tx_buf[UART_TX_MESSAGE_QUEUES_NUM][UART_TX_BUF_SIZE];  //[3][256]dma通信接口的缓冲区
ATTR_PLACE_AT_NONCACHEABLE_BSS_WITH_ALIGNMENT(4) uint8_t uart_com_fc_tx_buf[UART_TX_MESSAGE_QUEUES_NUM][UART_TX_BUF_SIZE];  //[3][256]dma通信接口的缓冲区

uart_info_t uart_dbg_info =     //调试 消息结构体，串口1 调试用的
{
    .tx_pbuf = {(void *)0},         //
    .tx_buf_len = {0},
    .tx_index = 0,
    .tx_queue_index = 0,
 
};

uart_info_t uart_com_ma_info =  //串口，跟MA摄像头通信的
{
    .tx_pbuf = {(void *)0},
    .tx_buf_len = {0},
    .tx_index = 0,
    .tx_queue_index = 0,
 
};

uart_info_t uart_com_fc_info =  //串口，跟 飞控 通信的
{
    .tx_pbuf = {(void *)0},
    .tx_buf_len = {0},
    .tx_index = 0,
    .tx_queue_index = 0,
 
};

static void bsp_uart_dbg_init(void);    //初始化调试和通信的 UART 模块
static void bsp_uart_com_ma_init(void); //初始化通信 UART 接口  与MA摄像头相连
static void bsp_uart_com_fc_init(void); //和飞控通信

void bsp_uart_init(void)
{
    uint8_t i = 0;

    for (i = 0; i < UART_TX_MESSAGE_QUEUES_NUM; i++)        //3  将全局定义的非缓存缓冲区（uart_dbg_tx_buf 和 uart_com_ma_tx_buf）的地址存储到对应结构体中
    {
        uart_dbg_info.tx_pbuf[i] = &uart_dbg_tx_buf[i][0];
    }

      for (i = 0; i < UART_TX_MESSAGE_QUEUES_NUM; i++)
    {
        uart_com_ma_info.tx_pbuf[i] = &uart_com_ma_tx_buf[i][0];
    }
    for (i = 0; i < UART_TX_MESSAGE_QUEUES_NUM; i++)
    {
        uart_com_fc_info.tx_pbuf[i] = &uart_com_fc_tx_buf[i][0];
    }

    bsp_uart_dbg_init();        //初始化调试 UART 接口
    bsp_uart_com_ma_init();     //初始化通信 UART 接口 跟摄像头通信的
    bsp_uart_com_fc_init();
}

//初始化调试用 UART 接口    串口0
static void bsp_uart_dbg_init(void)
{
    uart_config_t config;           //uart初始化配置结构体
    hpm_stat_t stat;
    uint32_t freq = 0U;

    // GPIO初始化
    HPM_IOC->PAD[BOARD_DBG_UART_RX_GPIO_PIN].FUNC_CTL = BOARD_DBG_UART_RX_GPIO_FUNC;    //PA01 使用 HPM_IOC（引脚复用配置寄存器）设置 UART RX 和 TX 引脚功能
    HPM_IOC->PAD[BOARD_DBG_UART_TX_GPIO_PIN].FUNC_CTL = BOARD_DBG_UART_TX_GPIO_FUNC;    //PA00

    // 串口初始化
    clock_set_source_divider(BOARD_DBG_UART_CLK_NAME, clk_src_osc24m, 1);               //使用内部 osc24m 时钟源为 UART 配置时钟，分频系数为 1
    clock_add_to_group(BOARD_DBG_UART_CLK_NAME, 0);                                     //将 UART 时钟添加到 0 时钟组（组编号为 0）
    freq = clock_get_frequency(BOARD_DBG_UART_CLK_NAME);                                //获取 UART 实际工作频率并存储到变量 freq 中

    // 串口默认设置
    config.baudrate = BOARD_DBG_UART_BAUDRATE;          //波特率为 46 0800 UL
    config.word_length = word_length_8_bits;            //数据格式为 8位数据
    config.parity = parity_none;                        //无校验位
    config.num_of_stop_bits = stop_bits_1;              //1 位停止位
    config.fifo_enable = true;                          //启用 FIFO，提高数据传输效率
    config.rx_fifo_level = uart_rx_fifo_trg_not_empty;  //接收 FIFO 非空时触发中断或 DMA
    config.tx_fifo_level = uart_tx_fifo_trg_not_full;   //发送 FIFO 未满时触发中断或 DMA
    config.dma_enable = false;                      //禁用DMA
    config.modem_config.auto_flow_ctrl_en = false;  //禁用硬件自动流量控制（RTS/CTS）
    config.modem_config.loop_back_en = false;       //禁用回环模式
    config.modem_config.set_rts_high = false;       //不设置 RTS 引脚为高电平

    config.rxidle_config.detect_enable = false;         //禁用 RX 空闲检测功能。
    config.rxidle_config.detect_irq_enable = false;     //禁用 当线路空闲时触发中断或唤醒设备
    config.rxidle_config.idle_cond = uart_rxline_idle_cond_rxline_logic_one;    //表示 RX 线路保持高电平时检测空闲。这是 UART 线路默认的空闲逻辑（高电平表示空闲）
    config.rxidle_config.threshold = 10; /* 10-bit for typical UART configuration (8-N-1) */ //默认设置空闲检测阈值为 10

    config.txidle_config.detect_enable = false;         //禁用 TX 空闲检测功能。
    config.txidle_config.detect_irq_enable = false;     //禁用 当线路空闲时触发中断或唤醒设备
    config.txidle_config.idle_cond = uart_rxline_idle_cond_rxline_logic_one;    //表示 TX 线路保持高电平时检测空闲。这是 UART 线路默认的空闲逻辑（高电平表示空闲）
    config.txidle_config.threshold = 10; /* 10-bit for typical UART configuration (8-N-1) */

    config.rx_enable = true;                            //启用接收功能，允许 UART 接收数据
    
    // 串口配置
    config.fifo_enable = true;          //启用 FIFO 缓存功能，提高数据传输性能
    config.dma_enable = true;           //启用 DMA 支持
    config.src_freq_in_hz = freq;       //串口频率
    config.tx_fifo_level = uart_tx_fifo_trg_not_full;   //发送 FIFO 未满时触发中断或 DMA
    config.rx_fifo_level = uart_rx_fifo_trg_not_empty;  //接收 FIFO 非空时触发中断或 DMA
    stat = uart_init(BOADR_DBG_UART_BASE, &config);     //HPM_UART0 初始化 UART 模块，使用 config 结构体配置 UART 属性
    if (stat != status_success) {               //如果初始化失败 
        printf("failed to initialize uart\n");
        while (1) {
        }
    }
  
    bsp_dma_set_uart_dbg_type(BOADR_DBG_UART_BASE);      //调试 UART 配置 DMA 模块
}


//连接摄像头的串口 串口7
static void bsp_uart_com_ma_init(void)
{
    uart_config_t config;       //uart初始化配置结构体
    hpm_stat_t stat;
    uint32_t freq = 0U;

    // GPIO初始化
    HPM_IOC->PAD[BOARD_COM_MA_UART_RX_GPIO_PIN].FUNC_CTL = BOARD_COM_MA_UART_RX_GPIO_FUNC;      //PA30 使用 HPM_IOC（引脚复用配置寄存器）设置 UART RX 和 TX 引脚功能
    HPM_IOC->PAD[BOARD_COM_MA_UART_TX_GPIO_PIN].FUNC_CTL = BOARD_COM_MA_UART_TX_GPIO_FUNC;      //PA31

    // 串口初始化
    clock_set_source_divider(BOARD_COM_MA_UART_CLK_NAME, clk_src_osc24m, 1);    //使用内部 osc24m 时钟源为 UART 配置时钟，分频系数为 1
    clock_add_to_group(BOARD_COM_MA_UART_CLK_NAME, 0);                          //将 UART 时钟添加到 0 时钟组（组编号为 0）
    freq = clock_get_frequency(BOARD_COM_MA_UART_CLK_NAME);                     //获取 UART 实际工作频率并存储到变量 freq 中

    // 串口默认设置
    config.baudrate = BOARD_COM_MA_UART_BAUDRATE;           //波特率为 115200UL UL
    config.word_length = word_length_8_bits;                //数据格式为 8位数据
    config.parity = parity_none;                            //无校验位
    config.num_of_stop_bits = stop_bits_1;                  //1 位停止位
    config.fifo_enable = true;                              //启用 FIFO，提高数据传输效率
    config.rx_fifo_level = uart_rx_fifo_trg_not_empty;      //接收 FIFO 非空时触发中断或 DMA
    config.tx_fifo_level = uart_tx_fifo_trg_not_full;       //发送 FIFO 未满时触发中断或 DMA
    config.dma_enable = false;                      //禁用DMA
    config.modem_config.auto_flow_ctrl_en = false;  //禁用硬件自动流量控制（RTS/CTS）
    config.modem_config.loop_back_en = false;       //禁用回环模式
    config.modem_config.set_rts_high = false;       //不设置 RTS 引脚为高电平

    config.rxidle_config.detect_enable = false;     //禁用 RX 空闲检测功能。
    config.rxidle_config.detect_irq_enable = false; //禁用 当线路空闲时触发中断或唤醒设备
    config.rxidle_config.idle_cond = uart_rxline_idle_cond_rxline_logic_one;        //表示 RX 线路保持高电平时检测空闲。这是 UART 线路默认的空闲逻辑（高电平表示空闲）
    config.rxidle_config.threshold = 10; /* 10-bit for typical UART configuration (8-N-1) */    //默认设置空闲检测阈值为 10

    config.txidle_config.detect_enable = false;       //禁用 TX 空闲检测功能。
    config.txidle_config.detect_irq_enable = false;   //禁用 当线路空闲时触发中断或唤醒设备
    config.txidle_config.idle_cond = uart_rxline_idle_cond_rxline_logic_one;        //表示 TX 线路保持高电平时检测空闲。这是 UART 线路默认的空闲逻辑（高电平表示空闲）
    config.txidle_config.threshold = 10; /* 10-bit for typical UART configuration (8-N-1) */    //默认设置空闲检测阈值为 10

    config.rx_enable = true;                        //启用接收功能，允许 UART 接收数据
    
    // 串口配置
    config.fifo_enable = true;      //启用 FIFO 缓存功能，提高数据传输性能
    config.dma_enable = true;       //启用 DMA 支持
    config.src_freq_in_hz = freq;   //串口频率
    config.tx_fifo_level = uart_tx_fifo_trg_not_full;   //发送 FIFO 未满时触发中断或 DMA
    config.rx_fifo_level = uart_rx_fifo_trg_not_empty;  //接收 FIFO 非空时触发中断或 DMA
    stat = uart_init(BOADR_COM_MA_UART_BASE, &config);  //HPM_UART7 初始化 UART 模块，使用 config 结构体配置 UART 属性
    if (stat != status_success) {                   //如果初始化失败 
        printf("failed to initialize uart\n");
        while (1) {
        }
    }
  
    bsp_dma_set_uart_com_ma_type(BOADR_COM_MA_UART_BASE);   //调试 UART 配置 DMA 模块
}


/* 飞控 串口 */
//连接 飞控 的串口 串口3
static void bsp_uart_com_fc_init(void)
{
    uart_config_t config;       //uart初始化配置结构体
    hpm_stat_t stat;
    uint32_t freq = 0U;

    // GPIO初始化
    HPM_IOC->PAD[BOARD_COM_FC_UART_RX_GPIO_PIN].FUNC_CTL = BOARD_COM_FC_UART_RX_GPIO_FUNC;      //PA30 使用 HPM_IOC（引脚复用配置寄存器）设置 UART RX 和 TX 引脚功能
    HPM_IOC->PAD[BOARD_COM_FC_UART_TX_GPIO_PIN].FUNC_CTL = BOARD_COM_FC_UART_TX_GPIO_FUNC;      //PA31

    // 串口初始化
    clock_set_source_divider(BOARD_COM_FC_UART_CLK_NAME, clk_src_osc24m, 1);    //使用内部 osc24m 时钟源为 UART 配置时钟，分频系数为 1
    clock_add_to_group(BOARD_COM_FC_UART_CLK_NAME, 0);                          //将 UART 时钟添加到 0 时钟组（组编号为 0）
    freq = clock_get_frequency(BOARD_COM_FC_UART_CLK_NAME);                     //获取 UART 实际工作频率并存储到变量 freq 中

    // 串口默认设置
    config.baudrate = BOARD_COM_FC_UART_BAUDRATE;           //波特率为 115200UL UL
    config.word_length = word_length_8_bits;                //数据格式为 8位数据
    config.parity = parity_none;                            //无校验位
    config.num_of_stop_bits = stop_bits_1;                  //1 位停止位
    config.fifo_enable = true;                              //启用 FIFO，提高数据传输效率
    config.rx_fifo_level = uart_rx_fifo_trg_not_empty;      //接收 FIFO 非空时触发中断或 DMA
    config.tx_fifo_level = uart_tx_fifo_trg_not_full;       //发送 FIFO 未满时触发中断或 DMA
    config.dma_enable = false;                      //禁用DMA
    config.modem_config.auto_flow_ctrl_en = false;  //禁用硬件自动流量控制（RTS/CTS）
    config.modem_config.loop_back_en = false;       //禁用回环模式
    config.modem_config.set_rts_high = false;       //不设置 RTS 引脚为高电平

    config.rxidle_config.detect_enable = false;     //禁用 RX 空闲检测功能。
    config.rxidle_config.detect_irq_enable = false; //禁用 当线路空闲时触发中断或唤醒设备
    config.rxidle_config.idle_cond = uart_rxline_idle_cond_rxline_logic_one;        //表示 RX 线路保持高电平时检测空闲。这是 UART 线路默认的空闲逻辑（高电平表示空闲）
    config.rxidle_config.threshold = 10; /* 10-bit for typical UART configuration (8-N-1) */    //默认设置空闲检测阈值为 10

    config.txidle_config.detect_enable = false;       //禁用 TX 空闲检测功能。
    config.txidle_config.detect_irq_enable = false;   //禁用 当线路空闲时触发中断或唤醒设备
    config.txidle_config.idle_cond = uart_rxline_idle_cond_rxline_logic_one;        //表示 TX 线路保持高电平时检测空闲。这是 UART 线路默认的空闲逻辑（高电平表示空闲）
    config.txidle_config.threshold = 10; /* 10-bit for typical UART configuration (8-N-1) */    //默认设置空闲检测阈值为 10

    config.rx_enable = true;                        //启用接收功能，允许 UART 接收数据
    
    // 串口配置
    config.fifo_enable = true;      //启用 FIFO 缓存功能，提高数据传输性能
    config.dma_enable = true;       //启用 DMA 支持
    config.src_freq_in_hz = freq;   //串口频率
    config.tx_fifo_level = uart_tx_fifo_trg_not_full;   //发送 FIFO 未满时触发中断或 DMA
    config.rx_fifo_level = uart_rx_fifo_trg_not_empty;  //接收 FIFO 非空时触发中断或 DMA
    stat = uart_init(BOADR_COM_FC_UART_BASE, &config);  //HPM_UART7 初始化 UART 模块，使用 config 结构体配置 UART 属性
    if (stat != status_success) {                   //如果初始化失败 
        printf("failed to initialize uart\n");
        while (1) {
        }
    }
  
    bsp_dma_set_uart_com_fc_type(BOADR_COM_FC_UART_BASE);   //调试 UART 配置 DMA 模块
}







//调度和启动 UART 的 DMA 传输，确保按队列中数据顺序发送数据 
//发送 tx_pbuf 里面的数据 
//spbuf = bsp_uart_dbg_get_tx_pbuf();先调用 赋值需要发送的数据 
//bsp_uart_dbg_set_tx_size(pbuf_len);设置长度
void bsp_uart_dbg_schedule_transmit(void)
{
    uint8_t* dma_pbuf;


    if (uart_dbg_tx_dma_done)       //上一次 DMA 传输是否完成
    {
        dma_pbuf = bsp_dma_get_uart_dbg_tx_pbuf();  //获取 DMA 的目标缓冲区指针  uart_dbg_tx_dma_buf[256]

        //memcpy(bsp_dma_get_uart_tx_pbuf(), uart_dbg_info.tx_pbuf[uart_dbg_info.tx_index], 
        //      uart_dbg_info.tx_buf_len[uart_dbg_info.tx_index]);
        for(uint16_t i = 0; i < uart_dbg_info.tx_buf_len[uart_dbg_info.tx_index]; i++)  //从 UART 发送队列（uart_dbg_info.tx_pbuf）中取出当前索引的数据，并逐字节复制到 DMA 缓冲区。
        {
            dma_pbuf[i] = uart_dbg_info.tx_pbuf[uart_dbg_info.tx_index][i];             //把数据传输到DMA缓冲区 uart_dbg_info.tx_pbuf[i] = &uart_dbg_tx_buf[i][0];  uart_dbg_tx_buf[3][256]
        }
        
        uart_debug_tx_trigger_dma(BOARD_APP_HDMA, uart_dbg_info.tx_buf_len[uart_dbg_info.tx_index]);    //启动 DMA 传输

        if (++uart_dbg_info.tx_index >= UART_TX_MESSAGE_QUEUES_NUM)         //3
        {
            uart_dbg_info.tx_index = 0;
        }

        uart_dbg_tx_dma_done = false;       //上一次 DMA 传输标志清0
    }
}

//调度和启动 MA摄像头 的 DMA 传输，确保按队列中数据顺序发送数据
void bsp_uart_com_ma_schedule_transmit(void)
{
    uint8_t* dma_pbuf;          //dma的缓冲区


    if (uart_com_ma_tx_dma_done)        //上一次 DMA 传输是否完成
    {
        dma_pbuf = bsp_dma_get_uart_com_ma_tx_pbuf();       //获取 DMA 的目标缓冲区指针  uart_com_ma_tx_dma_buf[256]

        //memcpy(bsp_dma_get_uart_tx_pbuf(), uart_com_ma_info.tx_pbuf[uart_com_ma_info.tx_index], 
        //      uart_com_ma_info.tx_buf_len[uart_com_ma_info.tx_index]);
        for(uint16_t i = 0; i < uart_com_ma_info.tx_buf_len[uart_com_ma_info.tx_index]; i++)    //从 UART 发送队列（uart_com_ma_info.tx_pbuf）中取出当前索引的数据，并逐字节复制到 DMA 缓冲区。
        {
            dma_pbuf[i] = uart_com_ma_info.tx_pbuf[uart_com_ma_info.tx_index][i];           //uart_com_ma_info.tx_pbuf[i] = &uart_com_ma_tx_buf[i][0]; uart_com_ma_tx_buf[3][256]
        }
        
        uart_com_ma_tx_trigger_dma(BOARD_APP_DMAMUX, uart_com_ma_info.tx_buf_len[uart_com_ma_info.tx_index]);    //!!!!!!
        //uart_debug_tx_trigger_dma(BOARD_APP_HDMA, uart_com_ma_info.tx_buf_len[uart_com_ma_info.tx_index]);//启动 DMA 传输

        if (++uart_com_ma_info.tx_index >= UART_TX_MESSAGE_QUEUES_NUM)  //3 
        {
            uart_com_ma_info.tx_index = 0;  
        }

        uart_com_ma_tx_dma_done = false;        //上一次 DMA 传输标志清0
    }
}

/* 飞控发送函数 */
//调度和启动 飞控 的 DMA 传输，确保按队列中数据顺序发送数据
void bsp_uart_com_fc_schedule_transmit(void)
{
    uint8_t* dma_pbuf;          //dma的缓冲区


    if (uart_com_fc_tx_dma_done)        //上一次 DMA 传输是否完成
    {
        dma_pbuf = bsp_dma_get_uart_com_fc_tx_pbuf();       //获取 DMA 的目标缓冲区指针  uart_com_ma_tx_dma_buf[256]

        //memcpy(bsp_dma_get_uart_tx_pbuf(), uart_com_ma_info.tx_pbuf[uart_com_ma_info.tx_index], 
        //      uart_com_ma_info.tx_buf_len[uart_com_ma_info.tx_index]);
        for(uint16_t i = 0; i < uart_com_fc_info.tx_buf_len[uart_com_fc_info.tx_index]; i++)    //从 UART 发送队列（uart_com_ma_info.tx_pbuf）中取出当前索引的数据，并逐字节复制到 DMA 缓冲区。
        {
            dma_pbuf[i] = uart_com_fc_info.tx_pbuf[uart_com_fc_info.tx_index][i];           //uart_com_ma_info.tx_pbuf[i] = &uart_com_ma_tx_buf[i][0]; uart_com_ma_tx_buf[3][256]
        }
        
        uart_com_fc_tx_trigger_dma(BOARD_APP_DMAMUX, uart_com_fc_info.tx_buf_len[uart_com_fc_info.tx_index]);    //!!!!!!
        //uart_debug_tx_trigger_dma(BOARD_APP_HDMA, uart_com_ma_info.tx_buf_len[uart_com_ma_info.tx_index]);//启动 DMA 传输

        if (++uart_com_fc_info.tx_index >= UART_TX_MESSAGE_QUEUES_NUM)  //3 
        {
            uart_com_fc_info.tx_index = 0;  
        }

        uart_com_fc_tx_dma_done = false;        //上一次 DMA 传输标志清0
    }
}




//获的 uart_dbg_info.tx_pbuf
uint8_t* bsp_uart_dbg_get_tx_pbuf(void)
{
    return uart_dbg_info.tx_pbuf[uart_dbg_info.tx_queue_index];
}

//获的 uart_com_ma_info.tx_pbuf
uint8_t* bsp_uart_com_ma_get_tx_pbuf(void)
{
    return uart_com_ma_info.tx_pbuf[uart_com_ma_info.tx_queue_index];
}

/*获的 uart_com_fc_info.tx_pbuf*/
uint8_t* bsp_uart_com_fc_get_tx_pbuf(void)
{
    return uart_com_fc_info.tx_pbuf[uart_com_fc_info.tx_queue_index];
}




void bsp_uart_dbg_set_tx_size(uint16_t len)
{
    uart_dbg_info.tx_buf_len[uart_dbg_info.tx_queue_index] = len;

    if (++uart_dbg_info.tx_queue_index >= UART_TX_MESSAGE_QUEUES_NUM) 
    {
        uart_dbg_info.tx_queue_index = 0;
    }
}

void bsp_uart_com_ma_set_tx_size(uint16_t len)
{
    uart_com_ma_info.tx_buf_len[uart_com_ma_info.tx_queue_index] = len;

    if (++uart_com_ma_info.tx_queue_index >= UART_TX_MESSAGE_QUEUES_NUM) 
    {
        uart_com_ma_info.tx_queue_index = 0;
    }
}

/*飞控*/
void bsp_uart_com_fc_set_tx_size(uint16_t len)
{
    uart_com_fc_info.tx_buf_len[uart_com_fc_info.tx_queue_index] = len;

    if (++uart_com_fc_info.tx_queue_index >= UART_TX_MESSAGE_QUEUES_NUM) 
    {
        uart_com_fc_info.tx_queue_index = 0;
    }
}


