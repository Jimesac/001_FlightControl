#include "bsp_dma.h"
#include "user_config.h"
#include <stdio.h>
#include "Flightcontrol_meg.h"

bool uart_dbg_tx_dma_done = false;      //UART 调试（dbg）的发送是否完成
bool uart_dbg_rx_dma_done = false;      //UART 调试（dbg）的接收是否完成
bool uart_com_ma_tx_dma_done = false;   //通信模块（com_ma）的 发送是否完成
bool uart_com_ma_rx_dma_done = false;   //通信模块（com_ma）的 接收是否完成

bool uart_com_fc_tx_dma_done = false;   //飞控模块（com_fc）发送是否完成
bool uart_com_fc_rx_dma_done = false;   //飞控模块（com_fc）的 接收是否完成

UART_Type* uart_dbg_type = (void *) 0;  // UART 硬件资源的指针变量 debug
UART_Type* uart_com_ma_type = (void *) 0;   // UART 硬件资源的指针变量 MA 摄像头
UART_Type* uart_com_fc_type = (void *) 0;   // UART 硬件资源的指针变量 MA 摄像头

ATTR_PLACE_AT_NONCACHEABLE_BSS_WITH_ALIGNMENT(4) uint8_t uart_dbg_rx_dma_buf[BOARD_DBG_UART_RX_DMA_BUF_SIZE] = {0}; //512U 接收缓冲区,分配在非缓存区（NONCACHEABLE_BSS）内存
ATTR_PLACE_AT_NONCACHEABLE_BSS_WITH_ALIGNMENT(4) uint8_t uart_dbg_tx_dma_buf[BOARD_DBG_UART_TX_DMA_BUF_SIZE] = {0}; //256  发送缓冲区
ATTR_PLACE_AT_NONCACHEABLE_BSS_WITH_ALIGNMENT(4) uint8_t uart_com_ma_rx_dma_buf[BOARD_COM_MA_UART_RX_DMA_BUF_SIZE] = {0};   //576U 接收缓冲区
ATTR_PLACE_AT_NONCACHEABLE_BSS_WITH_ALIGNMENT(4) uint8_t uart_com_ma_tx_dma_buf[BOARD_COM_MA_UART_TX_DMA_BUF_SIZE] = {0};   //256U 发送缓冲区
ATTR_PLACE_AT_NONCACHEABLE_BSS_WITH_ALIGNMENT(4) uint8_t uart_com_fc_rx_dma_buf[BOARD_COM_FC_UART_RX_DMA_BUF_SIZE] = {0};   //512U 接收缓冲区
ATTR_PLACE_AT_NONCACHEABLE_BSS_WITH_ALIGNMENT(4) uint8_t uart_com_fc_tx_dma_buf[BOARD_COM_FC_UART_TX_DMA_BUF_SIZE] = {0};   //212U 发送缓冲区

ATTR_PLACE_AT_NONCACHEABLE_BSS_WITH_ALIGNMENT(8) dma_linked_descriptor_t uart_dbg_rx_descriptors[2];        //定义一个 DMA 链接描述符 数组，用于配置调试 UART 的接收 DMA 链接
ATTR_PLACE_AT_NONCACHEABLE_BSS_WITH_ALIGNMENT(8) dma_linked_descriptor_t uart_com_ma_rx_descriptors[2];        //定义一个 DMA 链接描述符 数组，用于配置调试 UART 的接收 DMA 链接
ATTR_PLACE_AT_NONCACHEABLE_BSS_WITH_ALIGNMENT(8) dma_linked_descriptor_t uart_com_fc_rx_descriptors[2];        //定义一个 DMA 链接描述符 数组，用于配置调试 UART 的接收 DMA 链接

//dma 初始化
void bsp_dma_init(void)
{
    if (uart_dbg_type == (void *) 0 || uart_com_ma_type == (void *) 0)  //dma两个dma都没初始化，uart初始化没设置
    {
        printf("failed to initialize dma\n"); //
        while (1) { 
        }
    }
    uart_debug_rx_circle_dma(BOARD_APP_HDMA, (uint32_t)(&uart_dbg_rx_dma_buf[0]), BOARD_DBG_UART_RX_DMA_BUF_SIZE);       //配置 uart_debug_rx 环形接收 DMA
    //uart_debug_rx_circle_dma(BOARD_APP_HDMA, (uint32_t)(&uart_com_ma_rx_dma_buf[0]), BOARD_COM_MA_UART_RX_DMA_BUF_SIZE);
    uart_com_ma_rx_circle_dma(BOARD_APP_HDMA, (uint32_t)(&uart_com_ma_rx_dma_buf[0]), BOARD_COM_MA_UART_RX_DMA_BUF_SIZE); // !!!!!!!
    uart_com_fc_rx_circle_dma(BOARD_APP_HDMA, (uint32_t)(&uart_com_fc_rx_dma_buf[0]), BOARD_COM_FC_UART_RX_DMA_BUF_SIZE); // 飞控
    

    intc_m_enable_irq_with_priority(BOARD_HDMA_IRQ, BOARD_HDMA_IRQ_LEVEL);      //启用 DMA 中断，并设置优先级为6
    //配置 DMA 多路复用器（DMAMUX）以连接 DMA 控制器与硬件外设（UART）
    //BOARD_APP_DMAMUX： 多路复用器的句柄 BOARD_DBG_UART_RX_DMAMUX_CHN： DMAMUX 通道编号，用于调试 UART 接收
    //BOARD_DBG_UART_RX_DMA_REQ： 接收 DMA 请求信号来源
    //true： 启用对应的 DMA 通道
    dmamux_config(BOARD_APP_DMAMUX, BOARD_DBG_UART_RX_DMAMUX_CHN, BOARD_DBG_UART_RX_DMA_REQ, true);
    dmamux_config(BOARD_APP_DMAMUX, BOARD_DBG_UART_TX_DMAMUX_CHN, BOARD_DBG_UART_TX_DMA_REQ, true);
    dmamux_config(BOARD_APP_DMAMUX, BOARD_COM_MA_UART_RX_DMAMUX_CHN, BOARD_COM_MA_UART_RX_DMA_REQ, true);
    dmamux_config(BOARD_APP_DMAMUX, BOARD_COM_MA_UART_TX_DMAMUX_CHN, BOARD_COM_MA_UART_TX_DMA_REQ, true);

    dmamux_config(BOARD_APP_DMAMUX, BOARD_COM_FC_UART_RX_DMAMUX_CHN, BOARD_COM_FC_UART_RX_DMA_REQ, true);   //飞控
    dmamux_config(BOARD_APP_DMAMUX, BOARD_COM_FC_UART_TX_DMAMUX_CHN, BOARD_COM_FC_UART_TX_DMA_REQ, true);

    uart_dbg_tx_dma_done = true;        //调试 UART 的发送 DMA 当前初始化完成     用于在发送中断
    uart_dbg_rx_dma_done = false;       //调试 UART 的接收 DMA 当前初始化未完成
    uart_com_ma_tx_dma_done = true;     //摄像头 UART 的发送 DMA 当前初始化未完成
    uart_com_ma_rx_dma_done = false;    //摄像头 UART 的接收 DMA 当前初始化未完成

    uart_com_fc_tx_dma_done = true;     //飞控
    uart_com_fc_rx_dma_done = false;
}

//处理 UART 的调试 DMA 任务，包括接收和发送两个方向的中断
void bsp_dma_isr(void)
{
    volatile hpm_stat_t stat_uart_dbg_rx_chn, stat_uart_dbg_rx_tx_chn;  //
    volatile hpm_stat_t stat_uart_com_ma_rx_chn, stat_uart_com_ma_tx_chn;  //
    volatile hpm_stat_t stat_uart_com_fc_rx_chn, stat_uart_com_fc_tx_chn;  //

    //串口0 的中断   接收
    stat_uart_dbg_rx_chn = dma_check_transfer_status(BOARD_APP_HDMA, BOARD_DBG_UART_RX_DMA_CHN);    // 获取调试 UART 接收 DMA 通道(1)的传输状态
    if (stat_uart_dbg_rx_chn & DMA_CHANNEL_STATUS_TC) {                 //判断通道状态是否为传输完成
        dma_clear_transfer_status(BOARD_APP_HDMA, BOARD_DBG_UART_RX_DMA_CHN);   //清除当前 DMA 传输完成的状态标志
        uart_dbg_rx_dma_done = true;                                    //接收 DMA 任务已完成，应用程序可以处理接收到的数据 要置uart_dbg_rx_dma_done=false
        //调用接收数据处理函数
    }

    stat_uart_dbg_rx_tx_chn = dma_check_transfer_status(BOARD_APP_HDMA, BOARD_DBG_UART_TX_DMA_CHN); // 获取调试 UART 接收 DMA 通道(0)的传输状态
    if (stat_uart_dbg_rx_tx_chn & DMA_CHANNEL_STATUS_TC) {              //判断通道状态是否为传输完成
        dma_clear_transfer_status(BOARD_APP_HDMA, BOARD_DBG_UART_TX_DMA_CHN);   //清除当前 DMA 传输完成的状态标志
        uart_dbg_tx_dma_done = true;                                     //接收 DMA 任务已完成，应用程序可以处理接收到的数据
    }


    //串口7 的中断  摄像头 接收
    stat_uart_com_ma_rx_chn = dma_check_transfer_status(BOARD_APP_HDMA, BOARD_COM_MA_UART_RX_DMA_CHN);    // 获取调试 UART 接收 DMA 通道(1)的传输状态
    if (stat_uart_com_ma_rx_chn & DMA_CHANNEL_STATUS_TC) {                 //判断通道状态是否为传输完成
        dma_clear_transfer_status(BOARD_APP_HDMA, BOARD_COM_MA_UART_RX_DMA_CHN);   //清除当前 DMA 传输完成的状态标志
        uart_com_ma_rx_dma_done = true;                                    //接收 DMA 任务已完成，应用程序可以处理接收到的数据
        //调用接收数据处理函数
    }

    stat_uart_com_ma_tx_chn = dma_check_transfer_status(BOARD_APP_HDMA, BOARD_COM_MA_UART_TX_DMA_CHN); // 获取调试 UART 接收 DMA 通道(0)的传输状态
    if (stat_uart_com_ma_tx_chn & DMA_CHANNEL_STATUS_TC) {              //判断通道状态是否为传输完成
        dma_clear_transfer_status(BOARD_APP_HDMA, BOARD_COM_MA_UART_TX_DMA_CHN);   //清除当前 DMA 传输完成的状态标志
        uart_com_ma_tx_dma_done = true;                                     //接收 DMA 任务已完成，应用程序可以处理接收到的数据
    }

    //串口3 的中断  飞控通信中断 接收
    stat_uart_com_fc_rx_chn = dma_check_transfer_status(BOARD_APP_HDMA, BOARD_COM_FC_UART_RX_DMA_CHN);    // 获取调试 UART 接收 DMA 通道(1)的传输状态
    if (stat_uart_com_fc_rx_chn & DMA_CHANNEL_STATUS_TC) {                 //判断通道状态是否为传输完成
        dma_clear_transfer_status(BOARD_APP_HDMA, BOARD_COM_FC_UART_RX_DMA_CHN);   //清除当前 DMA 传输完成的状态标志
        uart_com_fc_rx_dma_done = true;                                    //接收 DMA 任务已完成，应用程序可以处理接收到的数据
        //调用接收数据处理函数
        mavlink_receive_message(0,uart_com_fc_rx_dma_buf,MAVLINK_MAX_PACKET_LEN); //利用通道0
        uart_com_fc_rx_dma_done = false;
    }

    stat_uart_com_fc_tx_chn = dma_check_transfer_status(BOARD_APP_HDMA, BOARD_COM_FC_UART_TX_DMA_CHN); // 获取调试 UART 接收 DMA 通道(0)的传输状态
    if (stat_uart_com_fc_tx_chn & DMA_CHANNEL_STATUS_TC) {              //判断通道状态是否为传输完成
        dma_clear_transfer_status(BOARD_APP_HDMA, BOARD_COM_FC_UART_TX_DMA_CHN);   //清除当前 DMA 传输完成的状态标志
        uart_com_fc_tx_dma_done = true;                                     //接收 DMA 任务已完成，应用程序可以处理接收到的数据
    }
}
SDK_DECLARE_EXT_ISR_M(BOARD_HDMA_IRQ, bsp_dma_isr)                      //将中断服务函数 bsp_dma_isr 注册到指定的中断号 BOARD_HDMA_IRQ

//配置并启动调试 UART 的发送 DMA 传输任务 使用 DMA 将数据从内存缓冲区传输到 UART 发送寄存器
hpm_stat_t uart_debug_tx_trigger_dma(DMA_Type *dma_ptr, uint32_t size)
{
    dma_handshake_config_t config;  //DMA 配置结构体

    dma_default_handshake_config(dma_ptr, &config); //初始化配置结构体为默认值
    config.ch_index = BOARD_DBG_UART_TX_DMA_CHN;    //0 指定 DMA 通道索引号
    config.dst = (uint32_t)&uart_dbg_type->THR;     //设置目标地址为 UART 发送保持寄存器（THR），即 DMA 传输数据的目标是 UART 的发送缓冲区
    config.dst_fixed = true;                        // 指定目标地址是否固定
    config.src = (uint32_t)uart_dbg_tx_dma_buf;     //设置源地址为 uart_dbg_tx_dma_buf，即 DMA 传输数据的起始位置是发送缓冲区
    config.src_fixed = false;                       //指定源地址是否固定  false：源地址递增，适合内存数组的场景
    config.data_width = DMA_TRANSFER_WIDTH_BYTE;    //指定传输数据的单位宽度为 1 字节（BYTE），符合 UART 字节流的传输特性
    config.size_in_byte = size;                     //指定总传输数据的大小，单位为字节，由 size 参数传入

    return dma_setup_handshake(dma_ptr, &config, true); //完成 DMA 配置并启动传输
}

//配置并启动MA 摄像头 UART 的发送 DMA 传输任务 使用 DMA 将数据从内存缓冲区传输到 UART 发送寄存器
hpm_stat_t uart_com_ma_tx_trigger_dma(DMA_Type *dma_ptr, uint32_t size)
{
    dma_handshake_config_t config;                  //DMA 配置结构体

    dma_default_handshake_config(dma_ptr, &config); //初始化配置结构体为默认值
    config.ch_index = BOARD_COM_MA_UART_TX_DMA_CHN; //2 指定 DMA 通道索引号
    config.dst = (uint32_t)&uart_com_ma_type->THR;  //设置目标地址为 UART 发送保持寄存器（THR），即 DMA 传输数据的目标是 UART 的发送缓冲区
    config.dst_fixed = true;                        // 指定目标地址是否固定
    config.src = (uint32_t)uart_com_ma_tx_dma_buf;  //设置源地址为 uart_com_ma_tx_dma_buf ，即 DMA 传输数据的起始位置是发送缓冲区
    config.src_fixed = false;                       //指定源地址是否固定  false：源地址递增，适合内存数组的场景
    config.data_width = DMA_TRANSFER_WIDTH_BYTE;    //指定传输数据的单位宽度为 1 字节（BYTE），符合 UART 字节流的传输特性
    config.size_in_byte = size;                     //指定总传输数据的大小，单位为字节，由 size 参数传入

    return dma_setup_handshake(dma_ptr, &config, true); //完成 DMA 配置并启动传输
}

//飞控
//配置并启动飞控 UART 的发送 DMA 传输任务 使用 DMA 将数据从内存缓冲区传输到 UART 发送寄存器
hpm_stat_t uart_com_fc_tx_trigger_dma(DMA_Type *dma_ptr, uint32_t size)
{
    dma_handshake_config_t config;                  //DMA 配置结构体

    dma_default_handshake_config(dma_ptr, &config); //初始化配置结构体为默认值
    config.ch_index = BOARD_COM_FC_UART_TX_DMA_CHN; //2 指定 DMA 通道索引号
    config.dst = (uint32_t)&uart_com_fc_type->THR;  //设置目标地址为 UART 发送保持寄存器（THR），即 DMA 传输数据的目标是 UART 的发送缓冲区
    config.dst_fixed = true;                        // 指定目标地址是否固定
    config.src = (uint32_t)uart_com_fc_tx_dma_buf;  //设置源地址为 uart_com_ma_tx_dma_buf ，即 DMA 传输数据的起始位置是发送缓冲区
    config.src_fixed = false;                       //指定源地址是否固定  false：源地址递增，适合内存数组的场景
    config.data_width = DMA_TRANSFER_WIDTH_BYTE;    //指定传输数据的单位宽度为 1 字节（BYTE），符合 UART 字节流的传输特性
    config.size_in_byte = size;                     //指定总传输数据的大小，单位为字节，由 size 参数传入

    return dma_setup_handshake(dma_ptr, &config, true); //完成 DMA 配置并启动传输
}



//配置并启动调试 UART 的接收 DMA 传输任务 使用 DMA 将数据从内存缓冲区传输到 UART 发送寄存器
//dma_ptr：指向 DMA 控制器的指针。
//dst：数据传输目标地址（内存缓冲区起始地址）。
//size：需要传输的数据大小，单位为字节
hpm_stat_t uart_debug_rx_trigger_dma(DMA_Type *dma_ptr, uint32_t dst, uint32_t size)
{
    dma_handshake_config_t config;                  //DMA 配置结构体              

    dma_default_handshake_config(dma_ptr, &config); //初始化配置结构体为默认值
    config.ch_index = BOARD_DBG_UART_RX_DMA_CHN;    //1 指定 DMA 通道索引号
    config.dst = dst;                               //设置目标地址为接收缓冲区的起始地址
    config.dst_fixed = false;                       // 指定目标地址是否固定
    config.src = (uint32_t)&uart_dbg_type->RBR;     //设置源地址为 UART 的接收缓冲寄存器（RBR），即 DMA 传输数据的来源
    config.src_fixed = true;                        //指定源地址是否固定：true：源地址固定，适合外设寄存器的场景。
    config.data_width = DMA_TRANSFER_WIDTH_BYTE;    //设置数据宽度为 1 字节，符合 UART 字节流的特性
    config.size_in_byte = size;                     //指定总传输数据的大小，单位为字节，由 size 参数传入

    return dma_setup_handshake(dma_ptr, &config, true); //完成 DMA 配置并启动传输
}

//配置并启动MA 摄像头 UART 的接收 DMA 传输任务 使用 DMA 将数据从内存缓冲区传输到 UART 发送寄存器
hpm_stat_t uart_com_ma_rx_trigger_dma(DMA_Type *dma_ptr, uint32_t dst, uint32_t size)
{
    dma_handshake_config_t config;                  //DMA 配置结构体              

    dma_default_handshake_config(dma_ptr, &config); //初始化配置结构体为默认值
    config.ch_index = BOARD_COM_MA_UART_RX_DMA_CHN; //1 指定 DMA 通道索引号
    config.dst = dst;                               //设置目标地址为接收缓冲区的起始地址
    config.dst_fixed = false;                       // 指定目标地址是否固定
    config.src = (uint32_t)&uart_com_ma_type->RBR;  //设置源地址为 UART 的接收缓冲寄存器（RBR），即 DMA 传输数据的来源
    config.src_fixed = true;                        //指定源地址是否固定：true：源地址固定，适合外设寄存器的场景。
    config.data_width = DMA_TRANSFER_WIDTH_BYTE;    //设置数据宽度为 1 字节，符合 UART 字节流的特性
    config.size_in_byte = size;                     //指定总传输数据的大小，单位为字节，由 size 参数传入

    return dma_setup_handshake(dma_ptr, &config, true); //完成 DMA 配置并启动传输
}

//飞控
//配置并启动 飞控 UART 的接收 DMA 传输任务 使用 DMA 将数据从内存缓冲区传输到 UART 发送寄存器
hpm_stat_t uart_com_fc_rx_trigger_dma(DMA_Type *dma_ptr, uint32_t dst, uint32_t size)
{
    dma_handshake_config_t config;                  //DMA 配置结构体              

    dma_default_handshake_config(dma_ptr, &config); //初始化配置结构体为默认值
    config.ch_index = BOARD_COM_FC_UART_RX_DMA_CHN; //1 指定 DMA 通道索引号
    config.dst = dst;                               //设置目标地址为接收缓冲区的起始地址
    config.dst_fixed = false;                       // 指定目标地址是否固定
    config.src = (uint32_t)&uart_com_fc_type->RBR;  //设置源地址为 UART 的接收缓冲寄存器（RBR），即 DMA 传输数据的来源
    config.src_fixed = true;                        //指定源地址是否固定：true：源地址固定，适合外设寄存器的场景。
    config.data_width = DMA_TRANSFER_WIDTH_BYTE;    //设置数据宽度为 1 字节，符合 UART 字节流的特性
    config.size_in_byte = size;                     //指定总传输数据的大小，单位为字节，由 size 参数传入

    return dma_setup_handshake(dma_ptr, &config, true); //完成 DMA 配置并启动传输
}




//配置 UART 调试接收的循环 DMA（Ring Buffer）功能  使用链式描述符实现循环 DMA，从 UART 接收寄存器读取数据并存储到目标内存缓冲区
//dma_ptr：指向 DMA 控制器的指针。dst：目标内存缓冲区起始地址。size：接收缓冲区的大小（字节）
hpm_stat_t uart_debug_rx_circle_dma(DMA_Type *dma_ptr, uint32_t dst, uint32_t size)
{
    hpm_stat_t stat;
    dma_channel_config_t config = {0};  //声明并清零  config 结构体

    /* 1.1 config chain descriptors */
    dma_default_channel_config(dma_ptr, &config);       //填充默认配置
    config.src_addr = (uint32_t)&uart_dbg_type->RBR;    //指定源地址为 UART 的接收寄存器（RBR）
    config.src_width = DMA_TRANSFER_WIDTH_BYTE; /*  In DMA handshake case, source width and destination width must be BYTE. */  //设置源数据宽度为 0 字节
    config.src_addr_ctrl = DMA_ADDRESS_CONTROL_FIXED;   //设置源地址为固定，适用于外设寄存器
    config.src_mode = DMA_HANDSHAKE_MODE_HANDSHAKE;     //设置为握手模式，与 UART 外设协同工作
    config.dst_addr = core_local_mem_to_sys_address(HPM_CORE0, (uint32_t)dst);  //设置目标地址为接收缓冲区（转换为系统地址）
    config.dst_width = DMA_TRANSFER_WIDTH_BYTE; /*  In DMA handshake case, source width and destination width must be BYTE. */  //目标数据宽度
    config.dst_addr_ctrl = DMA_ADDRESS_CONTROL_INCREMENT;   //目标地址递增，适合将数据存储到连续的缓冲区
    config.dst_mode = DMA_HANDSHAKE_MODE_NORMAL;            //设置为普通模式
    config.size_in_byte = size;                             //指定缓冲区大小
    config.src_burst_size = DMA_NUM_TRANSFER_PER_BURST_1T; /*  In DMA handshake case, source burst size must be 1 transfer, that is 0. */   //设置源突发传输大小为 1（符合握手模式）
    config.linked_ptr = core_local_mem_to_sys_address(HPM_CORE0, (uint32_t)&uart_dbg_rx_descriptors[1]);    //链式描述符的下一个地址
    stat = dma_config_linked_descriptor(dma_ptr, &uart_dbg_rx_descriptors[0], BOARD_DBG_UART_RX_DMA_CHN, &config);  //配置 DMA 链式描述符 第一次链式描述符连接到 uart_dbg_rx_descriptors[1]
    if (stat != status_success) {
        while (1) {
        };
    }

    config.linked_ptr = core_local_mem_to_sys_address(HPM_CORE0, (uint32_t)&uart_dbg_rx_descriptors[0]);
    stat = dma_config_linked_descriptor(dma_ptr, &uart_dbg_rx_descriptors[1], BOARD_DBG_UART_RX_DMA_CHN, &config);
    if (stat != status_success) {
        while (1) {
        };
    }

    /* 1.2 config rx dma */
    config.linked_ptr = core_local_mem_to_sys_address(HPM_CORE0, (uint32_t)&uart_dbg_rx_descriptors[0]);
    stat = dma_setup_channel(dma_ptr, BOARD_DBG_UART_RX_DMA_CHN, &config, true);    //启动 DMA 通道
    if (stat != status_success) {
        while (1) {
        };
    }
}

//配置 UART 调试接收的循环 DMA（Ring Buffer）功能  使用链式描述符实现循环 DMA，从 UART 接收寄存器读取数据并存储到目标内存缓冲区
//dma_ptr：指向 DMA 控制器的指针。dst：目标内存缓冲区起始地址。size：接收缓冲区的大小（字节）
hpm_stat_t uart_com_ma_rx_circle_dma(DMA_Type *dma_ptr, uint32_t dst, uint32_t size)
{
    hpm_stat_t stat;                    //
    dma_channel_config_t config = {0};  //声明并清零  config 结构体

    /* 1.1 config chain descriptors */
    dma_default_channel_config(dma_ptr, &config);           //填充默认配置
    config.src_addr = (uint32_t)&uart_com_ma_type->RBR;     //指定源地址为 MA UART 的接收寄存器（RBR）
    config.src_width = DMA_TRANSFER_WIDTH_BYTE; /*  In DMA handshake case, source width and destination width must be BYTE. */  //设置源数据宽度为 0 字节
    config.src_addr_ctrl = DMA_ADDRESS_CONTROL_FIXED;       //设置源地址为固定，适用于外设寄存器
    config.src_mode = DMA_HANDSHAKE_MODE_HANDSHAKE;         //设置为握手模式，与 UART 外设协同工作
    config.dst_addr = core_local_mem_to_sys_address(HPM_CORE0, (uint32_t)dst);      //设置目标地址为接收缓冲区（转换为系统地址）
    config.dst_width = DMA_TRANSFER_WIDTH_BYTE; /*  In DMA handshake case, source width and destination width must be BYTE. */ //设置目标地址为接收缓冲区（转换为系统地址） 
    config.dst_addr_ctrl = DMA_ADDRESS_CONTROL_INCREMENT;   //目标地址递增，适合将数据存储到连续的缓冲区
    config.dst_mode = DMA_HANDSHAKE_MODE_NORMAL;            //设置为普通模式
    config.size_in_byte = size;                             //指定缓冲区大小
    config.src_burst_size = DMA_NUM_TRANSFER_PER_BURST_1T; /*  In DMA handshake case, source burst size must be 1 transfer, that is 0. */ //设置源突发传输大小为 1（符合握手模式）
    config.linked_ptr = core_local_mem_to_sys_address(HPM_CORE0, (uint32_t)&uart_com_ma_rx_descriptors[1]);    //链式描述符的下一个地址 uart_dbg_rx_descriptors !!!!!!!!!
    stat = dma_config_linked_descriptor(dma_ptr, &uart_com_ma_rx_descriptors[0], BOARD_COM_MA_UART_RX_DMA_CHN, &config);    //!
    if (stat != status_success) {
        while (1) {
        };
    }

    config.linked_ptr = core_local_mem_to_sys_address(HPM_CORE0, (uint32_t)&uart_com_ma_rx_descriptors[0]); //!
    stat = dma_config_linked_descriptor(dma_ptr, &uart_com_ma_rx_descriptors[1], BOARD_COM_MA_UART_RX_DMA_CHN, &config);    //!
    if (stat != status_success) {
        while (1) {
        };
    }

    /* 1.2 config rx dma */
    config.linked_ptr = core_local_mem_to_sys_address(HPM_CORE0, (uint32_t)&uart_com_ma_rx_descriptors[0]); //!
    stat = dma_setup_channel(dma_ptr, BOARD_COM_MA_UART_RX_DMA_CHN, &config, true);
    if (stat != status_success) {                           //启动 DMA 通道
        while (1) {
        };
    }
}

//飞控
//配置 UART 调试接收的循环 DMA（Ring Buffer）功能  使用链式描述符实现循环 DMA，从 UART 接收寄存器读取数据并存储到目标内存缓冲区
//dma_ptr：指向 DMA 控制器的指针。dst：目标内存缓冲区起始地址。size：接收缓冲区的大小（字节）
hpm_stat_t uart_com_fc_rx_circle_dma(DMA_Type *dma_ptr, uint32_t dst, uint32_t size)
{
    hpm_stat_t stat;                    //
    dma_channel_config_t config = {0};  //声明并清零  config 结构体

    /* 1.1 config chain descriptors */
    dma_default_channel_config(dma_ptr, &config);           //填充默认配置
    config.src_addr = (uint32_t)&uart_com_fc_type->RBR;     //指定源地址为 MA UART 的接收寄存器（RBR）
    config.src_width = DMA_TRANSFER_WIDTH_BYTE; /*  In DMA handshake case, source width and destination width must be BYTE. */  //设置源数据宽度为 0 字节
    config.src_addr_ctrl = DMA_ADDRESS_CONTROL_FIXED;       //设置源地址为固定，适用于外设寄存器
    config.src_mode = DMA_HANDSHAKE_MODE_HANDSHAKE;         //设置为握手模式，与 UART 外设协同工作
    config.dst_addr = core_local_mem_to_sys_address(HPM_CORE0, (uint32_t)dst);      //设置目标地址为接收缓冲区（转换为系统地址）
    config.dst_width = DMA_TRANSFER_WIDTH_BYTE; /*  In DMA handshake case, source width and destination width must be BYTE. */ //设置目标地址为接收缓冲区（转换为系统地址） 
    config.dst_addr_ctrl = DMA_ADDRESS_CONTROL_INCREMENT;   //目标地址递增，适合将数据存储到连续的缓冲区
    config.dst_mode = DMA_HANDSHAKE_MODE_NORMAL;            //设置为普通模式
    config.size_in_byte = size;                             //指定缓冲区大小
    config.src_burst_size = DMA_NUM_TRANSFER_PER_BURST_1T; /*  In DMA handshake case, source burst size must be 1 transfer, that is 0. */ //设置源突发传输大小为 1（符合握手模式）
    config.linked_ptr = core_local_mem_to_sys_address(HPM_CORE0, (uint32_t)&uart_com_fc_rx_descriptors[1]);    //链式描述符的下一个地址 uart_dbg_rx_descriptors !!!!!!!!!
    stat = dma_config_linked_descriptor(dma_ptr, &uart_com_fc_rx_descriptors[0], BOARD_COM_FC_UART_RX_DMA_CHN, &config);    //!
    if (stat != status_success) {
        while (1) {
        };
    }

    config.linked_ptr = core_local_mem_to_sys_address(HPM_CORE0, (uint32_t)&uart_com_fc_rx_descriptors[0]); //!
    stat = dma_config_linked_descriptor(dma_ptr, &uart_com_fc_rx_descriptors[1], BOARD_COM_FC_UART_RX_DMA_CHN, &config);    //!
    if (stat != status_success) {
        while (1) {
        };
    }

    /* 1.2 config rx dma */
    config.linked_ptr = core_local_mem_to_sys_address(HPM_CORE0, (uint32_t)&uart_com_fc_rx_descriptors[0]); //!
    stat = dma_setup_channel(dma_ptr, BOARD_COM_FC_UART_RX_DMA_CHN, &config, true);
    if (stat != status_success) {                           //启动 DMA 通道
        while (1) {
        };
    }
}




void bsp_dma_set_uart_dbg_type(UART_Type *uart_ptr)
{
    uart_dbg_type = uart_ptr;
}

void bsp_dma_set_uart_com_ma_type(UART_Type *uart_ptr)
{
    uart_com_ma_type = uart_ptr;
}

/* 飞控*/
void bsp_dma_set_uart_com_fc_type(UART_Type *uart_ptr)
{
    uart_com_fc_type = uart_ptr;
}



uint8_t* bsp_dma_get_uart_dbg_tx_pbuf(void)
{
    return uart_dbg_tx_dma_buf;
}

uint8_t* bsp_dma_get_uart_com_ma_tx_pbuf(void)
{
    return uart_com_ma_tx_dma_buf;
}

/*飞控*/
uint8_t* bsp_dma_get_uart_com_fc_tx_pbuf(void)
{
    return uart_com_fc_tx_dma_buf;
}

