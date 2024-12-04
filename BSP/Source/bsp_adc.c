#include "bsp_adc.h"
#include "user_config.h"
#include "filter.h"
#include <stdio.h>


/* ************************** */
// 用户手册上写明ADC16 转换时钟最大频率为 50MHz

#define ADC_HALL_FILTER_SIZE   (7U)

static void bsp_adc_version_init(void);
static void bsp_adc_vbus_init(void);
static void bsp_adc_hall_init(void);

uint16_t adc_version_sample = 0, adc_version_mv = 0;
struct{
   uint32_t res: 16;
   uint32_t valid: 1;
   uint32_t rsvd: 15;
}adc_pmt_dma_buf[16] = {0};

uint16_t adc_hall_smp[2][2] = {0};
uint16_t adc_hall_filter[2][2] = {0};
uint16_t adc_hall_median[2][2][ADC_HALL_FILTER_SIZE] = {0};
uint16_t adc_hall_median_cnt;

uint32_t hall_adc_isr_cnt = 0;

void bsp_adc_init(void)
{
    bsp_adc_version_init();
    bsp_adc_vbus_init();
    bsp_adc_hall_init();
}

//初始化 ADC（模数转换器）硬件模块    查看版本
static void bsp_adc_version_init(void)
{
    adc16_config_t cfg;             //存储 ADC 模块的配置参数
    adc16_channel_config_t ch_cfg;  //存储 ADC 通道的配置参数
    uint32_t freq = 0;              //存储 ADC 模块的时钟频率

 
    /* ADC pin initialization */
    HPM_IOC->PAD[BOARD_ADC16_VERSION_GPIO_PIN].FUNC_CTL = BOARD_ADC16_VERSION_GPIO_FUNC;    // 配置 ADC 引脚的功能 PB14

    /* Configure the DAC clock to 180MHz */
    clock_set_adc_source(BOARD_ADC16_VERSION_CLK_NAME, clk_dac_src_ahb0);                   // 配置 ADC 模块的时钟源为 AHB0
    freq = clock_get_frequency(BOARD_ADC16_VERSION_CLK_NAME);                               // 获取 ADC 模块的时钟频率

    /* initialize an ADC instance */
    adc16_get_default_config(&cfg);                                     // 获取默认的 ADC 配置参数  
    cfg.res            = adc16_res_16_bits;                             // 设置 ADC 分辨率为 16 位
    cfg.conv_mode      = adc16_conv_mode_oneshot;                       // 设置 ADC 转换模式为单次转换模式
    cfg.adc_clk_div    = adc16_clock_divider_4;                         // 设置 ADC 时钟分频为 4
    cfg.sel_sync_ahb   = (clk_adc_src_ahb0 == freq) ? true : false;     // 判断是否启用 AHB 时钟同步
    if (cfg.conv_mode == adc16_conv_mode_sequence ||                    // 如果 ADC 转换模式是 序列模式 
        cfg.conv_mode == adc16_conv_mode_preemption) {                  // 如果 ADC 转换模式是 抢占模式
        cfg.adc_ahb_en = true;                                          // 启用 AHB 时钟支持
    }
    /* adc16 initialization */
    if (adc16_init(BOARD_ADC16_VERSION_BASE, &cfg) == status_success) { //初始化 ADC 模块，并判断是否成功
        printf("initialization adc version success!\n");
    } else {
        printf("initialization adc version failed!\n");
        return;
    }


    /* get a default channel config */
    adc16_get_channel_default_config(&ch_cfg);                          // 获取默认的 ADC 通道配置参数
    /* initialize an ADC channel */
    ch_cfg.ch           = BOARD_ADC16_VERSION_CH;                       // 设置 ADC 通道为指定通道
    ch_cfg.sample_cycle = 200;                                          // 设置 ADC 通道的采样周期为 200
    adc16_init_channel(BOARD_ADC16_VERSION_BASE, &ch_cfg);              // 初始化 ADC 通道
    adc16_set_nonblocking_read(BOARD_ADC16_VERSION_BASE);               // 启用非阻塞读取模式

    /* enable oneshot mode */
    adc16_enable_oneshot_mode(BOARD_ADC16_VERSION_BASE);                // 启用一次性模式

    while (adc16_get_oneshot_result(BOARD_ADC16_VERSION_BASE, BOARD_ADC16_VERSION_CH, &adc_version_sample) != status_success) // 等待一次性模式转换完成
    {
        if (adc16_is_nonblocking_mode(BOARD_ADC16_VERSION_BASE)) {      // 判断是否处于非阻塞模式
            adc16_get_oneshot_result(BOARD_ADC16_VERSION_BASE, BOARD_ADC16_VERSION_CH, &adc_version_sample);    // 获取一次性模式转换结果
        }
        adc_version_mv = ((uint32_t)adc_version_sample*825) >> 14;      // 将采样值 adc_version_sample 转换为电压值（单位：mV），假设 ADC 参考电压为 825mV，右移 14 位表示数据分辨率为 16 位。
        printf("Version ADC - Result : %dmV\n", adc_version_mv);        // 打印转换结果
        return;
    }
}

//初始化 ADC 模块以测量 VBUS（电源总线）电压
static void bsp_adc_vbus_init(void)
{
    adc16_config_t cfg;             //存储 ADC 模块的配置参数
    adc16_channel_config_t ch_cfg;  //存储 ADC 通道的配置参数
    adc16_prd_config_t prd_cfg;     //存储 ADC 周期模式的配置参数
    uint32_t freq = 0;              //存储 ADC 模块的时钟频率

 
    /* ADC pin initialization */
    HPM_IOC->PAD[BOARD_ADC16_VBUS_GPIO_PIN].FUNC_CTL = BOARD_ADC16_VBUS_GPIO_FUNC;          // 配置 ADC 引脚的功能，将引脚配置为 ADC 模式PB15

    /* Configure the DAC clock to 180MHz */
    clock_set_adc_source(BOARD_ADC16_VBUS_CLK_NAME, clk_dac_src_ahb0);                      // 配置 ADC 模块的时钟源为 AHB0
    freq = clock_get_frequency(BOARD_ADC16_VBUS_CLK_NAME);                                  // 获取 ADC 模块的时钟频率

    /* initialize an ADC instance */
    adc16_get_default_config(&cfg);                 // 获取默认的 ADC 配置参数
    cfg.res            = adc16_res_16_bits;         // 设置 ADC 分辨率为 16 位
    cfg.conv_mode      = adc16_conv_mode_period;    // 设置 ADC 转换模式为周期模式
    cfg.adc_clk_div    = adc16_clock_divider_4;     // 设置 ADC 时钟分频为 4
    cfg.sel_sync_ahb   = (clk_adc_src_ahb0 == freq) ? true : false; // 判断是否启用 AHB 时钟同步
    if (cfg.conv_mode == adc16_conv_mode_sequence ||                // 如果模式是 序列模式
        cfg.conv_mode == adc16_conv_mode_preemption) {              // 如果模式是 抢占模式
        cfg.adc_ahb_en = true;                                      // 启用 AHB 时钟支持
    }
    /* adc16 initialization */
    if (adc16_init(BOARD_ADC16_VBUS_BASE, &cfg) == status_success) {// 初始化 ADC 模块，并判断是否成功
        printf("initialization adc vbus success!\n");
    } else {
        printf("initialization adc vbus failed!\n");
        return;
    }


    /* get a default channel config */
    adc16_get_channel_default_config(&ch_cfg);                      // 获取默认的 ADC 通道配置参数
    /* initialize an ADC channel */
    ch_cfg.ch           = BOARD_ADC16_VBUS_CH;                      // 设置 ADC 通道为指定通道
    ch_cfg.sample_cycle = 20;                                       // 设置 ADC 通道的采样周期为 20
    adc16_init_channel(BOARD_ADC16_VBUS_BASE, &ch_cfg);             // 初始化 ADC 通道
    prd_cfg.ch           = BOARD_ADC16_VBUS_CH;                     // 设置 ADC 通道为指定通道
    prd_cfg.prescale     = 16;    /* Set divider: 2^22 clocks */    // 设置 ADC 周期模式的预分频值为 16
    prd_cfg.period_count = 3;     /* 0.983ms when AHB clock at 200MHz is ADC clock source */    // 设置 ADC 周期模式的周期计数为 3
    adc16_set_prd_config(BOARD_ADC16_VBUS_BASE, &prd_cfg);          // 设置 ADC 周期模式的配置参数
}

//初始化霍尔传感器的 ADC 模块
static void bsp_adc_hall_init(void)
{
    adc16_config_t cfg;                 //存储 ADC 模块的配置参数
    adc16_channel_config_t ch_cfg;      //存储 ADC 通道的配置参数
    adc16_prd_config_t prd_cfg;         //存储 ADC 周期模式的配置参数
    uint32_t freq = 0;                  //存储 ADC 模块的时钟频率

 
    /* ADC pin initialization */
    HPM_IOC->PAD[BOARD_ADC16_HALL_P1_GPIO_PIN].FUNC_CTL = BOARD_ADC16_HALL_GPIO_FUNC;   // 配置 ADC 引脚的功能，将引脚配置为 ADC 模式PB10

    /* Configure the DAC clock to 180MHz */
    clock_set_adc_source(BOARD_ADC16_HALL_CLK_NAME, clk_dac_src_ahb0);                  // 配置 ADC 模块的时钟源为 AHB0
    freq = clock_get_frequency(BOARD_ADC16_HALL_CLK_NAME);                              // 获取 ADC 模块的时钟频率

    /* initialize an ADC instance */
    adc16_get_default_config(&cfg);                                     // 获取默认的 ADC 配置参数
    cfg.res            = adc16_res_16_bits;                             // 设置 ADC 分辨率为 16 位
    cfg.conv_mode      = adc16_conv_mode_preemption;                    // 设置 ADC 转换模式为抢占模式
    cfg.adc_clk_div    = adc16_clock_divider_4;                         // 设置 ADC 时钟分频为 4
    cfg.sel_sync_ahb   = (clk_adc_src_ahb0 == freq) ? true : false;     // 判断是否启用 AHB 时钟同步
    if (cfg.conv_mode == adc16_conv_mode_sequence ||                    // 如果模式是 序列模式
        cfg.conv_mode == adc16_conv_mode_preemption) {                  // 如果模式是 抢占模式
        cfg.adc_ahb_en = true;                                          // 启用 AHB 时钟支持
    }

    /* adc16 initialization */
    if (adc16_init(BOARD_ADC16_HALL_BASE, &cfg) == status_success) {    // 初始化 ADC 模块，并判断是否成功
        printf("initialization adc sox success!\n");
    } else {
        printf("initialization adc sox failed!\n");
        return;
    }

    /* get a default channel config */
    adc16_get_channel_default_config(&ch_cfg);                          // 获取默认的 ADC 通道配置参数
    /* initialize an ADC channel */
    ch_cfg.sample_cycle = 20;                                           // 设置 ADC 通道的采样周期为 20
    ch_cfg.ch           = BOARD_ADC16_HALL_P1_CH;                       // 设置 ADC 通道为指定通道
    adc16_init_channel(BOARD_ADC16_HALL_BASE, &ch_cfg);                 // 初始化 ADC 通道
    ch_cfg.ch           = BOARD_ADC16_HALL_P2_CH;                       // 设置 ADC 通道为指定通道
    adc16_init_channel(BOARD_ADC16_HALL_BASE, &ch_cfg);                 // 初始化 ADC 通道
    ch_cfg.ch           = BOARD_ADC16_HALL_R1_CH;                       // 设置 ADC 通道为指定通道
    adc16_init_channel(BOARD_ADC16_HALL_BASE, &ch_cfg);                 // 初始化 ADC 通道
    ch_cfg.ch           = BOARD_ADC16_HALL_R2_CH;
    adc16_init_channel(BOARD_ADC16_HALL_BASE, &ch_cfg);

    // adc触发设置
    //// trigger mux
    //trgm_output_t trgm_output_cfg;
    //trgm_output_cfg.invert = false;
    //trgm_output_cfg.type   = trgm_output_same_as_input;
    //trgm_output_cfg.input  = BOARD_HALL_TRIGMUX_IN_NUM;
    //trgm_output_config(BOARD_ADC16_HALL_TRGM, BOARD_HALL_TRG_NUM, &trgm_output_cfg);

    // trigger target
    adc16_pmt_config_t pmt_cfg;                     //存储 ADC 触发模式的配置参数 配置 PMT（Preemption Mode Triggering）模块
    pmt_cfg.trig_len = 4;                           // 触发信号长度，设置为 4
    pmt_cfg.adc_ch[0] = BOARD_ADC16_HALL_P1_CH;     // 设置 ADC 通道为指定通道
    pmt_cfg.adc_ch[1] = BOARD_ADC16_HALL_P2_CH;
    pmt_cfg.adc_ch[2] = BOARD_ADC16_HALL_R1_CH;
    pmt_cfg.adc_ch[3] = BOARD_ADC16_HALL_R2_CH;
    pmt_cfg.trig_ch = BOARD_HALL_ADC_TRG;           // 设置 ADC 触发模式的通道为指定通道
    pmt_cfg.inten[0] = false;                       // 配置触发完成后是否启用中断
    pmt_cfg.inten[1] = false;                       
    pmt_cfg.inten[2] = false;
    pmt_cfg.inten[3] = true;
    adc16_set_pmt_config(BOARD_ADC16_HALL_BASE, &pmt_cfg);                          // 应用 PMT 配置
    adc16_set_pmt_queue_enable(BOARD_ADC16_HALL_BASE, BOARD_HALL_ADC_TRG, true);    // 并启用 PMT 触发队列

    adc16_init_pmt_dma(BOARD_ADC16_HALL_BASE, (uint32_t )adc_pmt_dma_buf);          // 配置 ADC 数据采样的 DMA 缓冲区 adc_pmt_dma_buf，将采样结果通过 DMA 传输到指定缓冲区
    
    /* Enable trigger complete interrupt */
    adc16_enable_interrupts(BOARD_ADC16_HALL_BASE, BOARD_ADC16_HALL_PMT_IRQ_EVENT); // 启用 ADC 触发完成中断（PMT 中断）
    intc_m_enable_irq_with_priority(BOARD_ADC16_HALL_IRQn, BOARD_ADC16_HALL_IRQn_LEVEL);    //设置中断优先级并启用相应的 NVIC 中断通道
}

void hall_adc16_isr(void)     // 中断处理函数，用于处理 ADC 触发完成中断事件
{
    uint32_t status;                        // 存储中断状态的变量
    uint32_t adc_hall_temp[2][2] = {0};     // 用于计算均值滤波的中间变量，记录临时求和结果

    status = adc16_get_status_flags(BOARD_ADC16_HALL_BASE);     // 获取 ADC 触发完成中断的状态标志

    /* Clear status */
    adc16_clear_status_flags(BOARD_ADC16_HALL_BASE, status);    // 清除 ADC 触发完成中断的状态标志

    if (ADC16_INT_STS_TRIG_CMPT_GET(status)) {                  // 检查 ADC 触发完成中断的状态标志是否为触发完成中断
        /* Set flag to read memory data */
        //trig_complete_flag = 1;
        hall_adc_isr_cnt++;
        //adc_hall_smp[0] = adc_pmt_dma_buf[0].res;
        //adc_hall_smp[1] = adc_pmt_dma_buf[1].res;
        //adc_hall_smp[2] = adc_pmt_dma_buf[2].res;
        //adc_hall_smp[3] = adc_pmt_dma_buf[3].res;
        
        adc_hall_median[PITCH][0][adc_hall_median_cnt] = adc_pmt_dma_buf[0].res;    // 将 ADC 采样结果存储到 adc_hall_median 数组中
        adc_hall_median[PITCH][1][adc_hall_median_cnt] = adc_pmt_dma_buf[1].res;    
        adc_hall_median[ROLL][0][adc_hall_median_cnt] = adc_pmt_dma_buf[3].res;
        adc_hall_median[ROLL][1][adc_hall_median_cnt] = adc_pmt_dma_buf[2].res;

#if (ADC_HALL_FILTER_SIZE == 3)
        adc_hall_filter[PITCH][0] = filter_median_u16w3(&adc_hall_median[PITCH][0][0]);
        adc_hall_filter[PITCH][1] = filter_median_u16w3(&adc_hall_median[PITCH][1][0]);
        adc_hall_filter[ROLL][0] = filter_median_u16w3(&adc_hall_median[ROLL][0][0]);
        adc_hall_filter[ROLL][1] = filter_median_u16w3(&adc_hall_median[ROLL][1][0]);
#elif (ADC_HALL_FILTER_SIZE == 5)
        adc_hall_filter[PITCH][0] = filter_median_u16w5(&adc_hall_median[PITCH][0][0]);
        adc_hall_filter[PITCH][1] = filter_median_u16w5(&adc_hall_median[PITCH][1][0]);
        adc_hall_filter[ROLL][0] = filter_median_u16w5(&adc_hall_median[ROLL][0][0]);
        adc_hall_filter[ROLL][1] = filter_median_u16w5(&adc_hall_median[ROLL][1][0]);
#elif (ADC_HALL_FILTER_SIZE == 7)
        adc_hall_filter[PITCH][0] = filter_median_u16w7(&adc_hall_median[PITCH][0][0]);     // 计算滤波后的值，七个数的中值
        adc_hall_filter[PITCH][1] = filter_median_u16w7(&adc_hall_median[PITCH][1][0]);
        adc_hall_filter[ROLL][0] = filter_median_u16w7(&adc_hall_median[ROLL][0][0]);
        adc_hall_filter[ROLL][1] = filter_median_u16w7(&adc_hall_median[ROLL][1][0]);
#else
        for (uint8_t i = 0; i < 2; i++)        // 循环计算滤波后的值
        {
            for (uint8_t j = 0; j < ADC_HALL_FILTER_SIZE; j++)  
            {
                adc_hall_temp[i][0] += adc_hall_median[i][0][j];
                adc_hall_temp[i][1] += adc_hall_median[i][1][j];
            }
            adc_hall_filter[i][0] = adc_hall_temp[i][0] / ADC_HALL_FILTER_SIZE;
            adc_hall_filter[i][1] = adc_hall_temp[i][1] / ADC_HALL_FILTER_SIZE;
        }
#endif
        if (++adc_hall_median_cnt >= ADC_HALL_FILTER_SIZE) adc_hall_median_cnt = 0;  // 循环使用 adc_hall_median 数组 ADC_HALL_FILTER_SIZE=7

    }

}
SDK_DECLARE_EXT_ISR_M(BOARD_ADC16_HALL_IRQn, hall_adc16_isr)


uint16_t *bsp_adc_get_hall_pitch_filter_ptr(void)   
{
    return &adc_hall_filter[PITCH][0];    // 返回 adc_hall_filter 数组的指针 Pitch
}

uint16_t *bsp_adc_get_hall_roll_filter_ptr(void)
{
    return &adc_hall_filter[ROLL][0];     // 返回 adc_hall_filter 数组的指针 Roll
}

