#include "bsp_timer.h"
#include "hpm_sysctl_drv.h"
#include "user_config.h"
#include <stdio.h>
#include "bsp_adc.h"

static void bsp_timer_schedule_init(void);
static void bsp_timer_clock_init(void);

bool schedule_ptmr_update_flag = false;
schedule_state_t schedule_ptmr_state = 0;
uint32_t schedule_ptmr_count;

void bsp_timer_init(void)
{
    bsp_timer_schedule_init();
    bsp_timer_clock_init();
}

static void bsp_timer_schedule_init(void)
{
    gptmr_channel_config_t config;              // 定时器配置
    clk_src_t clk_src;                          // 时钟源
    uint32_t gptmr_freq;                        // 定时器的时钟频率

    gptmr_channel_get_default_config(BOARD_PTMR_SCHEDULE_BASE, &config);    //获取默认的 GPTMR 配置，用于初始化 config 结构体
    clk_src = clock_get_source(BOARD_PTMR_SCHEDULE_CLK_NAME);               // 获取时钟源
    gptmr_freq = clock_get_frequency(BOARD_PTMR_SCHEDULE_CLK_NAME);         // 获取时钟频率
    printf("timer sechedule freq: %d\n", gptmr_freq);
    config.reload = gptmr_freq / TIMER_SCHEDULE_FREQ;                       // 计算定时器的重载值，使其达到所需的定时周期 gptmr_freq / 3840
    config.enable_software_sync = true;                                     //启用软件同步功能，确保定时器通道能够通过软件触发同步
    gptmr_channel_config(BOARD_PTMR_SCHEDULE_BASE, BOARD_PTMR_SCHEDULE_PTMR_CH, &config, false);    // 配置 GPTMR 通道，使用 config 结构体中的配置参数
    gptmr_channel_reset_count(BOARD_PTMR_SCHEDULE_BASE, BOARD_PTMR_SCHEDULE_PTMR_CH);               // 重置 GPTMR 通道的计数器

    gptmr_enable_irq(BOARD_PTMR_SCHEDULE_BASE, GPTMR_CH_RLD_IRQ_MASK(BOARD_PTMR_SCHEDULE_PTMR_CH)); // 使能 GPTMR 通道的重载中断
    intc_m_enable_irq_with_priority(BOARD_PTMR_SCHEDULE_IRQ, BOARD_PTMR_SCHEDULE_IRQ_LEVEL);        // 使能中断请求，设置中断优先级

    gptmr_start_counter(BOARD_PTMR_SCHEDULE_BASE, BOARD_PTMR_SCHEDULE_PTMR_CH);                     // 启动 GPTMR 通道的计数器

    gptmr_trigger_channel_software_sync(BOARD_PTMR_SCHEDULE_BASE, GPTMR_CH_GCR_SWSYNCT_MASK(BOARD_PTMR_SCHEDULE_PTMR_CH));   // 触发 GPTMR 通道的软件同步，确保定时器通道能够正常工作
    gptmr_enable_cmp_output(BOARD_PTMR_SCHEDULE_BASE, BOARD_PTMR_SCHEDULE_PTMR_CH);                 // 使能 GPTMR 通道的比较输出
}

void schedule_ptmr_isr(void)
{
    if (gptmr_check_status(BOARD_PTMR_SCHEDULE_BASE, GPTMR_CH_RLD_STAT_MASK(BOARD_PTMR_SCHEDULE_PTMR_CH))) {
        gptmr_clear_status(BOARD_PTMR_SCHEDULE_BASE, GPTMR_CH_RLD_STAT_MASK(BOARD_PTMR_SCHEDULE_PTMR_CH));

        adc16_trigger_pmt_by_sw(BOARD_ADC16_HALL_BASE, BOARD_HALL_ADC_TRG);

        schedule_ptmr_update_flag = true;
        schedule_ptmr_count++;
        if(++schedule_ptmr_state >= SCHEDULE_SECTION_NUM) schedule_ptmr_state = 0;
    }
}
SDK_DECLARE_EXT_ISR_M(BOARD_PTMR_SCHEDULE_IRQ, schedule_ptmr_isr)

void bsp_timer_schedule_reset_update_flag(void)
{
    schedule_ptmr_update_flag = false;
}

bool bsp_timer_schedule_get_update_flag(void)
{
    return schedule_ptmr_update_flag;
}

uint32_t bsp_timer_schedule_get_update_cnt(void)
{
    return schedule_ptmr_count;
}


static void bsp_timer_clock_init(void)
{
    gptmr_channel_config_t config;
    clk_src_t clk_src;
    hpm_stat_t stat;
    uint32_t gptmr_freq;

    gptmr_channel_get_default_config(BOARD_GPTMR_CLOCK_BASE, &config);
    clk_src = clock_get_source(BOARD_GPTMR_CLOCK_CLK_NAME);
    stat = clock_set_source_divider(BOARD_GPTMR_CLOCK_CLK_NAME, clk_src, 80);
    gptmr_freq = clock_get_frequency(BOARD_GPTMR_CLOCK_CLK_NAME);
    printf("timer clock freq: %d\n", gptmr_freq);
    config.reload = 0xFFFFFFFF;

    gptmr_channel_config(BOARD_GPTMR_CLOCK_BASE, BOARD_GPTMR_CLOCK_GPTMR_CH, &config, false);
    gptmr_channel_reset_count(BOARD_GPTMR_CLOCK_BASE, BOARD_GPTMR_CLOCK_GPTMR_CH);
    gptmr_start_counter(BOARD_GPTMR_CLOCK_BASE, BOARD_GPTMR_CLOCK_GPTMR_CH);
}

uint32_t bsp_timer_clock_start_0p1us(void)
{
    return BOARD_GPTMR_CLOCK_BASE->CHANNEL[BOARD_GPTMR_CLOCK_GPTMR_CH].CNT;
    //return gptmr_channel_get_counter(BOARD_GPTMR_CLOCK_BASE, BOARD_GPTMR_CLOCK_GPTMR_CH, gptmr_counter_type_normal);
}

uint32_t bsp_timer_clock_end_0p1us(uint32_t start_cnt)
{
    return BOARD_GPTMR_CLOCK_BASE->CHANNEL[BOARD_GPTMR_CLOCK_GPTMR_CH].CNT - start_cnt;
    //uint32_t counter = BOARD_GPTMR_CLOCK_BASE->CHANNEL[BOARD_GPTMR_CLOCK_GPTMR_CH].CNT;

    //if (counter > start_cnt) {
    //    return (counter - start_cnt);
    //}
    //else {
    //    return 0xffffffff - start_cnt + counter;
    //} 
}

