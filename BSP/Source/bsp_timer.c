#include "bsp_timer.h"
#include "hpm_sysctl_drv.h"
#include <stdio.h>

static void bsp_timer_schedule_init(void);
static void bsp_timer_clock_init(void);

bool schedule_ptmr_update_flag = false;
uint32_t schedule_ptmr_count;

void bsp_timer_init(void)
{
    bsp_timer_schedule_init();
    bsp_timer_clock_init();


}

static void bsp_timer_schedule_init(void)
{
    gptmr_channel_config_t config;
    clk_src_t clk_src;
    uint32_t gptmr_freq;

    gptmr_channel_get_default_config(BOARD_PTMR_SCHEDULE_BASE, &config);
    clk_src = clock_get_source(BOARD_PTMR_SCHEDULE_CLK_NAME);
    gptmr_freq = clock_get_frequency(BOARD_PTMR_SCHEDULE_CLK_NAME);
    printf("timer sechedule freq: %d\n", gptmr_freq);
    config.reload = gptmr_freq / TIMER_SCHEDULE_FREQ;
    gptmr_channel_config(BOARD_PTMR_SCHEDULE_BASE, BOARD_PTMR_SCHEDULE_PTMR_CH, &config, false);
    gptmr_channel_reset_count(BOARD_PTMR_SCHEDULE_BASE, BOARD_PTMR_SCHEDULE_PTMR_CH);

    gptmr_enable_irq(BOARD_PTMR_SCHEDULE_BASE, GPTMR_CH_RLD_IRQ_MASK(BOARD_PTMR_SCHEDULE_PTMR_CH));
    intc_m_enable_irq_with_priority(BOARD_PTMR_SCHEDULE_IRQ, 4);

    gptmr_start_counter(BOARD_PTMR_SCHEDULE_BASE, BOARD_PTMR_SCHEDULE_PTMR_CH);

}

void schedule_ptmr_isr(void)
{
    if (gptmr_check_status(BOARD_PTMR_SCHEDULE_BASE, GPTMR_CH_RLD_STAT_MASK(BOARD_PTMR_SCHEDULE_PTMR_CH))) {
        gptmr_clear_status(BOARD_PTMR_SCHEDULE_BASE, GPTMR_CH_RLD_STAT_MASK(BOARD_PTMR_SCHEDULE_PTMR_CH));
        schedule_ptmr_update_flag = true;
        schedule_ptmr_count++;
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
    return gptmr_channel_get_counter(BOARD_GPTMR_CLOCK_BASE, BOARD_GPTMR_CLOCK_GPTMR_CH, gptmr_counter_type_normal);
}

uint32_t bsp_timer_clock_end_0p1us(uint32_t start_cnt)
{
    uint32_t counter = gptmr_channel_get_counter(BOARD_GPTMR_CLOCK_BASE, BOARD_GPTMR_CLOCK_GPTMR_CH, gptmr_counter_type_normal);

    if (counter > start_cnt) {
        return (counter - start_cnt);
    }
    else {
        return 0xffffffff - start_cnt + counter;
    } 
}

