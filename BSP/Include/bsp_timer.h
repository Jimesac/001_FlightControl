#ifndef __BSP_TIMER_H
#define __BSP_TIMER_H

#include "hpm_soc.h"
#include "hpm_clock_drv.h"
#include "hpm_gptmr_drv.h"
#include "hpm_interrupt.h"
#include "user_config.h"

#define TIMER_SCHEDULE_FREQ   (SELTIMER_SCHEDULE_FREQ)

#define BOARD_GPTMR_CLOCK_BASE       HPM_GPTMR3
#define BOARD_GPTMR_CLOCK_GPTMR_CH   (0)
#define BOARD_GPTMR_CLOCK_CLK_NAME   (clock_gptmr3)

#define BOARD_PTMR_SCHEDULE_BASE      HPM_GPTMR2
#define BOARD_PTMR_SCHEDULE_PTMR_CH   (2)
#define BOARD_PTMR_SCHEDULE_CLK_NAME  clock_gptmr2

extern schedule_state_t schedule_ptmr_state;

void bsp_timer_init(void);

void bsp_timer_schedule_reset_update_flag(void);
bool bsp_timer_schedule_get_update_flag(void);
uint32_t bsp_timer_schedule_get_update_cnt(void);

uint32_t bsp_timer_clock_start_0p1us(void);
uint32_t bsp_timer_clock_end_0p1us(uint32_t start_cnt);


void board_timer_create(uint32_t ms);

#endif /* __BSP_TIMER_H */