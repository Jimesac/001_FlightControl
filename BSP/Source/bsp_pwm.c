#include "bsp_pwm.h"
#include <stdio.h>

uint32_t pwm_reload_value = 0;

void bsp_pwm_init(void)
{
    pwm_cmp_config_t cmp_config[8] = {0};
    pwm_config_t pwm_config = {0};
    hpm_stat_t stat;
    uint32_t pwm_clk_freq;

    HPM_IOC->PAD[BOARD_YAW_PWM_1_GPIO_PIN].FUNC_CTL = BOARD_YAW_PWM_1_GPIO_FUNC;
    HPM_IOC->PAD[BOARD_YAW_PWM_2_GPIO_PIN].FUNC_CTL = BOARD_YAW_PWM_2_GPIO_FUNC;
    HPM_IOC->PAD[BOARD_YAW_PWM_3_GPIO_PIN].FUNC_CTL = BOARD_YAW_PWM_3_GPIO_FUNC;

    // 手册里没有提到pwm的分频，这里暂不分频
    pwm_clk_freq = clock_get_frequency(BOARD_YAW_PWM_CLOCK_NAME);
    pwm_reload_value = pwm_clk_freq/PWM_UPDATE_FREQ - 1;

    pwm_deinit(BOARD_YAW_PWM_BASE);
    pwm_stop_counter(BOARD_YAW_PWM_BASE);
    pwm_enable_reload_at_synci(BOARD_YAW_PWM_BASE);

    /*
     * reload and start counter
     */
    pwm_set_reload(BOARD_YAW_PWM_BASE, 0, pwm_reload_value);
    pwm_set_start_count(BOARD_YAW_PWM_BASE, 0, 0);

    /*
     * config cmp
     */
    for (uint8_t i = 0; i < 6; i++) {
      cmp_config[i].mode = pwm_cmp_mode_output_compare;
      cmp_config[i].cmp = pwm_reload_value+1;
      cmp_config[i].update_trigger = pwm_shadow_register_update_on_hw_event;
    }
    cmp_config[6].mode = pwm_cmp_mode_output_compare;
    cmp_config[6].cmp = pwm_reload_value;
    cmp_config[6].update_trigger = pwm_shadow_register_update_on_modify;

    pwm_get_default_pwm_config(BOARD_YAW_PWM_BASE, &pwm_config);
    pwm_config.enable_output = true;
    pwm_config.dead_zone_in_half_cycle = 0;
    pwm_config.invert_output = false;

    /*
     * config pwm
     */
    if (status_success != pwm_setup_waveform(BOARD_YAW_PWM_BASE, BOARD_YAW_PWM_1_CH, &pwm_config, 0, &cmp_config[0], 2)) {
        printf("failed to setup waveform\n");
        while (1) {
        };
    }
    if (status_success != pwm_setup_waveform(BOARD_YAW_PWM_BASE, BOARD_YAW_PWM_2_CH, &pwm_config, 2, &cmp_config[2], 2)) {
        printf("failed to setup waveform\n");
        while (1) {
        };
    }
    if (status_success != pwm_setup_waveform(BOARD_YAW_PWM_BASE, BOARD_YAW_PWM_3_CH, &pwm_config, 4, &cmp_config[4], 2)) {
        printf("failed to setup waveform\n");
        while (1) {
        };
    }

    pwm_load_cmp_shadow_on_match(BOARD_YAW_PWM_BASE, 20,  &cmp_config[6]);
    pwm_start_counter(BOARD_YAW_PWM_BASE);
    pwm_issue_shadow_register_lock_event(BOARD_YAW_PWM_BASE);
    //duty_step = reload / TEST_LOOP;
    //duty = reload / TEST_LOOP;
    //for (uint32_t i = 0; i < TEST_LOOP; i++) {

    //    if ((duty + duty_step) >= reload) {
    //        duty = duty_step;
    //    } else {
    //        duty += duty_step;
    //    }
    //    pwm_update_raw_hrcmp_central_aligned(HRPWM, cmp_index, cmp_index + 1, (reload - duty) >> 1, (reload + duty) >> 1, 0, HRPWM_SET_IN_PWM_CLK);
    //    board_delay_ms(100);
    //}
    pwm_update_raw_cmp_central_aligned(BOARD_YAW_PWM_BASE, 0, 1, (pwm_reload_value - 200) >> 1, (pwm_reload_value + 200) >> 1);
    pwm_update_raw_cmp_central_aligned(BOARD_YAW_PWM_BASE, 2, 3, (pwm_reload_value - 200) >> 1, (pwm_reload_value + 200) >> 1);
    pwm_update_raw_cmp_central_aligned(BOARD_YAW_PWM_BASE, 4, 5, (pwm_reload_value - 200) >> 1, (pwm_reload_value + 200) >> 1);
}

void bsp_pwm_enable_all_pwm_output(void)
{
    pwm_enable_output(BOARD_YAW_PWM_BASE, BOARD_YAW_PWM_1_CH);
    pwm_enable_output(BOARD_YAW_PWM_BASE, BOARD_YAW_PWM_2_CH);
    pwm_enable_output(BOARD_YAW_PWM_BASE, BOARD_YAW_PWM_3_CH);
}

void bsp_pwm_disable_all_pwm_output(void)
{
    pwm_disable_output(BOARD_YAW_PWM_BASE, BOARD_YAW_PWM_1_CH);
    pwm_disable_output(BOARD_YAW_PWM_BASE, BOARD_YAW_PWM_2_CH);
    pwm_disable_output(BOARD_YAW_PWM_BASE, BOARD_YAW_PWM_3_CH);
}
