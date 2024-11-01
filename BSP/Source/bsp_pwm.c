#include "bsp_pwm.h"
#include "user_config.h"
#include <stdio.h>

uint32_t pwm_reload_value = 0;
uint32_t pwm_reload_half_value = 0;
uint32_t pwm_test_cnt[2] = {0};

uint16_t pwm_pwm_dutycycle_temp[3][3];
uint16_t *pwm_pitch_dutycycle_ptr;
uint16_t *pwm_roll_dutycycle_ptr;
uint16_t *pwm_yaw_dutycycle_ptr;

uint32_t pwm_roll_cmp[3][2] = {0};
uint32_t pwm_yaw_cmp[3][2] = {0};
uint32_t pwm_pitch_cmp[3][2] = {0};

void bsp_pwm_init(void)
{

    pwm_pitch_dutycycle_ptr = &pwm_pwm_dutycycle_temp[PITCH][0];
    pwm_roll_dutycycle_ptr = &pwm_pwm_dutycycle_temp[ROLL][0];
    pwm_yaw_dutycycle_ptr = &pwm_pwm_dutycycle_temp[YAW][0];

    pwm_cmp_config_t cmp_config[4] = {0};
    pwm_config_t pwm_config = {0};
    pwm_output_channel_t pwm_output_ch_cfg;
    hpm_stat_t stat;
    uint32_t pwm_clk_freq;
    
    HPM_IOC->PAD[BOARD_PITCH_PWM_1_GPIO_PIN].FUNC_CTL = BOARD_PITCH_PWM_1_GPIO_FUNC;
    HPM_IOC->PAD[BOARD_PITCH_PWM_2_GPIO_PIN].FUNC_CTL = BOARD_PITCH_PWM_2_GPIO_FUNC;
    HPM_IOC->PAD[BOARD_PITCH_PWM_3_GPIO_PIN].FUNC_CTL = BOARD_PITCH_PWM_3_GPIO_FUNC;

    HPM_IOC->PAD[BOARD_ROLL_PWM_1_GPIO_PIN].FUNC_CTL = BOARD_ROLL_PWM_1_GPIO_FUNC;
    HPM_IOC->PAD[BOARD_ROLL_PWM_2_GPIO_PIN].FUNC_CTL = BOARD_ROLL_PWM_2_GPIO_FUNC;
    HPM_IOC->PAD[BOARD_ROLL_PWM_3_GPIO_PIN].FUNC_CTL = BOARD_ROLL_PWM_3_GPIO_FUNC;

    HPM_IOC->PAD[BOARD_YAW_PWM_1_GPIO_PIN].FUNC_CTL = BOARD_YAW_PWM_1_GPIO_FUNC;
    HPM_IOC->PAD[BOARD_YAW_PWM_2_GPIO_PIN].FUNC_CTL = BOARD_YAW_PWM_2_GPIO_FUNC;
    HPM_IOC->PAD[BOARD_YAW_PWM_3_GPIO_PIN].FUNC_CTL = BOARD_YAW_PWM_3_GPIO_FUNC;

    // 手册里没有提到pwm的分频，这里暂不分频
    pwm_clk_freq = clock_get_frequency(BOARD_PWM_CLOCK_NAME);
    pwm_reload_value = pwm_clk_freq/PWM_UPDATE_FREQ - 1;
    pwm_reload_half_value = (pwm_reload_value+1)/2;

    pwm_deinit(BOARD_ROLL_YAW_PWM_BASE);
    pwm_stop_counter(BOARD_ROLL_YAW_PWM_BASE);
    pwm_enable_reload_at_synci(BOARD_ROLL_YAW_PWM_BASE);

    pwm_deinit(BOARD_PITCH_PWM_BASE);
    pwm_stop_counter(BOARD_PITCH_PWM_BASE);
    pwm_enable_reload_at_synci(BOARD_PITCH_PWM_BASE);

    /*
     * reload and start counter
     */
    pwm_set_reload(BOARD_ROLL_YAW_PWM_BASE, 0, pwm_reload_value);
    pwm_set_start_count(BOARD_ROLL_YAW_PWM_BASE, 0, 0);

    pwm_set_reload(BOARD_PITCH_PWM_BASE, 0, pwm_reload_value);
    pwm_set_start_count(BOARD_PITCH_PWM_BASE, 0, 0);

    /*
     * config cmp
     */

    cmp_config[0].mode = pwm_cmp_mode_output_compare;
    cmp_config[0].cmp = pwm_reload_value+1;
    cmp_config[0].update_trigger = pwm_shadow_register_update_on_hw_event;

    cmp_config[1].mode = pwm_cmp_mode_output_compare;
    cmp_config[1].cmp = pwm_reload_value+1;
    cmp_config[1].update_trigger = pwm_shadow_register_update_on_hw_event;

    cmp_config[2].mode = pwm_cmp_mode_output_compare;
    cmp_config[2].cmp = pwm_reload_value;
    cmp_config[2].update_trigger = pwm_shadow_register_update_on_modify;

    cmp_config[3].mode = pwm_cmp_mode_output_compare;
    cmp_config[3].cmp = pwm_reload_value;
    cmp_config[3].update_trigger = pwm_shadow_register_update_on_modify;


    pwm_get_default_pwm_config(BOARD_ROLL_YAW_PWM_BASE, &pwm_config);
    pwm_get_default_pwm_config(BOARD_PITCH_PWM_BASE, &pwm_config);
    pwm_config.enable_output = true;
    pwm_config.dead_zone_in_half_cycle = 0;
    pwm_config.invert_output = false;

    /*
     * config pwm
     */
    if (status_success != pwm_setup_waveform(BOARD_ROLL_YAW_PWM_BASE, BOARD_ROLL_PWM_1_CH, &pwm_config, 0, &cmp_config[0], 2)) {
        printf("failed to setup waveform\n");
        while (1) {
        };
    }
    if (status_success != pwm_setup_waveform(BOARD_ROLL_YAW_PWM_BASE, BOARD_ROLL_PWM_2_CH, &pwm_config, 2, &cmp_config[0], 2)) {
        printf("failed to setup waveform\n");
        while (1) {
        };
    }
    if (status_success != pwm_setup_waveform(BOARD_ROLL_YAW_PWM_BASE, BOARD_ROLL_PWM_3_CH, &pwm_config, 4, &cmp_config[0], 2)) {
        printf("failed to setup waveform\n");
        while (1) {
        };
    }
    if (status_success != pwm_setup_waveform(BOARD_ROLL_YAW_PWM_BASE, BOARD_YAW_PWM_1_CH, &pwm_config, 6, &cmp_config[0], 2)) {
        printf("failed to setup waveform\n");
        while (1) {
        };
    }
    if (status_success != pwm_setup_waveform(BOARD_ROLL_YAW_PWM_BASE, BOARD_YAW_PWM_2_CH, &pwm_config, 8, &cmp_config[0], 2)) {
        printf("failed to setup waveform\n");
        while (1) {
        };
    }
    if (status_success != pwm_setup_waveform(BOARD_ROLL_YAW_PWM_BASE, BOARD_YAW_PWM_3_CH, &pwm_config, 10, &cmp_config[0], 2)) {
        printf("failed to setup waveform\n");
        while (1) {
        };
    }
    pwm_load_cmp_shadow_on_match(BOARD_ROLL_YAW_PWM_BASE, 12,  &cmp_config[2]);

    if (status_success != pwm_setup_waveform(BOARD_PITCH_PWM_BASE, BOARD_PITCH_PWM_1_CH, &pwm_config, 0, &cmp_config[0], 2)) {
        printf("failed to setup waveform\n");
        while (1) {
        };
    }
    if (status_success != pwm_setup_waveform(BOARD_PITCH_PWM_BASE, BOARD_PITCH_PWM_2_CH, &pwm_config, 2, &cmp_config[0], 2)) {
        printf("failed to setup waveform\n");
        while (1) {
        };
    }
    if (status_success != pwm_setup_waveform(BOARD_PITCH_PWM_BASE, BOARD_PITCH_PWM_3_CH, &pwm_config, 4, &cmp_config[0], 2)) {
        printf("failed to setup waveform\n");
        while (1) {
        };
    }
    pwm_load_cmp_shadow_on_match(BOARD_PITCH_PWM_BASE, 6,  &cmp_config[3]);
    
    pwm_enable_irq(BOARD_ROLL_YAW_PWM_BASE, PWM_IRQ_HALF_RELOAD);
    intc_m_enable_irq_with_priority(BOARD_ROLL_YAW_PWM_IRQ, BOARD_ROLL_YAW_PWM_IRQ_LEVEL);

    pwm_enable_irq(BOARD_PITCH_PWM_BASE, PWM_IRQ_HALF_RELOAD);
    intc_m_enable_irq_with_priority(BOARD_PITCH_PWM_IRQ, BOARD_PITCH_PWM_IRQ_LEVEL);

    
    pwm_start_counter(BOARD_ROLL_YAW_PWM_BASE);
    pwm_start_counter(BOARD_PITCH_PWM_BASE);
    


    bsp_pwm_enable_all_pwm_output();
    //pwm_update_raw_cmp_central_aligned(BOARD_ROLL_YAW_PWM_BASE, 0, 1, (pwm_reload_value - 2000) >> 1, (pwm_reload_value + 2000) >> 1);
    //pwm_update_raw_cmp_central_aligned(BOARD_ROLL_YAW_PWM_BASE, 2, 3, (pwm_reload_value - 4000) >> 1, (pwm_reload_value + 4000) >> 1);
    //pwm_update_raw_cmp_central_aligned(BOARD_ROLL_YAW_PWM_BASE, 4, 5, (pwm_reload_value - 6000) >> 1, (pwm_reload_value + 6000) >> 1);

    //pwm_update_raw_cmp_central_aligned(BOARD_ROLL_YAW_PWM_BASE, 6, 7, (pwm_reload_value - 2000) >> 1, (pwm_reload_value + 2000) >> 1);
    //pwm_update_raw_cmp_central_aligned(BOARD_ROLL_YAW_PWM_BASE, 8, 9, (pwm_reload_value - 4000) >> 1, (pwm_reload_value + 4000) >> 1);
    //pwm_update_raw_cmp_central_aligned(BOARD_ROLL_YAW_PWM_BASE, 10, 11, (pwm_reload_value - 5000) >> 1, (pwm_reload_value + 5000) >> 1);

    //pwm_update_raw_cmp_central_aligned(BOARD_PITCH_PWM_BASE, 0, 1, (pwm_reload_value - 2000) >> 1, (pwm_reload_value + 2000) >> 1);
    //pwm_update_raw_cmp_central_aligned(BOARD_PITCH_PWM_BASE, 2, 3, (pwm_reload_value - 2000) >> 1, (pwm_reload_value + 2000) >> 1);
    //pwm_update_raw_cmp_central_aligned(BOARD_PITCH_PWM_BASE, 4, 5, (pwm_reload_value - 6000) >> 1, (pwm_reload_value + 6000) >> 1);
    clock_cpu_delay_ms(10);

}

void roll_yaw_isr(void)
{
    if (pwm_get_status(BOARD_ROLL_YAW_PWM_BASE)) 
    {
        pwm_clear_status(BOARD_ROLL_YAW_PWM_BASE, PWM_IRQ_HALF_RELOAD);
        pwm_test_cnt[0]++;
        
        pwm_roll_cmp[0][0] = pwm_reload_half_value - (pwm_roll_dutycycle_ptr[0]>>1);
        pwm_roll_cmp[1][0] = pwm_reload_half_value - (pwm_roll_dutycycle_ptr[1]>>1);
        pwm_roll_cmp[2][0] = pwm_reload_half_value - (pwm_roll_dutycycle_ptr[2]>>1);
        pwm_roll_cmp[0][1] = pwm_reload_value - pwm_roll_cmp[0][0];
        pwm_roll_cmp[1][1] = pwm_reload_value - pwm_roll_cmp[1][0];
        pwm_roll_cmp[2][1] = pwm_reload_value - pwm_roll_cmp[2][0];
        pwm_update_raw_cmp_central_aligned(BOARD_ROLL_YAW_PWM_BASE, 0, 1, pwm_roll_cmp[0][0], pwm_roll_cmp[0][1]);
        pwm_update_raw_cmp_central_aligned(BOARD_ROLL_YAW_PWM_BASE, 2, 3, pwm_roll_cmp[1][0], pwm_roll_cmp[1][1]);
        pwm_update_raw_cmp_central_aligned(BOARD_ROLL_YAW_PWM_BASE, 4, 5, pwm_roll_cmp[2][0], pwm_roll_cmp[2][1]);

        pwm_yaw_cmp[0][0] = pwm_reload_half_value - (pwm_yaw_dutycycle_ptr[0]>>1);
        pwm_yaw_cmp[1][0] = pwm_reload_half_value - (pwm_yaw_dutycycle_ptr[1]>>1);
        pwm_yaw_cmp[2][0] = pwm_reload_half_value - (pwm_yaw_dutycycle_ptr[2]>>1);
        pwm_yaw_cmp[0][1] = pwm_reload_value - pwm_yaw_cmp[0][0];
        pwm_yaw_cmp[1][1] = pwm_reload_value - pwm_yaw_cmp[1][0];
        pwm_yaw_cmp[2][1] = pwm_reload_value - pwm_yaw_cmp[2][0];
        pwm_update_raw_cmp_central_aligned(BOARD_ROLL_YAW_PWM_BASE, 6, 7, pwm_yaw_cmp[1][0], pwm_yaw_cmp[1][1]);
        pwm_update_raw_cmp_central_aligned(BOARD_ROLL_YAW_PWM_BASE, 8, 9, pwm_yaw_cmp[0][0], pwm_yaw_cmp[0][1]);
        pwm_update_raw_cmp_central_aligned(BOARD_ROLL_YAW_PWM_BASE, 10, 11, pwm_yaw_cmp[2][0], pwm_yaw_cmp[2][1]);
    }
}
SDK_DECLARE_EXT_ISR_M(BOARD_ROLL_YAW_PWM_IRQ, roll_yaw_isr)

void pitch_pwm_isr(void)
{
    if (pwm_get_status(BOARD_PITCH_PWM_BASE)) 
    {
        pwm_clear_status(BOARD_PITCH_PWM_BASE, PWM_IRQ_HALF_RELOAD);
        pwm_test_cnt[1]++;
        
        pwm_pitch_cmp[0][0] = pwm_reload_half_value - (pwm_pitch_dutycycle_ptr[0]>>1);
        pwm_pitch_cmp[1][0] = pwm_reload_half_value - (pwm_pitch_dutycycle_ptr[1]>>1);
        pwm_pitch_cmp[2][0] = pwm_reload_half_value - (pwm_pitch_dutycycle_ptr[2]>>1);
        pwm_pitch_cmp[0][1] = pwm_reload_value - pwm_pitch_cmp[0][0];
        pwm_pitch_cmp[1][1] = pwm_reload_value - pwm_pitch_cmp[1][0];
        pwm_pitch_cmp[2][1] = pwm_reload_value - pwm_pitch_cmp[2][0];
        pwm_update_raw_cmp_central_aligned(BOARD_PITCH_PWM_BASE, 0, 1, pwm_pitch_cmp[1][0], pwm_pitch_cmp[1][1]);
        pwm_update_raw_cmp_central_aligned(BOARD_PITCH_PWM_BASE, 2, 3, pwm_pitch_cmp[0][0], pwm_pitch_cmp[0][1]);
        pwm_update_raw_cmp_central_aligned(BOARD_PITCH_PWM_BASE, 4, 5, pwm_pitch_cmp[2][0], pwm_pitch_cmp[2][1]);
    }
}
SDK_DECLARE_EXT_ISR_M(BOARD_PITCH_PWM_IRQ, pitch_pwm_isr)

void bsp_pwm_enable_all_pwm_output(void)
{
    pwm_enable_output(BOARD_ROLL_YAW_PWM_BASE, BOARD_ROLL_PWM_1_CH);
    pwm_enable_output(BOARD_ROLL_YAW_PWM_BASE, BOARD_ROLL_PWM_2_CH);
    pwm_enable_output(BOARD_ROLL_YAW_PWM_BASE, BOARD_ROLL_PWM_3_CH);

    pwm_enable_output(BOARD_ROLL_YAW_PWM_BASE, BOARD_YAW_PWM_1_CH);
    pwm_enable_output(BOARD_ROLL_YAW_PWM_BASE, BOARD_YAW_PWM_2_CH);
    pwm_enable_output(BOARD_ROLL_YAW_PWM_BASE, BOARD_YAW_PWM_3_CH);

    pwm_enable_output(BOARD_PITCH_PWM_BASE, BOARD_PITCH_PWM_1_CH);
    pwm_enable_output(BOARD_PITCH_PWM_BASE, BOARD_PITCH_PWM_2_CH);
    pwm_enable_output(BOARD_PITCH_PWM_BASE, BOARD_PITCH_PWM_3_CH);
}

void bsp_pwm_disable_all_pwm_output(void)
{
    pwm_disable_output(BOARD_ROLL_YAW_PWM_BASE, BOARD_ROLL_PWM_1_CH);
    pwm_disable_output(BOARD_ROLL_YAW_PWM_BASE, BOARD_ROLL_PWM_2_CH);
    pwm_disable_output(BOARD_ROLL_YAW_PWM_BASE, BOARD_ROLL_PWM_3_CH);

    pwm_disable_output(BOARD_ROLL_YAW_PWM_BASE, BOARD_YAW_PWM_1_CH);
    pwm_disable_output(BOARD_ROLL_YAW_PWM_BASE, BOARD_YAW_PWM_2_CH);
    pwm_disable_output(BOARD_ROLL_YAW_PWM_BASE, BOARD_YAW_PWM_3_CH);

    pwm_disable_output(BOARD_PITCH_PWM_BASE, BOARD_PITCH_PWM_1_CH);
    pwm_disable_output(BOARD_PITCH_PWM_BASE, BOARD_PITCH_PWM_2_CH);
    pwm_disable_output(BOARD_PITCH_PWM_BASE, BOARD_PITCH_PWM_3_CH);
}

void remo_pwm_set_pwm_dutycycle_ptr(uint16_t *ptr, uint8_t index)
{
    if (index == PITCH)
    {
        pwm_pitch_dutycycle_ptr = ptr;
    }
    else if (index == ROLL)
    {
        pwm_roll_dutycycle_ptr = ptr;
    }
    else if (index == YAW)
    {
        pwm_yaw_dutycycle_ptr = ptr;
    }
}

uint32_t remo_pwm_get_pwm_reload_value(void)
{
    return pwm_reload_value;
}