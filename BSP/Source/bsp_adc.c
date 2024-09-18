#include "bsp_adc.h"
#include <stdio.h>

static void bsp_adc_version_init(void);
static void bsp_adc_vbus_init(void);

uint16_t adc_version_sample = 0, adc_version_mv = 0;

void bsp_adc_init(void)
{
    bsp_adc_version_init();
    bsp_adc_vbus_init();
}

static void bsp_adc_version_init(void)
{
    adc16_config_t cfg;
    adc16_channel_config_t ch_cfg;
    uint32_t freq = 0;

 
    /* ADC pin initialization */
    HPM_IOC->PAD[BOARD_ADC16_VERSION_GPIO_PIN].FUNC_CTL = BOARD_ADC16_VERSION_GPIO_FUNC;

    /* Configure the DAC clock to 180MHz */
    clock_set_adc_source(BOARD_ADC16_VERSION_CLK_NAME, clk_dac_src_ahb0);
    freq = clock_get_frequency(BOARD_ADC16_VERSION_CLK_NAME);

    /* initialize an ADC instance */
    adc16_get_default_config(&cfg);
    cfg.res            = adc16_res_16_bits;
    cfg.conv_mode      = adc16_conv_mode_oneshot;
    cfg.adc_clk_div    = adc16_clock_divider_4;
    cfg.sel_sync_ahb   = (clk_adc_src_ahb0 == freq) ? true : false;
    if (cfg.conv_mode == adc16_conv_mode_sequence ||
        cfg.conv_mode == adc16_conv_mode_preemption) {
        cfg.adc_ahb_en = true;
    }
    /* adc16 initialization */
    if (adc16_init(BOARD_ADC16_VERSION_BASE, &cfg) == status_success) {
        printf("initialization adc version success!\n");
    } else {
        printf("initialization adc version failed!\n");
        return;
    }


    /* get a default channel config */
    adc16_get_channel_default_config(&ch_cfg);
    /* initialize an ADC channel */
    ch_cfg.ch           = BOARD_ADC16_VERSION_CH;
    ch_cfg.sample_cycle = 200;
    adc16_init_channel(BOARD_ADC16_VERSION_BASE, &ch_cfg);
    adc16_set_nonblocking_read(BOARD_ADC16_VERSION_BASE);

    /* enable oneshot mode */
    adc16_enable_oneshot_mode(BOARD_ADC16_VERSION_BASE);

    while (adc16_get_oneshot_result(BOARD_ADC16_VERSION_BASE, BOARD_ADC16_VERSION_CH, &adc_version_sample) != status_success) 
    {
        if (adc16_is_nonblocking_mode(BOARD_ADC16_VERSION_BASE)) {
            adc16_get_oneshot_result(BOARD_ADC16_VERSION_BASE, BOARD_ADC16_VERSION_CH, &adc_version_sample);
        }
        adc_version_mv = ((uint32_t)adc_version_sample*825) >> 14;
        printf("Version ADC - Result : %dmV\n", adc_version_mv);
        return;
    }
}

static void bsp_adc_vbus_init(void)
{
    adc16_config_t cfg;
    adc16_channel_config_t ch_cfg;
    adc16_prd_config_t prd_cfg;
    uint32_t freq = 0;

 
    /* ADC pin initialization */
    HPM_IOC->PAD[BOARD_ADC16_VBUS_GPIO_PIN].FUNC_CTL = BOARD_ADC16_VBUS_GPIO_FUNC;

    /* Configure the DAC clock to 180MHz */
    clock_set_adc_source(BOARD_ADC16_VBUS_CLK_NAME, clk_dac_src_ahb0);
    freq = clock_get_frequency(BOARD_ADC16_VBUS_CLK_NAME);

    /* initialize an ADC instance */
    adc16_get_default_config(&cfg);
    cfg.res            = adc16_res_16_bits;
    cfg.conv_mode      = adc16_conv_mode_period;
    cfg.adc_clk_div    = adc16_clock_divider_4;
    cfg.sel_sync_ahb   = (clk_adc_src_ahb0 == freq) ? true : false;
    if (cfg.conv_mode == adc16_conv_mode_sequence ||
        cfg.conv_mode == adc16_conv_mode_preemption) {
        cfg.adc_ahb_en = true;
    }
    /* adc16 initialization */
    if (adc16_init(BOARD_ADC16_VBUS_BASE, &cfg) == status_success) {
        printf("initialization adc vbus success!\n");
    } else {
        printf("initialization adc vbus failed!\n");
        return;
    }


    /* get a default channel config */
    adc16_get_channel_default_config(&ch_cfg);
    /* initialize an ADC channel */
    ch_cfg.ch           = BOARD_ADC16_VBUS_CH;
    ch_cfg.sample_cycle = 20;
    adc16_init_channel(BOARD_ADC16_VBUS_BASE, &ch_cfg);
    prd_cfg.ch           = BOARD_ADC16_VBUS_CH;
    prd_cfg.prescale     = 16;    /* Set divider: 2^22 clocks */
    prd_cfg.period_count = 3;     /* 0.983ms when AHB clock at 200MHz is ADC clock source */
    adc16_set_prd_config(BOARD_ADC16_VBUS_BASE, &prd_cfg);
}

static void bsp_adc_yaw_current_init(void)
{
}



