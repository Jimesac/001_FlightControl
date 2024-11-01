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

static void bsp_adc_hall_init(void)
{
    adc16_config_t cfg;
    adc16_channel_config_t ch_cfg;
    adc16_prd_config_t prd_cfg;
    uint32_t freq = 0;

 
    /* ADC pin initialization */
    HPM_IOC->PAD[BOARD_ADC16_HALL_P1_GPIO_PIN].FUNC_CTL = BOARD_ADC16_HALL_GPIO_FUNC;

    /* Configure the DAC clock to 180MHz */
    clock_set_adc_source(BOARD_ADC16_HALL_CLK_NAME, clk_dac_src_ahb0);
    freq = clock_get_frequency(BOARD_ADC16_HALL_CLK_NAME);

    /* initialize an ADC instance */
    adc16_get_default_config(&cfg);
    cfg.res            = adc16_res_16_bits;
    cfg.conv_mode      = adc16_conv_mode_preemption;
    cfg.adc_clk_div    = adc16_clock_divider_4;
    cfg.sel_sync_ahb   = (clk_adc_src_ahb0 == freq) ? true : false;
    if (cfg.conv_mode == adc16_conv_mode_sequence ||
        cfg.conv_mode == adc16_conv_mode_preemption) {
        cfg.adc_ahb_en = true;
    }

    /* adc16 initialization */
    if (adc16_init(BOARD_ADC16_HALL_BASE, &cfg) == status_success) {
        printf("initialization adc sox success!\n");
    } else {
        printf("initialization adc sox failed!\n");
        return;
    }

    /* get a default channel config */
    adc16_get_channel_default_config(&ch_cfg);
    /* initialize an ADC channel */
    ch_cfg.sample_cycle = 20;
    ch_cfg.ch           = BOARD_ADC16_HALL_P1_CH;
    adc16_init_channel(BOARD_ADC16_HALL_BASE, &ch_cfg);
    ch_cfg.ch           = BOARD_ADC16_HALL_P2_CH;
    adc16_init_channel(BOARD_ADC16_HALL_BASE, &ch_cfg);
    ch_cfg.ch           = BOARD_ADC16_HALL_R1_CH;
    adc16_init_channel(BOARD_ADC16_HALL_BASE, &ch_cfg);
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
    adc16_pmt_config_t pmt_cfg;
    pmt_cfg.trig_len = 4;
    pmt_cfg.adc_ch[0] = BOARD_ADC16_HALL_P1_CH;
    pmt_cfg.adc_ch[1] = BOARD_ADC16_HALL_P2_CH;
    pmt_cfg.adc_ch[2] = BOARD_ADC16_HALL_R1_CH;
    pmt_cfg.adc_ch[3] = BOARD_ADC16_HALL_R2_CH;
    pmt_cfg.trig_ch = BOARD_HALL_ADC_TRG;
    pmt_cfg.inten[0] = false;
    pmt_cfg.inten[1] = false;
    pmt_cfg.inten[2] = false;
    pmt_cfg.inten[3] = true;
    adc16_set_pmt_config(BOARD_ADC16_HALL_BASE, &pmt_cfg);
    adc16_set_pmt_queue_enable(BOARD_ADC16_HALL_BASE, BOARD_HALL_ADC_TRG, true);

    adc16_init_pmt_dma(BOARD_ADC16_HALL_BASE, (uint32_t )adc_pmt_dma_buf);
    
    /* Enable trigger complete interrupt */
    adc16_enable_interrupts(BOARD_ADC16_HALL_BASE, BOARD_ADC16_HALL_PMT_IRQ_EVENT);
    intc_m_enable_irq_with_priority(BOARD_ADC16_HALL_IRQn, BOARD_ADC16_HALL_IRQn_LEVEL);
}

void hall_adc16_isr(void)
{
    uint32_t status;
    uint32_t adc_hall_temp[2][2] = {0};

    status = adc16_get_status_flags(BOARD_ADC16_HALL_BASE);

    /* Clear status */
    adc16_clear_status_flags(BOARD_ADC16_HALL_BASE, status);

    if (ADC16_INT_STS_TRIG_CMPT_GET(status)) {
        /* Set flag to read memory data */
        //trig_complete_flag = 1;
        hall_adc_isr_cnt++;
        //adc_hall_smp[0] = adc_pmt_dma_buf[0].res;
        //adc_hall_smp[1] = adc_pmt_dma_buf[1].res;
        //adc_hall_smp[2] = adc_pmt_dma_buf[2].res;
        //adc_hall_smp[3] = adc_pmt_dma_buf[3].res;
        
        adc_hall_median[PITCH][0][adc_hall_median_cnt] = adc_pmt_dma_buf[0].res;
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
        adc_hall_filter[PITCH][0] = filter_median_u16w7(&adc_hall_median[PITCH][0][0]);
        adc_hall_filter[PITCH][1] = filter_median_u16w7(&adc_hall_median[PITCH][1][0]);
        adc_hall_filter[ROLL][0] = filter_median_u16w7(&adc_hall_median[ROLL][0][0]);
        adc_hall_filter[ROLL][1] = filter_median_u16w7(&adc_hall_median[ROLL][1][0]);
#else
        for (uint8_t i = 0; i < 2; i++)
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
        if (++adc_hall_median_cnt >= ADC_HALL_FILTER_SIZE) adc_hall_median_cnt = 0;

    }

}
SDK_DECLARE_EXT_ISR_M(BOARD_ADC16_HALL_IRQn, hall_adc16_isr)


uint16_t *bsp_adc_get_hall_pitch_filter_ptr(void)
{
    return &adc_hall_filter[PITCH][0];
}

uint16_t *bsp_adc_get_hall_roll_filter_ptr(void)
{
    return &adc_hall_filter[ROLL][0];
}

