/*********************************************************************
*                    SEGGER Microcontroller GmbH                     *
*                        The Embedded Experts                        *
**********************************************************************

-------------------------- END-OF-HEADER -----------------------------

File    : main.c
Purpose : Generic application start

*/

#include <stdio.h>
#include <math.h>

#include "hpm_clock_drv.h"
#include "hpm_pcfg_drv.h"
#include "hpm_pllctlv2_drv.h"
#include "hpm_usb_drv.h"

#include "bsp_adc.h"
#include "bsp_led.h"
#include "bsp_timer.h"
#include "bsp_uart.h"
#include "bsp_spi.h"
#include "bsp_pwm.h"
#include "bsp_dma.h"
#include "bsp_flash.h"

#include "angle_encoder.h"
#include "imu.h"
#include "svpwm.h"
#include "logger.h"

#include "esc_ctrl.h"
#include "ahrs.h"
#include "ctrl_loop.h"
#include "calibrate.h"
#include "user_config.h"

#if defined(FLASH_XIP) && FLASH_XIP
__attribute__ ((section(".nor_cfg_option"))) const uint32_t option[4] = {0xfcf90002, 0x00000006, 0x1000, 0x0};
#endif

static void board_init_clock(void);
static void board_special_gpio_init(void);
static void board_test_gpio_init(void);

#define BOARD_TEST_GPIO_ON       gpio_write_pin(HPM_GPIO0,  GPIO_DI_GPIOB, IOC_PAD_PB09, 1);
#define BOARD_TEST_GPIO_OFF      gpio_write_pin(HPM_GPIO0,  GPIO_DI_GPIOB, IOC_PAD_PB09, 0);

uint32_t main_cnt = 0;
uint8_t spi_com_tx_data[5] = {0x12, 0x23, 0x34, 0x45, 0x67};
uint8_t spi_com_rx_data[5] = {0};

uint16_t adc_vbus_test_smp = 0;

uint32_t main_test_us[10][2] = {0};
float f_test[2] = {1.0f, 1.0f};

float flash_test[2][2] = {{1.025f, -0.135f}, {0.0f, 0.0f}};
/*********************************************************************
*
*       main()
*
*  Function description
*   Application entry point.
*/  
int main(void) {
    int i;

    board_init_clock();             //始终初始化
    board_special_gpio_init();      //GPIO Y的初始化
    board_test_gpio_init();         //设置PB9为高电平


    ///* FPU TEST */                //浮点运算的测试
    //while(1){
    //    f_test[0] = 1.0f;
    //    f_test[1] = -0.9f; 
    //    for (uint16_t i = 0; i < 2000; i++)
    //    {
    //        //f_test[0] = f_test[1] - f_test[0];
    //        f_test[0] = fabs(f_test[1]);
    //    }
    //    main_cnt++;
    //}

    disable_irq_from_intc();        //禁用中断
    bsp_flash_init();               //初始化闪存
    bsp_uart_init();                //初始化 UART 通信接口          !!!!!!!!
    bsp_adc_init();                 //初始化模数转换器（ADC）
    bsp_led_init();                 //初始化 LED
    bsp_timer_init();               //初始化定时器
    bsp_spi_init();                 //初始化 SPI 接口
    bsp_pwm_init();                 //初始化 PWM，用于电机控制
    bsp_dma_init();                 //初始化 DMA，用于高效数据传输

    //bsp_flash_user_params_write_words(0, (const uint32_t *)&flash_test[0][0], 2);
    //bsp_flash_user_params_read_words(0, (uint32_t *)&flash_test[1][0], 2);
    
    angle_encoder_init();           //初始化角度编码器，用于测量角度
    remo_imu_init();                //初始化惯性测量单元（IMU）
    svpwm_init();                   //初始化 SVPWM（空间矢量脉宽调制）算法，用于无刷电机控制

    bsp_magencoder_imu_spi_receive_init();  //初始化 SVPWM（空间矢量脉宽调制）算法，用于无刷电机控制

    remo_esc_ctrl_init();           //初始化电调控制模块
    remo_ctrl_loop_params_init();   // 配置控制环路的参数
    remo_cali_init();               //初始化校准模块

    enable_irq_from_intc();         //启用中断  

    ahrs_init();                    //初始化航向参考系统（AHRS）
    


    logger_printf_convert_to_usart_debugger(kInfo, 100, "debug test %d %d %s %f\n", -50, 300, "test", -0.256);  //使用串口调试器输出格式化调试信息

    printf("Hello World %d!\n", 0);

    do {
        
        //if (main_cnt % 300000 == 0)
        if (bsp_timer_schedule_get_update_flag())           //检查定时器调度标志是否被置位
        {
            main_cnt++;                                     //主计数器加1

            main_test_us[0][1] = bsp_timer_clock_end_0p1us(main_test_us[0][0]);     // 结束计时
            main_test_us[0][0] = bsp_timer_clock_start_0p1us();                     // 开始测量0.1us的时间
            
            // imu的加速度计值和磁编码的值交替采样
            BOARD_TEST_GPIO_OFF                                                     // 关闭PB9的输出     
            main_test_us[1][0] = bsp_timer_clock_start_0p1us();                     // 开始计时
            remo_imu_update();                                                      // 更新惯性测量单元（IMU）的数据
            main_test_us[1][1] = bsp_timer_clock_end_0p1us(main_test_us[1][0]);     // 结束计时
            if (schedule_ptmr_state % 2 == MAG_ENCODE_SAMPLE_SCHD_INDEX)            // 0
            {
                main_test_us[2][0] = bsp_timer_clock_start_0p1us();                 // 重新开始计时
                angle_encoder_update();                                             // 更新角度编码器的数据
                main_test_us[2][1] = bsp_timer_clock_end_0p1us(main_test_us[2][0]); // 结束计时
            }
            BOARD_TEST_GPIO_ON      // 打开PB9的输出

            switch(schedule_ptmr_state)                                             // 选择调度状态
            {
                case SCHEDULE_ATT_UPDATA:                                           // 更新姿态估计
                    main_test_us[3][0] = bsp_timer_clock_start_0p1us();             // 开始计时
                    ahrs_update_attitude(SCHEDULE_SECTION_TIME);                    // 更新姿态估计
                    main_test_us[3][1] = bsp_timer_clock_end_0p1us(main_test_us[3][0]); // 结束计时
                    break;
                case SCHEDULE_POS_PLAN:                                             // 执行校准操作
                    main_test_us[4][0] = bsp_timer_clock_start_0p1us();
                    remo_cali_run();                                                // 执行校准操作
                    ahrs_dcm_euler_update();                                        // 更新方向余弦矩阵和欧拉角
                    remo_ctrl_loop_pos_ctrl();                                      // 执行位置控制
                    main_test_us[4][1] = bsp_timer_clock_end_0p1us(main_test_us[4][0]);
                    break;
                case SCHEDULE_COM:                                                  // 执行通信操作 
                    main_test_us[5][0] = bsp_timer_clock_start_0p1us();
                    main_test_us[5][1] = bsp_timer_clock_end_0p1us(main_test_us[5][0]);
                    break;
                case SCJEDULE_LOGGER:                                               // 执行日志记录操作 
                    break;
                default:
                    break;
            }
            
            main_test_us[6][0] = bsp_timer_clock_start_0p1us();
            remo_ctrl_loop_vel_ctrl();                                              // 执行速度控制
            main_test_us[6][0] =  bsp_timer_clock_end_0p1us(main_test_us[6][0]);

            if (main_cnt % 600 == 0)                    // 每600次循环执行一次
            {
                BOARD_LED1_TOGGLE                       // 切换LED1的状态 PA2
            }

            svpwm_update();                             // 执行SVPWM算法
            bsp_timer_schedule_reset_update_flag();
        }

        bsp_uart_dbg_schedule_transmit();               // 执行串口调试器的调度传输
        
    } while (1);

}


//时钟初始化
static void board_init_clock(void)
{
    uint32_t cpu0_freq = clock_get_frequency(clock_cpu0);   //获取当前 CPU0 的时钟频率 

    if (cpu0_freq == PLLCTL_SOC_PLL_REFCLK_FREQ) {          // 若 CPU 时钟频率等于外部参考时钟频率 24MHz
        /* Configure the External OSC ramp-up time: ~9ms */
        pllctlv2_xtal_set_rampup_time(HPM_PLLCTLV2, 32UL * 1000UL * 9U);    // 配置外部晶振的上升时间为 9ms  //32MHZ的话    32*1000*9/32 000 000=9ms

        /* Select clock setting preset1 */
        sysctl_clock_set_preset(HPM_SYSCTL, 2);                             // 应用预设的时钟配置方案 （编号为 2）
    }

    /* group0[0] */
    clock_add_to_group(clock_cpu0, 0);
    clock_add_to_group(clock_ahb, 0);
    clock_add_to_group(clock_lmm0, 0);
    clock_add_to_group(clock_mchtmr0, 0);
    clock_add_to_group(clock_rom, 0);
    //clock_add_to_group(clock_can0, 0);
    //clock_add_to_group(clock_can1, 0);
    //clock_add_to_group(clock_can2, 0);
    //clock_add_to_group(clock_can3, 0);
    clock_add_to_group(clock_ptpc, 0);
    clock_add_to_group(clock_gptmr0, 0);
    clock_add_to_group(clock_gptmr1, 0);
    clock_add_to_group(clock_gptmr2, 0);
    clock_add_to_group(clock_gptmr3, 0);
    clock_add_to_group(clock_i2c0, 0);
    clock_add_to_group(clock_i2c1, 0);
    clock_add_to_group(clock_i2c2, 0);
    clock_add_to_group(clock_i2c3, 0);
    clock_add_to_group(clock_spi0, 0);
    clock_add_to_group(clock_spi1, 0);
    clock_add_to_group(clock_spi2, 0);
    clock_add_to_group(clock_spi3, 0);
    clock_add_to_group(clock_uart0, 0);
    clock_add_to_group(clock_uart1, 0);
    clock_add_to_group(clock_uart2, 0);
    clock_add_to_group(clock_uart3, 0);
    clock_add_to_group(clock_uart4, 0);
    clock_add_to_group(clock_uart5, 0);
    clock_add_to_group(clock_uart6, 0);
    /* group0[1] */
    clock_add_to_group(clock_uart7, 0);
    clock_add_to_group(clock_watchdog0, 0);
    clock_add_to_group(clock_watchdog1, 0);
    clock_add_to_group(clock_mbx0, 0);
    clock_add_to_group(clock_tsns, 0);
    clock_add_to_group(clock_crc0, 0);
    clock_add_to_group(clock_adc0, 0);
    clock_add_to_group(clock_adc1, 0);
    //clock_add_to_group(clock_dac0, 0);
    //clock_add_to_group(clock_dac1, 0);
    clock_add_to_group(clock_acmp, 0);
    clock_add_to_group(clock_opa0, 0);
    clock_add_to_group(clock_opa1, 0);
    clock_add_to_group(clock_mot0, 0);
    clock_add_to_group(clock_rng, 0);
    clock_add_to_group(clock_sdp, 0);
    clock_add_to_group(clock_kman, 0);
    clock_add_to_group(clock_gpio, 0);
    clock_add_to_group(clock_hdma, 0);
    clock_add_to_group(clock_xpi0, 0);
    clock_add_to_group(clock_usb0, 0);

    /* Connect Group0 to CPU0 */
    clock_connect_group_to_cpu(0, 0);         // 将时钟组0连接到 CPU0

    /* Bump up DCDC voltage to 1175mv */
    pcfg_dcdc_set_voltage(HPM_PCFG, 1175);    // 设置 DCDC 电压为 1175mv

    /* Configure CPU to 480MHz, AXI/AHB to 160MHz */
    sysctl_config_cpu0_domain_clock(HPM_SYSCTL, clock_source_pll0_clk0, 2, 3);  // 配置 CPU0 的时钟频率为 480MHz，AXI/AHB 的时钟频率为 160MHz
    /* Configure PLL0 Post Divider */
    pllctlv2_set_postdiv(HPM_PLLCTLV2, 0, 0, 0);    /* PLL0CLK0: 960MHz */      // 配置 PLL0CLK0 的时钟频率为 960MHz
    pllctlv2_set_postdiv(HPM_PLLCTLV2, 0, 1, 3);    /* PLL0CLK1: 600MHz */      // 600/3=200MHz
    pllctlv2_set_postdiv(HPM_PLLCTLV2, 0, 2, 7);    /* PLL0CLK2: 400MHz */      // 400/7=57MHz
    /* Configure PLL0 Frequency to 960MHz */
    pllctlv2_init_pll_with_freq(HPM_PLLCTLV2, 0, 960000000);                    // 配置 PLL0 的时钟频率为 960MHz

    clock_update_core_clock();                                                  // 更新核心时钟

    /* Configure mchtmr to 24MHz */
    clock_set_source_divider(clock_mchtmr0, clk_src_osc24m, 1);                 // 配置 mchtmr0 的时钟频率为 24MHz
}

static void board_test_gpio_init(void)
{
    HPM_IOC->PAD[IOC_PAD_PB09].FUNC_CTL = IOC_PB09_FUNC_CTL_GPIO_B_09;              // 配置 PB09 引脚的功能为 GPIO_B_09
    gpio_set_pin_output_with_initial(HPM_GPIO0,  GPIO_DI_GPIOB, IOC_PAD_PB09, 1);   // 将 PB09 引脚设置为输出模式，并初始化为高电平
}



////GPIO Y的初始化
static void board_special_gpio_init(void)
{
/*
 * Note:
 *  PY and PZ IOs: if any SOC pin function needs to be routed to these IOs, PY和PZ io:如果任何SOC引脚功能需要路由到这些io，
 *  besides of IOC, PIOC/BIOC needs to be configured SOC_GPIO_X_xx, so that
 *  expected SoC function can be enabled on these IOs.
 * 除了IOC，PIOC/BIOC需要配置SOC_GPIO_X_xx，以便在这些IO上启用预期的SOC功能
 *
 */
    HPM_PIOC->PAD[IOC_PAD_PY00].FUNC_CTL = PIOC_PY00_FUNC_CTL_SOC_GPIO_Y_00;        //配置PY00引脚的功能为SOC_GPIO_Y_00
    HPM_PIOC->PAD[IOC_PAD_PY01].FUNC_CTL = PIOC_PY01_FUNC_CTL_SOC_GPIO_Y_01;        //配置PY01引脚的功能为SOC_GPIO_Y_01
    HPM_PIOC->PAD[IOC_PAD_PY02].FUNC_CTL = PIOC_PY02_FUNC_CTL_SOC_GPIO_Y_02;        //配置PY02引脚的功能为SOC_GPIO_Y_02
    HPM_PIOC->PAD[IOC_PAD_PY03].FUNC_CTL = PIOC_PY03_FUNC_CTL_SOC_GPIO_Y_03;        //配置PY03引脚的功能为SOC_GPIO_Y_03

    HPM_USB0->PHY_CTRL0 |= 0x001000E0u;                                         //配置USB0的PHY_CTRL0寄存器
    clock_disable(clock_usb0);                                                  //禁用USB0的时钟
}

/*************************** End of file ****************************/



