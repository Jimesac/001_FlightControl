#include "svpwm.h"
#include "bsp_pwm.h"
#include "esc_ctrl.h"
#include "user_config.h"

#define BOARD_MOTOR_DRIVER_OFF     gpio_write_pin(BOARD_MOTOR_DRIVER_GPIO_CTRL,  BOARD_MOTOR_DRIVER_ENABLE_GPIO_INDEX, BOARD_MOTOR_DRIVER_ENABLE_GPIO_PIN, 0);
#define BOARD_MOTOR_DRIVER_ON      gpio_write_pin(BOARD_MOTOR_DRIVER_GPIO_CTRL,  BOARD_MOTOR_DRIVER_ENABLE_GPIO_INDEX, BOARD_MOTOR_DRIVER_ENABLE_GPIO_PIN, 1);

#define MOTOR_PITCH_DRIVER_FAULT_FLAG (bool)(0 == gpio_read_pin(BOARD_MOTOR_DRIVER_GPIO_CTRL, BOARD_MOTOR_PITCH_DRIVER_FAULT_GPIO_INDEX, BOARD_MOTOR_PITCH_DRIVER_FAULT_GPIO_PIN))
#define MOTOR_ROLL_DRIVER_FAULT_FLAG  (bool)(0 == gpio_read_pin(BOARD_MOTOR_DRIVER_GPIO_CTRL, BOARD_MOTOR_ROLL_DRIVER_FAULT_GPIO_INDEX, BOARD_MOTOR_ROLL_DRIVER_FAULT_GPIO_PIN))
#define MOTOR_YAW_DRIVER_FAULT_FLAG   (bool)(0 == gpio_read_pin(BOARD_MOTOR_DRIVER_GPIO_CTRL, BOARD_MOTOR_YAW_DRIVER_FAULT_GPIO_INDEX, BOARD_MOTOR_YAW_DRIVER_FAULT_GPIO_PIN))

typedef struct{
    uint8_t pwm_output_flag;      // 定时器pwm输出标志
    uint8_t pwm_output_flag_last;
    uint8_t pwm_test_flag;        // 定时器pwm测试标志
    
    uint8_t driven_fault_flag;
    uint16_t driven_reset_cnt;

    uint16_t pwm_test_output[3];
    uint16_t pwm_output[3];       // 定时器pwm三相输出
}svpwm_params_t;

svpwm_params_t svpwm_params[3] = {
    {
        .driven_fault_flag = 0,
        .driven_reset_cnt = 0,

        .pwm_output_flag = 0,
        .pwm_output_flag_last = 0,
        .pwm_test_flag = 0,
        .pwm_test_output = {0, 0, 0},
        .pwm_output = {0, 0, 0},
    },
    {
        .driven_fault_flag = 0,
        .driven_reset_cnt = 0,

        .pwm_output_flag = 0,
        .pwm_output_flag_last = 0,
        .pwm_test_flag = 0,
        .pwm_test_output = {0, 0, 0},
        .pwm_output = {0, 0, 0},
    },
    {
        .driven_fault_flag = 0,
        .driven_reset_cnt = 0,

        .pwm_output_flag = 0,
        .pwm_output_flag_last = 0,
        .pwm_test_flag = 0,
        .pwm_test_output = {0, 0, 0},
        .pwm_output = {0, 0, 0},
    },
};

void svpwm_init(void)
{
    gpio_interrupt_trigger_t trigger;

    HPM_IOC->PAD[BOARD_MOTOR_DRIVER_ENABLE_GPIO_PIN].FUNC_CTL = BOARD_MOTOR_DRIVER_ENABLE_GPIO_FUNC;
    gpio_set_pin_output_with_initial(BOARD_MOTOR_DRIVER_GPIO_CTRL,  BOARD_MOTOR_DRIVER_ENABLE_GPIO_INDEX, BOARD_MOTOR_DRIVER_ENABLE_GPIO_PIN, 0);


    HPM_IOC->PAD[BOARD_MOTOR_PITCH_DRIVER_FAULT_GPIO_PIN].FUNC_CTL = BOARD_MOTOR_PITCH_DRIVER_FAULT_GPIO_FUNC;
    gpio_set_pin_input(BOARD_MOTOR_DRIVER_GPIO_CTRL, BOARD_MOTOR_PITCH_DRIVER_FAULT_GPIO_INDEX, BOARD_MOTOR_PITCH_DRIVER_FAULT_GPIO_PIN);

    HPM_IOC->PAD[BOARD_MOTOR_ROLL_DRIVER_FAULT_GPIO_PIN].FUNC_CTL = BOARD_MOTOR_ROLL_DRIVER_FAULT_GPIO_FUNC;
    gpio_set_pin_input(BOARD_MOTOR_DRIVER_GPIO_CTRL, BOARD_MOTOR_ROLL_DRIVER_FAULT_GPIO_INDEX, BOARD_MOTOR_ROLL_DRIVER_FAULT_GPIO_PIN);

    HPM_IOC->PAD[BOARD_MOTOR_YAW_DRIVER_FAULT_GPIO_PIN].FUNC_CTL = BOARD_MOTOR_YAW_DRIVER_FAULT_GPIO_FUNC;
    gpio_set_pin_input(BOARD_MOTOR_DRIVER_GPIO_CTRL, BOARD_MOTOR_YAW_DRIVER_FAULT_GPIO_INDEX, BOARD_MOTOR_YAW_DRIVER_FAULT_GPIO_PIN);
    
    trigger = gpio_interrupt_trigger_edge_falling;

    //gpio_config_pin_interrupt(BOARD_MOTOR_DRIVER_GPIO_CTRL, BOARD_MOTOR_PITCH_DRIVER_FAULT_GPIO_INDEX, BOARD_MOTOR_PITCH_DRIVER_FAULT_GPIO_PIN, trigger);
    //gpio_enable_pin_interrupt(BOARD_MOTOR_DRIVER_GPIO_CTRL, BOARD_MOTOR_PITCH_DRIVER_FAULT_GPIO_INDEX, BOARD_MOTOR_PITCH_DRIVER_FAULT_GPIO_PIN);
    //intc_m_enable_irq_with_priority(BOARD_MOTOR_PITCH_DRIVER_FAULT_GPIO_IRQ,  BOARD_MOTOR_PITCH_DRIVER_FAULT_GPIO_IRQ_LEVEL);

    //gpio_config_pin_interrupt(BOARD_MOTOR_DRIVER_GPIO_CTRL, BOARD_MOTOR_ROLL_DRIVER_FAULT_GPIO_INDEX, BOARD_MOTOR_ROLL_DRIVER_FAULT_GPIO_PIN, trigger);
    //gpio_enable_pin_interrupt(BOARD_MOTOR_DRIVER_GPIO_CTRL, BOARD_MOTOR_ROLL_DRIVER_FAULT_GPIO_INDEX, BOARD_MOTOR_ROLL_DRIVER_FAULT_GPIO_PIN);
    //intc_m_enable_irq_with_priority(BOARD_MOTOR_ROLL_DRIVER_FAULT_GPIO_IRQ,  BOARD_MOTOR_ROLL_DRIVER_FAULT_GPIO_IRQ_LEVEL);

    //gpio_config_pin_interrupt(BOARD_MOTOR_DRIVER_GPIO_CTRL, BOARD_MOTOR_YAW_DRIVER_FAULT_GPIO_INDEX, BOARD_MOTOR_YAW_DRIVER_FAULT_GPIO_PIN, trigger);
    //gpio_enable_pin_interrupt(BOARD_MOTOR_DRIVER_GPIO_CTRL, BOARD_MOTOR_YAW_DRIVER_FAULT_GPIO_INDEX, BOARD_MOTOR_YAW_DRIVER_FAULT_GPIO_PIN);
    //intc_m_enable_irq_with_priority(BOARD_MOTOR_YAW_DRIVER_FAULT_GPIO_IRQ,  BOARD_MOTOR_YAW_DRIVER_FAULT_GPIO_IRQ_LEVEL);
    
    BOARD_MOTOR_DRIVER_ON

    clock_cpu_delay_ms(20);
    if (!MOTOR_PITCH_DRIVER_FAULT_FLAG && !MOTOR_ROLL_DRIVER_FAULT_FLAG && !MOTOR_YAW_DRIVER_FAULT_FLAG)
    {
        bsp_pwm_enable_all_pwm_output();
        //svpwm_params[0].pwm_output_flag = true;
        //svpwm_params[1].pwm_output_flag = true;
        //svpwm_params[2].pwm_output_flag = true;
    }

    remo_esc_set_pwm_output_ptr(svpwm_params[PITCH].pwm_output, PITCH);
    remo_esc_set_pwm_output_ptr(svpwm_params[ROLL].pwm_output, ROLL);
    remo_esc_set_pwm_output_ptr(svpwm_params[YAW].pwm_output, YAW);

    remo_pwm_set_pwm_dutycycle_ptr(svpwm_params[PITCH].pwm_output, PITCH);
    remo_pwm_set_pwm_dutycycle_ptr(svpwm_params[ROLL].pwm_output, ROLL);
    remo_pwm_set_pwm_dutycycle_ptr(svpwm_params[YAW].pwm_output, YAW);

    
}

void svpwm_update(void)
{
    uint8_t fault_flag = 0;
    remo_esc_ctrl_update();
    
    for (uint8_t i = 0; i < 3; i++)
    {

        if (svpwm_params[i].driven_fault_flag)
        {
            if (i == PITCH) {fault_flag = MOTOR_PITCH_DRIVER_FAULT_FLAG;}
            else if (i == ROLL) {fault_flag = MOTOR_ROLL_DRIVER_FAULT_FLAG;}
            else if (i == YAW) {fault_flag = MOTOR_YAW_DRIVER_FAULT_FLAG;}
            svpwm_params[i].driven_reset_cnt++;
            if (svpwm_params[i].driven_reset_cnt == 1)
            {
                if (!fault_flag)
                {
                        bsp_pwm_enable_all_pwm_output();
                        svpwm_params[i].driven_fault_flag = 0;
                        svpwm_params[i].driven_reset_cnt = 0;
                        return;
                }
                else 
                {
                        remo_esc_set_esc_status(ESC_ERR, i);
                        bsp_pwm_disable_all_pwm_output();
                }
            }
            else if (svpwm_params[i].driven_reset_cnt == 500)
            {
                svpwm_params[i].driven_reset_cnt = 0;
                
                // RESET驱动芯片
                // ......
                
                if (!fault_flag)
                {
                        bsp_pwm_enable_all_pwm_output();
                        svpwm_params[i].driven_fault_flag = 0;
                        svpwm_params[i].driven_reset_cnt = 0;
                }
            }
        }
        
        if (svpwm_params[i].pwm_output_flag_last != svpwm_params[i].pwm_output_flag)
        {
            if (svpwm_params[i].pwm_output_flag)
            {
                bsp_pwm_enable_all_pwm_output();
                if (svpwm_params[i].pwm_test_flag)
                {
                    remo_pwm_set_pwm_dutycycle_ptr(svpwm_params[PITCH].pwm_test_output, PITCH);
                    remo_pwm_set_pwm_dutycycle_ptr(svpwm_params[ROLL].pwm_test_output, ROLL);
                    remo_pwm_set_pwm_dutycycle_ptr(svpwm_params[YAW].pwm_test_output, YAW);
                }
                else 
                {
                    remo_pwm_set_pwm_dutycycle_ptr(svpwm_params[PITCH].pwm_output, PITCH);
                    remo_pwm_set_pwm_dutycycle_ptr(svpwm_params[ROLL].pwm_output, ROLL);
                    remo_pwm_set_pwm_dutycycle_ptr(svpwm_params[YAW].pwm_output, YAW);
                }
            }
            else
            {
                bsp_pwm_disable_all_pwm_output();
            }
            svpwm_params[i].pwm_output_flag_last = svpwm_params[i].pwm_output_flag;
            return;
        }

        svpwm_params[i].pwm_output_flag_last = svpwm_params[i].pwm_output_flag;
    }
}

void motor_pitch_driver_fault_isr(void)
{
    gpio_clear_pin_interrupt_flag(BOARD_MOTOR_DRIVER_GPIO_CTRL, BOARD_MOTOR_PITCH_DRIVER_FAULT_GPIO_INDEX, BOARD_MOTOR_PITCH_DRIVER_FAULT_GPIO_PIN);
    if (MOTOR_PITCH_DRIVER_FAULT_FLAG)
    {
        svpwm_params[PITCH].driven_fault_flag = true;
        svpwm_params[PITCH].driven_reset_cnt = 0;
    }
}
SDK_DECLARE_EXT_ISR_M(BOARD_MOTOR_PITCH_DRIVER_FAULT_GPIO_IRQ, motor_pitch_driver_fault_isr)

void motor_roll_driver_fault_isr(void)
{
    gpio_clear_pin_interrupt_flag(BOARD_MOTOR_DRIVER_GPIO_CTRL, BOARD_MOTOR_ROLL_DRIVER_FAULT_GPIO_INDEX, BOARD_MOTOR_ROLL_DRIVER_FAULT_GPIO_PIN);
    if (MOTOR_ROLL_DRIVER_FAULT_FLAG)
    {
        svpwm_params[ROLL].driven_fault_flag = true;
        svpwm_params[ROLL].driven_reset_cnt = 0;
    }
}
SDK_DECLARE_EXT_ISR_M(BOARD_MOTOR_ROLL_DRIVER_FAULT_GPIO_IRQ, motor_roll_driver_fault_isr)

void motor_yaw_driver_fault_isr(void)
{
    gpio_clear_pin_interrupt_flag(BOARD_MOTOR_DRIVER_GPIO_CTRL, BOARD_MOTOR_YAW_DRIVER_FAULT_GPIO_INDEX, BOARD_MOTOR_YAW_DRIVER_FAULT_GPIO_PIN);
    if (MOTOR_YAW_DRIVER_FAULT_FLAG)
    {
        svpwm_params[YAW].driven_fault_flag = true;
        svpwm_params[YAW].driven_reset_cnt = 0;
    }
}
SDK_DECLARE_EXT_ISR_M(BOARD_MOTOR_YAW_DRIVER_FAULT_GPIO_IRQ, motor_yaw_driver_fault_isr)
