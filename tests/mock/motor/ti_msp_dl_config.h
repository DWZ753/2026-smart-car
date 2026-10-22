#ifndef TEST_MOTOR_CONFIG_H
#define TEST_MOTOR_CONFIG_H

#include "../ti_msp_dl_config.h"

#define GPIO_MOTORs_INT_IRQN 1
#define GPIO_MOTORs_GPIO_MOTOR1_OUT_A_PORT ((GPIO_Regs *)0)
#define GPIO_MOTORs_GPIO_MOTOR2_OUT_A_PORT ((GPIO_Regs *)0)
#define GPIO_MOTORs_GPIO_MOTOR1_OUT_B_PORT ((GPIO_Regs *)0)
#define GPIO_MOTORs_GPIO_MOTOR2_OUT_B_PORT ((GPIO_Regs *)0)
#define GPIO_MOTORs_GPIO_MOTOR1_OUT_A_PIN 1U
#define GPIO_MOTORs_GPIO_MOTOR2_OUT_A_PIN 2U
#define GPIO_MOTORs_GPIO_MOTOR1_OUT_B_PIN 4U
#define GPIO_MOTORs_GPIO_MOTOR2_OUT_B_PIN 8U
#define GPIO_MOTORs_GPIO_MOTOR1_OUT_A_IIDX 1U
#define GPIO_MOTORs_GPIO_MOTOR2_OUT_A_IIDX 2U
#define DL_INTERRUPT_GROUP_1 1U
#define DL_INTERRUPT_GROUP1_IIDX_GPIOA 1U

static inline uint32_t DL_Interrupt_getPendingGroup(uint32_t group)
{
	return 0U;
}

static inline uint32_t DL_GPIO_getEnabledInterruptStatus(
	GPIO_Regs *port_p, uint32_t pins)
{
	return 0U;
}

static inline void DL_GPIO_clearInterruptStatus(
	GPIO_Regs *port_p, uint32_t pins)
{
}

#endif
