#ifndef MOCK_TB6612_H
#define MOCK_TB6612_H

#include <stdint.h>

#define TB6612_PWM_PERIOD_COUNT 10000U

void TB6612_Init(void);
void TB6612_A_Forward(uint32_t duty);
void TB6612_A_Backward(uint32_t duty);
void TB6612_B_Forward(uint32_t duty);
void TB6612_B_Backward(uint32_t duty);
void TB6612_A_Stop(void);
void TB6612_B_Stop(void);
void TB6612_A_Brake(void);
void TB6612_B_Brake(void);

#endif
