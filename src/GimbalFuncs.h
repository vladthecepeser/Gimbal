#ifndef GIMBAL_FUNCS_H
#define GIMBAL_FUNCS_H

int getEncoderTheta();

void TIM1_PWM_Init(void);
void TIM8_PWM_Init(void);
void TIM20_PWM_Init(void);

void TIM1_CH1_SetDutyCycle(unsigned int dutyPercent);
void TIM1_CH2_SetDutyCycle(unsigned int dutyPercent);
void TIM1_CH3_SetDutyCycle(unsigned int dutyPercent);

void TIM8_CH1_SetDutyCycle(unsigned int dutyPercent);
void TIM8_CH2_SetDutyCycle(unsigned int dutyPercent);
void TIM8_CH3_SetDutyCycle(unsigned int dutyPercent);

void TIM20_CH1_SetDutyCycle(unsigned int dutyPercent);
void TIM20_CH2_SetDutyCycle(unsigned int dutyPercent);
void TIM20_CH3_SetDutyCycle(unsigned int dutyPercent);

#endif // GIMBAL_FUNCS_H