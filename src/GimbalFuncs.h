#ifndef GIMBAL_FUNCS_H
#define GIMBAL_FUNCS_H

int getEncoderTheta();

void Pitch_PWM_Init(void);
void Roll_PWM_Init(void);
void Yaw_PWM_Init(void);

void Pitch_CH1_SetDutyCycle(unsigned int dutyPercent);
void Pitch_CH2_SetDutyCycle(unsigned int dutyPercent);
void Pitch_CH3_SetDutyCycle(unsigned int dutyPercent);

void Roll_CH1_SetDutyCycle(unsigned int dutyPercent);
void Roll_CH2_SetDutyCycle(unsigned int dutyPercent);
void Roll_CH3_SetDutyCycle(unsigned int dutyPercent);

void Yaw_CH1_SetDutyCycle(unsigned int dutyPercent);
void Yaw_CH2_SetDutyCycle(unsigned int dutyPercent);
void Yaw_CH3_SetDutyCycle(unsigned int dutyPercent);

#endif // GIMBAL_FUNCS_H