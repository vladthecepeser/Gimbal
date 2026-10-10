#include "stm32g4xx.h"
#include "GimbalFuncs.h"

// Timer channel	Pin
// TIM1 CH1–CH3	    PC0, PC1, PA10
// TIM8 CH1–CH3	    PB6, PB8, PB9
// TIM20 CH1–CH3	PB2, PC2, PC8

//Don't need complimentary switching for DRV8313 -- handled internally by the DRV8313.  
//So we can use the normal PWM outputs without the complementary ones.

void TIM1_PWM_Init(void)
{
    // Assumes TIM1CLK = 170 MHz:
    // counter clock = 170 MHz / (33 + 1) = 5 MHz
    // center-aligned period = 2 * ARR = 200 ticks
    // PWM frequency = 5 MHz / 200 = 25 kHz
    // CCR1-3 values 0..100 provide approximately 1% duty increments.

    // Enable GPIOA/GPIOC and TIM1 clocks.
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    (void)RCC->APB2ENR; // Read back clock-enable register.

    // PC0/PC1 -> TIM1_CH1/CH2 (AF2); PA10 -> TIM1_CH3 (AF6).
    GPIOA->MODER =
        (GPIOA->MODER & ~GPIO_MODER_MODE10_Msk) |
        (2U << GPIO_MODER_MODE10_Pos);

    GPIOA->OSPEEDR =
        (GPIOA->OSPEEDR & ~GPIO_OSPEEDR_OSPEED10_Msk) |
        (3U << GPIO_OSPEEDR_OSPEED10_Pos);

    GPIOA->AFR[1] =
        (GPIOA->AFR[1] & ~GPIO_AFRH_AFSEL10_Msk) |
        (6U << GPIO_AFRH_AFSEL10_Pos);

    GPIOC->MODER =
        (GPIOC->MODER & ~(GPIO_MODER_MODE0_Msk | GPIO_MODER_MODE1_Msk)) |
        (2U << GPIO_MODER_MODE0_Pos) |
        (2U << GPIO_MODER_MODE1_Pos);

    GPIOC->OSPEEDR =
        (GPIOC->OSPEEDR & ~(GPIO_OSPEEDR_OSPEED0_Msk | GPIO_OSPEEDR_OSPEED1_Msk)) |
        (3U << GPIO_OSPEEDR_OSPEED0_Pos) |
        (3U << GPIO_OSPEEDR_OSPEED1_Pos);

    GPIOC->AFR[0] =
        (GPIOC->AFR[0] & ~(GPIO_AFRL_AFSEL0_Msk | GPIO_AFRL_AFSEL1_Msk)) |
        (2U << GPIO_AFRL_AFSEL0_Pos) |
        (2U << GPIO_AFRL_AFSEL1_Pos);

    // Configure TIM1 while stopped.
    TIM1->CR1 &= ~TIM_CR1_CEN;

    TIM1->PSC = 33U;
    TIM1->ARR = 100U;
    TIM1->RCR = 0U;

    // Center-aligned mode 1 (CMS = 01), initially count upward.
    TIM1->CR1 &= ~(TIM_CR1_CMS_Msk | TIM_CR1_DIR);
    TIM1->CR1 |= TIM_CR1_CMS_0 | TIM_CR1_ARPE;

    // CH1-3: output compare, PWM mode 1, preload enabled.
    TIM1->CCMR1 &= ~(TIM_CCMR1_CC1S_Msk | TIM_CCMR1_OC1M_Msk |
                     TIM_CCMR1_CC2S_Msk | TIM_CCMR1_OC2M_Msk);
    TIM1->CCMR1 |= (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE |
                   (6U << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;

    TIM1->CCMR2 &= ~(TIM_CCMR2_CC3S_Msk | TIM_CCMR2_OC3M_Msk);
    TIM1->CCMR2 |= (6U << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE;

    TIM1->CCR1 = 0U;
    TIM1->CCR2 = 0U;
    TIM1->CCR3 = 0U;

    // Active-high outputs.
    TIM1->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC1NP |
                    TIM_CCER_CC2P | TIM_CCER_CC2NP |
                    TIM_CCER_CC3P | TIM_CCER_CC3NP);

    // Load PSC/ARR/CCR preloads, then clear the update flag it sets.
    TIM1->EGR = TIM_EGR_UG;
    TIM1->SR = 0U;

    // Enable CH1-3 and TIM1's advanced-timer main output.
    TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E;
    TIM1->BDTR |= TIM_BDTR_MOE;

    // Start the counter.
    TIM1->CR1 |= TIM_CR1_CEN;
}

void TIM8_PWM_Init(void)
{
    // Assumes TIM8CLK = 170 MHz:
    // counter clock = 170 MHz / (33 + 1) = 5 MHz
    // center-aligned period = 2 * ARR = 200 ticks
    // PWM frequency = 5 MHz / 200 = 25 kHz
    // CCR1-3 values 0..100 provide approximately 1% duty increments.

    // Enable GPIOB/GPIOC and TIM8 clocks.
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;
    (void)RCC->APB2ENR;

    // PB6/PB8/PB9 -> TIM8_CH1/CH2/CH3 (AF5/AF10/AF10).
    GPIOB->MODER =
        (GPIOB->MODER &
         ~(GPIO_MODER_MODE6_Msk | GPIO_MODER_MODE8_Msk | GPIO_MODER_MODE9_Msk)) |
        (2U << GPIO_MODER_MODE6_Pos) |
        (2U << GPIO_MODER_MODE8_Pos) |
        (2U << GPIO_MODER_MODE9_Pos);

    GPIOB->OSPEEDR =
        (GPIOB->OSPEEDR &
         ~(GPIO_OSPEEDR_OSPEED6_Msk | GPIO_OSPEEDR_OSPEED8_Msk | GPIO_OSPEEDR_OSPEED9_Msk)) |
        (3U << GPIO_OSPEEDR_OSPEED6_Pos) |
        (3U << GPIO_OSPEEDR_OSPEED8_Pos) |
        (3U << GPIO_OSPEEDR_OSPEED9_Pos);

    GPIOB->AFR[0] =
        (GPIOB->AFR[0] & ~GPIO_AFRL_AFSEL6_Msk) |
        (5U << GPIO_AFRL_AFSEL6_Pos);

    GPIOB->AFR[1] =
        (GPIOB->AFR[1] & ~(GPIO_AFRH_AFSEL8_Msk | GPIO_AFRH_AFSEL9_Msk)) |
        (10U << GPIO_AFRH_AFSEL8_Pos) |
        (10U << GPIO_AFRH_AFSEL9_Pos);

    TIM8->CR1 &= ~TIM_CR1_CEN;

    TIM8->PSC = 33U;
    TIM8->ARR = 100U;
    TIM8->RCR = 0U;

    TIM8->CR1 &= ~(TIM_CR1_CMS_Msk | TIM_CR1_DIR);
    TIM8->CR1 |= TIM_CR1_CMS_0 | TIM_CR1_ARPE;

    TIM8->CCMR1 &= ~(TIM_CCMR1_CC1S_Msk | TIM_CCMR1_OC1M_Msk |
                     TIM_CCMR1_CC2S_Msk | TIM_CCMR1_OC2M_Msk);
    TIM8->CCMR1 |= (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE |
                   (6U << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;

    TIM8->CCMR2 &= ~(TIM_CCMR2_CC3S_Msk | TIM_CCMR2_OC3M_Msk);
    TIM8->CCMR2 |= (6U << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE;

    TIM8->CCR1 = 0U;
    TIM8->CCR2 = 0U;
    TIM8->CCR3 = 0U;
    TIM8->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC1NP |
                    TIM_CCER_CC2P | TIM_CCER_CC2NP |
                    TIM_CCER_CC3P | TIM_CCER_CC3NP);

    TIM8->EGR = TIM_EGR_UG;
    TIM8->SR = 0U;

    TIM8->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E;
    TIM8->BDTR |= TIM_BDTR_MOE;
    TIM8->CR1 |= TIM_CR1_CEN;
}

void TIM20_PWM_Init(void)
{
    // Assumes TIM20CLK = 170 MHz:
    // counter clock = 170 MHz / (33 + 1) = 5 MHz
    // center-aligned period = 2 * ARR = 200 ticks
    // PWM frequency = 5 MHz / 200 = 25 kHz
    // CCR1-3 values 0..100 provide approximately 1% duty increments.

    // Enable GPIOB/GPIOC and TIM20 clocks.
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM20EN;
    (void)RCC->APB2ENR;

    // PB2 -> TIM20_CH1 (AF3); PC2/PC8 -> TIM20_CH2/CH3 (AF6).
    GPIOB->MODER =
        (GPIOB->MODER & ~GPIO_MODER_MODE2_Msk) |
        (2U << GPIO_MODER_MODE2_Pos);

    GPIOB->OSPEEDR =
        (GPIOB->OSPEEDR & ~GPIO_OSPEEDR_OSPEED2_Msk) |
        (3U << GPIO_OSPEEDR_OSPEED2_Pos);

    GPIOB->AFR[0] =
        (GPIOB->AFR[0] & ~GPIO_AFRL_AFSEL2_Msk) |
        (3U << GPIO_AFRL_AFSEL2_Pos);

    GPIOC->MODER =
        (GPIOC->MODER & ~(GPIO_MODER_MODE2_Msk | GPIO_MODER_MODE8_Msk)) |
        (2U << GPIO_MODER_MODE2_Pos) |
        (2U << GPIO_MODER_MODE8_Pos);

    GPIOC->OSPEEDR =
        (GPIOC->OSPEEDR & ~(GPIO_OSPEEDR_OSPEED2_Msk | GPIO_OSPEEDR_OSPEED8_Msk)) |
        (3U << GPIO_OSPEEDR_OSPEED2_Pos) |
        (3U << GPIO_OSPEEDR_OSPEED8_Pos);

    GPIOC->AFR[0] =
        (GPIOC->AFR[0] & ~GPIO_AFRL_AFSEL2_Msk) |
        (6U << GPIO_AFRL_AFSEL2_Pos);

    GPIOC->AFR[1] =
        (GPIOC->AFR[1] & ~GPIO_AFRH_AFSEL8_Msk) |
        (6U << GPIO_AFRH_AFSEL8_Pos);

    TIM20->CR1 &= ~TIM_CR1_CEN;

    TIM20->PSC = 33U;
    TIM20->ARR = 100U;
    TIM20->RCR = 0U;

    TIM20->CR1 &= ~(TIM_CR1_CMS_Msk | TIM_CR1_DIR);
    TIM20->CR1 |= TIM_CR1_CMS_0 | TIM_CR1_ARPE;

    TIM20->CCMR1 &= ~(TIM_CCMR1_CC1S_Msk | TIM_CCMR1_OC1M_Msk |
                      TIM_CCMR1_CC2S_Msk | TIM_CCMR1_OC2M_Msk);
    TIM20->CCMR1 |= (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE |
                    (6U << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;

    TIM20->CCMR2 &= ~(TIM_CCMR2_CC3S_Msk | TIM_CCMR2_OC3M_Msk);
    TIM20->CCMR2 |= (6U << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE;

    TIM20->CCR1 = 0U;
    TIM20->CCR2 = 0U;
    TIM20->CCR3 = 0U;
    TIM20->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC1NP |
                     TIM_CCER_CC2P | TIM_CCER_CC2NP |
                     TIM_CCER_CC3P | TIM_CCER_CC3NP);

    TIM20->EGR = TIM_EGR_UG;
    TIM20->SR = 0U;

    TIM20->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E;
    TIM20->BDTR |= TIM_BDTR_MOE;
    TIM20->CR1 |= TIM_CR1_CEN;
}

void TIM1_CH1_SetDutyCycle(unsigned int dutyPercent)
{
    if (dutyPercent > 100U) dutyPercent = 100U;
    TIM1->CCR1 = dutyPercent;
}

void TIM1_CH2_SetDutyCycle(unsigned int dutyPercent)
{
    if (dutyPercent > 100U) dutyPercent = 100U;
    TIM1->CCR2 = dutyPercent;
}

void TIM1_CH3_SetDutyCycle(unsigned int dutyPercent)
{
    if (dutyPercent > 100U) dutyPercent = 100U;
    TIM1->CCR3 = dutyPercent;
}

void TIM8_CH1_SetDutyCycle(unsigned int dutyPercent)
{
    if (dutyPercent > 100U) dutyPercent = 100U;
    TIM8->CCR1 = dutyPercent;
}

void TIM8_CH2_SetDutyCycle(unsigned int dutyPercent)
{
    if (dutyPercent > 100U) dutyPercent = 100U;
    TIM8->CCR2 = dutyPercent;
}

void TIM8_CH3_SetDutyCycle(unsigned int dutyPercent)
{
    if (dutyPercent > 100U) dutyPercent = 100U;
    TIM8->CCR3 = dutyPercent;
}

void TIM20_CH1_SetDutyCycle(unsigned int dutyPercent)
{
    if (dutyPercent > 100U) dutyPercent = 100U;
    TIM20->CCR1 = dutyPercent;
}

void TIM20_CH2_SetDutyCycle(unsigned int dutyPercent)
{
    if (dutyPercent > 100U) dutyPercent = 100U;
    TIM20->CCR2 = dutyPercent;
}

void TIM20_CH3_SetDutyCycle(unsigned int dutyPercent)
{
    if (dutyPercent > 100U) dutyPercent = 100U;
    TIM20->CCR3 = dutyPercent;
}