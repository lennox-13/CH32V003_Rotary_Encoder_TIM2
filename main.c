#define TIM2_Polling
//#define TIM2_EXTI_Interrupt

#ifdef TIM2_Polling

#include "ch32v00x.h"

u16 ENC_Count = 0;

//*************************************************************************************************
//*    Port Init                                                                                  *
//*************************************************************************************************
void GPIO_INIT(void){
    GPIO_InitTypeDef  GPIO_InitStructure = {0};
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3 | GPIO_Pin_4;  // port D, enkoder A,B.
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;            // Input IPU
    GPIO_Init(GPIOD, &GPIO_InitStructure);
}

//*************************************************************************************************
//*    Init Enkoder Port D, TIMER 2                                                               *
//*    Pin PD3 encA, PD4 encB                                                                     *
//*************************************************************************************************
void Encoder_Init(void) {
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  TIM_TimeBaseInitTypeDef tim2;
  tim2.TIM_Period = 0xffff;
  tim2.TIM_Prescaler = 0;
  tim2.TIM_ClockDivision = 0;
  tim2.TIM_CounterMode = TIM_CounterMode_Up;
  tim2.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM2, &tim2);
  TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI1, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
  TIM_SetCounter(TIM2,255);//initial=255
  TIM_Cmd(TIM2, ENABLE);
}

//*************************************************************************************************
//*    Read Enkoder, TIMER2   (0 - 255)                                                           *
//*    Inputs port D, enkoder A PD3, enkoder B PD4                                                *
//*************************************************************************************************
u16 GetEncoder() {
    int16_t count = TIM_GetCounter(TIM2);

    if (count > 255) {      // direction CW
        TIM_SetCounter(TIM2, 255);
        if (ENC_Count < 255) ENC_Count++;
    }
    else if (count < 255) { // direction CCW
        TIM_SetCounter(TIM2, 255);
        if (ENC_Count > 0) ENC_Count--;
    }
    return ENC_Count;
}

int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Delay_Init();
    GPIO_INIT();
    Encoder_Init();

    USART_Printf_Init(115200);

    while(1){

    printf("CNT: %ld \n\r", GetEncoder());
    Delay_Ms(1);
    }
}

#endif


#ifdef TIM2_EXTI_Interrupt

#include "ch32v00x.h"

u16 ENC_Count = 0;

//*************************************************************************************************
//*    Port Init                                                                                  *
//*************************************************************************************************
void GPIO_INIT(void) {
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

    // Konfig PD3 and PD4 Input pull-up
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
}

//*******************************************************************************
// Konfig (EXTI)                                                                *
//*******************************************************************************
void ENC_GPIO_exti(void) {
    EXTI_InitTypeDef EXTI_InitStructure = {0};

    // Clock enable AFIO
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    // Map PD3 to EXTI3 a PD4 to EXTI4
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource3);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource4);

    // Konfig EXTI3 (PD3)
    EXTI_InitStructure.EXTI_Line = EXTI_Line3 | EXTI_Line4;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    // Konfig NVIC
    NVIC_InitTypeDef NVIC_InitStructure = {0};
    NVIC_InitStructure.NVIC_IRQChannel = EXTI7_0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

//*************************************************************************************************
//*    Init Enkoder Port D, TIMER 2                                                               *
//*    Pin PD3 encA, PD4 encB                                                                     *
//*************************************************************************************************
void Encoder_Init(void) {
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  TIM_TimeBaseInitTypeDef tim2;
  tim2.TIM_Period = 0xffff;
  tim2.TIM_Prescaler = 0;
  tim2.TIM_ClockDivision = 0;
  tim2.TIM_CounterMode = TIM_CounterMode_Up;
  tim2.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM2, &tim2);
  TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI1, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
  TIM_SetCounter(TIM2,255); //initial=1000
  TIM_Cmd(TIM2, ENABLE);
}

//*************************************************************************************************
//*    Read Enkoder, TIMER2   (0 - 255)                                                                    *
//*    Inputs port D, enkoder A PD3, enkoder B PD4                                                *
//*************************************************************************************************
void GetEncoder() {
    int16_t count = TIM_GetCounter(TIM2);

    if (count > 255) {       // direction CW
        TIM_SetCounter(TIM2, 255);
        if (ENC_Count < 255) ENC_Count++;
    }
    else if (count < 255) { // direction CCW
        TIM_SetCounter(TIM2, 255);
        if (ENC_Count > 0) ENC_Count--;
    }
}

int main(void) {
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Delay_Init();
    GPIO_INIT();
    ENC_GPIO_exti();
    Encoder_Init();

    USART_Printf_Init(115200);

    while(1) {

        printf("CNT: %ld \n\r", ENC_Count);
        Delay_Ms(1);
    }
}


//*******************************************************************************
// EXTI7_0_IRQHandler                                                           *
//*******************************************************************************
void EXTI7_0_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI7_0_IRQHandler(void) {
    if (EXTI_GetITStatus(EXTI_Line3) != RESET || EXTI_GetITStatus(EXTI_Line4) != RESET) {

    if (EXTI_GetITStatus(EXTI_Line3) != RESET) EXTI_ClearITPendingBit(EXTI_Line3);
    if (EXTI_GetITStatus(EXTI_Line4) != RESET) EXTI_ClearITPendingBit(EXTI_Line4);

        GetEncoder();
    }
}

#endif

