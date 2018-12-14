// 송신부 최종

// 블루투스 송신부
#include <stdio.h>
#include "stm32f4xx.h"
#include "led.h"
#include "llio.h"

#define TIMER_PRESCALER_FREQ        168000     // timer 입력 클력 1MHz
#define TIMER_FREQ                  100         // timer 반복 주기 50ms = 20Hz

//LED 용 GPIO 핀 초기화 함수들 
//LED 용 GPIO 핀, + x축 방향

unsigned char min(int a, int b) { return a < b ? a : b; }
void GPIO_Init_AllatOnce();

void Init_GPIO_gSelect1(){  //E0, GPIO for g-Select 1
    GPIO_InitTypeDef GPIO_InitStructure;                               
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);                   
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;                      
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;                      
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;                   
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                      
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                  
    GPIO_Init(GPIOE, &GPIO_InitStructure);                     
    GPIOE -> BSRRH = GPIO_Pin_0; //initial output signal -> 0, low
}
void Init_GPIO_gSelect2(){  //E1, GPIO for g-Select 2
    GPIO_InitTypeDef GPIO_InitStructure;                               
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);                   
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1;                      
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;                      
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;                   
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                      
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                  
    GPIO_Init(GPIOE, &GPIO_InitStructure);                     
    GPIOE -> BSRRH = GPIO_Pin_1;  //initial output signal -> 0
}
void Init_GPIO_SleepMode(){  //E2, GPIO for Sleep mode
    GPIO_InitTypeDef GPIO_InitStructure;                               
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);                   
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;                      
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;                      
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;                   
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                      
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                  
    GPIO_Init(GPIOE, &GPIO_InitStructure);                     
    GPIOE -> BSRRL = GPIO_Pin_2;      //initial output signal -> 1
}

void Init_TimerForSensing(){  //가속도 센서의 x, y, z 출력값을 얻기위한 타이머 초기화 함수, TIM3
    uint16_t PrescalerValue;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0d;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0f;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    SystemCoreClockUpdate();
    PrescalerValue = (uint16_t) (SystemCoreClock / TIMER_PRESCALER_FREQ) - 1;
    TIM_TimeBaseStructure.TIM_Period = TIMER_PRESCALER_FREQ / 50 - 1;           
    TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
}

void Init_ADC1(){  //가속도 센서의 x -> ADC 1  //PC0
    GPIO_InitTypeDef GPIO_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;
    ADC_CommonInitTypeDef ADC_CommonInitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;     
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStructure);

    /* ADC channel 10 (PC0) configuration */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* ADC channel configuration */
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    /* ADC1 regular channel10 configuration */
    ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_3Cycles);

    /* Enable ADC1 */
    ADC_Cmd(ADC1, ENABLE);
}
void Init_ADC2(){  //가속도 센서의 y -> ADC 2  //PC2
   //ADC2, C 포트 2번핀, 해상도 12bit, 채널 12
   GPIO_InitTypeDef GPIO_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;
    ADC_CommonInitTypeDef ADC_CommonInitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;     
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* ADC channel configuration */
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion = 1;
    ADC_Init(ADC2, &ADC_InitStructure);

    /* ADC1 regular channel10 configuration */
    ADC_RegularChannelConfig(ADC2, ADC_Channel_12, 1, ADC_SampleTime_3Cycles);

    /* Enable ADC1 */
    ADC_Cmd(ADC2, ENABLE);
}

unsigned int adc_x = 0;  //ADC value from ADC1
unsigned int adc_y = 0; //ADC value from ADC2

void Init_TimerSet() { //TIM 2
    uint16_t PrescalerValue;//클럭의 주파수를 설정할 변수
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;//TIMER 셋팅과 관련된 구조체
    NVIC_InitTypeDef NVIC_InitStructure;//NVIC 셋팅과 관련된 구조체
    //IRQ는 Interrupt ReQuest의 약자이다.
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    //TIM3를 InterruptReQuest 채널에 등록시킨다.
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0e;
    //선점형 우선순위 설정
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0f;
    //서브 우선순위를 설정
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    //IRQ 채널을 사용할지를 설정
    NVIC_Init(&NVIC_InitStructure);//TIM3에 대한 NVIC_Init을 실행시킨다.
    /* TIM3 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    //TIM3 clock을 생성한다.
    SystemCoreClockUpdate(); //168MHz --> 168000000
    PrescalerValue = (uint16_t)(SystemCoreClock / TIMER_PRESCALER_FREQ) - 1;
    // prescalerValue라는 값을 통하여 클럭의 주파수를 저장
    /* Time base configuration */ //TIM 구조체 초기화 
    TIM_TimeBaseStructure.TIM_Period = TIMER_PRESCALER_FREQ / TIMER_FREQ - 1;     // 펄스에 따른 인터럽트의 주기를 초기화
    TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
    //클럭의 주파수를 초기화
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    //증가하는 카운터로 설정한다.
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    //타이머의 기본 베이스를 구조체로서 초기화시킨다.
        /* TIM IT enable */
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    // 오버 플로우가 일어날 경우 인터럽트가 발생하게 한다(TIM3_SR 레지스터 0번 비트를 1로 설정시킨다.).
        /* TIM3 enable counter */
    TIM_Cmd(TIM2, ENABLE);// 카운트를 세기 시작한다.
}

void Init_USART(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // Enable peripheral
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    // Configure USART Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0f;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0f;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // GPIO AF(Alternative Function) config
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

    // Configure GPIO(UART TX/RX)
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_10;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Configure UART peripheral
    USART_InitStructure.USART_BaudRate   = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits   = USART_StopBits_1;
    USART_InitStructure.USART_Parity     = USART_Parity_No ;
    USART_InitStructure.USART_HardwareFlowControl
                                         = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode       = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART1, &USART_InitStructure);

    // Enable USART receive interrupt
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    USART_Cmd(USART1, ENABLE);
}

unsigned char c;
int main()
{
    LED_Init();
    //GPIO_Init_AllatOnce(); //LED용 GPIO 초기화 함수들 한번에 진행
    Init_GPIO_gSelect1();
    Init_GPIO_gSelect2();
    Init_GPIO_SleepMode();
    Init_TimerForSensing();
    Init_ADC1();
    Init_ADC2();
    Init_USART();
    Init_TimerSet();
    
    //printf("\r\nADC Test\r\n");


    while(1)
    {
        __WFI();
        //printf("x, y : %d, %d\r\n", adc_x,adc_y);     // ADC 결과 읽어 출력 ? UART ? ComportMaster
        //Control_LEDs(adc_x, adc_y, adc_z);
    }
}

void TIM3_IRQHandler(void) //Interrupt handler for Timer3
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
    {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        
        ADC_SoftwareStartConv(ADC1);  
        ADC_SoftwareStartConv(ADC2);  

        if (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == SET)       // ADC 완료까지 대기
        {
            adc_x = ADC_GetConversionValue(ADC1);
        }
        if (ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC) == SET)       // ADC 완료까지 대기
        {
            adc_y = ADC_GetConversionValue(ADC2);
        }
    
     }
}

void TIM2_IRQHandler(void) //타이머 인터럽트 핸들러
{
    static int i = 0;
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)//인터럽트가 발생할 경우
    {
        //Timx_SR 레지스터 0번 비트가 다시 0으로 세팅되면서 다시 인터럽트를 받을 준비를 한다.
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        
        unsigned char lv = 0, ld = 0, rv = 0, rd = 0;
        
        c = 0;
        if (adc_x >= 3328) ld = 0, rd = 1;
        else ld = 1, rd = 0;
        
        if (adc_x < 2557 || adc_x >= 3842) lv = rv = 3;
        else if (adc_x < 2814 || adc_x >= 3585) lv = rv = 2;
        else if (adc_x < 3071 || adc_x >= 3328) lv = rv = 1;
        else lv = rv = 0;
        
        //if (adc_y >= 2000) c |= 1 << 3;
        if (adc_y < 2557) lv = 0;
        else if (adc_y < 2814) lv = min(lv, 1);
        else if (adc_y < 3071) lv = min(lv, 2);
        else if (adc_y < 3328) lv = min(lv, 3), rv = min(rv, 3);
        else if (adc_y < 3585) rv = min(rv, 2);
        else if (adc_y < 3842) rv = min(rv, 1);
        else rv = 0;
        
        c = ld << 7 | lv << 4 | rd << 3 | rv;
        USART_SendData(USART1, c);
        
        if (i & 1) LED_R_ON();
        else LED_R_OFF();
        
        i ^= 1;
    }
}