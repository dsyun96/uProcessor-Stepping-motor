// ���ź� ����

// ���ź�
#include <stdio.h>
#include "stm32f4xx.h"
#include "led.h"
#include "llio.h"

#define TIMER_PRESCALER_FREQ        168000     // timer �Է� Ŭ�� 1MHz
#define TIMER_FREQ                  20         // timer �ݺ� �ֱ� 50ms = 20Hz

void ChangeTimerFrequency2(int freq); //Ÿ�̸� �ֱ� ���� �Լ� 
void ChangeTimerFrequency3(int freq);

void Init_GPIOforPulseL(){  //E ��Ʈ 0����
   //GPIO �ʱ�ȭ �ڵ�, output ���, nopull, gpio speed : 50MHz
    GPIO_InitTypeDef GPIO_InitStructure; //GPIO���ð� ���õ� ����ü
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; //GPIO Pin 0���� ���
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; //Out��带 ���
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; // PULL��� ������� ����
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //Output Type ����
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // 50MHz ����
    
    GPIO_Init(GPIOE, &GPIO_InitStructure);
}
void Init_GPIOforPulseR(){  //E ��Ʈ 1����
   //GPIO �ʱ�ȭ �ڵ�, output ���, nopull, gpio speed : 50MHz
    GPIO_InitTypeDef GPIO_InitStructure; //GPIO���ð� ���õ� ����ü
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1; //GPIO Pin 1���� ���
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; //Out��带 ���
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; // PULL��� ������� ����
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //Output Type ����
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // 50MHz ����
    
    GPIO_Init(GPIOE, &GPIO_InitStructure);
}
void Init_GPIO_DriverEnable(){  //Motor driver enable or disable // E2
   //GPIO �ʱ�ȭ �ڵ�, output ���, nopull, gpio speed : 50MHz
    GPIO_InitTypeDef GPIO_InitStructure; //GPIO���ð� ���õ� ����ü
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //GPIO Pin 2���� ���
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; //Out��带 ���
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; // PULL��� ������� ����
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //Output Type ����
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // 50MHz ����
    
    GPIO_Init(GPIOE, &GPIO_InitStructure);
    
    GPIOE->BSRRH = GPIO_Pin_2;  //�ʱ� ���� a motor driver enable
}
void Init_GPIO_StepMode0(){  //Step Mode 0 // E3
   //GPIO �ʱ�ȭ �ڵ�, output ���, nopull, gpio speed : 50MHz
    GPIO_InitTypeDef GPIO_InitStructure; //GPIO���ð� ���õ� ����ü
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; //GPIO Pin 3���� ���
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; //Out��带 ���
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; // PULL��� ������� ����
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //Output Type ����
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // 50MHz ����
    
    GPIO_Init(GPIOE, &GPIO_InitStructure);
    
    GPIOE->BSRRH = GPIO_Pin_3;  //�ʱ� ����
}
void Init_GPIO_StepMode1(){  //Step Mode 1 // E4
   //GPIO �ʱ�ȭ �ڵ�, output ���, nopull, gpio speed : 50MHz
    GPIO_InitTypeDef GPIO_InitStructure; //GPIO���ð� ���õ� ����ü
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4; //GPIO Pin 4���� ���
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; //Out��带 ���
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; // PULL��� ������� ����
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //Output Type ����
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // 50MHz ����
    
    GPIO_Init(GPIOE, &GPIO_InitStructure);
    
    GPIOE->BSRRH = GPIO_Pin_4;  //�ʱ⼳��
}
void Init_GPIO_Direction_Left(){ //Direction of left wheel //E5
    //GPIO �ʱ�ȭ �ڵ�, output ���, nopull, gpio speed : 50MHz
    GPIO_InitTypeDef GPIO_InitStructure; //GPIO���ð� ���õ� ����ü
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; //GPIO Pin 5���� ���
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; //Out��带 ���
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; // PULL��� ������� ����
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //Output Type ����
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // 50MHz ����
    
    GPIO_Init(GPIOE, &GPIO_InitStructure);
    
    GPIOE->BSRRH = GPIO_Pin_5;  //�ʱ⼳��
}
void Init_GPIO_Direction_Right(){ //Direction of  right wheel //E6
   //GPIO �ʱ�ȭ �ڵ�, output ���, nopull, gpio speed : 50MHz
    GPIO_InitTypeDef GPIO_InitStructure; //GPIO���ð� ���õ� ����ü
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; //GPIO Pin 6���� ���
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; //Out��带 ���
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; // PULL��� ������� ����
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //Output Type ����
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // 50MHz ����
    
    GPIO_Init(GPIOE, &GPIO_InitStructure);
    
    GPIOE->BSRRH = GPIO_Pin_6; //�ʱ⼳��
}

void Init_TimerForPulseL(){   //���� ���� ������ ���� �޽� ������ Ÿ�̸� Timer2
    uint16_t PrescalerValue;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0f;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0f;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    SystemCoreClockUpdate();
    PrescalerValue = (uint16_t) (SystemCoreClock / TIMER_PRESCALER_FREQ) - 1;
    TIM_TimeBaseStructure.TIM_Period = TIMER_PRESCALER_FREQ / TIMER_FREQ - 1; 
    TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
}
void Init_TimerForPulseR(){   //������ ���� ������ ���� �޽� ������ Ÿ�̸� Timer3
    uint16_t PrescalerValue;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0e;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0f;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    SystemCoreClockUpdate();
    PrescalerValue = (uint16_t) (SystemCoreClock / TIMER_PRESCALER_FREQ) - 1;
    TIM_TimeBaseStructure.TIM_Period = TIMER_PRESCALER_FREQ / TIMER_FREQ - 1;           
    TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
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
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0e;
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

int main()
{
    LED_Init();
    
    //printf("\r\nADC Test\r\n");
    //���ܸ��͸� ����ϴ� GPIO �ʱ�ȭ
    Init_GPIOforPulseL();
    Init_GPIOforPulseR();
    Init_GPIO_DriverEnable();
    Init_GPIO_StepMode0();
    Init_GPIO_StepMode1();
    Init_GPIO_Direction_Left();
    Init_GPIO_Direction_Right();
    Init_USART();
    Init_TimerForPulseL();
    Init_TimerForPulseR();

    while(1)
    {
        __WFI();
// ADC ��� �о� ��� a UART a ComportMaster
    }
}

void USART1_IRQHandler(void)
{
    unsigned char c;
    static int i = 0;
    
    if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
    {
        // �����͸� �޾Ҵ���
        c = USART_ReceiveData(USART1);
        if (i & 1) LED_R_ON();
        else LED_R_OFF();
        
        i ^= 1;
        
        // (����1)(�ӵ�1) (����2)(�ӵ�2)
        // ������ 1�� �ð�, 0�� �ݽð�
        if ((c >> 4 & 3) == 0 && (c & 3) == 0) GPIOE->BSRRH = GPIO_Pin_2;
        else GPIOE->BSRRL = GPIO_Pin_2;
        
        if (c & 1 << 7) GPIOE->BSRRH = GPIO_Pin_5;
        else GPIOE->BSRRL = GPIO_Pin_5;
        
        if (c >> 4 & 3) ChangeTimerFrequency2(3000 * (1 << (c >> 4 & 3) - 1));
        
        if (c & 1 << 3) GPIOE->BSRRH = GPIO_Pin_6;
        else GPIOE->BSRRL = GPIO_Pin_6;
        
        if (c & 3) ChangeTimerFrequency3(3000 * (1 << (c & 3) - 1));
    }
}

void TIM2_IRQHandler(void) //Interrupt handler for Timer2
{
    static int j = 0;
    
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        
        //�޽� ����
        if (j & 1) GPIOE->BSRRH = GPIO_Pin_0;
        else GPIOE->BSRRL = GPIO_Pin_0;
        
        j ^= 1;
    }
}

void TIM3_IRQHandler(void) //Interrupt handler for Timer3
{
    static int k = 0;
    
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
    {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        
        //�޽� ����
        if (k & 1) GPIOE->BSRRH = GPIO_Pin_1;
        else GPIOE->BSRRL = GPIO_Pin_1;
        
        k ^= 1;
    }
}

void ChangeTimerFrequency2(int freq){ //Ÿ�̸� �ֱ� �ǽð� ��ȭ
    uint16_t PrescalerValue;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    SystemCoreClockUpdate();
    PrescalerValue = (uint16_t) (SystemCoreClock / TIMER_PRESCALER_FREQ) - 1;
    TIM_TimeBaseStructure.TIM_Period = TIMER_PRESCALER_FREQ / freq - 1;           // 20Hz timer
    TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
}

void ChangeTimerFrequency3(int freq){ //Ÿ�̸� �ֱ� �ǽð� ��ȭ
    uint16_t PrescalerValue;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    SystemCoreClockUpdate();
    PrescalerValue = (uint16_t) (SystemCoreClock / TIMER_PRESCALER_FREQ) - 1;
    TIM_TimeBaseStructure.TIM_Period = TIMER_PRESCALER_FREQ / freq - 1;           // 20Hz timer
    TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
}
