/* Includes */
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_usart.h>
#include <stm32f4xx_i2c.h>
#include <stm32f4xx_tim.h>
#include <stm32f4xx_adc.h>

/* Private macro */

/* Private function prototypes -----------------------------------------------*/

// zmienne do pwm
void UstawieniePWM(void);
uint16_t CCR1_Val = 333;	//pulse 1
uint16_t CCR2_Val = 249;	//pulse 2
uint16_t PrescalerValue = 0;

/* Private variables */
uint8_t sendFlag=0;	// flaga s³u¿¹ca do sprawdzania czy przycisk nie zosta³ ju¿ wciœniêty

char MESSAGE[] = "dupa";	// widomosc do USART2



/* Private function prototypes */

/* Private functions */
void UstawieniePIN(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIOMode_TypeDef mode);

// dioda PA1 - taka do zabawy/testÃ³w
#define Test1_Pin GPIO_Pin_1
#define Test1_GPIO GPIOA
void ZapalDiode1();
void ZgasDiode1();

// USART 2 - PA1(TX) , PA2(RX)
void UstawienieUSART1(); // PA9(Tx), PA10(Rx)

void SendPacket(uint8_t *data, uint16_t length); // dwie funkcje do wysyslania danch przez USART
void Send_Byte (uint8_t c);



/* Private define */


/**
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/
int main(void)
{

  /* Enable clocks */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);


  /* Initialize LEDs */
  UstawieniePIN(GPIOD,GPIO_Pin_12,GPIO_Mode_OUT); //zielona
  UstawieniePIN(GPIOD,GPIO_Pin_13,GPIO_Mode_OUT); //pomarañczowa
  UstawieniePIN(GPIOD,GPIO_Pin_14,GPIO_Mode_OUT); //czerwona
  UstawieniePIN(GPIOD,GPIO_Pin_15,GPIO_Mode_OUT); //niebieska
  /* Diody testowe */
  UstawieniePIN(GPIOA,GPIO_Pin_1,GPIO_Mode_OUT);
  /* USER Button */
  UstawieniePIN(GPIOA,GPIO_Pin_0,GPIO_Mode_IN);
  GPIO_SetBits(GPIOD, GPIO_Pin_14);

  /* Ustawienie timerów pod PWM , pod prêdkoœc obrotow¹ silnikow*/
  UstawieniePWM();




  /* Transmisja USART2 */
  void UstawienieUSART1();

  char c;

  /* Infinite loop */
  while (1)
  {


	if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0) == 1)
	{
		GPIO_SetBits(GPIOD, GPIO_Pin_12);
		GPIO_ResetBits(GPIOD, GPIO_Pin_14);
		ZapalDiode1();


		if(sendFlag == 0)
		{
			//USART_SendData(USART1, "d");
			//Send_Byte("d");


			sendFlag = 1;// flaga do sprawdzania czy przycisk wcisniety raz  i nie przytrzymywany
		}

	}
	else
	{
			GPIO_ResetBits(GPIOD, GPIO_Pin_12);
			GPIO_SetBits(GPIOD, GPIO_Pin_14);
			ZgasDiode1();

			sendFlag=0; // flaga do sprawdzania czy przycisk wcisniety raz  i nie przytrzymywany

	}
  }
}

/*
 * Ustawiamy podstawowe parametry dla wybranego Pina
 */
__INLINE void UstawieniePIN(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIOMode_TypeDef mode)
{

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin;
	GPIO_InitStructure.GPIO_Mode = mode; //  Moze byc IN, OUT , AF- alternative function, AN - analog in/out
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //Ensure output is push-pull(PP) vs open drain(OD)
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //No internal pullup resistors required
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // sa 3 lvl do wyboru albo mozna po GPIO_Speed_50MHz;
	GPIO_Init(GPIOx, &GPIO_InitStructure);
}
/*
 * Ustawiamy podstawowe parametry dla wybranego Pina
 */
__INLINE void UstawienieUSART1()
{
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

    // GPIO AF config
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);


    // Configure GPIO(UART TX/RX)
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Configure UART peripheral
	USART_StructInit(&USART_InitStructure);
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART1, &USART_InitStructure);

    // Enable USART receive interrupt
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	USART_Cmd(USART1,ENABLE);


}


/*
 * Zapala diode testow¹ 1
 */
void ZapalDiode1()
{
	GPIO_SetBits(Test1_GPIO, Test1_Pin);
}
/*
 * Gasi diode testowa 1
 */
void ZgasDiode1()
{
	GPIO_ResetBits(Test1_GPIO, Test1_Pin);
}
/**
**===========================================================================
**
**  Abstract: Send_Byte USART
**
**===========================================================================
*/
void Send_Byte (uint8_t c)
{
  //  while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
 //   USART_SendData(USART1, c);

    // Wait until transmit data register is empty
   // while (!USART_GetFlagStatus(USART1, USART_FLAG_TXE));
    while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);
    USART_SendData(USART1, c);

}
/**
**===========================================================================
**
**  Abstract: SendPacket USART
**
**===========================================================================
*/
void SendPacket(uint8_t *data, uint16_t length)
{
  uint16_t i;
  i = 0;
  while (i < length)
  {
    Send_Byte(data[i]);
    i++;
  }
}
void USART1_IRQHandler(void)
{
    char c;

    if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
    {
        c = USART_ReceiveData(USART1);

        USART_SendData(USART1, 'c');
    }
}
void UstawieniePWM(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;

  /* TIM3 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  /* GPIOC and GPIOB clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

  /* GPIOC Configuration: TIM3 CH1 (PC6) and TIM3 CH2 (PC7) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* Connect TIM3 pins to AF2 */
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3);


  /* Compute the prescaler value */
  PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 28000000) - 1;

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 665;
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(TIM3, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel2 */
  TIM_OCInitStructure.TIM_Pulse = CCR2_Val;

  TIM_OC2Init(TIM3, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM3, ENABLE);

  /* TIM3 enable counter */
  TIM_Cmd(TIM3, ENABLE);
}

/*
 * Callback used by stm32f4_discovery_audio_codec.c.
 * Refer to stm32f4_discovery_audio_codec.h for more info.
 */
void EVAL_AUDIO_TransferComplete_CallBack(uint32_t pBuffer, uint32_t Size){
  /* TODO, implement your code here */
  return;
}

/*
 * Callback used by stm324xg_eval_audio_codec.c.
 * Refer to stm324xg_eval_audio_codec.h for more info.
 */
uint16_t EVAL_AUDIO_GetSampleCallBack(void){
  /* TODO, implement your code here */
  return -1;
}
