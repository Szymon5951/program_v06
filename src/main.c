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
/* Private variables */
/* Private function prototypes */
/* Private functions */
/* Private define */


//Itemki podstawowe
uint8_t sendFlag=0;	// flaga s³u¿¹ca do sprawdzania czy przycisk nie zosta³ ju¿ wciœniêty
void UstawieniePIN(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIOMode_TypeDef mode);


// itemki do pwm
void UstawieniePWM(void);
uint16_t CCR1_Val = 333;	//pulse 1
uint16_t CCR2_Val = 249;	//pulse 2
uint16_t PrescalerValue = 0;


// itemki pod uart
char MESSAGE[] = "dupa";	// widomosc do USART1
void UstawienieUSART1(); // PB6(Tx), PB7(Rx)
void Wyslij_zdanie(uint8_t *data, uint16_t length); // dwie funkcje do wysyslania danch przez USART
void Wyslij_znak(uint8_t c);
void OdpowiedzUART();


//itemki pod adc
void UstawienieADC3();
#define ADC_CDR_ADDRESS    ((uint32_t)0x40012308)
#define ADC3_DR_ADDRESS     ((uint32_t)0x4001224C)
__IO uint16_t ADC3ConvertedValue = 0;
__IO uint32_t ADC3ConvertedVoltage = 0;
void ObslugaCzujkaOdleglosci();


// itemki testowe
#define Test1_Pin GPIO_Pin_1
#define Test1_GPIO GPIOA
void ZapalDiode1();
void ZgasDiode1();



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
  //UstawieniePWM();

  /* Transmisja USART2 */
  UstawienieUSART1();

  //ustawienie adc
  UstawienieADC3();

  /* Infinite loop */
  while (1)
  {

	OdpowiedzUART();

	if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0) == 1)
	{
		GPIO_SetBits(GPIOD, GPIO_Pin_12);
		GPIO_ResetBits(GPIOD, GPIO_Pin_14);
		//ZapalDiode1();



		if(sendFlag == 0)
		{
			//USART_SendData(USART1, "d");
			//Send_Byte("d");
			//Wyslij_zdanie(MESSAGE,4);

			ObslugaCzujkaOdleglosci(); // wysyla jaka odleglosc  - w przyszlosci sprawdzanie odleglosci i zatrzymanie robota


			sendFlag = 1;// flaga do sprawdzania czy przycisk wcisniety raz  i nie przytrzymywany
		}

	}
	else
	{
			GPIO_ResetBits(GPIOD, GPIO_Pin_12);
			GPIO_SetBits(GPIOD, GPIO_Pin_14);
			//ZgasDiode1();

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
 * U
 */
__INLINE void UstawienieUSART1()
{
	/* --------------------------- System Clocks Configuration -----------------*/
	  /* USART1 clock enable */
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	  /* GPIOB clock enable */
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	  /**************************************************************************************/

	  GPIO_InitTypeDef GPIO_InitStructure;
	  /*-------------------------- GPIO Configuration ----------------------------*/
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOB, &GPIO_InitStructure);
	  /* Connect USART pins to AF */
	  GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1); // USART1_TX
	  GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1); // USART1_RX

	  USART_InitTypeDef USART_InitStructure;
	  /* USARTx configuration ------------------------------------------------------*/
	  /* USARTx configured as follow:
	  - BaudRate = 9600 baud
	  - Word Length = 8 Bits
	  - One Stop Bit
	  - No parity
	  - Hardware flow control disabled (RTS and CTS signals)
	  - Receive and transmit enabled
	  */
	  USART_InitStructure.USART_BaudRate = 9600;
	  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	  USART_InitStructure.USART_StopBits = USART_StopBits_1;
	  USART_InitStructure.USART_Parity = USART_Parity_No;
	  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	  USART_Init(USART1, &USART_InitStructure);
	  USART_Cmd(USART1, ENABLE);

}
void OdpowiedzUART()
{
	if(USART_GetFlagStatus(USART1,USART_FLAG_RXNE))
	{
		char c = USART_ReceiveData(USART1);
	    USART_SendData(USART1,c );
	    GPIO_SetBits(GPIOB, GPIO_Pin_10);
	    if(c == 'g')
	    {
	    	GPIO_SetBits(GPIOB, GPIO_Pin_11);
	    }


    }
}
void UstawienieADC3()
{
	ADC_InitTypeDef       ADC_InitStructure;
	  ADC_CommonInitTypeDef ADC_CommonInitStructure;
	  DMA_InitTypeDef       DMA_InitStructure;
	  GPIO_InitTypeDef      GPIO_InitStructure;

	  /* Enable ADC3, DMA2 and GPIO clocks ****************************************/
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOC, ENABLE);
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);

	  /* DMA2 Stream0 channel0 configuration **************************************/
	  DMA_InitStructure.DMA_Channel = DMA_Channel_2;
	  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC3_DR_ADDRESS;
	  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC3ConvertedValue;
	  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	  DMA_InitStructure.DMA_BufferSize = 1;
	  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
	  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	  DMA_Init(DMA2_Stream0, &DMA_InitStructure);
	  DMA_Cmd(DMA2_Stream0, ENABLE);

	  /* Configure ADC3 Channel12 pin as analog input ******************************/
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	  GPIO_Init(GPIOC, &GPIO_InitStructure);

	  /* ADC Common Init **********************************************************/
	  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	  ADC_CommonInit(&ADC_CommonInitStructure);

	  /* ADC3 Init ****************************************************************/
	  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	  ADC_InitStructure.ADC_NbrOfConversion = 1;
	  ADC_Init(ADC3, &ADC_InitStructure);

	  /* ADC3 regular channel12 configuration *************************************/
	  ADC_RegularChannelConfig(ADC3, ADC_Channel_12, 1, ADC_SampleTime_3Cycles);

	 /* Enable DMA request after last transfer (Single-ADC mode) */
	  ADC_DMARequestAfterLastTransferCmd(ADC3, ENABLE);

	  /* Enable ADC3 DMA */
	  ADC_DMACmd(ADC3, ENABLE);

	  /* Enable ADC3 */
	  ADC_Cmd(ADC3, ENABLE);

	  /* Start ADC3 Software Conversion */
	  ADC_SoftwareStartConv(ADC3);
}

void ObslugaCzujkaOdleglosci()
{
	ADC3ConvertedVoltage = ADC3ConvertedValue *3300/0xFFF;
	uint32_t v=0,mv=0;
	uint8_t text[50];

	// HEH przeliczanie tego gÃ³wna
	v=(ADC3ConvertedVoltage)/1000;
	mv = (ADC3ConvertedVoltage%1000)/100;
	sprintf((char*)text,"   ADC = %d,%d V   ",v,mv);	// podobno uzywanie sprintf do wysylki przez uart mega obciaza procka
	Wyslij_zdanie(text,20);
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
void Wyslij_znak (uint8_t c)
{
	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); // Wait for Empty
	USART_SendData(USART1, c);
}
/**
**===========================================================================
**
**  Abstract: SendPacket USART
**
**===========================================================================
*/
void Wyslij_zdanie(uint8_t *data, uint16_t length)
{
  uint16_t i;
  i = 0;
  while (i < length)
  {
    Wyslij_znak(data[i]);
    i++;
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
