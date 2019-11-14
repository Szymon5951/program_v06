/* Includes */
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_usart.h>
#include <stm32f4xx_i2c.h>
#include <stm32f4xx_tim.h>
#include <stm32f4xx_adc.h>

#include "tm_stm32f4_hmc5883l.h"


/* Private macro */

#define HMC5883L_Address 0x1E	// adress magnetometru

/* Private function prototypes -----------------------------------------------*/
/* Private variables */
/* Private function prototypes */
/* Private functions */
/* Private define */


//Itemki podstawowe
uint8_t sendFlag=0;	// flaga s³u¿¹ca do sprawdzania czy przycisk nie zosta³ ju¿ wciœniêty
void UstawieniePIN(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIOMode_TypeDef mode);
__IO uint32_t TimmingDelay;

// funkja zawieraj¹ca wszystkie ustawiania
void Ustawianie();

// itemki do pwm i enkodery
void UstawieniePWM(void);
int Pulse_lewy = 100;	//pulse 1	LEWY
int Pulse_prawy = 105;	//pulse 2	PRAWY
char Pulse_lewy_char[] = " ";	//pulse 1	LEWY
char Pulse_prawy_char[] = " ";	//pulse 2	PRAWY

uint16_t PeroidValue = 255;
uint16_t PrescalerValue = 392;

// kurde wrzystko to jak narazie pod regulacje predkosci silników
int licznik_lewy = 0;
char licznik_lewy_char[] = "0";
int licznik_prawy = 0;
char licznik_prawy_char[] = "0";
void UstawieniePrzerwanPE0();
void UstawieniePrzerwanPE1();

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
int silniki_testy = 0;




// itemki pod sterownik silnika
void UstawienieSilnikow();

void Silniki_Przod();
void Silniki_Stop();
void Silniki_Tyl();
void Silniki_Lewo();
void Silniki_Prawo();

// Funkcje i zmienne pod algorytm
void Sprawdz_odleglosc()	;		// zwraca int zaleznie od strefy w której widzi przeszkode
void Jedz_impuls(int impuls); 			// jedzie o do przodu o impuls enkodera

int Strefa_odleglosc = 0;   // zmienna zapisujaca aktualna strefe przed robotem
int CzyMogeJechac();	// sprawdza jaka strefa przed nami

int jazda =0;

void Algorytm_jazdy(); 			// jedziemy kilka razy i omijamy przeszkody :D






TM_HMC5883L_t HMC5883L;

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
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);


  /* Ustawienie ledów, przycisków itp*/
  Ustawianie();

  /* Ustawienie pinow pod silniki*/
  UstawienieSilnikow();

  /* Ustawienie timerów pod PWM , pod prêdkoœc obrotow¹ silnikow*/
  UstawieniePWM();

  /* Ustawienie enkoderów*/
  UstawieniePIN(GPIOE, GPIO_Pin_7, GPIO_Mode_IN);	//LEWY B
  UstawieniePIN(GPIOE, GPIO_Pin_9, GPIO_Mode_IN);	//PRAWY B

  //UstawieniePrzerwanEnkoderow();
  UstawieniePrzerwanPE0();
  UstawieniePrzerwanPE1();

  /* Transmisja USART1 */
  UstawienieUSART1();

  //ustawienie adc
  UstawienieADC3();

/*
	// Init HMC5883L sensor /
	if (TM_HMC5883L_Init(&HMC5883L, TM_HMC5883L_Gain_1_3, TM_HMC5883L_OutputRate_15Hz) == TM_HMC5883L_Result_Ok) {
		//* Device OK /
		//printf("Device Initialized\n");
		Wyslij_zdanie("dziala ",6);
	} else {
		//printf("Device Error\n");
		Wyslij_zdanie("hujaaa",6);

		// Infinite loop /
		while (1);
	}
*/

	//STM_EVAL_LEDOn(LED4);
	//GPIO_SetBits(GPIOD, GPIO_Pin_13);


	SysTick_Config(SystemCoreClock/1000);
	//SystemCoreClockUpdate();
	//ii = SystemCoreClock;
	//ii=0;


  /* Infinite loop */
  while (1)
  {

	OdpowiedzUART(); // funkcja z automatu odsyla wiadomosci odebrane :P

	if(jazda == 1)
	{
	Algorytm_jazdy();
	}



				// DO USTAWIANIA PWM NA OBU KOLACH
/*
	if((licznik_lewy%2000) == 5)
		{

			// pulsy na pwm
			sprintf(Pulse_prawy_char,"%d",Pulse_prawy);
			sprintf(Pulse_lewy_char,"%d",Pulse_lewy);

			Wyslij_zdanie("Lewy: ",6);
			Wyslij_zdanie(Pulse_lewy_char,3);
			Wyslij_zdanie("  ",2);
			Wyslij_zdanie("Prawy: ",7);
			Wyslij_zdanie(Pulse_prawy_char,3);
			Wyslij_zdanie("\r\n",4);


			//obroty na kolach
			sprintf(licznik_lewy_char,"%d",licznik_lewy);
			Wyslij_zdanie("Lewy: ",6);
			Wyslij_zdanie(licznik_lewy_char,10);
			Wyslij_zdanie("  ",2);

			sprintf(licznik_prawy_char,"%d",licznik_prawy);
			Wyslij_zdanie("Prawy: ",7);
			Wyslij_zdanie(licznik_prawy_char,10);
			Wyslij_zdanie("\r\n",4);
			Wyslij_zdanie("\r\n",4);


		}
		*/

/*
	if(licznik_lewy == 3000)
	{
		Silniki_Stop();
		licznik_lewy =0;
		licznik_prawy = 0;
	}

*/

	if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0) == 1) // przycisk
	{
		GPIO_SetBits(GPIOD, GPIO_Pin_12);
		GPIO_ResetBits(GPIOD, GPIO_Pin_14);
		//ZapalDiode1();





		if(sendFlag == 0)
		{
			//USART_SendData(USART1, "d");
			//Send_Byte("d");
			//Wyslij_zdanie(MESSAGE,4);

			//ObslugaCzujkaOdleglosci(); // wysyla jaka odleglosc  - w przyszlosci sprawdzanie odleglosci i zatrzymanie robota
			//Sprawdz_odleglosc();


			//Jedz_impuls(300);

			jazda = 1;

			sendFlag = 1;// flaga do sprawdzania czy przycisk wcisniety raz  i nie przytrzymywany


			/*
					if(silniki_testy == 0)
					{
						Silniki_Przod();
						silniki_testy = 1;
					}
					else if(silniki_testy == 1 )
					{
						Silniki_Stop();
						silniki_testy = 0;
					}

*/


/*
					Pulse_lewy = Pulse_lewy + 50;	//pulse 1	LEWY
					Pulse_prawy = Pulse_prawy + 50;	//pulse 2	PRAWY
					if(Pulse_lewy == 250)
					{
						Pulse_lewy = 50;
						Pulse_prawy = 50;
					}


					  UstawieniePWM();

*/


			/*

					if(silniki_testy == 0)
					{
						Silniki_Przod();
						silniki_testy = 1;
					}
					else if(silniki_testy == 1)
					{
						Silniki_Tyl();
						silniki_testy = 2;
					}
					else if(silniki_testy == 2)
					{
						Silniki_Stop();
						silniki_testy = 3;
					}
					else if(silniki_testy == 3)
					{
						Silniki_Lewo();
						silniki_testy = 4;
					}
					else if(silniki_testy == 4)
					{
						Silniki_Prawo();
						silniki_testy = 0;
					}
					*/


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
	uint8_t text2[50];

	// HEH przeliczanie tego gÃ³wna
	v=(ADC3ConvertedVoltage)/1000;
	mv = (ADC3ConvertedVoltage%1000)/100;
	sprintf((char*)text,"   ADC = %d,%d V   ",v,mv);	// podobno uzywanie sprintf do wysylki przez uart mega obciaza procka
	sprintf((char*)text2," zobaczmy : %d  ",ADC3ConvertedVoltage);

//	Wyslij_zdanie(text,20);
//	Wyslij_zdanie("\r\n",4);

	Wyslij_zdanie(text2,20);
	Wyslij_zdanie("\r\n",4);



}
void Sprawdz_odleglosc()
{
	ADC3ConvertedVoltage = ADC3ConvertedValue *3300/0xFFF;


	if(ADC3ConvertedVoltage > 1250)
	{
		Wyslij_zdanie("Widze cos do 20cm.",20);
		Wyslij_zdanie("\r\n",4);
		Strefa_odleglosc = 1;
	}
	else if(ADC3ConvertedVoltage < 600)
	{
		Wyslij_zdanie("Widze cos za  40cm.",20);
		Wyslij_zdanie("\r\n",4);
		Strefa_odleglosc = 3;
	}
	else
	{
		Wyslij_zdanie("Widze cos miedzy 20 a 40cm.",20);
		Wyslij_zdanie("\r\n",4);
		Strefa_odleglosc = 2;

	}


}
void Jedz_impuls(int impuls)
{
	int zadane =  licznik_lewy + impuls;
	Silniki_Przod();
	while(zadane > licznik_lewy)
	{}

	Silniki_Stop();
}
int CzyMogeJechac()
{
	Sprawdz_odleglosc();
	if(Strefa_odleglosc == 3 || Strefa_odleglosc == 2)
	{
		Wyslij_zdanie("Jedziemy",20);
		Wyslij_zdanie("\r\n",4);
		return 1;

	}
	else
	{
		Wyslij_zdanie("Przeszkoda - nie mozna jechac.",20);
		Wyslij_zdanie("\r\n",4);
		return 0;
	}

}

void Algorytm_jazdy()
{
	GPIO_ToggleBits(GPIOD,GPIO_Pin_13);
	Delay(5000);		// 5 sekund
	Sprawdz_odleglosc()	;		// zwraca int zaleznie od strefy w której widzi przeszkode
	ObslugaCzujkaOdleglosci();
	GPIO_ToggleBits(GPIOD,GPIO_Pin_13);
	if(CzyMogeJechac() == 0)
	{
		// Infinite loop /
		while (1);
	}

	Jedz_impuls(300); 			// jedzie o do przodu o impuls enkodera


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
	/*
  uint16_t i;
  i = 0;
  while (i < length)
  {
    Wyslij_znak(data[i]);
    i++;
  }
  */
  while(*data)
	  Wyslij_znak(*data++);
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




  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = PeroidValue;//19999;//665;
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;//4799;//PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = Pulse_lewy;//9999;//CCR1_Val;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(TIM3, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel2 */
  TIM_OCInitStructure.TIM_Pulse = Pulse_prawy;//CCR2_Val;

  TIM_OC2Init(TIM3, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM3, ENABLE);

  /* TIM3 enable counter */
  TIM_Cmd(TIM3, ENABLE);
}

void UstawieniePrzerwanPE0()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable clocks */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	/* Set pin as input */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

	/* Connect EXTI Line0 to PA0 pin */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource0);

	/* Configure EXTI Line0 */
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);


	/* Enable and set EXTI Line0 Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);





}

void UstawieniePrzerwanPE1()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable clocks */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	/* Set pin as input */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

	/* Connect EXTI Line0 to PA0 pin */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource1);

	/* Configure EXTI Line1 */
	EXTI_InitStructure.EXTI_Line = EXTI_Line1;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Enable and set EXTI Line1 Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void Ustawianie()
{
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
}
void UstawienieSilnikow()
{
	 UstawieniePIN(GPIOE, GPIO_Pin_8, GPIO_Mode_OUT);	//IN1
	 UstawieniePIN(GPIOE, GPIO_Pin_10, GPIO_Mode_OUT);	//IN2
	 UstawieniePIN(GPIOE, GPIO_Pin_12, GPIO_Mode_OUT);	//IN3
	 UstawieniePIN(GPIOE, GPIO_Pin_14, GPIO_Mode_OUT);	//IN3

}
void Silniki_Przod()
{
	//Silnik1
	GPIO_SetBits(GPIOE, GPIO_Pin_8);	//LEWY
	GPIO_ResetBits(GPIOE, GPIO_Pin_10);

	//Silnik 2
	GPIO_SetBits(GPIOE, GPIO_Pin_12);	//PRAWY
    GPIO_ResetBits(GPIOE, GPIO_Pin_14);
}
void Silniki_Stop()
{
	//Silnik1
	GPIO_ResetBits(GPIOE, GPIO_Pin_8);	//LEWY
	GPIO_ResetBits(GPIOE, GPIO_Pin_10);

	//Silnik 2
	GPIO_ResetBits(GPIOE, GPIO_Pin_12);	//PRAWY
    GPIO_ResetBits(GPIOE, GPIO_Pin_14);
}
void Silniki_Tyl()
{
	//Silnik1
	GPIO_ResetBits(GPIOE, GPIO_Pin_8);	//LEWY
	GPIO_SetBits(GPIOE, GPIO_Pin_10);

	//Silnik 2
	GPIO_ResetBits(GPIOE, GPIO_Pin_12);	//PRAWY
    GPIO_SetBits(GPIOE, GPIO_Pin_14);

}
void Silniki_Lewo()
{
	//Silnik1
	GPIO_ResetBits(GPIOE, GPIO_Pin_8);	//LEWY
	GPIO_SetBits(GPIOE, GPIO_Pin_10);

	//Silnik 2
	GPIO_SetBits(GPIOE, GPIO_Pin_12);	//PRAWY
    GPIO_ResetBits(GPIOE, GPIO_Pin_14);


}
void Silniki_Prawo()
{
	//Silnik1
	GPIO_SetBits(GPIOE, GPIO_Pin_8);	//LEWY
	GPIO_ResetBits(GPIOE, GPIO_Pin_10);

	//Silnik 2
	GPIO_ResetBits(GPIOE, GPIO_Pin_12);	//PRAWY
    GPIO_SetBits(GPIOE, GPIO_Pin_14);

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
/* Set interrupt handlers */
/* Handle PD0 interrupt */
void EXTI0_IRQHandler(void) {
    /* Make sure that interrupt flag is set */
    if (EXTI_GetITStatus(EXTI_Line0) != RESET) {
        /* Do your stuff when PD0 is changed */
    	//GPIO_ToggleBits(GPIOD,GPIO_Pin_15);


    			if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_7) == 1)
    			{
    				licznik_lewy++;

    			}
    			else if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_7) == 0)
    			{
    				licznik_lewy--;
    			}

    		if(licznik_lewy > 30000 || licznik_lewy <-30000)
    		{
    			licznik_lewy = 0;
    		}


        /* Clear interrupt flag */
        EXTI_ClearITPendingBit(EXTI_Line0);
    }
}
void EXTI1_IRQHandler(void) {
    /* Make sure that interrupt flag is set */
    if (EXTI_GetITStatus(EXTI_Line1) != RESET) {
        /* Do your stuff when PE1 is changed */
    	//GPIO_ToggleBits(GPIOD,GPIO_Pin_15);


    			if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_9) == 1)
    			{
    				licznik_prawy--;

    			}
    			else if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_9) == 0)
    			{
    				licznik_prawy++;
    			}

    		if(licznik_prawy > 30000 || licznik_prawy <-30000)
    		{
    			licznik_prawy = 0;
    		}


        /* Clear interrupt flag */
        EXTI_ClearITPendingBit(EXTI_Line1);
    }
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */



void SysTick_Handler(void)
{
  if(TimmingDelay !=0)
  {
    TimmingDelay --;
   }
}
void Delay(__IO uint32_t time)
{
  TimmingDelay = time;
  while(TimmingDelay !=0);
}
