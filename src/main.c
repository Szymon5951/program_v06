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
void UstawienieUSART2(); // PA9(Tx), PA10(Rx)

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


  /**
  *  IMPORTANT NOTE!
  *  The symbol VECT_TAB_SRAM needs to be defined when building the project
  *  if code has been located to RAM and interrupts are used. 
  *  Otherwise the interrupt table located in flash will be used.
  *  See also the <system_*.c> file and how the SystemInit() function updates 
  *  SCB->VTOR register.  
  *  E.g.  SCB->VTOR = 0x20000000;  
  */

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

  /* Transmisja USART2 */
  void UstawienieUSART2();


  /* Infinite loop */
  while (1)
  {
	//i++;
	if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0) == 1)
	{
		GPIO_SetBits(GPIOD, GPIO_Pin_12);
		GPIO_ResetBits(GPIOD, GPIO_Pin_14);
		ZapalDiode1();


		if(sendFlag == 0)
		{
			//USART_SendData(USART2, 'd');
			Send_Byte('d');
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
__INLINE void UstawienieUSART2()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

    /* Init clock for GPIOA, with USART connected */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    /* Init clock for DMA 1, with use RX transfer data */
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    /* Init clock for USART2 */
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

	 /* (1) Select AF mode (10) on PA9 and PA10 */
	 GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_StructInit(&USART_InitStructure);
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(USART2, &USART_InitStructure);
	USART_Cmd(USART2,ENABLE);


    /* Configure IT */
    /* (3) Set priority for USART1_IRQn */
    /* (4) Enable USART1_IRQn */
    NVIC_SetPriority(USART2_IRQn, 0); /* (3) */
    NVIC_EnableIRQ(USART2_IRQn); /* (4) */

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
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    USART_SendData(USART2, c);
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
void USART2_IRQHandler(void)
{

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
