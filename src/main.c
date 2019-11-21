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

// i2c DO WYWALENIA
#define I2Cx                      I2C1//I2C3  //Selected I2C peripheral
#define RCC_APB1Periph_I2Cx       RCC_APB1Periph_I2C1//RCC_APB1Periph_I2C3 //Bus where the peripheral is connected
#define RCC_AHB1Periph_GPIO_SCL   RCC_AHB1Periph_GPIOB//RCC_AHB1Periph_GPIOA  //Bus for GPIO Port of SCL
#define RCC_AHB1Periph_GPIO_SDA   RCC_AHB1Periph_GPIOB//RCC_AHB1Periph_GPIOC  //Bus for GPIO Port of SDA
#define GPIO_AF_I2Cx              GPIO_AF_I2C1//GPIO_AF_I2C3    //Alternate function for GPIO pins
#define GPIO_SCL                  GPIOB//GPIOA
#define GPIO_SDA                  GPIOB//GPIOC
#define GPIO_Pin_SCL              GPIO_Pin_6//GPIO_Pin_8
#define GPIO_Pin_SDA              GPIO_Pin_9
#define GPIO_PinSource_SCL        GPIO_PinSource6//GPIO_PinSource8
#define GPIO_PinSource_SDA        GPIO_PinSource9

#define HMC5883L_Address 0x1E//0x3C/ adress magnetometru

/* Private macro */
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

uint16_t PeroidValue = 255;		// to bylo liczone
uint16_t PrescalerValue = 392;

// itemki pod regulacje liczników
int licznik_lewy = 0;	// trzeba by go resetowac czesciej niz 30000	LICZNIK ENKODERA
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
void Wyslij_int(char* wiadomosc,int data);
void OdpowiedzUART();	// odpowiada na porcie szeregowym tym co pojdzie na rx

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
int Sprawdz_odleglosc();		// zwraca int zaleznie od strefy w której widzi przeszkode
int Sprawdz_lewy();
int Sprawdz_prawy();
int Sprawdz_przod();	// sprawdza jaka strefa przed nami

void Skret_Lewo();
void Skret_Prawo();

void Wykonaj_ruch();

int kierunek = 1;	//1 - Prosto, 2- Prawo, 3 - TYL, 4 - LEWO
int ruch_pion = 0;
int ruch_poziom = 0;



void Jedz_impuls(int impuls); 			// jedzie o do przodu o impuls enkodera

int Strefa_odleglosc = 0;   // zmienna zapisujaca aktualna strefe przed robotem


int jazda =0;





int Omijanie_Przeszkody();

void Algorytm_jazdy(int powtorzenia); 			// jedziemy kilka razy i omijamy przeszkody :D






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
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
  SysTick_Config(SystemCoreClock/1000);


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






  /* Infinite loop */
  while (1)
  {

	OdpowiedzUART(); // funkcja z automatu odsyla wiadomosci odebrane :P




	// TU MUSIMY ZAPODAC ZADANIE : PRZEJEDZ ODLEGLOSC.

	if(jazda == 1)
	{
		Wykonaj_ruch();
	}



	if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0) == 1) // przycisk
	{
		if(sendFlag == 0)	// TU SIE DZIEJE RAZ PO WCISNIECIU PRZYCISKU
		{
			GPIO_SetBits(GPIOD, GPIO_Pin_12);
			GPIO_ResetBits(GPIOD, GPIO_Pin_14);
			//ZapalDiode1();

			jazda = 1;
			sendFlag = 1;// flaga do sprawdzania czy przycisk wcisniety raz  i nie przytrzymywany




		//	Jedz_odleglosc(132);

		//	Wyslij_int("Zlecona odleglosc: %d",odleglosc);
		//	Wyslij_int("Odcinkow po 10cm: %d",odcinek);



		//	Wyslij_zdanie("siemaneczko",20);
		//	Wyslij_zdanie("\r\n",4);


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
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	  /**************************************************************************************/

	  GPIO_InitTypeDef GPIO_InitStructure;
	  /*-------------------------- GPIO Configuration ----------------------------*/
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;//GPIO_Pin_9 | GPIO_Pin_10;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOB, &GPIO_InitStructure);
	//  GPIO_Init(GPIOA, &GPIO_InitStructure);
	  /* Connect USART pins to AF */
	  GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1); // USART1_TX
	  GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1); // USART1_RX

	//  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1); // USART1_TX
//	  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1); // USART1_RX

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
	   // GPIO_SetBits(GPIOD, GPIO_Pin_15);
	    if(c == 'g')
	    {
	    //	GPIO_SetBits(GPIOD, GPIO_Pin_13);
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
	sprintf((char*)text2," odleglosc w mv : %d  ",ADC3ConvertedVoltage);

//	Wyslij_zdanie(text,20);
//	Wyslij_zdanie("\r\n",4);

	Wyslij_zdanie(text2,20);
	Wyslij_zdanie("\r\n",4);



}
int Sprawdz_odleglosc()
{
	ADC3ConvertedVoltage = ADC3ConvertedValue *3300/0xFFF;


	if(ADC3ConvertedVoltage > 1250)
	{
		Wyslij_zdanie("Widze cos do 20cm.",20);
		Wyslij_zdanie("\r\n",4);
		//Strefa_odleglosc = 1;
		return 1;
	}
	else if(ADC3ConvertedVoltage < 600)
	{
		Wyslij_zdanie("Widze cos za  40cm.",20);
		Wyslij_zdanie("\r\n",4);
		//Strefa_odleglosc = 3;
		return 3;
	}
	else
	{
		Wyslij_zdanie("Widze cos miedzy 20 a 40cm.",20);
		Wyslij_zdanie("\r\n",4);
		//Strefa_odleglosc = 2;
		return 2;
	}


}
int Sprawdz_lewy() // Zwraca 0 jak jest przeszkoda i 1 jak nnie ma
{
	if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_1) == 0) // PC1 to jest lewy czujnik optycczny
	{
		//GPIO_SetBits(GPIOD, GPIO_Pin_15);
		Wyslij_zdanie("Przeszkoda po lewej stronie.",30);
		Wyslij_zdanie("\r\n",4);
		return 0;
	}
	else
	{
		//GPIO_ResetBits(GPIOD, GPIO_Pin_15);
		Wyslij_zdanie("Brak przeszkody po lewej stronie.",30);
		Wyslij_zdanie("\r\n",4);
		return 1;
	}

}
int Sprawdz_prawy()	// Zwraca 0 jak jest przeszkoda i 1 jak nnie ma
{
	if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_3) == 0) // PC1 to jest lewy czujnik optycczny
	{
		//GPIO_SetBits(GPIOD, GPIO_Pin_15);
		Wyslij_zdanie("Przeszkoda po prawej stronie.",30);
		Wyslij_zdanie("\r\n",4);
		return 0;
	}
	else
	{
		//GPIO_ResetBits(GPIOD, GPIO_Pin_15);
		Wyslij_zdanie("Brak przeszkody po prawej stronie.",30);
		Wyslij_zdanie("\r\n",4);
		return 1;
	}

}



void Jedz_impuls(int impuls)
{
	Wyslij_zdanie("Jedziemy prosto",20);
	Wyslij_zdanie("\r\n",4);
	Delay(1000);	//odczekamy sobie dwie sekundy

	licznik_lewy = 0;

	//int zadane =  licznik_lewy + impuls;		// w sumie nie istotne jaki bedzie licznik_lewy
	Silniki_Przod();
	while(impuls > licznik_lewy)
	{}
	Silniki_Stop();


}
void Skret_Lewo()
{
	Wyslij_zdanie("Krecimy w lewo",20);
	Wyslij_zdanie("\r\n",4);
	Delay(3000);
	int zadane =  licznik_prawy + 525;
	Silniki_Lewo();
	while(zadane > licznik_prawy)
	{}
	Silniki_Stop();

}

void Skret_Prawo()
{
		Wyslij_zdanie("Krecimy w prawo",20);
		Wyslij_zdanie("\r\n",4);
	Delay(3000);
	int zadane =  licznik_prawy - 590;
	Silniki_Prawo();
	while(zadane < licznik_prawy)
	{}
	Silniki_Stop();

}


int Sprawdz_przod()	// zwraca 1 jak mozna jechac, 0 jak nie
{
	Strefa_odleglosc = Sprawdz_odleglosc();
	if(Strefa_odleglosc == 3 || Strefa_odleglosc == 2)
	{
		Wyslij_zdanie("Brak przeszkody z przodu - jedziemy.",20);
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
void Wykonaj_ruch()
{
	Delay(2000);
	// Generalnie ruch przebiega poprzez wybrór dnego z ruchów : prosto/lewo/prawo
	// Napierw sprawdzane jest przemieszczenie w poziomie	: 3 opcje
	// Zaleznie od tego sprawdzane jest w jakim kierunku zwrocony jest robot	: 4 opcje
	// Algorytm wybiera jeden z 12 scenariuszy : 3 przemieszczenia x 4 opcje kierunku

	if(ruch_poziom == 0)	// robot na trasie
	{
		if(kierunek == 1)	// robot na trasie zwrocony do przodu
		{
			if(Sprawdz_przod() == 1)
			{
				Jedz_impuls(300);
				ruch_pion = ruch_pion + 10;
			}
			else
			{
				if(Sprawdz_prawy() == 1)
				{
					Skret_Prawo();
					kierunek = 2;
				}
				else
				{
					if(Sprawdz_lewy() == 1)
					{
						Skret_Lewo();
						kierunek = 4;
					}
					else
					{
						Wyslij_zdanie("Slepy zaulek - STOP.",20);
						Wyslij_zdanie("\r\n",4);
					}
				}
			}
			Wyslij_zdanie("Scenariusz 1 - robot na trasie zwrocony do przodu.",20);
			Wyslij_zdanie("\r\n",4);

		}
		else if(kierunek == 2)	//robot na	trasie zwrocony w prawo
		{
			if(Sprawdz_lewy() == 1)
						{
							Skret_Lewo();
							kierunek = 1;
						}
						else
						{
							if(Sprawdz_przod() == 1)
							{
								Jedz_impuls(300);
								ruch_poziom = ruch_poziom + 10;
							}
							else
							{
								if(Sprawdz_prawy() == 1)
								{
									Skret_Prawo();
									kierunek = 3;
								}
								else
								{
									Wyslij_zdanie("Slepy zaulek - STOP.",20);
									Wyslij_zdanie("\r\n",4);
								}
							}
						}
						Wyslij_zdanie("Scenariusz 2 - robot na	trasie zwrocony w prawo.",20);
						Wyslij_zdanie("\r\n",4);


		}
		else if(kierunek == 3)	//robot na trasie zwrocony w tyl
		{
			if(Sprawdz_prawy() == 1)
			{
				Skret_Prawo();
				kierunek = 4;
			}
			else
			{
				if(Sprawdz_lewy() == 1)
				{
					Skret_Lewo();
					kierunek = 2;
				}
				else
				{
					if(Sprawdz_przod() == 1)
					{
						Jedz_impuls(300);
						ruch_pion = ruch_pion -10;
					}
					else
					{
						Wyslij_zdanie("Slepy zaulek - STOP.",20);
						Wyslij_zdanie("\r\n",4);
					}
				}
			}
			Wyslij_zdanie("Scenariusz 3 - robot na trasie zwrocony w tyl.",20);
			Wyslij_zdanie("\r\n",4);

		}
		else if(kierunek == 4)	//robot na trasie zwrocony w lewo
		{
			if(Sprawdz_prawy() == 1)
			{
				Skret_Prawo();
				kierunek = 1;
			}
			else
			{
				if(Sprawdz_przod() == 1)
				{
					Jedz_impuls(300);
					ruch_poziom = ruch_poziom - 10;
				}
				else
				{
					if(Sprawdz_lewy() == 1)
					{
						Skret_Lewo();
						kierunek = 3;
					}
					else
					{
						Wyslij_zdanie("Slepy zaulek - STOP.",20);
						Wyslij_zdanie("\r\n",4);
					}
				}
			}
			Wyslij_zdanie("Scenariusz 4 - robot na trasie zwrocony w lewo.",20);
			Wyslij_zdanie("\r\n",4);

		}

	}
	else if(ruch_poziom > 0)	// robot zjechal w prawo
	{
		if(kierunek == 1)	// robot zjechal w prawo zwrocony do przodu
		{
			if(Sprawdz_lewy() == 1)
						{
							Skret_Lewo();
							kierunek = 4;
						}
						else
						{
							if(Sprawdz_przod() == 1)
							{
								Jedz_impuls(300);
								ruch_pion = ruch_pion + 10;
							}
							else
							{
								if(Sprawdz_prawy() == 1)
								{
									Skret_Prawo();
									kierunek = 2;
								}
								else
								{
									Wyslij_zdanie("Slepy zaulek - STOP.",20);
									Wyslij_zdanie("\r\n",4);
								}
							}
						}
						Wyslij_zdanie("Scenariusz 5 - robot zjechal w prawo zwrocony do przodu.",20);
						Wyslij_zdanie("\r\n",4);

		}
		else if(kierunek == 2)	// robot zjechal w prawo zwrocony w prawo
		{
			if(Sprawdz_lewy() == 1)
						{
							Skret_Lewo();
							kierunek = 1;
							Jedz_impuls(300);
							Jedz_impuls(300);	// Wyj¹tek
							Jedz_impuls(300);
							ruch_pion = ruch_pion + 30;
						}
						else
						{
							if(Sprawdz_przod() == 1)
							{
								Jedz_impuls(300);
								ruch_poziom = ruch_poziom + 10;
							}
							else
							{
								if(Sprawdz_prawy() == 1)
								{
									Skret_Prawo();
									kierunek = 3;
								}
								else
								{
									Wyslij_zdanie("Slepy zaulek - STOP.",20);
									Wyslij_zdanie("\r\n",4);
								}
							}
						}
						Wyslij_zdanie("Scenariusz 6 - robot zjechal w prawo zwrocony w prawo.",20);
						Wyslij_zdanie("\r\n",4);

		}
		else if(kierunek == 3)	// robot zjechal w prawo zwrocony w tyl
		{
			if(Sprawdz_prawy() == 1)
						{
							Skret_Prawo();
							kierunek = 4;
						}
						else
						{
							if(Sprawdz_lewy() == 1)
							{
								Skret_Lewo();
								kierunek = 2;
							}
							else
							{
								if(Sprawdz_przod() == 1)
								{
									Jedz_impuls(300);
									ruch_pion = ruch_pion - 10;
								}
								else
								{
									Wyslij_zdanie("Slepy zaulek - STOP.",20);
									Wyslij_zdanie("\r\n",4);
								}
							}
						}
						Wyslij_zdanie("Scenariusz 7 - robot zjechal w prawo zwrocony w tyl.",20);
						Wyslij_zdanie("\r\n",4);

		}
		else if(kierunek == 4)	// robot zjechal w prawo zwrocony w lewo
		{
			if(Sprawdz_przod() == 1)
						{
							Jedz_impuls(300);
							ruch_poziom = ruch_poziom - 10;
						}
						else
						{
							if(Sprawdz_prawy() == 1)
							{
								Skret_Prawo();
								kierunek = 1;
							}
							else
							{
								if(Sprawdz_lewy() == 1)
								{
									Skret_Lewo();
									kierunek = 3;
								}
								else
								{
									Wyslij_zdanie("Slepy zaulek - STOP.",20);
									Wyslij_zdanie("\r\n",4);
								}
							}
						}
						Wyslij_zdanie("Scenariusz 8 - robot zjechal w prawo zwrocony w lewo.",20);
						Wyslij_zdanie("\r\n",4);

		}

	}
	else if(ruch_poziom < 0)	// robot zjechal w lewo
	{
		if(kierunek == 1)	// robot zjechal w lewo zwrocony do przodu
		{
			if(Sprawdz_prawy() == 1)
						{
							Skret_Prawo();
							kierunek = 2;
						}
						else
						{
							if(Sprawdz_przod() == 1)
							{
								Jedz_impuls(300);
								ruch_pion = ruch_pion + 10;
							}
							else
							{
								if(Sprawdz_lewy() == 1)
								{
									Skret_Lewo();
									kierunek = 4;
								}
								else
								{
									Wyslij_zdanie("Slepy zaulek - STOP.",20);
									Wyslij_zdanie("\r\n",4);
								}
							}
						}
						Wyslij_zdanie("Scenariusz 9 - robot zjechal w lewo zwrocony do przodu.",20);
						Wyslij_zdanie("\r\n",4);

		}
		else if(kierunek == 2)	// robot zjechal w lewo zwrocony w prawo
		{
			if(Sprawdz_przod() == 1)
						{
							Jedz_impuls(300);
							ruch_poziom = ruch_poziom + 10;
						}
						else
						{
							if(Sprawdz_lewy() == 1)
							{
								Skret_Lewo();
								kierunek = 1;
							}
							else
							{
								if(Sprawdz_prawy() == 1)
								{
									Skret_Prawo();
									kierunek = 3;
								}
								else
								{
									Wyslij_zdanie("Slepy zaulek - STOP.",20);
									Wyslij_zdanie("\r\n",4);
								}
							}
						}
						Wyslij_zdanie("Scenariusz 10 - robot zjechal w lewo zwrocony w prawo.",20);
						Wyslij_zdanie("\r\n",4);

		}
		else if(kierunek == 3)	// robot zjechal w lewo zwrocony w tyl
		{
			if(Sprawdz_lewy() == 1)
						{
							Skret_Lewo();
							kierunek = 2;
						}
						else
						{
							if(Sprawdz_prawy() == 1)
							{
								Skret_Prawo();
								kierunek = 4;
							}
							else
							{
								if(Sprawdz_przod() == 1)
								{
									Jedz_impuls(300);
									ruch_pion = ruch_pion + 10;
								}
								else
								{
									Wyslij_zdanie("Slepy zaulek - STOP.",20);
									Wyslij_zdanie("\r\n",4);
								}
							}
						}
						Wyslij_zdanie("Scenariusz 11 - robot zjechal w lewo zwrocony w tyl.",20);
						Wyslij_zdanie("\r\n",4);

		}
		else if(kierunek == 4) // robot zjechal w lewo zwrocony w tyl
		{
			if(Sprawdz_prawy() == 1)
						{
							Skret_Prawo();
							kierunek = 1;
							Jedz_impuls(300);
							Jedz_impuls(300);	//Wyjatek
							Jedz_impuls(300);
							ruch_pion = ruch_pion + 30;
						}
						else
						{
							if(Sprawdz_przod() == 1)
							{
								Jedz_impuls(300);
								ruch_poziom = ruch_poziom - 10;
							}
							else
							{
								if(Sprawdz_lewy() == 1)
								{
									Skret_Lewo();
									kierunek = 3;

								}
								else
								{
									Wyslij_zdanie("Slepy zaulek - STOP.",20);
									Wyslij_zdanie("\r\n",4);
								}
							}
						}
						Wyslij_zdanie("Scenariusz 12 - robot zjechal w lewo zwrocony w tyl.",20);
						Wyslij_zdanie("\r\n",4);

		}

	}


	Wyslij_int("Przejechane w pionie: %d.", ruch_pion);
	Wyslij_int("Przejechane w poziomie: %d.", ruch_poziom);
	Wyslij_int("Obrany kiernek: %d.", kierunek);


}


void Algorytm_jazdy(int powtorzenia)
{
	for(int i =0;i<powtorzenia;i++)
	{
		GPIO_ToggleBits(GPIOD,GPIO_Pin_13);
			Delay(5000);		// 5 sekund
			Sprawdz_odleglosc()	;		// zwraca int zaleznie od strefy w której widzi przeszkode
			ObslugaCzujkaOdleglosci();

			if(CzyMogeJechac() == 0)	// wykrywa przeszkodê - w srodku omijanie
			{
				int n ;
				n = Omijanie_Przeszkody();
				powtorzenia = powtorzenia + n;
			}
			else {

				Jedz_impuls(300); 			// jedzie o do przodu o impuls enkodera
			}
	}
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
void Wyslij_int(char* wiadomosc,int data)	// pamietaj zeby w wiadomosci dac " %d "
{
	uint8_t text[50];
	sprintf((char*)text,wiadomosc,data);	// podobno uzywanie sprintf do wysylki przez uart mega obciaza procka
	Wyslij_zdanie(text,20);
	Wyslij_zdanie("\r\n",4);

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

	  UstawieniePIN(GPIOC,GPIO_Pin_1,GPIO_Mode_IN);	//Sprawdz lewy
	  UstawieniePIN(GPIOC,GPIO_Pin_3,GPIO_Mode_IN);	//Sprawdz lewy
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

