#include "stm32f4xx.h"
#include "stdio.h"
#include "string.h"
int ncount;
char BUFFER[100];

int n;
int i ;
int j;
char BUFFER_REC[20];


int ADC2_DMA_Buffer[3];

#define ADC_BASE_ADR        ((uint32_t)0x40012000)
#define ADC_ADC2_OFFSET     ((uint32_t)0x00000100)
#define ADC_REG_DR_OFFSET     0x4C

#define ADC2_DR_ADDRESS    (ADC_BASE_ADR | ADC_ADC2_OFFSET | ADC_REG_DR_OFFSET)


#define DMA_Channel_1                      0x1<<25
#define DMA_DIR_PeripheralToMemory         0x0<<6
#define DMA_PeripheralInc_Disable          0x0<<9
#define DMA_MemoryInc_Enable               0x1<<10
#define DMA_PeripheralDataSize_Word        0x2<<11
#define DMA_Mode_Circular                  0x1<<8
#define DMA_Priority_High                  0x2<<16
#define DMA_MemoryBurst_Single             0x0<<23
#define DMA_PeripheralBurst_Single         0x0<<21
#define DMA_MemoryDataSize_Word            0x2<<13

#define DMA_FIFOMode_Disable              0x0<<1
#define DMA_FIFOThreshold_HalfFull        0x1<<0


#define RCC_APB2Periph_ADC2              ((uint32_t)0x00000200)
#define RCC_AHB1Periph_DMA2              ((uint32_t)0x00400000)

#define ADC_Mode_Independent 0x00000000
#define ADC2d_VORTEILER   0x00010000
#define ADC_DMAAccessMode_Disabled  0x00000000
#define ADC_TwoSamplingDelay_5Cycles  0x00000000





void DMA_init_CHANNEL1_STREAM2(void)
{
  //DMA2 clock enabled
  RCC->AHB1ENR |= RCC_AHB1Periph_DMA2; //RCC->AHB1ENR |= 0x00400000;


  /*------------------------- DMAy Streamx CR Configuration ------------------*/
  DMA2_Stream2->CR|=DMA_Channel_1;
  DMA2_Stream2->CR|=DMA_DIR_PeripheralToMemory;
  DMA2_Stream2->CR|=DMA_PeripheralInc_Disable;
  DMA2_Stream2->CR|=DMA_MemoryInc_Enable;
  DMA2_Stream2->CR|=DMA_PeripheralDataSize_Word;
  DMA2_Stream2->CR|=DMA_MemoryDataSize_Word;
  DMA2_Stream2->CR|= DMA_Mode_Circular;
  DMA2_Stream2->CR|=DMA_Priority_High;
  DMA2_Stream2->CR|=DMA_MemoryBurst_Single;
  DMA2_Stream2->CR|=DMA_PeripheralBurst_Single;

  /*****************************NUMBER DATA*********************************/
   DMA2_Stream2->NDTR =3;
   /*------------------------- DMAy Streamx PAR Configuration -----------------*/

  DMA2_Stream2->PAR = ADC2_DR_ADDRESS;// DMA2_Stream2->PAR=0x40012000|0x00000100|0x4C;

   /*------------------------- DMAy Streamx M0AR Configuration ----------------*/
   DMA2_Stream2->M0AR = &ADC2_DMA_Buffer; //DMA2_Stream2->M0AR = &ADC2_DMA_Buffer;

  /*------------------------- DMAy Streamx FCR Configuration -----------------*/
  // DISABLE FIFO MODE
  DMA2_Stream2->FCR|= DMA_FIFOMode_Disable;

  /*****************************INTERRUPT ENABLE****************************/
  DMA2_Stream2->CR |=  0x00000010;  //TCIE=1 transfer complete interrupt enable
  NVIC_EnableIRQ(DMA2_Stream2_IRQn);

  /* Enable the selected DMAy Streamx by setting EN bit */
   DMA2_Stream2->CR |= (uint32_t)DMA_SxCR_EN;  //DMA2_Stream2->CR |= 0x00000001;

}




void P_ADC2d_InitADC(void)
{

  //Clock Enable (PCLK2=42MHz)
  RCC->APB2ENR |= RCC_APB2Periph_ADC2;

  //ADC2 PRESCALER=8
  ADC->CCR|= 3<<16;

  //ADC2 MODE SCAN (MULTICHANNEL)
  ADC2->CR1|= 0x00000100;

  // enable mode DISCONT 3 channels à la fois aprés le start
  ADC2->CR1|= 0x00004800;

  //ADC2(disable mode CONT )

  ADC2->CR2|= 0x00000000;

   //ADC2 LENGTH CHANNEL=3
   ADC2->SQR1 = 2<<20;

  //PA3 CHANNEL3 ADC2
   ADC2->SMPR2=0x00000000;
   ADC2->SMPR1=0x00000000;

   ADC2->SQR3=0x00003dC3;

   /* Enable the selected ADC DMA request after last transfer */
   ADC2->CR2 |= (uint32_t)ADC_CR2_DDS;

   /* Enable the selected ADC DMA request */
   ADC2->CR2 |= (uint32_t)ADC_CR2_DMA;

   // Enable clock for GPIOA & GPIOC
   RCC->AHB1ENR |= 0x00000005;
   //PA3 are analog inputs
   GPIOA->MODER |= 0x000000C0;
   //PC4 PC5 are analog inputs
   GPIOC->MODER |= 0x00000F00;

   /* Set the ADON bit to wake up the ADC from power down mode */
   ADC2->CR2 |= (uint32_t)ADC_CR2_ADON;
}


//--------------------------------------------------------------
// internal function
// Enable and start from ADC and DMA
//--------------------------------------------------------------
void P_ADC2d_Start(void)
{
    /* Enable the selected ADC conversion for regular group */
    ADC2->CR2 |= (uint32_t)ADC_CR2_SWSTART;
}

void UsartInit()
{
  // Enable clock for GPIOB
  RCC->AHB1ENR |= 0x00000002;
  // Enable clock for USART3
  RCC->APB1ENR|=0x00040000;
  //enable USART3_TX to PB10 and USART3_RX to PB11
  GPIOB->AFR[1]=0x00007700;
  // configuring the USART3 ALTERNATE function  to PB10 and PB11
  GPIOB->MODER|=0x2AA00000;
  //  BaudRate:9600
  USART3->BRR=0x1120;
  //USART3->BRR     = 0x0890;      // 19200 baud
  //USART4->BRR     = 0x0450;      // 38400 baud
  //USART4->BRR     = 0x02e0;      // 57600 baud
  //USART4->BRR     = 0x016d;      // 115200 baud

  // USART3 enable
  USART3->CR1|=0x0000203C;
  NVIC_EnableIRQ(USART3_IRQn);
}



/**********************************************************************************
    //TRANSMISION 1 seul caractère
 **********************************************************************************/

void SendChar(char Tx)
{
	while((USART3->SR&0x80)==0);  // On attend à ce que TXBUFF soit vide (libere) ou (FLAG TXNE=1) ou Le caractere est envoyé
	USART3->DR=Tx;
}

/**********************************************************************************
    //TRANSMISION d'une chaine caractère: STRING
 **********************************************************************************/
void SendTxt(char *Adr)
{
  while(*Adr)
  {
    SendChar(*Adr);
    Adr++;
  }
}


void SystemInit_1() /// CLK_TIMER=84MHz
{

  RCC->CFGR |= 0x00009400;        // AHB_presc=1  APB1_presc=4
  RCC->CR |= 0x00010000;          // HSE_ON
  while (!(RCC->CR & 0x00020000));// wait until HSE READY

  RCC->PLLCFGR = 0x07402A04;      // PLL  M=4, N=168, P=2  Q=7 yapalim   168 Mhz
  RCC->CR |= 0x01000000;          // PLL_ON
  while(!(RCC->CR & 0x02000000)); // wait Pll_ready

  FLASH->ACR = 0x00000605;        // Flash ROM icin 5 Wait state secelim ve ART yi aktif edelim (Rehber Sayfa 55)
  RCC->CFGR |= 0x00000002;        // Sistem Clk u PLL uzerinden besleyelim
  while ((RCC->CFGR & 0x0000000F) != 0x0000000A); // Besleninceye kadar bekle

}


void config_TIMER3()
{
	 // Enable TIM3 clock (RCC_APB1ENR[1]=1)
	 RCC->APB1ENR |= 0x00000002;//le bit [0] du registre APB1ENR
	 // Set prescaler to 7999
	 TIM3->PSC = 7999; // prescaler=8000 sachant syst clk=HSE	                   // => counter clk= 1000Hz
	 // Set ARR to 999
	 TIM3->ARR = 999; // le compteur compte de 0 à 999
	 //ENABLE counter CCR=<ARR , de CH1,CH2,CH3
	 TIM3->CCER|=0x0111;
	 // CH1,CH2 output mode 1
	 TIM3->CCMR1|=0x6060;
	 // CH3 output mode 1
	 TIM3->CCMR2|=0x0060;

	 //ACTIVATION CLOCK GPIOB <-------------------------------------UP DATE
	 RCC->AHB1ENR |= 0x00000003;

	 //PA6 et PA7 en mode alternate function
	 GPIOA->MODER|=0x0000A000;
	 // lier TIM3 CH1(AF2) à PA6 et CH2(AF2) à PA7 connectée
	 GPIOA->AFR[0]|=0x22000000;

	 //CONFIG PIN0 GPIOB EN MODE ALTERNATE FUNCTION <------------------------UPDATE
	 GPIOB->MODER|=0x00000002;
	 // lier TIM3 CH3 (AF2) à PB0
	 GPIOB->AFR[0]|=0x00000002;


	 // ENABLE counter CNT=ARR bit tim2[0]= CEN=1
	TIM3->CR1= 0x0001;

}


void config_TIMER6()
{
	 // Enable TIM6 clock (RCC_APB1ENR[4]=1)
	 RCC->APB1ENR |= 0x00000010;//le bit [4] du registre APB1ENR

	 // Set prescaler to 7999
	 TIM6->PSC = 9999; // prescaler=8000 sachant syst clk=168MHz
	                   // => counter clk= 16.8kHZ
	 // Set ARR to 999
	 TIM6->ARR = 4999; // le compteur compte de 0 à 9999
	                   // -->puis UIF passe 1
	 //INTERRUPT ENABLE (TIM6: DIER[0]=UIE=1)
     TIM6->DIER |= 0x0001; // TIM6 passe en inteerupt qd UIF=1
     //ENABLE INTERRIUPT REQUEST
	 NVIC_EnableIRQ(TIM6_DAC_IRQn); //interrupt request = 1
	 //ENABLE TIMER6

}





//--------------------------------------------------------------
// interne Funktionen
//--------------------------------------------------------------
void P_ADC2d_InitIO(void);
void P_ADC2d_InitDMA(void);
void P_ADC2d_InitADC(void);
void P_ADC2d_Start(void);


int main(void)
{
 //System clock : HSExPLL=8x21=168MHz
  SystemInit_1();

  UsartInit();
  config_TIMER3();
  DMA_init_CHANNEL1_STREAM2();
  P_ADC2d_InitADC();
  config_TIMER6();


  while(1);
}


void DMA2_Stream2_IRQHandler()
{

  if(DMA2->LISR&(1<<21))
  {
	   TIM3->CCR1= 0.243*ADC2_DMA_Buffer[0];
       TIM3->CCR2= 0.243*ADC2_DMA_Buffer[1];
       TIM3->CCR3= 0.243*ADC2_DMA_Buffer[2];





	      sprintf(BUFFER,"ADC_CH3=%d ADC_CH14=%d ADC_CH15=%d\r\n",ADC2_DMA_Buffer[0],ADC2_DMA_Buffer[1],ADC2_DMA_Buffer[2]);
	      SendTxt(BUFFER);


       DMA2->LIFCR|=1<<21;
  }
}






void TIM6_DAC_IRQHandler()
{

	if((TIM6->SR & 0x0001) != 0) //TEST  if TIMER6 FLAG UIF is set
    {
		 ADC2->CR2 |= (uint32_t)ADC_CR2_SWSTART;


    }
    	//Clear FLAG UIF (UIF=0)
    	TIM6->SR &= 0x0000;

    }

void   USART3_IRQHandler()
{

	if((USART3->SR & 0x0020)!=0) //TEST FLAG RXNE
    {
	  j++;
	  BUFFER_REC[j-1]=USART3->DR;
	}

	if((USART3->SR & 0x0010)!=0) //TEST FLAG IDLE
	{
		 n=strlen(BUFFER_REC);

		if(strstr(BUFFER_REC,"ON"))
		{
			TIM6->CR1= 0x0001;
		}
		if(strstr(BUFFER_REC,"OFF"))
		{
			TIM6->CR1= 0x0000;
			       TIM3->CCR1= 0x0000;
			       TIM3->CCR2= 0x0000;
			       TIM3->CCR3= 0x0000;

		}


	   for(i=0;i<n;i++)
	   {
		   BUFFER_REC[i]=0;
	   }

           j=0;
	   USART3->SR;
           USART3->DR;
    }

}









