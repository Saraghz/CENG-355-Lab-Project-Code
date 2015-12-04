

//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------
// School: University of Victoria, Canada.
// Course: CENG 355 "Microprocessor-Based Systems".
//
//
// See "system/include/cmsis/stm32f0xx.h" for register/bit definitions.
// See "system/src/cmsis/vectors_stm32f0xx.c" for handler declarations.
// ----------------------------------------------------------------------------

#include <stdio.h>
#include <string.h>
#include "diag/Trace.h"
#include "cmsis/cmsis_device.h"

// ----------------------------------------------------------------------------
//
// STM32F0 empty sample (trace via $(trace)).
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the $(trace) output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"


/* Clock prescaler for TIM2 timer: no prescaling */
#define myTIM2_PRESCALER ((uint16_t)0x0000)
/* Clock prescaler for TIM3 timer: 1Khz prescaling */
#define myTIM3_PRESCALER ((uint16_t) 47999)
/* Maximum possible setting for overflow */
#define myTIM2_PERIOD ((uint32_t)0xFFFFFFFF)


//Declarations for Initializations
void myGPIOA_Init(void);
void myTIM2_Init(void);
void myTIM3_Init(void);
void myEXTI_Init(void);
void myADC_Init(void);
void myDAC_Init(void);
void mySPI_Init(void);


// LCD related functions
void myLCD_Init(void);
void myrefresh_LCD(void);
void myDelay(uint32_t);
void myLCD_Config(void);
void myWrite_Data(char);
void myWrite_Cmd(char);
void HC595_Write(char);
void Position_LCD(unsigned short, unsigned short);
void Write_LCD_Text(char *);
void Write_LCD_Num(uint16_t);
void itoa(uint16_t, char []);

// Your global variables...
/* Global variable keeping track of elapsed timer pulses */
volatile uint32_t timerPulses = 0;
/* Global variable indicating if TIM2 has been started */
volatile uint32_t timerActive = 0;

volatile uint16_t DACconvertedval = 0;
unsigned int frequency = 0;
unsigned int Voltage = 0;
uint16_t resistance = 0;

int main(int argc, char* argv[])
{

    trace_printf("This is Part 2 of Introductory Lab...\n");
    trace_printf("System clock: %u Hz\n", SystemCoreClock);

    myGPIOA_Init();   	 /* Initialize I/O port PA */
    myTIM2_Init();   	 /* Initialize timer TIM2 */
    myTIM3_Init();		 /* Initialize timer TIM3 */
    myEXTI_Init();   	 /* Initialize EXTI */
    myADC_Init();   	 /* Initialize ADC Port PC1 Input*/
    myDAC_Init();   	 /* Initialize DAC Port PA4 Output*/
	myLCD_Config();		 /* Set up LCD */

   while (1)
    {
		/*Trigger ADC conversion in Software */
	   	ADC1->CR |= 0x00000004;

		/* Test EOC flag */
		while((ADC1->ISR & 0x00000004)==0);

		/* calculate the voltage */
		Voltage = (unsigned int)(ADC1->DR);
		DAC->DHR12R1=(Voltage);
		/* Get ADC1 converted data */
		DACconvertedval = DAC->DOR1;

		/* Resistance calculations */
		float rescalc = 5000*((4095 - ((float)DACconvertedval))/4095 );
		resistance = ((uint16_t)rescalc);

		/*Computer Console prints */
		trace_printf("Fq:%d\n",frequency);
		trace_printf("Voltage:%d\n",Voltage);
		trace_printf("Resistance:%d\n\n",resistance);

		/* Display converted data on the LCD */
		myrefresh_LCD();

    }

    return 0;

}


void myGPIOA_Init()
{
    /* Enable clock for GPIOA peripheral */
    // Relevant register: RCC->AHBENR

    RCC->AHBENR |= 0x00020000;

    /* Configure PA1 as input */
    // Relevant register: GPIOA->MODER

    GPIOA->MODER |= 0x28000000;

    /* Ensure no pull-up/pull-down for PA1 */
    // Relevant register: GPIOA->PUPDR

    GPIOA->PUPDR |= 0x24000000;
}


void myrefresh_LCD(void)
{

	Position_LCD(1,1);

	Write_LCD_Text("R:");

	Write_LCD_Num(resistance);

	Position_LCD(1,7);

	Write_LCD_Text("Oh");

	Position_LCD(2,1);

	Write_LCD_Text("F:");

	Write_LCD_Num(frequency);

	Position_LCD(2,7);

	Write_LCD_Text("Hz");

	myDelay(250); /* 250­ms refresh period (LCD visual perception) */

}


void myDelay(uint32_t d)
{

	TIM3->CNT |= 0x00000000;		/* Clear timer */

	TIM3->ARR = d;					/* Time­out value */

    TIM3->EGR |= 0x0001; 			/* Update registers */

    TIM3->CR1 |= TIM_CR1_CEN;		/* Start timer */


	while ((TIM3->SR & TIM_SR_UIF) == 0) {} /* Wait for time­out delay */

  	TIM3->SR &= ~(TIM_SR_UIF);		/* Clear update interrupt flag */
  	TIM3 -> CR1	&= ~(TIM_CR1_CEN);  /* Stop timer */
}


void myLCD_Config()
{
	mySPI_Init();		 /* Initialize SPI */
	myLCD_Init();		 /* Initialize LCD */
	trace_printf("\n LCD initialization has completed... \n");
}


void myLCD_Init()
{


	HC595_Write(0x02); // Let EN = 0, RS = 0, DB[7:4] = 0010
	HC595_Write(0x82); // Let EN = 1, RS = 0, DB[7:4] = 0010
	HC595_Write(0x02); // Let EN = 0, RS = 0, DB[7:4] = 0010

	myDelay(20);

	myWrite_Cmd(0x28); // 4­bits, 2 lines, 5x7 font
	myWrite_Cmd(0x0E); // Display ON, No cursors
	myWrite_Cmd(0x06); // Entry mode­ Auto­increment, No Display shifting
	myWrite_Cmd(0x01); // Clear display

	myDelay(20);
}


void myWrite_Cmd(char cmd)
{
	char Low_Nibble = (cmd & 0x0f);
	char High_Nibble = ((cmd & 0xf0) >> 4);
	//High
	HC595_Write(0x00 + High_Nibble); // Let RS = 0 and EN = 0
	HC595_Write(0x80 + High_Nibble); // Let RS = 0 and EN = 1
	HC595_Write(0x00 + High_Nibble); // Let RS = 0 and EN = 0


	//Low
	HC595_Write(0x00 + Low_Nibble); // Let RS = 0 and EN = 0
	HC595_Write(0x80 + Low_Nibble); // Let RS = 0 and EN = 1
	HC595_Write(0x00 + Low_Nibble); // Let RS = 0 and EN = 0

}


void myWrite_Data(char data)
{
	char Low_Nibble = (data & 0x0f);
	char High_Nibble = ((data & 0xf0) >> 4);
	//High
	HC595_Write(0x40 + High_Nibble ); // Let RS = 1 and EN = 0
	HC595_Write(0xc0 + High_Nibble ); // Let RS = 1 and EN = 1
	HC595_Write(0x40 + High_Nibble ); // Let RS = 1 and EN = 0


	//Low
	HC595_Write(0x40 + Low_Nibble ); // Let RS = 1 and EN = 0
	HC595_Write(0xc0 + Low_Nibble ); // Let RS = 1 and EN = 1
	HC595_Write(0x40 + Low_Nibble ); // Let RS = 1 and EN = 0

}


void HC595_Write(char data)
{

	// LCK = 0
	GPIOB->BRR = GPIO_Pin_4;

	// Wait until not busy
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY)==SET){;}

	SPI_SendData8(SPI1,data);

	// Wait until not busy
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY)==SET){;}

	// LCK = 1 (rising edge)
	GPIOB->BSRR = GPIO_Pin_4;

	//myDelay(2); // 2­ms delay (slow LCD)


}


void Position_LCD(unsigned short x, unsigned short y)

{
	unsigned short temp = 127 + y;
	if (x == 2)
		temp = temp + 64;
	myWrite_Cmd(temp); //moves cursor
}



void Write_LCD_Text(char *StrData)
{
	unsigned short p = 0;
	unsigned short q = strlen(StrData); //Calculates the string length
	while(p < q){
		myWrite_Data(StrData[p]);  		//Writes the string data to the LCD

		p++;
	}
}


void Write_LCD_Num(uint16_t num)
{
	char Pot_st[5]={'0','0','0','0',0};
	itoa(num,Pot_st); 			//Converters the num in to ASCII and puts it in the array
	Write_LCD_Text(Pot_st);
}


//Converts integer to character
void itoa(uint16_t n, char s[])
{
	//uint16_t count = 3;
	//off set by 0x30 or '0' for dec to ascii
	s[0] = n/1000 +'0';
	s[1] =((n%1000)/100) + '0';
	s[2] =((n%100)/10) +'0';
	s[3] = n%10 + '0';
}




void myTIM2_Init()
{
    /* Enable clock for TIM2 peripheral */
    // Relevant register: RCC->APB1ENR

    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    /* Configure TIM2: buffer auto-reload, count up, stop on overflow,
     * enable update events, interrupt on overflow only */
    // Relevant register: TIM2->CR1

    //TIM2->CR1 |= 0x00C8;     //set auto-reload
    TIM2->CR1 = ((uint16_t) 0x008C);

    //TIM2->CR1 |= 0x0004;    //interrupt on overflow only
    //TIM2->CR1 |= 0x0008;	//stop on overflow
    //TIM2->CR1 &= ~(0x0060); //set CMS to follow DIR bit
    //TIM2->CR1 &= ~(0x0010); //set DIR bit as upcounter
    //TIM2->CR1 &= ~(0x0002);    //enable update events

    /* Set clock prescaler value */
    TIM2->PSC = myTIM2_PRESCALER;
    /* Set auto-reloaded delay */
    TIM2->ARR = myTIM2_PERIOD;

    /* Update timer registers */
    // Relevant register: TIM2->EGR

    TIM2->EGR |= 0x0001;

    /* Assign TIM2 interrupt priority = 0 in NVIC */
    // Relevant register: NVIC->IP[3], or use NVIC_SetPriority

    NVIC_SetPriority(TIM2_IRQn, 0);

    /* Enable TIM2 interrupts in NVIC */
    // Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ

    NVIC_EnableIRQ(TIM2_IRQn);

    /* Enable update interrupt generation */
    // Relevant register: TIM2->DIER

    TIM2->DIER |= TIM_DIER_UIE;
    //TIM2->CR1 |= TIM_CR1_CEN;
}


void myTIM3_Init(void)
{

	/* Enable clock for TIM3 peripheral */

    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;


	/* Configure TIM3: buffer auto­reload, count up, stop on overflow,

	 * enable update events, interrupt on overflow only */

    TIM2->CR1 = ((uint16_t) 0x008C);


	/* Set clock prescaler value: 48MHz/(47999+1) = 1 KHz */
    TIM3->PSC = myTIM3_PRESCALER;

	/* Default auto­reloaded delay: 100 ms */
    TIM3->ARR = 100;

	/* Update timer registers */
    TIM3->EGR |= 0x0001;


}



void myEXTI_Init()
{

	/*/* Enable GPIOA clock
    RCC->AHBENR |= 0x00020000;

	// Clear PA1 field = IN mode
    GPIOA->MODER |= 0x28000000;

	//Clear PA1 field = NO pull­up/down
    GPIOA->PUPDR |= 0x24000000;
    */
    /* Map EXTI1 line to PA1 */
    // Relevant register: SYSCFG->EXTICR[0]
    SYSCFG->EXTICR[0] &=  ~(SYSCFG_EXTICR1_EXTI1); //set EXTI0[3:0] to 0000

    /* EXTI1 l
     * define interrupts: set rising-edge trigger */
    // Relevant register: EXTI->RTSR
    EXTI->RTSR |= EXTI_RTSR_TR1;

    /* Unmask interrupts from EXTI1 line */
    // Relevant register: EXTI->IMR
    EXTI->IMR |= EXTI_IMR_MR1;

    /* Assign EXTI1 interrupt priority = 0 in NVIC */
    // Relevant register: NVIC->IP[1], or use NVIC_SetPriority
    NVIC_SetPriority(EXTI0_1_IRQn,0);

    /* Enable EXTI1 interrupts in NVIC */
    // Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
    NVIC_EnableIRQ(EXTI0_1_IRQn);
}



void myADC_Init()
{
    //make pc1 input and analog
    //check ADC -> SR for the EOC bit, if 1 then read the data, otherwise do nothing
    RCC->AHBENR |= 0x00080000;    //enable clock on port C

    GPIOC->MODER |= 0x000000c0; //set mode to analog input
	GPIOC->PUPDR |= 0x24000000;

	RCC->APB2ENR |=0x00000200; //enable ADC clock



	ADC1->CFGR1 &=~(0x00000020); //continuous
	ADC1->CFGR1 |=0x00002000;

	ADC1->SMPR |= 0x00000007; //sample length

	ADC1->CHSELR |=0x00000800; //mapped to port c1

	ADC1->CR |= 0x80000000; //12 bit resolution
	while((ADC1->CR & 0x80000000) !=0);
	ADC1->CR |=0x00000001;
	while((ADC1->ISR & 0x00000001) ==0);
}

void myDAC_Init()
{

    RCC->APB1ENR|= 0x20000000; // Enable DAC clock

    GPIOC->MODER|= 0x00000300;

    DAC->CR &= 0xfffffcff;

    DAC->CR|=0x00000000; // set bit

    DAC->CR|=0x00000001; //enable bit to channel 1


}

void mySPI_Init()
{

	//Clock enable
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;


    GPIO_InitTypeDef GPIO_InitStruct;

    // RCC config
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB,ENABLE);

    // GPIO Alternate Function config
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_0);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_0);

    // Configure PB3 and PB5 alternate function
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_5;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Configure PB4 in output mode to be used as storage clock input in 74HC595
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStruct);



	/* Configure pins required for SPI peripheral */
	/* Enable clock for GPIOB peripheral */
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

	/* Configure PB3 as alternate function [SCK]*/
	GPIOB->MODER |= GPIO_MODER_MODER3_1;

	/* Select alternate function for PB3  */
	/* Set AFRy[3:0] to 0 to select AF0 for PB3 */
	GPIOB->AFR[0] |= GPIO_AFRL_AFR0;

	/* Configure PB4 as output [LCK] */
	GPIOB->MODER |= GPIO_MODER_MODER4_0;

	/* Ensure no pull-up/pull-down for PB4 */
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR4);

	/* Configure PB5 as alternate function [MOSI]*/
	GPIOB->MODER |= GPIO_MODER_MODER5_1;

	/* Select alternate function for PB5 */
	GPIOB->AFR[0] |= GPIO_AFRL_AFR0;




    /*Create SPI Struct*/
    SPI_InitTypeDef SPI_InitStructInfo;

   /*Initialize our SPI Struct with the struct Info*/
    SPI_InitTypeDef* SPI_InitStruct = &SPI_InitStructInfo;

    //FROM DALERS SLIDES 21
    /*Specifies the SPI unidirectional or bidirectional data mode*/
    SPI_InitStruct->SPI_Direction= SPI_Direction_1Line_Tx;

    /*Turn Master mode on*/
    SPI_InitStruct->SPI_Mode= SPI_Mode_Master;

    /*Set Data size*/
    SPI_InitStruct->SPI_DataSize = SPI_DataSize_8b;

    /*Initialize serial clock steady state*/
    SPI_InitStruct->SPI_CPOL = SPI_CPOL_Low;

    /*Initialize the clock active edge for the bit capture*/
    SPI_InitStruct->SPI_CPHA = SPI_CPHA_1Edge;

    /*Initialize the NSS signal to be measured by hardware*/
    SPI_InitStruct->SPI_NSS = SPI_NSS_Soft;

    /*Specifies the Baud Rate prescaler value which will be
    used to configure the transmit and receive SCK clock.*/
    SPI_InitStruct->SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;

    /*Specifies the data transfer to start from the MSB*/
    SPI_InitStruct ->SPI_FirstBit= SPI_FirstBit_MSB;

    /*Specify the polynomial used for calculation*/
    SPI_InitStruct ->SPI_CRCPolynomial = 7;

    //Initialize SPI1 with the SPI_InitStruct
    SPI_Init(SPI1, SPI_InitStruct);

    //Enable SPI1
    SPI_Cmd(SPI1, ENABLE);
}


/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void TIM2_IRQHandler()
{
    /* Check if update interrupt flag is indeed set */
    if ((TIM2->SR & TIM_SR_UIF) != 0)
    {
   	 trace_printf("\n*** Overflow! ***\n");

   	 /* Clear update interrupt flag */
   	 // Relevant register: TIM2->SR

   	 TIM2->SR &= ~(TIM_SR_UIF);

   	 /* Restart stopped timer */
   	 // Relevant register: TIM2->CR1

   	 TIM2->CR1 |= TIM_CR1_CEN;
    }
}


/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void EXTI0_1_IRQHandler()
{
    // Your local variables...
	double sigFrequency = 0;

    /* Check if EXTI1 interrupt pending flag is indeed set */
    if ((EXTI->PR & EXTI_PR_PR1) != 0)
    {
   	 //
   	 // 1. If this is the first edge:
   	 //    - Clear count register (TIM2->CNT).
   	 //    - Start timer (TIM2->CR1).
   	 //	Else (this is the second edge):
   	 //    - Stop timer (TIM2->CR1).
   	 //    - Read out count register (TIM2->CNT).
   	 //    - Calculate signal period and frequency.
   	 //    - Print calculated values to the console.
   	 //      NOTE: Function trace_printf does not work
   	 //      with floating-point numbers: you must use
   	 //      "unsigned int" type to print your signal
   	 //      period and frequency.
   	 //
   	 // 2. Clear EXTI1 interrupt pending flag (EXTI->PR).
   	 //
    	/* Stop timer */
    	TIM2->CR1 &= ~(TIM_CR1_CEN);
    	if(timerActive == 0 ){
    		timerPulses = 0;
    		timerActive = 1;
    		TIM2->CNT = 0x00000000;
    		TIM2->CR1 |= TIM_CR1_CEN;

    	} else {
    	    EXTI->IMR &= ~(EXTI_IMR_MR1);  //Mask EXTI1 interrupts
    		TIM2->CR1 &= ~(TIM_CR1_CEN);   //stop the timer
    		timerPulses = TIM2->CNT;
    		timerActive = 0;
    		sigFrequency = ((double)SystemCoreClock)/((double)timerPulses);
    		frequency = (unsigned int)sigFrequency;


 			//	  NOTE: Function trace_printf does not work
 			//	  with floating-point numbers: you must use
 			//	  "unsigned int" type to print your signal
 			//	  period and frequency.

    		EXTI->IMR |= EXTI_IMR_MR1; 		//Unmask EXTI interrupts
    		//trace_printf("Frequency: %f (Hz)\n\n",  frequency);
    	}
    }

    EXTI->PR |= (EXTI_PR_PR1); /*Clear interrupt flag*/
}





#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
