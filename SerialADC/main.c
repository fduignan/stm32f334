/* User LED on PF0 and button PF1 */
// This program outputs ADC conversions for ADC1 Channel1 and Channel 2 (pins 6 and 7 resp)
// Data is output over the serial port at 9600 bps
#include <stdint.h>
#include "../include/cortexm4.h"
#include "../include/STM32F3x4.h"
#include "serial.h"
void delay(uint32_t dly)
{
    while(dly--);
}
void initClocks()
{
    // Bump the clock speed up to 64MHz
    // Assumption: Using HSI clock (8MHz)
	// PLL is driven by HSI/2 = 4MHz
	// Multiply x 16  = 64MHz
	
	// Must set flash latency (2 wait states) to allow
	// for higher speed clock	
	Flash->ACR |= 2; // add two wait states to flash
	
	RCC->CR &= ~BIT24; // turn off PLL
	delay(10);	
    while(RCC->CFGR & (BIT25) ); // wait for PLL to be ready
	RCC->CFGR |= ((0xf) << 18);	// PLL multiply = 16	
	RCC->CFGR |= BIT10;  // APB1 Low speed Clock = HCLK/2 (32MHz)	
	delay(10);
	RCC->CR |= BIT24; // turn on PLL
	delay(10);
	RCC->CFGR |= BIT1;  // switch to PLL as clock

}
void initADC()
{
    // Enable 2 channels of ADC1 : Ch1 and Ch2 on pins PA0 (pin 7), PA1 (pin 8)
    RCC->AHBENR |= (BIT28); // enabled the ADCs
    GPIOA->MODER |= (BIT0 + BIT1 + BIT2 + BIT3); // GPIOA bits 0 and 1 in analog mode 
    ADC_Common->ADC1_CCR |= BIT17; // set ADC Clock = HCLK / 2
    ADC_Common->ADC1_CCR &= ~BIT16;
    
    
    ADC1->CR &= ~(BIT29+BIT28); // enable the ADC voltage regulator
    ADC1->CR |= (BIT28);
    
    //ADC_Common->ADC1_CCR |= BIT22; // enable the voltage reference
    delay(1000); // wait at least 10us for the voltage regulator to stabilize
    ADC1->CR &= ~(BIT30); // single ended mode
    ADC1->CR |= BIT31; // start calibration
    while((ADC1->CR & BIT31)!=0); // wait for calibration to complete
    
    
    ADC1->CR |= BIT0; 
    while( (ADC1->ISR & BIT0)==0); // wait for ADC to become ready
  
    
}
uint32_t readADC(uint32_t ChannelNumber)
{
    ADC1->SQR1 = (ChannelNumber << 6); // single channel conversion on specified channel
    ADC1->CR |= BIT2; // start conversions
    while((ADC1->CR & BIT2) != 0); // wait for conversion to complete
    return ADC1->DR;
    
}
void initUI()
{
    // The "user interface" consists of an LED and a button attached
    // to Port F bits 0 and 1 resp.
    RCC->AHBENR |= (1 << 22); // turn on the clock for GPIOA
    GPIOF->MODER |= (1 << 0); // Make bit 13 an output
    GPIOF->MODER &= ~(1 << 1);
    GPIOF->MODER &= ~(1 << 2);
    GPIOF->MODER &= ~(1 << 3);
}
int main()
{
    delay(1000000);

    
    while ((GPIOF->IDR & (1 << 1)) != 0); // wait for a button press
    initClocks();
    initUI();
    initUART(9600);
    initADC();
    while(1)
    {
        eputs("CH1 : ");
        eputShort(readADC(1));
        eputs(" CH2 : ");
        eputShort(readADC(2));
        eputs("\r\n");
        GPIOF->ODR |= (1 << 0); // bit high
        delay(1000000);
        GPIOF->ODR &= ~(1 << 0); // bit low
        delay(1000000);
       
    }
}
    
