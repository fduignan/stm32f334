/* User LED on PF0 and button on PF1 */
/* This program sends & receives data over USART2 on PA2 (pin 8) and PA3 (pin 9)*/


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
char str[20];
int main()
{
    delay(1000000);
    initClocks();
    initUART(9600);
    RCC->AHBENR |= (1 << 22); // turn on the clock for GPIOF
    GPIOF->MODER |= (1 << 0); // Make bit 0 an output
    GPIOF->MODER &= ~(1 << 1);
    GPIOF->MODER &= ~(BIT2+BIT3); // Make bit 1 an input
    while(1)
    {
        eputs("Enter a string:");
        egets(str,10);
        eputs("\r\nYou entered :");
        eputs(str);
        eputs("\r\n");
        GPIOF->ODR |= (1 << 0); // bit high
        delay(1000000);
        GPIOF->ODR &= ~(1 << 0); // bit low
        delay(1000000);
       
    }
}
    
