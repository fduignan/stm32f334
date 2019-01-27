/* User LED on PF0 and, button PF1 */
/* Blinky with a button */
// This program flashes an LED attached on GPIOF bit 0 after the 
// user presses the button attached to GPIOF bit 1.
// It also bumps the MCU speed up to 64MHz using the PLC

#include <stdint.h>
#include "../include/cortexm4.h"
#include "../include/STM32F3x4.h"
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
	
	// Must set flash latency to 2 wait states to allow
	// for higher speed clock.  Prefetch is enabled on reset
	Flash->ACR |= 2; // add two wait states to flash
	
	RCC->CR &= ~BIT24; // turn off PLL
    while(RCC->CFGR & (BIT25) ); // wait for PLL to be ready for configuration
	RCC->CFGR |= ((0xf) << 18);	// PLL multiply = 16	
	RCC->CFGR |= BIT10;  // APB1 Low speed Clock = HCLK/2 (32MHz)	
	RCC->CR |= BIT24; // turn on PLL
	RCC->CFGR |= BIT1;  // switch to PLL as system clock

}

int main()
{
    delay(1000000); // This is here to allow the system to be rescued in the event that clock config locks it up.
    initClocks();
    RCC->AHBENR |= (1 << 22); // turn on the clock for GPIOF
    GPIOF->MODER |= (1 << 0); // Make bit 0 an output
    GPIOF->MODER &= ~(1 << 1);
    GPIOF->MODER &= ~(BIT2+BIT3); // Make bit 1 an input
    
    while ((GPIOF->IDR & (1 << 1)) != 0); // wait for a button press
    
    while(1)
    {
        GPIOF->ODR |= (1 << 0); // bit high
        delay(1000000);
        GPIOF->ODR &= ~(1 << 0); // bit low
        delay(1000000);
    }
}
    
