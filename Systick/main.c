/* User LED on PF0 and Button on PF1 */
// This program makes use of the systick interrupt.  It configures the
// MCU to run at 64MHz and the systick timer to generate an IRQ every 
// millisecond.  After 1000 interrupts (a second) the LED is toggled.
#include <stdint.h>
#include "../include/cortexm4.h"
#include "../include/STM32F3x4.h"
uint32_t systick_counter=0;
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
void initSystick()
{	
    
    SysTick->CTRL |= ( BIT2 | BIT1 | BIT0); // enable systick, source = cpu clock, enable interrupt
// SysTick clock source = 64MHz.  Divide this down to create 1 millisecond period
	SysTick->LOAD = (64000);   
	SysTick->VAL = 10; // don't want long wait for counter to count down from initial high unknown value
}
void Systick_Handler()
{
    systick_counter++;
    if (systick_counter >= 1000) // 1 second passed?
    {
        systick_counter = 0;
        GPIOF->ODR ^= BIT0; // Toggle the LED
    }
}
int main()
{
    delay(1000000); // precautionary delay in case clock configuration locks up the MCU
    initClocks();
    initSystick();
    enable_interrupts();
    RCC->AHBENR |= (1 << 22); // turn on the clock for GPIOF
    GPIOF->MODER |= (1 << 0); // Make bit 0 an output
    GPIOF->MODER &= ~(1 << 1);
    GPIOF->MODER &= ~(BIT2+BIT3); // Make bit 1 an input
    while(1)
    {
        
    }
}
    
