void init(void);
void Default_Handler(void);
void Systick_Handler(void);
int main(void);

// The following are 'declared' in the linker script
extern unsigned char  INIT_DATA_VALUES;
extern unsigned char  INIT_DATA_START;
extern unsigned char  INIT_DATA_END;
extern unsigned char  BSS_START;
extern unsigned char  BSS_END;
// the section "vectors" is placed at the beginning of flash
// by the linker script
const void * Vectors[] __attribute__((section(".vectors"))) = {
  (void *)0x20003000,   /* Top of stack (12k) */
  init,             /* Reset Handler 0004*/
  Default_Handler,  /* NMI 0008 */
  Default_Handler,  /* Hard Fault 000C */
  Default_Handler,  /* MemManage 0010*/
  Default_Handler,  /* Prefetch fail, memory access fault  0014 */
  Default_Handler,  /* Undefined instruction or illegal state 0018 */
  Default_Handler,  /* Reserved 001C */
  Default_Handler,  /* Reserved 0020 */
  Default_Handler,  /* Reserved 0024 */
  Default_Handler,  /* Reserved 0028 */
  Default_Handler,  /* SVCall 002C */
  Default_Handler,  /* Reserved 0030 */
  Default_Handler,  /* Reserved 0034*/
  Default_Handler,  /* PendSV 0038 */
  Systick_Handler,  /* SysTick 003C */
  /* External interrupt handlers follow */
  Default_Handler,  /* 0: WWDG */
  Default_Handler,  /* 1: PVD */
  Default_Handler,  /* 2: Tamper */
  Default_Handler,  /* 3: RTC */
  Default_Handler,  /* 4: Flash */
  Default_Handler,  /* 5: RCC */
  Default_Handler,  /* 6: EXTI Line 0 */
  Default_Handler,  /* 7: EXTI Line 1 */
  Default_Handler,  /* 8: EXTI Line 2 */
  Default_Handler,  /* 9: EXTI Line 3 */
  Default_Handler,  /* 10: EXTI Line 4 */
  Default_Handler,  /* 11: DMA1 Channel 1 */
  Default_Handler,  /* 12: DMA1 Channel 2 */
  Default_Handler,  /* 13: DMA1 Channel 3 */
  Default_Handler,  /* 14: DMA1 Channel 4 */
  Default_Handler,  /* 15: DMA1 Channel 5 */
  Default_Handler,  /* 16: DMA1 Channel 6 */
  Default_Handler,  /* 17: DMA1 Channel 7 */
  Default_Handler,  /* 18: ADC1 & ADC   */
  Default_Handler,  /* 19: CAN1 TX interrupt */
  Default_Handler,  /* 20: CAN1 RX0 interrupt */
  Default_Handler,  /* 21: CAN1 RX1 interrupt */
  Default_Handler,  /* 22: CAN1 SCE interrupt */
  Default_Handler,  /* 23: EXTI Line[9:5] interrupts */
  Default_Handler,  /* 24: TIM1 Break interrupt and TIM15 global*/
  Default_Handler,  /* 25: TIM1 Update interrupt and and TIM16 global*/
  Default_Handler,  /* 26: TIM1 Trigger and Commutation/TIM17 interrupts */
  Default_Handler,  /* 27: TIM1 Capture Compare interrupt */
  Default_Handler,  /* 28: TIM2 global interrupt */
  Default_Handler,  /* 29: TIM3 global interrupt */
  Default_Handler,  /* 30: Reserved */
  Default_Handler,  /* 31: I2C1 event & EXTI Line 32 interrupt*/
  Default_Handler,  /* 32: I2C1 error interrupt */
  Default_Handler,  /* 33: Reserved */
  Default_Handler,  /* 34: Reserved */
  Default_Handler,  /* 35: SPI1 global interrupt */
  Default_Handler,  /* 36: Reserved */
  Default_Handler,  /* 37: USART1 global & EXTI Line 25 interrupt */
  Default_Handler,  /* 38: USART2 global & EXTI Line 26 interrupt */
  Default_Handler,  /* 39: USART3 global & EXTI Line 28 interrupt */
  Default_Handler,  /* 40: EXTI Line[15:10] interrupts */
  Default_Handler,  /* 41: RTC Alarm interrupt */
  Default_Handler,  /* 42: Reserved */
  Default_Handler,  /* 43: Reserved */
  Default_Handler,  /* 44: Reserved */
  Default_Handler,  /* 45: Reserved */
  Default_Handler,  /* 46: Reserved */
  Default_Handler,  /* 47: Reserved */
  Default_Handler,  /* 48: Reserved */
  Default_Handler,  /* 49: Reserved */
  Default_Handler,  /* 50: Reserved */
  Default_Handler,  /* 51: Reserved */
  Default_Handler,  /* 52: Reserved */
  Default_Handler,  /* 53: Reserved */
  Default_Handler,  /* 54: TIM6 global and DAC1 underrun error interrupt */
  Default_Handler,  /* 55: TIM7 global and DAC2 underrun errot interrupt */
  Default_Handler,  /* 56: Reserved */
  Default_Handler,  /* 57: Reserved */
  Default_Handler,  /* 58: Reserved */
  Default_Handler,  /* 59: Reserved */
  Default_Handler,  /* 60: Reserved */
  Default_Handler,  /* 61: Reserved */
  Default_Handler,  /* 62: Reserved */
  Default_Handler,  /* 63: Reserved*/
  Default_Handler,  /* 64: COMP2 & EXTI Line 22 interrupt */
  Default_Handler,  /* 65: COMP2, COMP6 & EXTI lines 30 & 32 interrupt */
  Default_Handler,  /* 66: Reserved */
  Default_Handler,  /* 67: HRTIM master timer interrupt */
  Default_Handler,  /* 68: HRTIM timer A interrupt  */
  Default_Handler,  /* 69: HRTIM timer B interrupt  */
  Default_Handler,  /* 70: HRTIM timer C interrupt  */
  Default_Handler,  /* 71: HRTIM timer D interrupt  */
  Default_Handler,  /* 72: HRTIM timer E interrupt  */
  Default_Handler,  /* 73: HRTIM fault interrupt  */
  Default_Handler,  /* 74: Reserved */
  Default_Handler,  /* 75: Reserved */
  Default_Handler,  /* 76: Reserved */
  Default_Handler,  /* 77: Reserved */
  Default_Handler,  /* 78: Reserved */
  Default_Handler,  /* 79: Reserved */
  Default_Handler,  /* 80: Reserved */
  Default_Handler,  /* 81: FPU global interrupt  */

};
void init()
{
  // do global/static data initialization
  unsigned char *src;
  unsigned char *dest;
  unsigned len;
  src = &INIT_DATA_VALUES;
  dest = &INIT_DATA_START;
  len = &INIT_DATA_END - &INIT_DATA_START;
  while (len--)
    *dest++ = *src++;
  // zero out the uninitialized global/static variables
  dest = &BSS_START;
  len = &BSS_END - &BSS_START;
  while (len--)
    *dest++ = 0;
  main();
  while (1);
}

void Default_Handler()
{
  while (1);
}
