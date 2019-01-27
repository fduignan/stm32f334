
/****************************************************************************************************//**
 * @file     STM32F3x4.h
 *
 * @brief    CMSIS Cortex-M4 Peripheral Access Layer Header File for
 *           STM32F3x4 from <unknown Vendor>.
 *
 * @version  V1.4
 * @date     23. January 2019
 *
 * @note     Generated with SVDConv V2.75 
 *           from CMSIS SVD File 'STM32F3x4.svd' Version 1.4,
 *******************************************************************************************************/



/** @addtogroup (null)
  * @{
  */

/** @addtogroup STM32F3x4
  * @{
  */

#ifndef STM32F3X4_H
#define STM32F3X4_H

#ifdef __cplusplus
extern "C" {
#endif


/* -------------------------  Interrupt Number Definition  ------------------------ */

typedef enum {
/* -------------------  Cortex-M4 Processor Exceptions Numbers  ------------------- */
  Reset_IRQn                    = -15,              /*!<   1  Reset Vector, invoked on Power up and warm reset                 */
  NonMaskableInt_IRQn           = -14,              /*!<   2  Non maskable Interrupt, cannot be stopped or preempted           */
  HardFault_IRQn                = -13,              /*!<   3  Hard Fault, all classes of Fault                                 */
  MemoryManagement_IRQn         = -12,              /*!<   4  Memory Management, MPU mismatch, including Access Violation
                                                         and No Match                                                          */
  BusFault_IRQn                 = -11,              /*!<   5  Bus Fault, Pre-Fetch-, Memory Access Fault, other address/memory
                                                         related Fault                                                         */
  UsageFault_IRQn               = -10,              /*!<   6  Usage Fault, i.e. Undef Instruction, Illegal State Transition    */
  SVCall_IRQn                   =  -5,              /*!<  11  System Service Call via SVC instruction                          */
  DebugMonitor_IRQn             =  -4,              /*!<  12  Debug Monitor                                                    */
  PendSV_IRQn                   =  -2,              /*!<  14  Pendable request for system service                              */
  SysTick_IRQn                  =  -1,              /*!<  15  System Tick Timer                                                */
/* --------------------  STM32F3x4 Specific Interrupt Numbers  -------------------- */
  WWDG_IRQn                     =   0,              /*!<   0  Window Watchdog interrupt                                        */
  PVD_IRQn                      =   1,              /*!<   1  PVD through EXTI line detection interrupt                        */
  TAMP_STAMP_IRQn               =   2,              /*!<   2  Tamper and TimeStamp interrupts                                  */
  RTC_WKUP_IRQn                 =   3,              /*!<   3  RTC Wakeup interrupt through the EXTI line                       */
  FLASH_IRQn                    =   4,              /*!<   4  Flash global interrupt                                           */
  RCC_IRQn                      =   5,              /*!<   5  RCC global interrupt                                             */
  EXTI0_IRQn                    =   6,              /*!<   6  EXTI Line0 interrupt                                             */
  EXTI1_IRQn                    =   7,              /*!<   7  EXTI Line3 interrupt                                             */
  EXTI2_TSC_IRQn                =   8,              /*!<   8  EXTI Line2 and Touch sensing interrupts                          */
  EXTI3_IRQn                    =   9,              /*!<   9  EXTI Line3 interrupt                                             */
  EXTI4_IRQn                    =  10,              /*!<  10  EXTI Line4 interrupt                                             */
  DMA1_CH1_IRQn                 =  11,              /*!<  11  DMA1 channel 1 interrupt                                         */
  DMA1_CH2_IRQn                 =  12,              /*!<  12  DMA1 channel 2 interrupt                                         */
  DMA1_CH3_IRQn                 =  13,              /*!<  13  DMA1 channel 3 interrupt                                         */
  DMA1_CH4_IRQn                 =  14,              /*!<  14  DMA1 channel 4 interrupt                                         */
  DMA1_CH5_IRQn                 =  15,              /*!<  15  DMA1 channel 5 interrupt                                         */
  DMA1_CH6_IRQn                 =  16,              /*!<  16  DMA1 channel 6 interrupt                                         */
  DMA1_CH7_IRQn                 =  17,              /*!<  17  DMA1 channel 7interrupt                                          */
  ADC1_2_IRQn                   =  18,              /*!<  18  ADC1 and ADC2 global interrupt                                   */
  USB_HP_CAN_TX_IRQn            =  19,              /*!<  19  USB High Priority/CAN_TX interrupts                              */
  USB_LP_CAN_RX0_IRQn           =  20,              /*!<  20  USB Low Priority/CAN_RX0 interrupts                              */
  CAN_RX1_IRQn                  =  21,              /*!<  21  CAN_RX1 interrupt                                                */
  CAN_SCE_IRQn                  =  22,              /*!<  22  CAN_SCE interrupt                                                */
  EXTI9_5_IRQn                  =  23,              /*!<  23  EXTI Line5 to Line9 interrupts                                   */
  TIM1_BRK_TIM15_IRQn           =  24,              /*!<  24  TIM1 Break/TIM15 global interruts                                */
  TIM1_UP_TIM16_IRQn            =  25,              /*!<  25  TIM1 Update/TIM16 global interrupts                              */
  TIM1_TRG_COM_TIM17_IRQn       =  26,              /*!<  26  TIM1 trigger and commutation/TIM17 interrupts                    */
  TIM1_CC_IRQn                  =  27,              /*!<  27  TIM1 capture compare interrupt                                   */
  TIM2_IRQn                     =  28,              /*!<  28  TIM2 global interrupt                                            */
  TIM3_IRQ_IRQn                 =  29,              /*!<  29  Timer 3 global interrupt                                         */
  I2C1_EV_EXTI23_IRQn           =  31,              /*!<  31  I2C1 event interrupt and EXTI Line23 interrupt                   */
  I2C1_ER_IRQn                  =  32,              /*!<  32  I2C1 error interrupt                                             */
  SPI1_IRQn                     =  35,              /*!<  35  SPI1 global interrupt                                            */
  USART1_EXTI25_IRQn            =  37,              /*!<  37  USART1 global interrupt and EXTI Line 25 interrupt               */
  USART2_EXTI26_IRQn            =  38,              /*!<  38  USART2 global interrupt and EXTI Line 26 interrupt               */
  USART3_EXTI28_IRQn            =  39,              /*!<  39  USART3 global interrupt and EXTI Line 28 interrupt               */
  EXTI15_10_IRQn                =  40,              /*!<  40  EXTI Line15 to Line10 interrupts                                 */
  RTCAlarm_IRQn                 =  41,              /*!<  41  RTC alarm interrupt                                              */
  TIM6_DAC1_IRQn                =  54,              /*!<  54  TIM6 global and DAC12 underrun interrupts                        */
  TIM7_DAC2_IRQn                =  55,              /*!<  55  TIM7 global interrupt                                            */
  FPU_IRQn                      =  81,              /*!<  81  Floating point unit interrupt                                    */
  HRTIM1_MST_IRQn               = 103,              /*!< 103  HRTIM1 master timer interrupt                                    */
  HRTIM1_TIMA_IRQn              = 104,              /*!< 104  HRTIM1 timer A interrupt                                         */
  HRTIM_TIMB_IRQn               = 105,              /*!< 105  HRTIM1 timer B interrupt                                         */
  HRTIM1_TIMC_IRQn              = 106,              /*!< 106  HRTIM1 timer C interrupt                                         */
  HRTIM1_TIMD_IRQn              = 107,              /*!< 107  HRTIM1 timer D interrupt                                         */
  HRTIM_TIME_IRQn               = 108,              /*!< 108  HRTIM1 timer E interrupt                                         */
  HRTIM1_FLT_IRQn               = 109               /*!< 109  HRTIM1 fault interrupt                                           */
} IRQn_Type;


/** @addtogroup Configuration_of_CMSIS
  * @{
  */


/* ================================================================================ */
/* ================      Processor and Core Peripheral Section     ================ */
/* ================================================================================ */

/* ----------------Configuration of the Cortex-M4 Processor and Core Peripherals---------------- */
#define __CM4_REV                 0x0100            /*!< Cortex-M4 Core Revision                                               */
#define __MPU_PRESENT                  0            /*!< MPU present or not                                                    */
#define __NVIC_PRIO_BITS               3            /*!< Number of Bits used for Priority Levels                               */
#define __Vendor_SysTickConfig         0            /*!< Set to 1 if different SysTick Config is used                          */
#define __FPU_PRESENT                  0            /*!< FPU present or not                                                    */
/** @} */ /* End of group Configuration_of_CMSIS */

#include <core_cm4.h>                               /*!< Cortex-M4 processor and core peripherals                              */
#include "system_STM32F3x4.h"                       /*!< STM32F3x4 System                                                      */


/* ================================================================================ */
/* ================       Device Specific Peripheral Section       ================ */
/* ================================================================================ */


/** @addtogroup Device_Peripheral_Registers
  * @{
  */


/* -------------------  Start of section using anonymous unions  ------------------ */
#if defined(__CC_ARM)
  #pragma push
  #pragma anon_unions
#elif defined(__ICCARM__)
  #pragma language=extended
#elif defined(__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined(__TMS470__)
/* anonymous unions are enabled by default */
#elif defined(__TASKING__)
  #pragma warning 586
#else
  #warning Not supported compiler type
#endif



/* ================================================================================ */
/* ================                      GPIOA                     ================ */
/* ================================================================================ */


/**
  * @brief General-purpose I/Os (GPIOA)
  */

typedef struct {                                    /*!< GPIOA Structure                                                       */
  __IO uint32_t  MODER;                             /*!< GPIO port mode register                                               */
  __IO uint32_t  OTYPER;                            /*!< GPIO port output type register                                        */
  __IO uint32_t  OSPEEDR;                           /*!< GPIO port output speed register                                       */
  __IO uint32_t  PUPDR;                             /*!< GPIO port pull-up/pull-down register                                  */
  __I  uint32_t  IDR;                               /*!< GPIO port input data register                                         */
  __IO uint32_t  ODR;                               /*!< GPIO port output data register                                        */
  __O  uint32_t  BSRR;                              /*!< GPIO port bit set/reset register                                      */
  __IO uint32_t  LCKR;                              /*!< GPIO port configuration lock register                                 */
  __IO uint32_t  AFRL;                              /*!< GPIO alternate function low register                                  */
  __IO uint32_t  AFRH;                              /*!< GPIO alternate function high register                                 */
  __O  uint32_t  BRR;                               /*!< Port bit reset register                                               */
} GPIOA_Type;


/* ================================================================================ */
/* ================                      GPIOB                     ================ */
/* ================================================================================ */


/**
  * @brief General-purpose I/Os (GPIOB)
  */

typedef struct {                                    /*!< GPIOB Structure                                                       */
  __IO uint32_t  MODER;                             /*!< GPIO port mode register                                               */
  __IO uint32_t  OTYPER;                            /*!< GPIO port output type register                                        */
  __IO uint32_t  OSPEEDR;                           /*!< GPIO port output speed register                                       */
  __IO uint32_t  PUPDR;                             /*!< GPIO port pull-up/pull-down register                                  */
  __I  uint32_t  IDR;                               /*!< GPIO port input data register                                         */
  __IO uint32_t  ODR;                               /*!< GPIO port output data register                                        */
  __O  uint32_t  BSRR;                              /*!< GPIO port bit set/reset register                                      */
  __IO uint32_t  LCKR;                              /*!< GPIO port configuration lock register                                 */
  __IO uint32_t  AFRL;                              /*!< GPIO alternate function low register                                  */
  __IO uint32_t  AFRH;                              /*!< GPIO alternate function high register                                 */
  __O  uint32_t  BRR;                               /*!< Port bit reset register                                               */
} GPIOB_Type;


/* ================================================================================ */
/* ================                      GPIOC                     ================ */
/* ================================================================================ */


/**
  * @brief General-purpose I/Os (GPIOC)
  */

typedef struct {                                    /*!< GPIOC Structure                                                       */
  __IO uint32_t  MODER;                             /*!< GPIO port mode register                                               */
  __IO uint32_t  OTYPER;                            /*!< GPIO port output type register                                        */
  __IO uint32_t  OSPEEDR;                           /*!< GPIO port output speed register                                       */
  __IO uint32_t  PUPDR;                             /*!< GPIO port pull-up/pull-down register                                  */
  __I  uint32_t  IDR;                               /*!< GPIO port input data register                                         */
  __IO uint32_t  ODR;                               /*!< GPIO port output data register                                        */
  __O  uint32_t  BSRR;                              /*!< GPIO port bit set/reset register                                      */
  __IO uint32_t  LCKR;                              /*!< GPIO port configuration lock register                                 */
  __IO uint32_t  AFRL;                              /*!< GPIO alternate function low register                                  */
  __IO uint32_t  AFRH;                              /*!< GPIO alternate function high register                                 */
  __O  uint32_t  BRR;                               /*!< Port bit reset register                                               */
} GPIOC_Type;


/* ================================================================================ */
/* ================                       TSC                      ================ */
/* ================================================================================ */


/**
  * @brief Touch sensing controller (TSC)
  */

typedef struct {                                    /*!< TSC Structure                                                         */
  __IO uint32_t  CR;                                /*!< control register                                                      */
  __IO uint32_t  IER;                               /*!< interrupt enable register                                             */
  __IO uint32_t  ICR;                               /*!< interrupt clear register                                              */
  __IO uint32_t  ISR;                               /*!< interrupt status register                                             */
  __IO uint32_t  IOHCR;                             /*!< I/O hysteresis control register                                       */
  __I  uint32_t  RESERVED0;
  __IO uint32_t  IOASCR;                            /*!< I/O analog switch control register                                    */
  __I  uint32_t  RESERVED1;
  __IO uint32_t  IOSCR;                             /*!< I/O sampling control register                                         */
  __I  uint32_t  RESERVED2;
  __IO uint32_t  IOCCR;                             /*!< I/O channel control register                                          */
  __I  uint32_t  RESERVED3;
  __IO uint32_t  IOGCSR;                            /*!< I/O group control status register                                     */
  __I  uint32_t  IOG1CR;                            /*!< I/O group x counter register                                          */
  __I  uint32_t  IOG2CR;                            /*!< I/O group x counter register                                          */
  __I  uint32_t  IOG3CR;                            /*!< I/O group x counter register                                          */
  __I  uint32_t  IOG4CR;                            /*!< I/O group x counter register                                          */
  __I  uint32_t  IOG5CR;                            /*!< I/O group x counter register                                          */
  __I  uint32_t  IOG6CR;                            /*!< I/O group x counter register                                          */
  __I  uint32_t  IOG7CR;                            /*!< I/O group x counter register                                          */
  __I  uint32_t  IOG8CR;                            /*!< I/O group x counter register                                          */
} TSC_Type;


/* ================================================================================ */
/* ================                       CRC                      ================ */
/* ================================================================================ */


/**
  * @brief cyclic redundancy check calculation unit (CRC)
  */

typedef struct {                                    /*!< CRC Structure                                                         */
  __IO uint32_t  DR;                                /*!< Data register                                                         */
  __IO uint32_t  IDR;                               /*!< Independent data register                                             */
  __IO uint32_t  CR;                                /*!< Control register                                                      */
  __I  uint32_t  RESERVED0;
  __IO uint32_t  INIT;                              /*!< Initial CRC value                                                     */
  __IO uint32_t  POL;                               /*!< CRC polynomial                                                        */
} CRC_Type;


/* ================================================================================ */
/* ================                      Flash                     ================ */
/* ================================================================================ */


/**
  * @brief Flash (Flash)
  */

typedef struct {                                    /*!< Flash Structure                                                       */
  __IO uint32_t  ACR;                               /*!< Flash access control register                                         */
  __O  uint32_t  KEYR;                              /*!< Flash key register                                                    */
  __O  uint32_t  OPTKEYR;                           /*!< Flash option key register                                             */
  __IO uint32_t  SR;                                /*!< Flash status register                                                 */
  __IO uint32_t  CR;                                /*!< Flash control register                                                */
  __O  uint32_t  AR;                                /*!< Flash address register                                                */
  __I  uint32_t  RESERVED0;
  __I  uint32_t  OBR;                               /*!< Option byte register                                                  */
  __I  uint32_t  WRPR;                              /*!< Write protection register                                             */
} Flash_Type;


/* ================================================================================ */
/* ================                       RCC                      ================ */
/* ================================================================================ */


/**
  * @brief Reset and clock control (RCC)
  */

typedef struct {                                    /*!< RCC Structure                                                         */
  __IO uint32_t  CR;                                /*!< Clock control register                                                */
  __IO uint32_t  CFGR;                              /*!< Clock configuration register (RCC_CFGR)                               */
  __IO uint32_t  CIR;                               /*!< Clock interrupt register (RCC_CIR)                                    */
  __IO uint32_t  APB2RSTR;                          /*!< APB2 peripheral reset register (RCC_APB2RSTR)                         */
  __IO uint32_t  APB1RSTR;                          /*!< APB1 peripheral reset register (RCC_APB1RSTR)                         */
  __IO uint32_t  AHBENR;                            /*!< AHB Peripheral Clock enable register (RCC_AHBENR)                     */
  __IO uint32_t  APB2ENR;                           /*!< APB2 peripheral clock enable register (RCC_APB2ENR)                   */
  __IO uint32_t  APB1ENR;                           /*!< APB1 peripheral clock enable register (RCC_APB1ENR)                   */
  __IO uint32_t  BDCR;                              /*!< Backup domain control register (RCC_BDCR)                             */
  __IO uint32_t  CSR;                               /*!< Control/status register (RCC_CSR)                                     */
  __IO uint32_t  AHBRSTR;                           /*!< AHB peripheral reset register                                         */
  __IO uint32_t  CFGR2;                             /*!< Clock configuration register 2                                        */
  __IO uint32_t  CFGR3;                             /*!< Clock configuration register 3                                        */
} RCC_Type;


/* ================================================================================ */
/* ================                      DMA1                      ================ */
/* ================================================================================ */


/**
  * @brief DMA controller 1 (DMA1)
  */

typedef struct {                                    /*!< DMA1 Structure                                                        */
  __I  uint32_t  ISR;                               /*!< DMA interrupt status register (DMA_ISR)                               */
  __O  uint32_t  IFCR;                              /*!< DMA interrupt flag clear register (DMA_IFCR)                          */
  __IO uint32_t  CCR1;                              /*!< DMA channel configuration register (DMA_CCR)                          */
  __IO uint32_t  CNDTR1;                            /*!< DMA channel 1 number of data register                                 */
  __IO uint32_t  CPAR1;                             /*!< DMA channel 1 peripheral address register                             */
  __IO uint32_t  CMAR1;                             /*!< DMA channel 1 memory address register                                 */
  __I  uint32_t  RESERVED0;
  __IO uint32_t  CCR2;                              /*!< DMA channel configuration register (DMA_CCR)                          */
  __IO uint32_t  CNDTR2;                            /*!< DMA channel 2 number of data register                                 */
  __IO uint32_t  CPAR2;                             /*!< DMA channel 2 peripheral address register                             */
  __IO uint32_t  CMAR2;                             /*!< DMA channel 2 memory address register                                 */
  __I  uint32_t  RESERVED1;
  __IO uint32_t  CCR3;                              /*!< DMA channel configuration register (DMA_CCR)                          */
  __IO uint32_t  CNDTR3;                            /*!< DMA channel 3 number of data register                                 */
  __IO uint32_t  CPAR3;                             /*!< DMA channel 3 peripheral address register                             */
  __IO uint32_t  CMAR3;                             /*!< DMA channel 3 memory address register                                 */
  __I  uint32_t  RESERVED2;
  __IO uint32_t  CCR4;                              /*!< DMA channel configuration register (DMA_CCR)                          */
  __IO uint32_t  CNDTR4;                            /*!< DMA channel 4 number of data register                                 */
  __IO uint32_t  CPAR4;                             /*!< DMA channel 4 peripheral address register                             */
  __IO uint32_t  CMAR4;                             /*!< DMA channel 4 memory address register                                 */
  __I  uint32_t  RESERVED3;
  __IO uint32_t  CCR5;                              /*!< DMA channel configuration register (DMA_CCR)                          */
  __IO uint32_t  CNDTR5;                            /*!< DMA channel 5 number of data register                                 */
  __IO uint32_t  CPAR5;                             /*!< DMA channel 5 peripheral address register                             */
  __IO uint32_t  CMAR5;                             /*!< DMA channel 5 memory address register                                 */
  __I  uint32_t  RESERVED4;
  __IO uint32_t  CCR6;                              /*!< DMA channel configuration register (DMA_CCR)                          */
  __IO uint32_t  CNDTR6;                            /*!< DMA channel 6 number of data register                                 */
  __IO uint32_t  CPAR6;                             /*!< DMA channel 6 peripheral address register                             */
  __IO uint32_t  CMAR6;                             /*!< DMA channel 6 memory address register                                 */
  __I  uint32_t  RESERVED5;
  __IO uint32_t  CCR7;                              /*!< DMA channel configuration register (DMA_CCR)                          */
  __IO uint32_t  CNDTR7;                            /*!< DMA channel 7 number of data register                                 */
  __IO uint32_t  CPAR7;                             /*!< DMA channel 7 peripheral address register                             */
  __IO uint32_t  CMAR7;                             /*!< DMA channel 7 memory address register                                 */
} DMA1_Type;


/* ================================================================================ */
/* ================                      TIM2                      ================ */
/* ================================================================================ */


/**
  * @brief General purpose timer (TIM2)
  */

typedef struct {                                    /*!< TIM2 Structure                                                        */
  __IO uint32_t  CR1;                               /*!< control register 1                                                    */
  __IO uint32_t  CR2;                               /*!< control register 2                                                    */
  __IO uint32_t  SMCR;                              /*!< slave mode control register                                           */
  __IO uint32_t  DIER;                              /*!< DMA/Interrupt enable register                                         */
  __IO uint32_t  SR;                                /*!< status register                                                       */
  __O  uint32_t  EGR;                               /*!< event generation register                                             */
  
  union {
    __IO uint32_t  CCMR1_Input;                     /*!< capture/compare mode register 1 (input mode)                          */
    __IO uint32_t  CCMR1_Output;                    /*!< capture/compare mode register 1 (output mode)                         */
  } ;
  
  union {
    __IO uint32_t  CCMR2_Input;                     /*!< capture/compare mode register 2 (input mode)                          */
    __IO uint32_t  CCMR2_Output;                    /*!< capture/compare mode register 2 (output mode)                         */
  } ;
  __IO uint32_t  CCER;                              /*!< capture/compare enable register                                       */
  __IO uint32_t  CNT;                               /*!< counter                                                               */
  __IO uint32_t  PSC;                               /*!< prescaler                                                             */
  __IO uint32_t  ARR;                               /*!< auto-reload register                                                  */
  __I  uint32_t  RESERVED0;
  __IO uint32_t  CCR1;                              /*!< capture/compare register 1                                            */
  __IO uint32_t  CCR2;                              /*!< capture/compare register 2                                            */
  __IO uint32_t  CCR3;                              /*!< capture/compare register 3                                            */
  __IO uint32_t  CCR4;                              /*!< capture/compare register 4                                            */
  __I  uint32_t  RESERVED1;
  __IO uint32_t  DCR;                               /*!< DMA control register                                                  */
  __IO uint32_t  DMAR;                              /*!< DMA address for full transfer                                         */
} TIM2_Type;


/* ================================================================================ */
/* ================                      TIM15                     ================ */
/* ================================================================================ */


/**
  * @brief General purpose timers (TIM15)
  */

typedef struct {                                    /*!< TIM15 Structure                                                       */
  __IO uint32_t  CR1;                               /*!< control register 1                                                    */
  __IO uint32_t  CR2;                               /*!< control register 2                                                    */
  __IO uint32_t  SMCR;                              /*!< slave mode control register                                           */
  __IO uint32_t  DIER;                              /*!< DMA/Interrupt enable register                                         */
  __IO uint32_t  SR;                                /*!< status register                                                       */
  __O  uint32_t  EGR;                               /*!< event generation register                                             */
  
  union {
    __IO uint32_t  CCMR1_Input;                     /*!< capture/compare mode register 1 (input mode)                          */
    __IO uint32_t  CCMR1_Output;                    /*!< capture/compare mode register (output mode)                           */
  } ;
  __I  uint32_t  RESERVED0;
  __IO uint32_t  CCER;                              /*!< capture/compare enable register                                       */
  __IO uint32_t  CNT;                               /*!< counter                                                               */
  __IO uint32_t  PSC;                               /*!< prescaler                                                             */
  __IO uint32_t  ARR;                               /*!< auto-reload register                                                  */
  __IO uint32_t  RCR;                               /*!< repetition counter register                                           */
  __IO uint32_t  CCR1;                              /*!< capture/compare register 1                                            */
  __IO uint32_t  CCR2;                              /*!< capture/compare register 2                                            */
  __I  uint32_t  RESERVED1[2];
  __IO uint32_t  BDTR;                              /*!< break and dead-time register                                          */
  __IO uint32_t  DCR;                               /*!< DMA control register                                                  */
  __IO uint32_t  DMAR;                              /*!< DMA address for full transfer                                         */
} TIM15_Type;


/* ================================================================================ */
/* ================                      TIM16                     ================ */
/* ================================================================================ */


/**
  * @brief General-purpose-timers (TIM16)
  */

typedef struct {                                    /*!< TIM16 Structure                                                       */
  __IO uint32_t  CR1;                               /*!< control register 1                                                    */
  __IO uint32_t  CR2;                               /*!< control register 2                                                    */
  __I  uint32_t  RESERVED0;
  __IO uint32_t  DIER;                              /*!< DMA/Interrupt enable register                                         */
  __IO uint32_t  SR;                                /*!< status register                                                       */
  __O  uint32_t  EGR;                               /*!< event generation register                                             */
  
  union {
    __IO uint32_t  CCMR1_Input;                     /*!< capture/compare mode register 1 (input mode)                          */
    __IO uint32_t  CCMR1_Output;                    /*!< capture/compare mode register (output mode)                           */
  } ;
  __I  uint32_t  RESERVED1;
  __IO uint32_t  CCER;                              /*!< capture/compare enable register                                       */
  __IO uint32_t  CNT;                               /*!< counter                                                               */
  __IO uint32_t  PSC;                               /*!< prescaler                                                             */
  __IO uint32_t  ARR;                               /*!< auto-reload register                                                  */
  __IO uint32_t  RCR;                               /*!< repetition counter register                                           */
  __IO uint32_t  CCR1;                              /*!< capture/compare register 1                                            */
  __I  uint32_t  RESERVED2[3];
  __IO uint32_t  BDTR;                              /*!< break and dead-time register                                          */
  __IO uint32_t  DCR;                               /*!< DMA control register                                                  */
  __IO uint32_t  DMAR;                              /*!< DMA address for full transfer                                         */
  __IO uint32_t  OR;                                /*!< option register                                                       */
} TIM16_Type;


/* ================================================================================ */
/* ================                      TIM17                     ================ */
/* ================================================================================ */


/**
  * @brief General purpose timer (TIM17)
  */

typedef struct {                                    /*!< TIM17 Structure                                                       */
  __IO uint32_t  CR1;                               /*!< control register 1                                                    */
  __IO uint32_t  CR2;                               /*!< control register 2                                                    */
  __I  uint32_t  RESERVED0;
  __IO uint32_t  DIER;                              /*!< DMA/Interrupt enable register                                         */
  __IO uint32_t  SR;                                /*!< status register                                                       */
  __O  uint32_t  EGR;                               /*!< event generation register                                             */
  
  union {
    __IO uint32_t  CCMR1_Input;                     /*!< capture/compare mode register 1 (input mode)                          */
    __IO uint32_t  CCMR1_Output;                    /*!< capture/compare mode register (output mode)                           */
  } ;
  __I  uint32_t  RESERVED1;
  __IO uint32_t  CCER;                              /*!< capture/compare enable register                                       */
  __IO uint32_t  CNT;                               /*!< counter                                                               */
  __IO uint32_t  PSC;                               /*!< prescaler                                                             */
  __IO uint32_t  ARR;                               /*!< auto-reload register                                                  */
  __IO uint32_t  RCR;                               /*!< repetition counter register                                           */
  __IO uint32_t  CCR1;                              /*!< capture/compare register 1                                            */
  __I  uint32_t  RESERVED2[3];
  __IO uint32_t  BDTR;                              /*!< break and dead-time register                                          */
  __IO uint32_t  DCR;                               /*!< DMA control register                                                  */
  __IO uint32_t  DMAR;                              /*!< DMA address for full transfer                                         */
} TIM17_Type;


/* ================================================================================ */
/* ================                     USART1                     ================ */
/* ================================================================================ */


/**
  * @brief Universal synchronous asynchronous receiver-transmitter (USART1)
  */

typedef struct {                                    /*!< USART1 Structure                                                      */
  __IO uint32_t  CR1;                               /*!< Control register 1                                                    */
  __IO uint32_t  CR2;                               /*!< Control register 2                                                    */
  __IO uint32_t  CR3;                               /*!< Control register 3                                                    */
  __IO uint32_t  BRR;                               /*!< Baud rate register                                                    */
  __IO uint32_t  GTPR;                              /*!< Guard time and prescaler register                                     */
  __IO uint32_t  RTOR;                              /*!< Receiver timeout register                                             */
  __IO uint32_t  RQR;                               /*!< Request register                                                      */
  __I  uint32_t  ISR;                               /*!< Interrupt & status register                                           */
  __IO uint32_t  ICR;                               /*!< Interrupt flag clear register                                         */
  __I  uint32_t  RDR;                               /*!< Receive data register                                                 */
  __IO uint32_t  TDR;                               /*!< Transmit data register                                                */
} USART1_Type;


/* ================================================================================ */
/* ================                      SPI2                      ================ */
/* ================================================================================ */


/**
  * @brief Serial peripheral interface/Inter-IC2 (SPI2)
  */

typedef struct {                                    /*!< SPI2 Structure                                                        */
  __IO uint32_t  CR1;                               /*!< control register 1                                                    */
  __IO uint32_t  CR2;                               /*!< control register 2                                                    */
  __IO uint32_t  SR;                                /*!< status register                                                       */
  __IO uint32_t  DR;                                /*!< data register                                                         */
  __IO uint32_t  CRCPR;                             /*!< CRC polynomial register                                               */
  __I  uint32_t  RXCRCR;                            /*!< RX CRC register                                                       */
  __I  uint32_t  TXCRCR;                            /*!< TX CRC register                                                       */
  __IO uint32_t  I2SCFGR;                           /*!< I2S configuration register                                            */
  __IO uint32_t  I2SPR;                             /*!< I2S prescaler register                                                */
} SPI2_Type;


/* ================================================================================ */
/* ================                      SPI3                      ================ */
/* ================================================================================ */


/**
  * @brief Serial peripheral interface/Inter-IC2 (SPI3)
  */

typedef struct {                                    /*!< SPI3 Structure                                                        */
  __IO uint32_t  CR1;                               /*!< control register 1                                                    */
  __IO uint32_t  CR2;                               /*!< control register 2                                                    */
  __IO uint32_t  SR;                                /*!< status register                                                       */
  __IO uint32_t  DR;                                /*!< data register                                                         */
  __IO uint32_t  CRCPR;                             /*!< CRC polynomial register                                               */
  __I  uint32_t  RXCRCR;                            /*!< RX CRC register                                                       */
  __I  uint32_t  TXCRCR;                            /*!< TX CRC register                                                       */
  __IO uint32_t  I2SCFGR;                           /*!< I2S configuration register                                            */
  __IO uint32_t  I2SPR;                             /*!< I2S prescaler register                                                */
} SPI3_Type;


/* ================================================================================ */
/* ================                      EXTI                      ================ */
/* ================================================================================ */


/**
  * @brief External interrupt/event controller (EXTI)
  */

typedef struct {                                    /*!< EXTI Structure                                                        */
  __IO uint32_t  IMR1;                              /*!< Interrupt mask register                                               */
  __IO uint32_t  EMR1;                              /*!< Event mask register                                                   */
  __IO uint32_t  RTSR1;                             /*!< Rising Trigger selection register                                     */
  __IO uint32_t  FTSR1;                             /*!< Falling Trigger selection register                                    */
  __IO uint32_t  SWIER1;                            /*!< Software interrupt event register                                     */
  __IO uint32_t  PR1;                               /*!< Pending register                                                      */
  __IO uint32_t  IMR2;                              /*!< Interrupt mask register                                               */
  __IO uint32_t  EMR2;                              /*!< Event mask register                                                   */
  __IO uint32_t  RTSR2;                             /*!< Rising Trigger selection register                                     */
  __IO uint32_t  FTSR2;                             /*!< Falling Trigger selection register                                    */
  __IO uint32_t  SWIER2;                            /*!< Software interrupt event register                                     */
  __IO uint32_t  PR2;                               /*!< Pending register                                                      */
} EXTI_Type;


/* ================================================================================ */
/* ================                       PWR                      ================ */
/* ================================================================================ */


/**
  * @brief Power control (PWR)
  */

typedef struct {                                    /*!< PWR Structure                                                         */
  __IO uint32_t  CR;                                /*!< power control register                                                */
  __IO uint32_t  CSR;                               /*!< power control/status register                                         */
} PWR_Type;


/* ================================================================================ */
/* ================                      I2C1                      ================ */
/* ================================================================================ */


/**
  * @brief Inter-integrated circuit (I2C1)
  */

typedef struct {                                    /*!< I2C1 Structure                                                        */
  __IO uint32_t  CR1;                               /*!< Control register 1                                                    */
  __IO uint32_t  CR2;                               /*!< Control register 2                                                    */
  __IO uint32_t  OAR1;                              /*!< Own address register 1                                                */
  __IO uint32_t  OAR2;                              /*!< Own address register 2                                                */
  __IO uint32_t  TIMINGR;                           /*!< Timing register                                                       */
  __IO uint32_t  TIMEOUTR;                          /*!< Status register 1                                                     */
  __IO uint32_t  ISR;                               /*!< Interrupt and Status register                                         */
  __O  uint32_t  ICR;                               /*!< Interrupt clear register                                              */
  __I  uint32_t  PECR;                              /*!< PEC register                                                          */
  __I  uint32_t  RXDR;                              /*!< Receive data register                                                 */
  __IO uint32_t  TXDR;                              /*!< Transmit data register                                                */
} I2C1_Type;


/* ================================================================================ */
/* ================                      IWDG                      ================ */
/* ================================================================================ */


/**
  * @brief Independent watchdog (IWDG)
  */

typedef struct {                                    /*!< IWDG Structure                                                        */
  __O  uint32_t  KR;                                /*!< Key register                                                          */
  __IO uint32_t  PR;                                /*!< Prescaler register                                                    */
  __IO uint32_t  RLR;                               /*!< Reload register                                                       */
  __I  uint32_t  SR;                                /*!< Status register                                                       */
  __IO uint32_t  WINR;                              /*!< Window register                                                       */
} IWDG_Type;


/* ================================================================================ */
/* ================                      WWDG                      ================ */
/* ================================================================================ */


/**
  * @brief Window watchdog (WWDG)
  */

typedef struct {                                    /*!< WWDG Structure                                                        */
  __IO uint32_t  CR;                                /*!< Control register                                                      */
  __IO uint32_t  CFR;                               /*!< Configuration register                                                */
  __IO uint32_t  SR;                                /*!< Status register                                                       */
} WWDG_Type;


/* ================================================================================ */
/* ================                       RTC                      ================ */
/* ================================================================================ */


/**
  * @brief Real-time clock (RTC)
  */

typedef struct {                                    /*!< RTC Structure                                                         */
  __IO uint32_t  TR;                                /*!< time register                                                         */
  __IO uint32_t  DR;                                /*!< date register                                                         */
  __IO uint32_t  CR;                                /*!< control register                                                      */
  __IO uint32_t  ISR;                               /*!< initialization and status register                                    */
  __IO uint32_t  PRER;                              /*!< prescaler register                                                    */
  __IO uint32_t  WUTR;                              /*!< wakeup timer register                                                 */
  __I  uint32_t  RESERVED0;
  __IO uint32_t  ALRMAR;                            /*!< alarm A register                                                      */
  __IO uint32_t  ALRMBR;                            /*!< alarm B register                                                      */
  __O  uint32_t  WPR;                               /*!< write protection register                                             */
  __I  uint32_t  SSR;                               /*!< sub second register                                                   */
  __O  uint32_t  SHIFTR;                            /*!< shift control register                                                */
  __I  uint32_t  TSTR;                              /*!< time stamp time register                                              */
  __I  uint32_t  TSDR;                              /*!< time stamp date register                                              */
  __I  uint32_t  TSSSR;                             /*!< timestamp sub second register                                         */
  __IO uint32_t  CALR;                              /*!< calibration register                                                  */
  __IO uint32_t  TAFCR;                             /*!< tamper and alternate function configuration register                  */
  __IO uint32_t  ALRMASSR;                          /*!< alarm A sub second register                                           */
  __IO uint32_t  ALRMBSSR;                          /*!< alarm B sub second register                                           */
  __I  uint32_t  RESERVED1;
  __IO uint32_t  BKP0R;                             /*!< backup register                                                       */
  __IO uint32_t  BKP1R;                             /*!< backup register                                                       */
  __IO uint32_t  BKP2R;                             /*!< backup register                                                       */
  __IO uint32_t  BKP3R;                             /*!< backup register                                                       */
  __IO uint32_t  BKP4R;                             /*!< backup register                                                       */
  __IO uint32_t  BKP5R;                             /*!< backup register                                                       */
  __IO uint32_t  BKP6R;                             /*!< backup register                                                       */
  __IO uint32_t  BKP7R;                             /*!< backup register                                                       */
  __IO uint32_t  BKP8R;                             /*!< backup register                                                       */
  __IO uint32_t  BKP9R;                             /*!< backup register                                                       */
  __IO uint32_t  BKP10R;                            /*!< backup register                                                       */
  __IO uint32_t  BKP11R;                            /*!< backup register                                                       */
  __IO uint32_t  BKP12R;                            /*!< backup register                                                       */
  __IO uint32_t  BKP13R;                            /*!< backup register                                                       */
  __IO uint32_t  BKP14R;                            /*!< backup register                                                       */
  __IO uint32_t  BKP15R;                            /*!< backup register                                                       */
  __IO uint32_t  BKP16R;                            /*!< backup register                                                       */
  __IO uint32_t  BKP17R;                            /*!< backup register                                                       */
  __IO uint32_t  BKP18R;                            /*!< backup register                                                       */
  __IO uint32_t  BKP19R;                            /*!< backup register                                                       */
  __IO uint32_t  BKP20R;                            /*!< backup register                                                       */
  __IO uint32_t  BKP21R;                            /*!< backup register                                                       */
  __IO uint32_t  BKP22R;                            /*!< backup register                                                       */
  __IO uint32_t  BKP23R;                            /*!< backup register                                                       */
  __IO uint32_t  BKP24R;                            /*!< backup register                                                       */
  __IO uint32_t  BKP25R;                            /*!< backup register                                                       */
  __IO uint32_t  BKP26R;                            /*!< backup register                                                       */
  __IO uint32_t  BKP27R;                            /*!< backup register                                                       */
  __IO uint32_t  BKP28R;                            /*!< backup register                                                       */
  __IO uint32_t  BKP29R;                            /*!< backup register                                                       */
  __IO uint32_t  BKP30R;                            /*!< backup register                                                       */
  __IO uint32_t  BKP31R;                            /*!< backup register                                                       */
} RTC_Type;


/* ================================================================================ */
/* ================                      TIM6                      ================ */
/* ================================================================================ */


/**
  * @brief Basic timers (TIM6)
  */

typedef struct {                                    /*!< TIM6 Structure                                                        */
  __IO uint32_t  CR1;                               /*!< control register 1                                                    */
  __IO uint32_t  CR2;                               /*!< control register 2                                                    */
  __I  uint32_t  RESERVED0;
  __IO uint32_t  DIER;                              /*!< DMA/Interrupt enable register                                         */
  __IO uint32_t  SR;                                /*!< status register                                                       */
  __O  uint32_t  EGR;                               /*!< event generation register                                             */
  __I  uint32_t  RESERVED1[3];
  __IO uint32_t  CNT;                               /*!< counter                                                               */
  __IO uint32_t  PSC;                               /*!< prescaler                                                             */
  __IO uint32_t  ARR;                               /*!< auto-reload register                                                  */
} TIM6_Type;


/* ================================================================================ */
/* ================                      DAC1                      ================ */
/* ================================================================================ */


/**
  * @brief Digital-to-analog converter (DAC1)
  */

typedef struct {                                    /*!< DAC1 Structure                                                        */
  __IO uint32_t  CR;                                /*!< control register                                                      */
  __O  uint32_t  SWTRIGR;                           /*!< software trigger register                                             */
  __IO uint32_t  DHR12R1;                           /*!< channel1 12-bit right-aligned data holding register                   */
  __IO uint32_t  DHR12L1;                           /*!< channel1 12-bit left aligned data holding register                    */
  __IO uint32_t  DHR8R1;                            /*!< channel1 8-bit right aligned data holding register                    */
  __IO uint32_t  DHR12R2;                           /*!< channel2 12-bit right aligned data holding register                   */
  __IO uint32_t  DHR12L2;                           /*!< channel2 12-bit left aligned data holding register                    */
  __IO uint32_t  DHR8R2;                            /*!< channel2 8-bit right-aligned data holding register                    */
  __IO uint32_t  DHR12RD;                           /*!< Dual DAC 12-bit right-aligned data holding register                   */
  __IO uint32_t  DHR12LD;                           /*!< DUAL DAC 12-bit left aligned data holding register                    */
  __IO uint32_t  DHR8RD;                            /*!< DUAL DAC 8-bit right aligned data holding register                    */
  __I  uint32_t  DOR1;                              /*!< channel1 data output register                                         */
  __I  uint32_t  DOR2;                              /*!< channel2 data output register                                         */
  __IO uint32_t  SR;                                /*!< status register                                                       */
} DAC1_Type;


/* ================================================================================ */
/* ================                     DBGMCU                     ================ */
/* ================================================================================ */


/**
  * @brief Debug support (DBGMCU)
  */

typedef struct {                                    /*!< DBGMCU Structure                                                      */
  __I  uint32_t  IDCODE;                            /*!< MCU Device ID Code Register                                           */
  __IO uint32_t  CR;                                /*!< Debug MCU Configuration Register                                      */
  __IO uint32_t  APB1FZ;                            /*!< APB Low Freeze Register                                               */
  __IO uint32_t  APB2FZ;                            /*!< APB High Freeze Register                                              */
} DBGMCU_Type;


/* ================================================================================ */
/* ================                      TIM1                      ================ */
/* ================================================================================ */


/**
  * @brief Advanced timer (TIM1)
  */

typedef struct {                                    /*!< TIM1 Structure                                                        */
  __IO uint32_t  CR1;                               /*!< control register 1                                                    */
  __IO uint32_t  CR2;                               /*!< control register 2                                                    */
  __IO uint32_t  SMCR;                              /*!< slave mode control register                                           */
  __IO uint32_t  DIER;                              /*!< DMA/Interrupt enable register                                         */
  __IO uint32_t  SR;                                /*!< status register                                                       */
  __O  uint32_t  EGR;                               /*!< event generation register                                             */
  
  union {
    __IO uint32_t  CCMR1_Input;                     /*!< capture/compare mode register 1 (input mode)                          */
    __IO uint32_t  CCMR1_Output;                    /*!< capture/compare mode register (output mode)                           */
  } ;
  
  union {
    __IO uint32_t  CCMR2_Input;                     /*!< capture/compare mode register 2 (input mode)                          */
    __IO uint32_t  CCMR2_Output;                    /*!< capture/compare mode register (output mode)                           */
  } ;
  __IO uint32_t  CCER;                              /*!< capture/compare enable register                                       */
  __IO uint32_t  CNT;                               /*!< counter                                                               */
  __IO uint32_t  PSC;                               /*!< prescaler                                                             */
  __IO uint32_t  ARR;                               /*!< auto-reload register                                                  */
  __IO uint32_t  RCR;                               /*!< repetition counter register                                           */
  __IO uint32_t  CCR1;                              /*!< capture/compare register 1                                            */
  __IO uint32_t  CCR2;                              /*!< capture/compare register 2                                            */
  __IO uint32_t  CCR3;                              /*!< capture/compare register 3                                            */
  __IO uint32_t  CCR4;                              /*!< capture/compare register 4                                            */
  __IO uint32_t  BDTR;                              /*!< break and dead-time register                                          */
  __IO uint32_t  DCR;                               /*!< DMA control register                                                  */
  __IO uint32_t  DMAR;                              /*!< DMA address for full transfer                                         */
  __I  uint32_t  RESERVED0;
  __IO uint32_t  CCMR3_Output;                      /*!< capture/compare mode register 3 (output mode)                         */
  __IO uint32_t  CCR5;                              /*!< capture/compare register 5                                            */
  __IO uint32_t  CCR6;                              /*!< capture/compare register 6                                            */
  __IO uint32_t  OR;                                /*!< option registers                                                      */
} TIM1_Type;


/* ================================================================================ */
/* ================                      ADC1                      ================ */
/* ================================================================================ */


/**
  * @brief Analog-to-Digital Converter (ADC1)
  */

typedef struct {                                    /*!< ADC1 Structure                                                        */
  __IO uint32_t  ISR;                               /*!< interrupt and status register                                         */
  __IO uint32_t  IER;                               /*!< interrupt enable register                                             */
  __IO uint32_t  CR;                                /*!< control register                                                      */
  __IO uint32_t  CFGR;                              /*!< configuration register                                                */
  __I  uint32_t  RESERVED0;
  __IO uint32_t  SMPR1;                             /*!< sample time register 1                                                */
  __IO uint32_t  SMPR2;                             /*!< sample time register 2                                                */
  __I  uint32_t  RESERVED1;
  __IO uint32_t  TR1;                               /*!< watchdog threshold register 1                                         */
  __IO uint32_t  TR2;                               /*!< watchdog threshold register                                           */
  __IO uint32_t  TR3;                               /*!< watchdog threshold register 3                                         */
  __I  uint32_t  RESERVED2;
  __IO uint32_t  SQR1;                              /*!< regular sequence register 1                                           */
  __IO uint32_t  SQR2;                              /*!< regular sequence register 2                                           */
  __IO uint32_t  SQR3;                              /*!< regular sequence register 3                                           */
  __IO uint32_t  SQR4;                              /*!< regular sequence register 4                                           */
  __I  uint32_t  DR;                                /*!< regular Data Register                                                 */
  __I  uint32_t  RESERVED3[2];
  __IO uint32_t  JSQR;                              /*!< injected sequence register                                            */
  __I  uint32_t  RESERVED4[4];
  __IO uint32_t  OFR1;                              /*!< offset register 1                                                     */
  __IO uint32_t  OFR2;                              /*!< offset register 2                                                     */
  __IO uint32_t  OFR3;                              /*!< offset register 3                                                     */
  __IO uint32_t  OFR4;                              /*!< offset register 4                                                     */
  __I  uint32_t  RESERVED5[4];
  __I  uint32_t  JDR1;                              /*!< injected data register 1                                              */
  __I  uint32_t  JDR2;                              /*!< injected data register 2                                              */
  __I  uint32_t  JDR3;                              /*!< injected data register 3                                              */
  __I  uint32_t  JDR4;                              /*!< injected data register 4                                              */
  __I  uint32_t  RESERVED6[4];
  __IO uint32_t  AWD2CR;                            /*!< Analog Watchdog 2 Configuration Register                              */
  __IO uint32_t  AWD3CR;                            /*!< Analog Watchdog 3 Configuration Register                              */
  __I  uint32_t  RESERVED7[2];
  __IO uint32_t  DIFSEL;                            /*!< Differential Mode Selection Register 2                                */
  __IO uint32_t  CALFACT;                           /*!< Calibration Factors                                                   */
} ADC1_Type;


/* ================================================================================ */
/* ================                SYSCFG_COMP_OPAMP               ================ */
/* ================================================================================ */


/**
  * @brief System configuration controller (SYSCFG_COMP_OPAMP)
  */

typedef struct {                                    /*!< SYSCFG_COMP_OPAMP Structure                                           */
  __IO uint32_t  SYSCFG_CFGR1;                      /*!< configuration register 1                                              */
  __IO uint32_t  SYSCFG_RCR;                        /*!< CCM SRAM protection register                                          */
  __IO uint32_t  SYSCFG_EXTICR1;                    /*!< external interrupt configuration register 1                           */
  __IO uint32_t  SYSCFG_EXTICR2;                    /*!< external interrupt configuration register 2                           */
  __IO uint32_t  SYSCFG_EXTICR3;                    /*!< external interrupt configuration register 3                           */
  __IO uint32_t  SYSCFG_EXTICR4;                    /*!< external interrupt configuration register 4                           */
  __IO uint32_t  SYSCFG_CFGR2;                      /*!< configuration register 2                                              */
  __I  uint32_t  RESERVED0;
  __IO uint32_t  COMP2_CSR;                         /*!< control and status register                                           */
  __I  uint32_t  RESERVED1;
  __IO uint32_t  COMP4_CSR;                         /*!< control and status register                                           */
  __I  uint32_t  RESERVED2;
  __IO uint32_t  COMP6_CSR;                         /*!< control and status register                                           */
  __I  uint32_t  RESERVED3[2];
  __IO uint32_t  OPAMP2_CSR;                        /*!< OPAMP2 control register                                               */
  __I  uint32_t  RESERVED4[4];
  __IO uint32_t  SYSCFG_CFGR3;                      /*!< configuration register 3                                              */
} SYSCFG_COMP_OPAMP_Type;


/* ================================================================================ */
/* ================                      TIM3                      ================ */
/* ================================================================================ */


/**
  * @brief General purpose timer (TIM3)
  */

typedef struct {                                    /*!< TIM3 Structure                                                        */
  __IO uint32_t  CR1;                               /*!< control register 1                                                    */
  __IO uint32_t  CR2;                               /*!< control register 2                                                    */
  __IO uint32_t  SMCR;                              /*!< slave mode control register                                           */
  __IO uint32_t  DIER;                              /*!< DMA/Interrupt enable register                                         */
  __IO uint32_t  SR;                                /*!< status register                                                       */
  __O  uint32_t  EGR;                               /*!< event generation register                                             */
  
  union {
    __IO uint32_t  CCMR1_Input;                     /*!< capture/compare mode register 1 (input mode)                          */
    __IO uint32_t  CCMR1_Output;                    /*!< capture/compare mode register 1 (output mode)                         */
  } ;
  
  union {
    __IO uint32_t  CCMR2_Input;                     /*!< capture/compare mode register 2 (input mode)                          */
    __IO uint32_t  CCMR2_Output;                    /*!< capture/compare mode register 2 (output mode)                         */
  } ;
  __IO uint32_t  CCER;                              /*!< capture/compare enable register                                       */
  __IO uint32_t  CNT;                               /*!< counter                                                               */
  __IO uint32_t  PSC;                               /*!< prescaler                                                             */
  __IO uint32_t  ARR;                               /*!< auto-reload register                                                  */
  __I  uint32_t  RESERVED0;
  __IO uint32_t  CCR1;                              /*!< capture/compare register 1                                            */
  __IO uint32_t  CCR2;                              /*!< capture/compare register 2                                            */
  __IO uint32_t  CCR3;                              /*!< capture/compare register 3                                            */
  __IO uint32_t  CCR4;                              /*!< capture/compare register 4                                            */
  __I  uint32_t  RESERVED1;
  __IO uint32_t  DCR;                               /*!< DMA control register                                                  */
  __IO uint32_t  DMAR;                              /*!< DMA address for full transfer                                         */
} TIM3_Type;


/* ================================================================================ */
/* ================                       CAN                      ================ */
/* ================================================================================ */


/**
  * @brief Controller area network (CAN)
  */

typedef struct {                                    /*!< CAN Structure                                                         */
  __IO uint32_t  MCR;                               /*!< master control register                                               */
  __IO uint32_t  MSR;                               /*!< master status register                                                */
  __IO uint32_t  TSR;                               /*!< transmit status register                                              */
  __IO uint32_t  RF0R;                              /*!< receive FIFO 0 register                                               */
  __IO uint32_t  RF1R;                              /*!< receive FIFO 1 register                                               */
  __IO uint32_t  IER;                               /*!< interrupt enable register                                             */
  __IO uint32_t  ESR;                               /*!< error status register                                                 */
  __IO uint32_t  BTR;                               /*!< bit timing register                                                   */
  __I  uint32_t  RESERVED0[88];
  __IO uint32_t  TI0R;                              /*!< TX mailbox identifier register                                        */
  __IO uint32_t  TDT0R;                             /*!< mailbox data length control and time stamp register                   */
  __IO uint32_t  TDL0R;                             /*!< mailbox data low register                                             */
  __IO uint32_t  TDH0R;                             /*!< mailbox data high register                                            */
  __IO uint32_t  TI1R;                              /*!< TX mailbox identifier register                                        */
  __IO uint32_t  TDT1R;                             /*!< mailbox data length control and time stamp register                   */
  __IO uint32_t  TDL1R;                             /*!< mailbox data low register                                             */
  __IO uint32_t  TDH1R;                             /*!< mailbox data high register                                            */
  __IO uint32_t  TI2R;                              /*!< TX mailbox identifier register                                        */
  __IO uint32_t  TDT2R;                             /*!< mailbox data length control and time stamp register                   */
  __IO uint32_t  TDL2R;                             /*!< mailbox data low register                                             */
  __IO uint32_t  TDH2R;                             /*!< mailbox data high register                                            */
  __I  uint32_t  RI0R;                              /*!< receive FIFO mailbox identifier register                              */
  __I  uint32_t  RDT0R;                             /*!< receive FIFO mailbox data length control and time stamp register      */
  __I  uint32_t  RDL0R;                             /*!< receive FIFO mailbox data low register                                */
  __I  uint32_t  RDH0R;                             /*!< receive FIFO mailbox data high register                               */
  __I  uint32_t  RI1R;                              /*!< receive FIFO mailbox identifier register                              */
  __I  uint32_t  RDT1R;                             /*!< receive FIFO mailbox data length control and time stamp register      */
  __I  uint32_t  RDL1R;                             /*!< receive FIFO mailbox data low register                                */
  __I  uint32_t  RDH1R;                             /*!< receive FIFO mailbox data high register                               */
  __I  uint32_t  RESERVED1[12];
  __IO uint32_t  FMR;                               /*!< filter master register                                                */
  __IO uint32_t  FM1R;                              /*!< filter mode register                                                  */
  __I  uint32_t  RESERVED2;
  __IO uint32_t  FS1R;                              /*!< filter scale register                                                 */
  __I  uint32_t  RESERVED3;
  __IO uint32_t  FFA1R;                             /*!< filter FIFO assignment register                                       */
  __I  uint32_t  RESERVED4;
  __IO uint32_t  FA1R;                              /*!< CAN filter activation register                                        */
  __I  uint32_t  RESERVED5[8];
  __IO uint32_t  F0R1;                              /*!< Filter bank 0 register 1                                              */
  __IO uint32_t  F0R2;                              /*!< Filter bank 0 register 2                                              */
  __IO uint32_t  F1R1;                              /*!< Filter bank 1 register 1                                              */
  __IO uint32_t  F1R2;                              /*!< Filter bank 1 register 2                                              */
  __IO uint32_t  F2R1;                              /*!< Filter bank 2 register 1                                              */
  __IO uint32_t  F2R2;                              /*!< Filter bank 2 register 2                                              */
  __IO uint32_t  F3R1;                              /*!< Filter bank 3 register 1                                              */
  __IO uint32_t  F3R2;                              /*!< Filter bank 3 register 2                                              */
  __IO uint32_t  F4R1;                              /*!< Filter bank 4 register 1                                              */
  __IO uint32_t  F4R2;                              /*!< Filter bank 4 register 2                                              */
  __IO uint32_t  F5R1;                              /*!< Filter bank 5 register 1                                              */
  __IO uint32_t  F5R2;                              /*!< Filter bank 5 register 2                                              */
  __IO uint32_t  F6R1;                              /*!< Filter bank 6 register 1                                              */
  __IO uint32_t  F6R2;                              /*!< Filter bank 6 register 2                                              */
  __IO uint32_t  F7R1;                              /*!< Filter bank 7 register 1                                              */
  __IO uint32_t  F7R2;                              /*!< Filter bank 7 register 2                                              */
  __IO uint32_t  F8R1;                              /*!< Filter bank 8 register 1                                              */
  __IO uint32_t  F8R2;                              /*!< Filter bank 8 register 2                                              */
  __IO uint32_t  F9R1;                              /*!< Filter bank 9 register 1                                              */
  __IO uint32_t  F9R2;                              /*!< Filter bank 9 register 2                                              */
  __IO uint32_t  F10R1;                             /*!< Filter bank 10 register 1                                             */
  __IO uint32_t  F10R2;                             /*!< Filter bank 10 register 2                                             */
  __IO uint32_t  F11R1;                             /*!< Filter bank 11 register 1                                             */
  __IO uint32_t  F11R2;                             /*!< Filter bank 11 register 2                                             */
  __IO uint32_t  F12R1;                             /*!< Filter bank 4 register 1                                              */
  __IO uint32_t  F12R2;                             /*!< Filter bank 12 register 2                                             */
  __IO uint32_t  F13R1;                             /*!< Filter bank 13 register 1                                             */
  __IO uint32_t  F13R2;                             /*!< Filter bank 13 register 2                                             */
  __IO uint32_t  F14R1;                             /*!< Filter bank 14 register 1                                             */
  __IO uint32_t  F14R2;                             /*!< Filter bank 14 register 2                                             */
  __IO uint32_t  F15R1;                             /*!< Filter bank 15 register 1                                             */
  __IO uint32_t  F15R2;                             /*!< Filter bank 15 register 2                                             */
  __IO uint32_t  F16R1;                             /*!< Filter bank 16 register 1                                             */
  __IO uint32_t  F16R2;                             /*!< Filter bank 16 register 2                                             */
  __IO uint32_t  F17R1;                             /*!< Filter bank 17 register 1                                             */
  __IO uint32_t  F17R2;                             /*!< Filter bank 17 register 2                                             */
  __IO uint32_t  F18R1;                             /*!< Filter bank 18 register 1                                             */
  __IO uint32_t  F18R2;                             /*!< Filter bank 18 register 2                                             */
  __IO uint32_t  F19R1;                             /*!< Filter bank 19 register 1                                             */
  __IO uint32_t  F19R2;                             /*!< Filter bank 19 register 2                                             */
  __IO uint32_t  F20R1;                             /*!< Filter bank 20 register 1                                             */
  __IO uint32_t  F20R2;                             /*!< Filter bank 20 register 2                                             */
  __IO uint32_t  F21R1;                             /*!< Filter bank 21 register 1                                             */
  __IO uint32_t  F21R2;                             /*!< Filter bank 21 register 2                                             */
  __IO uint32_t  F22R1;                             /*!< Filter bank 22 register 1                                             */
  __IO uint32_t  F22R2;                             /*!< Filter bank 22 register 2                                             */
  __IO uint32_t  F23R1;                             /*!< Filter bank 23 register 1                                             */
  __IO uint32_t  F23R2;                             /*!< Filter bank 23 register 2                                             */
  __IO uint32_t  F24R1;                             /*!< Filter bank 24 register 1                                             */
  __IO uint32_t  F24R2;                             /*!< Filter bank 24 register 2                                             */
  __IO uint32_t  F25R1;                             /*!< Filter bank 25 register 1                                             */
  __IO uint32_t  F25R2;                             /*!< Filter bank 25 register 2                                             */
  __IO uint32_t  F26R1;                             /*!< Filter bank 26 register 1                                             */
  __IO uint32_t  F26R2;                             /*!< Filter bank 26 register 2                                             */
  __IO uint32_t  F27R1;                             /*!< Filter bank 27 register 1                                             */
  __IO uint32_t  F27R2;                             /*!< Filter bank 27 register 2                                             */
} CAN_Type;


/* ================================================================================ */
/* ================                   ADC_Common                   ================ */
/* ================================================================================ */


/**
  * @brief ADC common registers (ADC_Common)
  */

typedef struct {                                    /*!< ADC_Common Structure                                                  */
  __I  uint32_t  ADC1_CSR;                          /*!< ADC Common status register                                            */
  __I  uint32_t  RESERVED0;
  __IO uint32_t  ADC1_CCR;                          /*!< ADC common control register                                           */
  __I  uint32_t  ADC1_CDR;                          /*!< ADC common regular data register for dual mode                        */
} ADC_Common_Type;


/* ================================================================================ */
/* ================                  HRTIM_Master                  ================ */
/* ================================================================================ */


/**
  * @brief High Resolution Timer: Master Timers (HRTIM_Master)
  */

typedef struct {                                    /*!< HRTIM_Master Structure                                                */
  __IO uint32_t  MCR;                               /*!< Master Timer Control Register                                         */
  __I  uint32_t  MISR;                              /*!< Master Timer Interrupt Status Register                                */
  __O  uint32_t  MICR;                              /*!< Master Timer Interrupt Clear Register                                 */
  __IO uint32_t  MDIER4;                            /*!< MDIER4                                                                */
  __IO uint32_t  MCNTR;                             /*!< Master Timer Counter Register                                         */
  __IO uint32_t  MPER;                              /*!< Master Timer Period Register                                          */
  __IO uint32_t  MREP;                              /*!< Master Timer Repetition Register                                      */
  __IO uint32_t  MCMP1R;                            /*!< Master Timer Compare 1 Register                                       */
  __I  uint32_t  RESERVED0;
  __IO uint32_t  MCMP2R;                            /*!< Master Timer Compare 2 Register                                       */
  __IO uint32_t  MCMP3R;                            /*!< Master Timer Compare 3 Register                                       */
  __IO uint32_t  MCMP4R;                            /*!< Master Timer Compare 4 Register                                       */
} HRTIM_Master_Type;


/* ================================================================================ */
/* ================                   HRTIM_TIMA                   ================ */
/* ================================================================================ */


/**
  * @brief High Resolution Timer: TIMA (HRTIM_TIMA)
  */

typedef struct {                                    /*!< HRTIM_TIMA Structure                                                  */
  __IO uint32_t  TIMACR;                            /*!< Timerx Control Register                                               */
  __I  uint32_t  TIMAISR;                           /*!< Timerx Interrupt Status Register                                      */
  __O  uint32_t  TIMAICR;                           /*!< Timerx Interrupt Clear Register                                       */
  __IO uint32_t  TIMADIER5;                         /*!< TIMxDIER5                                                             */
  __IO uint32_t  CNTAR;                             /*!< Timerx Counter Register                                               */
  __IO uint32_t  PERAR;                             /*!< Timerx Period Register                                                */
  __IO uint32_t  REPAR;                             /*!< Timerx Repetition Register                                            */
  __IO uint32_t  CMP1AR;                            /*!< Timerx Compare 1 Register                                             */
  __IO uint32_t  CMP1CAR;                           /*!< Timerx Compare 1 Compound Register                                    */
  __IO uint32_t  CMP2AR;                            /*!< Timerx Compare 2 Register                                             */
  __IO uint32_t  CMP3AR;                            /*!< Timerx Compare 3 Register                                             */
  __IO uint32_t  CMP4AR;                            /*!< Timerx Compare 4 Register                                             */
  __I  uint32_t  CPT1AR;                            /*!< Timerx Capture 1 Register                                             */
  __I  uint32_t  CPT2AR;                            /*!< Timerx Capture 2 Register                                             */
  __IO uint32_t  DTAR;                              /*!< Timerx Deadtime Register                                              */
  __IO uint32_t  SETA1R;                            /*!< Timerx Output1 Set Register                                           */
  __IO uint32_t  RSTA1R;                            /*!< Timerx Output1 Reset Register                                         */
  __IO uint32_t  SETA2R;                            /*!< Timerx Output2 Set Register                                           */
  __IO uint32_t  RSTA2R;                            /*!< Timerx Output2 Reset Register                                         */
  __IO uint32_t  EEFAR1;                            /*!< Timerx External Event Filtering Register 1                            */
  __IO uint32_t  EEFAR2;                            /*!< Timerx External Event Filtering Register 2                            */
  __IO uint32_t  RSTAR;                             /*!< TimerA Reset Register                                                 */
  __IO uint32_t  CHPAR;                             /*!< Timerx Chopper Register                                               */
  __IO uint32_t  CPT1ACR;                           /*!< Timerx Capture 2 Control Register                                     */
  __IO uint32_t  CPT2ACR;                           /*!< CPT2xCR                                                               */
  __IO uint32_t  OUTAR;                             /*!< Timerx Output Register                                                */
  __IO uint32_t  FLTAR;                             /*!< Timerx Fault Register                                                 */
} HRTIM_TIMA_Type;


/* ================================================================================ */
/* ================                   HRTIM_TIMB                   ================ */
/* ================================================================================ */


/**
  * @brief High Resolution Timer: TIMB (HRTIM_TIMB)
  */

typedef struct {                                    /*!< HRTIM_TIMB Structure                                                  */
  __IO uint32_t  TIMBCR;                            /*!< Timerx Control Register                                               */
  __I  uint32_t  TIMBISR;                           /*!< Timerx Interrupt Status Register                                      */
  __O  uint32_t  TIMBICR;                           /*!< Timerx Interrupt Clear Register                                       */
  __IO uint32_t  TIMBDIER5;                         /*!< TIMxDIER5                                                             */
  __IO uint32_t  CNTR;                              /*!< Timerx Counter Register                                               */
  __IO uint32_t  PERBR;                             /*!< Timerx Period Register                                                */
  __IO uint32_t  REPBR;                             /*!< Timerx Repetition Register                                            */
  __IO uint32_t  CMP1BR;                            /*!< Timerx Compare 1 Register                                             */
  __IO uint32_t  CMP1CBR;                           /*!< Timerx Compare 1 Compound Register                                    */
  __IO uint32_t  CMP2BR;                            /*!< Timerx Compare 2 Register                                             */
  __IO uint32_t  CMP3BR;                            /*!< Timerx Compare 3 Register                                             */
  __IO uint32_t  CMP4BR;                            /*!< Timerx Compare 4 Register                                             */
  __I  uint32_t  CPT1BR;                            /*!< Timerx Capture 1 Register                                             */
  __I  uint32_t  CPT2BR;                            /*!< Timerx Capture 2 Register                                             */
  __IO uint32_t  DTBR;                              /*!< Timerx Deadtime Register                                              */
  __IO uint32_t  SETB1R;                            /*!< Timerx Output1 Set Register                                           */
  __IO uint32_t  RSTB1R;                            /*!< Timerx Output1 Reset Register                                         */
  __IO uint32_t  SETB2R;                            /*!< Timerx Output2 Set Register                                           */
  __IO uint32_t  RSTB2R;                            /*!< Timerx Output2 Reset Register                                         */
  __IO uint32_t  EEFBR1;                            /*!< Timerx External Event Filtering Register 1                            */
  __IO uint32_t  EEFBR2;                            /*!< Timerx External Event Filtering Register 2                            */
  __IO uint32_t  RSTBR;                             /*!< TimerA Reset Register                                                 */
  __IO uint32_t  CHPBR;                             /*!< Timerx Chopper Register                                               */
  __IO uint32_t  CPT1BCR;                           /*!< Timerx Capture 2 Control Register                                     */
  __IO uint32_t  CPT2BCR;                           /*!< CPT2xCR                                                               */
  __IO uint32_t  OUTBR;                             /*!< Timerx Output Register                                                */
  __IO uint32_t  FLTBR;                             /*!< Timerx Fault Register                                                 */
} HRTIM_TIMB_Type;


/* ================================================================================ */
/* ================                   HRTIM_TIMC                   ================ */
/* ================================================================================ */


/**
  * @brief High Resolution Timer: TIMC (HRTIM_TIMC)
  */

typedef struct {                                    /*!< HRTIM_TIMC Structure                                                  */
  __IO uint32_t  TIMCCR;                            /*!< Timerx Control Register                                               */
  __I  uint32_t  TIMCISR;                           /*!< Timerx Interrupt Status Register                                      */
  __O  uint32_t  TIMCICR;                           /*!< Timerx Interrupt Clear Register                                       */
  __IO uint32_t  TIMCDIER5;                         /*!< TIMxDIER5                                                             */
  __IO uint32_t  CNTCR;                             /*!< Timerx Counter Register                                               */
  __IO uint32_t  PERCR;                             /*!< Timerx Period Register                                                */
  __IO uint32_t  REPCR;                             /*!< Timerx Repetition Register                                            */
  __IO uint32_t  CMP1CR;                            /*!< Timerx Compare 1 Register                                             */
  __IO uint32_t  CMP1CCR;                           /*!< Timerx Compare 1 Compound Register                                    */
  __IO uint32_t  CMP2CR;                            /*!< Timerx Compare 2 Register                                             */
  __IO uint32_t  CMP3CR;                            /*!< Timerx Compare 3 Register                                             */
  __IO uint32_t  CMP4CR;                            /*!< Timerx Compare 4 Register                                             */
  __I  uint32_t  CPT1CR;                            /*!< Timerx Capture 1 Register                                             */
  __I  uint32_t  CPT2CR;                            /*!< Timerx Capture 2 Register                                             */
  __IO uint32_t  DTCR;                              /*!< Timerx Deadtime Register                                              */
  __IO uint32_t  SETC1R;                            /*!< Timerx Output1 Set Register                                           */
  __IO uint32_t  RSTC1R;                            /*!< Timerx Output1 Reset Register                                         */
  __IO uint32_t  SETC2R;                            /*!< Timerx Output2 Set Register                                           */
  __IO uint32_t  RSTC2R;                            /*!< Timerx Output2 Reset Register                                         */
  __IO uint32_t  EEFCR1;                            /*!< Timerx External Event Filtering Register 1                            */
  __IO uint32_t  EEFCR2;                            /*!< Timerx External Event Filtering Register 2                            */
  __IO uint32_t  RSTCR;                             /*!< TimerA Reset Register                                                 */
  __IO uint32_t  CHPCR;                             /*!< Timerx Chopper Register                                               */
  __IO uint32_t  CPT1CCR;                           /*!< Timerx Capture 2 Control Register                                     */
  __IO uint32_t  CPT2CCR;                           /*!< CPT2xCR                                                               */
  __IO uint32_t  OUTCR;                             /*!< Timerx Output Register                                                */
  __IO uint32_t  FLTCR;                             /*!< Timerx Fault Register                                                 */
} HRTIM_TIMC_Type;


/* ================================================================================ */
/* ================                   HRTIM_TIMD                   ================ */
/* ================================================================================ */


/**
  * @brief High Resolution Timer: TIMD (HRTIM_TIMD)
  */

typedef struct {                                    /*!< HRTIM_TIMD Structure                                                  */
  __IO uint32_t  TIMDCR;                            /*!< Timerx Control Register                                               */
  __I  uint32_t  TIMDISR;                           /*!< Timerx Interrupt Status Register                                      */
  __O  uint32_t  TIMDICR;                           /*!< Timerx Interrupt Clear Register                                       */
  __IO uint32_t  TIMDDIER5;                         /*!< TIMxDIER5                                                             */
  __IO uint32_t  CNTDR;                             /*!< Timerx Counter Register                                               */
  __IO uint32_t  PERDR;                             /*!< Timerx Period Register                                                */
  __IO uint32_t  REPDR;                             /*!< Timerx Repetition Register                                            */
  __IO uint32_t  CMP1DR;                            /*!< Timerx Compare 1 Register                                             */
  __IO uint32_t  CMP1CDR;                           /*!< Timerx Compare 1 Compound Register                                    */
  __IO uint32_t  CMP2DR;                            /*!< Timerx Compare 2 Register                                             */
  __IO uint32_t  CMP3DR;                            /*!< Timerx Compare 3 Register                                             */
  __IO uint32_t  CMP4DR;                            /*!< Timerx Compare 4 Register                                             */
  __I  uint32_t  CPT1DR;                            /*!< Timerx Capture 1 Register                                             */
  __I  uint32_t  CPT2DR;                            /*!< Timerx Capture 2 Register                                             */
  __IO uint32_t  DTDR;                              /*!< Timerx Deadtime Register                                              */
  __IO uint32_t  SETD1R;                            /*!< Timerx Output1 Set Register                                           */
  __IO uint32_t  RSTD1R;                            /*!< Timerx Output1 Reset Register                                         */
  __IO uint32_t  SETD2R;                            /*!< Timerx Output2 Set Register                                           */
  __IO uint32_t  RSTD2R;                            /*!< Timerx Output2 Reset Register                                         */
  __IO uint32_t  EEFDR1;                            /*!< Timerx External Event Filtering Register 1                            */
  __IO uint32_t  EEFDR2;                            /*!< Timerx External Event Filtering Register 2                            */
  __IO uint32_t  RSTDR;                             /*!< TimerA Reset Register                                                 */
  __IO uint32_t  CHPDR;                             /*!< Timerx Chopper Register                                               */
  __IO uint32_t  CPT1DCR;                           /*!< Timerx Capture 2 Control Register                                     */
  __IO uint32_t  CPT2DCR;                           /*!< CPT2xCR                                                               */
  __IO uint32_t  OUTDR;                             /*!< Timerx Output Register                                                */
  __IO uint32_t  FLTDR;                             /*!< Timerx Fault Register                                                 */
} HRTIM_TIMD_Type;


/* ================================================================================ */
/* ================                   HRTIM_TIME                   ================ */
/* ================================================================================ */


/**
  * @brief High Resolution Timer: TIME (HRTIM_TIME)
  */

typedef struct {                                    /*!< HRTIM_TIME Structure                                                  */
  __IO uint32_t  TIMECR;                            /*!< Timerx Control Register                                               */
  __I  uint32_t  TIMEISR;                           /*!< Timerx Interrupt Status Register                                      */
  __O  uint32_t  TIMEICR;                           /*!< Timerx Interrupt Clear Register                                       */
  __IO uint32_t  TIMEDIER5;                         /*!< TIMxDIER5                                                             */
  __IO uint32_t  CNTER;                             /*!< Timerx Counter Register                                               */
  __IO uint32_t  PERER;                             /*!< Timerx Period Register                                                */
  __IO uint32_t  REPER;                             /*!< Timerx Repetition Register                                            */
  __IO uint32_t  CMP1ER;                            /*!< Timerx Compare 1 Register                                             */
  __IO uint32_t  CMP1CER;                           /*!< Timerx Compare 1 Compound Register                                    */
  __IO uint32_t  CMP2ER;                            /*!< Timerx Compare 2 Register                                             */
  __IO uint32_t  CMP3ER;                            /*!< Timerx Compare 3 Register                                             */
  __IO uint32_t  CMP4ER;                            /*!< Timerx Compare 4 Register                                             */
  __I  uint32_t  CPT1ER;                            /*!< Timerx Capture 1 Register                                             */
  __I  uint32_t  CPT2ER;                            /*!< Timerx Capture 2 Register                                             */
  __IO uint32_t  DTER;                              /*!< Timerx Deadtime Register                                              */
  __IO uint32_t  SETE1R;                            /*!< Timerx Output1 Set Register                                           */
  __IO uint32_t  RSTE1R;                            /*!< Timerx Output1 Reset Register                                         */
  __IO uint32_t  SETE2R;                            /*!< Timerx Output2 Set Register                                           */
  __IO uint32_t  RSTE2R;                            /*!< Timerx Output2 Reset Register                                         */
  __IO uint32_t  EEFER1;                            /*!< Timerx External Event Filtering Register 1                            */
  __IO uint32_t  EEFER2;                            /*!< Timerx External Event Filtering Register 2                            */
  __IO uint32_t  RSTER;                             /*!< TimerA Reset Register                                                 */
  __IO uint32_t  CHPER;                             /*!< Timerx Chopper Register                                               */
  __IO uint32_t  CPT1ECR;                           /*!< Timerx Capture 2 Control Register                                     */
  __IO uint32_t  CPT2ECR;                           /*!< CPT2xCR                                                               */
  __IO uint32_t  OUTER;                             /*!< Timerx Output Register                                                */
  __IO uint32_t  FLTER;                             /*!< Timerx Fault Register                                                 */
} HRTIM_TIME_Type;


/* ================================================================================ */
/* ================                  HRTIM_Common                  ================ */
/* ================================================================================ */


/**
  * @brief High Resolution Timer: Common functions (HRTIM_Common)
  */

typedef struct {                                    /*!< HRTIM_Common Structure                                                */
  __IO uint32_t  CR1;                               /*!< Control Register 1                                                    */
  __IO uint32_t  CR2;                               /*!< Control Register 2                                                    */
  __IO uint32_t  ISR;                               /*!< Interrupt Status Register                                             */
  __IO uint32_t  ICR;                               /*!< Interrupt Clear Register                                              */
  __IO uint32_t  IER;                               /*!< Interrupt Enable Register                                             */
  __O  uint32_t  OENR;                              /*!< Output Enable Register                                                */
  __IO uint32_t  ODISR;                             /*!< DISR                                                                  */
  __I  uint32_t  ODSR;                              /*!< Output Disable Status Register                                        */
  __IO uint32_t  BMCR;                              /*!< Burst Mode Control Register                                           */
  __IO uint32_t  BMTRGR;                            /*!< BMTRGR                                                                */
  __IO uint32_t  BMCMPR;                            /*!< BMCMPR                                                                */
  __IO uint32_t  BMPER;                             /*!< Burst Mode Period Register                                            */
  __IO uint32_t  EECR1;                             /*!< Timer External Event Control Register 1                               */
  __IO uint32_t  EECR2;                             /*!< Timer External Event Control Register 2                               */
  __IO uint32_t  EECR3;                             /*!< Timer External Event Control Register 3                               */
  __IO uint32_t  ADC1R;                             /*!< ADC Trigger 1 Register                                                */
  __IO uint32_t  ADC2R;                             /*!< ADC Trigger 2 Register                                                */
  __IO uint32_t  ADC3R;                             /*!< ADC Trigger 3 Register                                                */
  __IO uint32_t  ADC4R;                             /*!< ADC Trigger 4 Register                                                */
  __IO uint32_t  DLLCR;                             /*!< DLL Control Register                                                  */
  __IO uint32_t  FLTINR1;                           /*!< HRTIM Fault Input Register 1                                          */
  __IO uint32_t  FLTINR2;                           /*!< HRTIM Fault Input Register 2                                          */
  __IO uint32_t  BDMUPDR;                           /*!< BDMUPDR                                                               */
  __IO uint32_t  BDTAUPR;                           /*!< Burst DMA Timerx update Register                                      */
  __IO uint32_t  BDTBUPR;                           /*!< Burst DMA Timerx update Register                                      */
  __IO uint32_t  BDTCUPR;                           /*!< Burst DMA Timerx update Register                                      */
  __IO uint32_t  BDTDUPR;                           /*!< Burst DMA Timerx update Register                                      */
  __IO uint32_t  BDTEUPR;                           /*!< Burst DMA Timerx update Register                                      */
  __IO uint32_t  BDMADR;                            /*!< Burst DMA Data Register                                               */
} HRTIM_Common_Type;


/* ================================================================================ */
/* ================                      NVIC                      ================ */
/* ================================================================================ */


/**
  * @brief Nested Vectored Interrupt Controller (NVIC)
  */

typedef struct {                                    /*!< NVIC Structure                                                        */
  __IO uint32_t  ISER0;                             /*!< Interrupt Set-Enable Register                                         */
  __IO uint32_t  ISER1;                             /*!< Interrupt Set-Enable Register                                         */
  __IO uint32_t  ISER2;                             /*!< Interrupt Set-Enable Register                                         */
  __I  uint32_t  RESERVED0[29];
  __IO uint32_t  ICER0;                             /*!< Interrupt Clear-Enable Register                                       */
  __IO uint32_t  ICER1;                             /*!< Interrupt Clear-Enable Register                                       */
  __IO uint32_t  ICER2;                             /*!< Interrupt Clear-Enable Register                                       */
  __I  uint32_t  RESERVED1[29];
  __IO uint32_t  ISPR0;                             /*!< Interrupt Set-Pending Register                                        */
  __IO uint32_t  ISPR1;                             /*!< Interrupt Set-Pending Register                                        */
  __IO uint32_t  ISPR2;                             /*!< Interrupt Set-Pending Register                                        */
  __I  uint32_t  RESERVED2[29];
  __IO uint32_t  ICPR0;                             /*!< Interrupt Clear-Pending Register                                      */
  __IO uint32_t  ICPR1;                             /*!< Interrupt Clear-Pending Register                                      */
  __IO uint32_t  ICPR2;                             /*!< Interrupt Clear-Pending Register                                      */
  __I  uint32_t  RESERVED3[29];
  __I  uint32_t  IABR0;                             /*!< Interrupt Active Bit Register                                         */
  __I  uint32_t  IABR1;                             /*!< Interrupt Active Bit Register                                         */
  __I  uint32_t  IABR2;                             /*!< Interrupt Active Bit Register                                         */
  __I  uint32_t  RESERVED4[61];
  __IO uint32_t  IPR0;                              /*!< Interrupt Priority Register                                           */
  __IO uint32_t  IPR1;                              /*!< Interrupt Priority Register                                           */
  __IO uint32_t  IPR2;                              /*!< Interrupt Priority Register                                           */
  __IO uint32_t  IPR3;                              /*!< Interrupt Priority Register                                           */
  __IO uint32_t  IPR4;                              /*!< Interrupt Priority Register                                           */
  __IO uint32_t  IPR5;                              /*!< Interrupt Priority Register                                           */
  __IO uint32_t  IPR6;                              /*!< Interrupt Priority Register                                           */
  __IO uint32_t  IPR7;                              /*!< Interrupt Priority Register                                           */
  __IO uint32_t  IPR8;                              /*!< Interrupt Priority Register                                           */
  __IO uint32_t  IPR9;                              /*!< Interrupt Priority Register                                           */
  __IO uint32_t  IPR10;                             /*!< Interrupt Priority Register                                           */
  __IO uint32_t  IPR11;                             /*!< Interrupt Priority Register                                           */
  __IO uint32_t  IPR12;                             /*!< Interrupt Priority Register                                           */
  __IO uint32_t  IPR13;                             /*!< Interrupt Priority Register                                           */
  __IO uint32_t  IPR14;                             /*!< Interrupt Priority Register                                           */
  __IO uint32_t  IPR15;                             /*!< Interrupt Priority Register                                           */
  __IO uint32_t  IPR16;                             /*!< Interrupt Priority Register                                           */
  __IO uint32_t  IPR17;                             /*!< Interrupt Priority Register                                           */
  __IO uint32_t  IPR18;                             /*!< Interrupt Priority Register                                           */
  __IO uint32_t  IPR19;                             /*!< Interrupt Priority Register                                           */
  __IO uint32_t  IPR20;                             /*!< Interrupt Priority Register                                           */
} NVIC_Type;


/* ================================================================================ */
/* ================                       FPU                      ================ */
/* ================================================================================ */


/**
  * @brief Floting point unit (FPU)
  */

typedef struct {                                    /*!< FPU Structure                                                         */
  __IO uint32_t  FPCCR;                             /*!< Floating-point context control register                               */
  __IO uint32_t  FPCAR;                             /*!< Floating-point context address register                               */
  __IO uint32_t  FPSCR;                             /*!< Floating-point status control register                                */
} FPU_Type;


/* ================================================================================ */
/* ================                       MPU                      ================ */
/* ================================================================================ */


/**
  * @brief Memory protection unit (MPU)
  */

typedef struct {                                    /*!< MPU Structure                                                         */
  __I  uint32_t  MPU_TYPER;                         /*!< MPU type register                                                     */
  __I  uint32_t  MPU_CTRL;                          /*!< MPU control register                                                  */
  __IO uint32_t  MPU_RNR;                           /*!< MPU region number register                                            */
  __IO uint32_t  MPU_RBAR;                          /*!< MPU region base address register                                      */
  __IO uint32_t  MPU_RASR;                          /*!< MPU region attribute and size register                                */
} MPU_Type;


/* ================================================================================ */
/* ================                       STK                      ================ */
/* ================================================================================ */


/**
  * @brief SysTick timer (STK)
  */

typedef struct {                                    /*!< STK Structure                                                         */
  __IO uint32_t  CTRL;                              /*!< SysTick control and status register                                   */
  __IO uint32_t  LOAD;                              /*!< SysTick reload value register                                         */
  __IO uint32_t  VAL;                               /*!< SysTick current value register                                        */
  __IO uint32_t  CALIB;                             /*!< SysTick calibration value register                                    */
} STK_Type;


/* ================================================================================ */
/* ================                       SCB                      ================ */
/* ================================================================================ */


/**
  * @brief System control block (SCB)
  */

typedef struct {                                    /*!< SCB Structure                                                         */
  __I  uint32_t  CPUID;                             /*!< CPUID base register                                                   */
  __IO uint32_t  ICSR;                              /*!< Interrupt control and state register                                  */
  __IO uint32_t  VTOR;                              /*!< Vector table offset register                                          */
  __IO uint32_t  AIRCR;                             /*!< Application interrupt and reset control register                      */
  __IO uint32_t  SCR;                               /*!< System control register                                               */
  __IO uint32_t  CCR;                               /*!< Configuration and control register                                    */
  __IO uint32_t  SHPR1;                             /*!< System handler priority registers                                     */
  __IO uint32_t  SHPR2;                             /*!< System handler priority registers                                     */
  __IO uint32_t  SHPR3;                             /*!< System handler priority registers                                     */
  __IO uint32_t  SHCRS;                             /*!< System handler control and state register                             */
  __IO uint32_t  CFSR_UFSR_BFSR_MMFSR;              /*!< Configurable fault status register                                    */
  __IO uint32_t  HFSR;                              /*!< Hard fault status register                                            */
  __I  uint32_t  RESERVED0;
  __IO uint32_t  MMFAR;                             /*!< Memory management fault address register                              */
  __IO uint32_t  BFAR;                              /*!< Bus fault address register                                            */
  __IO uint32_t  AFSR;                              /*!< Auxiliary fault status register                                       */
} SCB_Type;


/* ================================================================================ */
/* ================                    NVIC_STIR                   ================ */
/* ================================================================================ */


/**
  * @brief Nested vectored interrupt controller (NVIC_STIR)
  */

typedef struct {                                    /*!< NVIC_STIR Structure                                                   */
  __IO uint32_t  STIR;                              /*!< Software trigger interrupt register                                   */
} NVIC_STIR_Type;


/* ================================================================================ */
/* ================                    FPU_CPACR                   ================ */
/* ================================================================================ */


/**
  * @brief Floating point unit CPACR (FPU_CPACR)
  */

typedef struct {                                    /*!< FPU_CPACR Structure                                                   */
  __IO uint32_t  CPACR;                             /*!< Coprocessor access control register                                   */
} FPU_CPACR_Type;


/* ================================================================================ */
/* ================                    SCB_ACTRL                   ================ */
/* ================================================================================ */


/**
  * @brief System control block ACTLR (SCB_ACTRL)
  */

typedef struct {                                    /*!< SCB_ACTRL Structure                                                   */
  __IO uint32_t  ACTRL;                             /*!< Auxiliary control register                                            */
} SCB_ACTRL_Type;


/* --------------------  End of section using anonymous unions  ------------------- */
#if defined(__CC_ARM)
  #pragma pop
#elif defined(__ICCARM__)
  /* leave anonymous unions enabled */
#elif defined(__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined(__TMS470__)
  /* anonymous unions are enabled by default */
#elif defined(__TASKING__)
  #pragma warning restore
#else
  #warning Not supported compiler type
#endif




/* ================================================================================ */
/* ================              Peripheral memory map             ================ */
/* ================================================================================ */

#define GPIOA_BASE                      0x48000000UL
#define GPIOB_BASE                      0x48000400UL
#define GPIOC_BASE                      0x48000800UL
#define GPIOD_BASE                      0x48000C00UL
#define GPIOF_BASE                      0x48001400UL
#define TSC_BASE                        0x40024000UL
#define CRC_BASE                        0x40023000UL
#define Flash_BASE                      0x40022000UL
#define RCC_BASE                        0x40021000UL
#define DMA1_BASE                       0x40020000UL
#define TIM2_BASE                       0x40000000UL
#define TIM15_BASE                      0x40014000UL
#define TIM16_BASE                      0x40014400UL
#define TIM17_BASE                      0x40014800UL
#define USART1_BASE                     0x40013800UL
#define USART2_BASE                     0x40004400UL
#define USART3_BASE                     0x40004800UL
#define SPI2_BASE                       0x40003800UL
#define I2S2ext_BASE                    0x40003400UL
#define I2S3ext_BASE                    0x40004000UL
#define SPI3_BASE                       0x40003C00UL
#define SPI1_BASE                       0x40013000UL
#define EXTI_BASE                       0x40010400UL
#define PWR_BASE                        0x40007000UL
#define I2C1_BASE                       0x40005400UL
#define I2C2_BASE                       0x40005800UL
#define I2C3_BASE                       0x40007800UL
#define IWDG_BASE                       0x40003000UL
#define WWDG_BASE                       0x40002C00UL
#define RTC_BASE                        0x40002800UL
#define TIM6_BASE                       0x40001000UL
#define TIM7_BASE                       0x40001400UL
#define DAC1_BASE                       0x40007400UL
#define DAC2_BASE                       0x40009800UL
#define DBGMCU_BASE                     0xE0042000UL
#define TIM1_BASE                       0x40012C00UL
#define ADC1_BASE                       0x50000000UL
#define ADC2_BASE                       0x50000100UL
#define SYSCFG_COMP_OPAMP_BASE          0x40010000UL
#define TIM3_BASE                       0x40000400UL
#define CAN_BASE                        0x40006400UL
#define ADC_Common_BASE                 0x50000300UL
#define HRTIM_Master_BASE               0x40017400UL
#define HRTIM_TIMA_BASE                 0x40017480UL
#define HRTIM_TIMB_BASE                 0x40017500UL
#define HRTIM_TIMC_BASE                 0x40017580UL
#define HRTIM_TIMD_BASE                 0x40017600UL
#define HRTIM_TIME_BASE                 0x40017680UL
#define HRTIM_Common_BASE               0x40017780UL
#define NVIC_BASE                       0xE000E100UL
#define FPU_BASE                        0xE000EF34UL
#define MPU_BASE                        0xE000ED90UL
#define STK_BASE                        0xE000E010UL
#define SCB_BASE                        0xE000ED00UL
#define NVIC_STIR_BASE                  0xE000EF00UL
#define FPU_CPACR_BASE                  0xE000ED88UL
#define SCB_ACTRL_BASE                  0xE000E008UL


/* ================================================================================ */
/* ================             Peripheral declaration             ================ */
/* ================================================================================ */

#define GPIOA                           ((GPIOA_Type              *) GPIOA_BASE)
#define GPIOB                           ((GPIOB_Type              *) GPIOB_BASE)
#define GPIOC                           ((GPIOC_Type              *) GPIOC_BASE)
#define GPIOD                           ((GPIOC_Type              *) GPIOD_BASE)
#define GPIOF                           ((GPIOC_Type              *) GPIOF_BASE)
#define TSC                             ((TSC_Type                *) TSC_BASE)
#define CRC                             ((CRC_Type                *) CRC_BASE)
#define Flash                           ((Flash_Type              *) Flash_BASE)
#define RCC                             ((RCC_Type                *) RCC_BASE)
#define DMA1                            ((DMA1_Type               *) DMA1_BASE)
#define TIM2                            ((TIM2_Type               *) TIM2_BASE)
#define TIM15                           ((TIM15_Type              *) TIM15_BASE)
#define TIM16                           ((TIM16_Type              *) TIM16_BASE)
#define TIM17                           ((TIM17_Type              *) TIM17_BASE)
#define USART1                          ((USART1_Type             *) USART1_BASE)
#define USART2                          ((USART1_Type             *) USART2_BASE)
#define USART3                          ((USART1_Type             *) USART3_BASE)
#define SPI2                            ((SPI2_Type               *) SPI2_BASE)
#define I2S2ext                         ((SPI2_Type               *) I2S2ext_BASE)
#define I2S3ext                         ((SPI2_Type               *) I2S3ext_BASE)
#define SPI3                            ((SPI3_Type               *) SPI3_BASE)
#define SPI1                            ((SPI3_Type               *) SPI1_BASE)
#define EXTI                            ((EXTI_Type               *) EXTI_BASE)
#define PWR                             ((PWR_Type                *) PWR_BASE)
#define I2C1                            ((I2C1_Type               *) I2C1_BASE)
#define I2C2                            ((I2C1_Type               *) I2C2_BASE)
#define I2C3                            ((I2C1_Type               *) I2C3_BASE)
#define IWDG                            ((IWDG_Type               *) IWDG_BASE)
#define WWDG                            ((WWDG_Type               *) WWDG_BASE)
#define RTC                             ((RTC_Type                *) RTC_BASE)
#define TIM6                            ((TIM6_Type               *) TIM6_BASE)
#define TIM7                            ((TIM6_Type               *) TIM7_BASE)
#define DAC1                            ((DAC1_Type               *) DAC1_BASE)
#define DAC2                            ((DAC1_Type               *) DAC2_BASE)
#define DBGMCU                          ((DBGMCU_Type             *) DBGMCU_BASE)
#define TIM1                            ((TIM1_Type               *) TIM1_BASE)
#define ADC1                            ((ADC1_Type               *) ADC1_BASE)
#define ADC2                            ((ADC1_Type               *) ADC2_BASE)
#define SYSCFG_COMP_OPAMP               ((SYSCFG_COMP_OPAMP_Type  *) SYSCFG_COMP_OPAMP_BASE)
#define TIM3                            ((TIM3_Type               *) TIM3_BASE)
#define CAN                             ((CAN_Type                *) CAN_BASE)
#define ADC_Common                      ((ADC_Common_Type         *) ADC_Common_BASE)
#define HRTIM_Master                    ((HRTIM_Master_Type       *) HRTIM_Master_BASE)
#define HRTIM_TIMA                      ((HRTIM_TIMA_Type         *) HRTIM_TIMA_BASE)
#define HRTIM_TIMB                      ((HRTIM_TIMB_Type         *) HRTIM_TIMB_BASE)
#define HRTIM_TIMC                      ((HRTIM_TIMC_Type         *) HRTIM_TIMC_BASE)
#define HRTIM_TIMD                      ((HRTIM_TIMD_Type         *) HRTIM_TIMD_BASE)
#define HRTIM_TIME                      ((HRTIM_TIME_Type         *) HRTIM_TIME_BASE)
#define HRTIM_Common                    ((HRTIM_Common_Type       *) HRTIM_Common_BASE)
#define NVIC                            ((NVIC_Type               *) NVIC_BASE)
#define FPU                             ((FPU_Type                *) FPU_BASE)
#define MPU                             ((MPU_Type                *) MPU_BASE)
#define STK                             ((STK_Type                *) STK_BASE)
#define SCB                             ((SCB_Type                *) SCB_BASE)
#define NVIC_STIR                       ((NVIC_STIR_Type          *) NVIC_STIR_BASE)
#define FPU_CPACR                       ((FPU_CPACR_Type          *) FPU_CPACR_BASE)
#define SCB_ACTRL                       ((SCB_ACTRL_Type          *) SCB_ACTRL_BASE)


/** @} */ /* End of group Device_Peripheral_Registers */
/** @} */ /* End of group STM32F3x4 */
/** @} */ /* End of group (null) */

#ifdef __cplusplus
}
#endif


#endif  /* STM32F3x4_H */

