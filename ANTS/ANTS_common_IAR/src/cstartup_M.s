/**************************************************
 *
 * Part one of the system initialization code, contains low-level
 * initialization, plain thumb variant.
 *
 * Copyright 2009 IAR Systems. All rights reserved.
 *
 * $Revision: 33181 $
 *
 **************************************************/

;
; The modules in this file are included in the libraries, and may be replaced
; by any user-defined modules that define the PUBLIC symbol _program_start or
; a user defined start symbol.
; To override the cstartup defined in the library, simply add your modified
; version to the workbench project.
;
; The vector table is normally located at address 0.
; When debugging in RAM, it can be located in RAM, aligned to at least 2^6.
; The name "__vector_table" has special meaning for C-SPY:
; it is where the SP start value is found, and the NVIC vector
; table register (VTOR) is initialized to this address if != 0.
;
; Cortex-M version
;

        MODULE  ?cstartup

        ;; Forward declaration of sections.
        SECTION CSTACK:DATA:NOROOT(3)

        SECTION .intvec:CODE:NOROOT(2)

        EXTERN  __iar_program_start
        PUBLIC  __vector_table
        PUBLIC  __vector_table_0x1c

        DATA
__vector_table
        DCD     sfe(CSTACK)                 ; Top of Stack
        DCD     __iar_program_start         ; Reset Handler
        DCD     NMI_Handler                 ; NMI Handler
        DCD     HardFault_Handler           ; Hard Fault Handler
        DCD     MemManage_Handler           ; MPU Fault Handler
        DCD     BusFault_Handler            ; Bus Fault Handler
        DCD     UsageFault_Handler          ; Usage Fault Handler
__vector_table_0x1c
        DCD     0                           ; Reserved
        DCD     0                           ; Reserved
        DCD     0                           ; Reserved
        DCD     0                           ; Reserved
        DCD     SVC_Handler                 ; SVCall Handler
        DCD     DebugMon_Handler            ; Debug Monitor Handler
        DCD     0                           ; Reserved
        DCD     PendSV_Handler              ; PendSV Handler
        DCD     SysTick_Handler             ; SysTick Handler
        DCD     WAKE_UP0_IRQHandler         ; WAKE_UP0 Handler
        DCD     WAKE_UP1_IRQHandler         ; WAKE_UP1 Handler
        DCD     WAKE_UP2_IRQHandler         ; WAKE_UP2 Handler
        DCD     WAKE_UP3_IRQHandler         ; WAKE_UP3 Handler
        DCD     WAKE_UP4_IRQHandler         ; WAKE_UP4 Handler
        DCD     WAKE_UP5_IRQHandler         ; WAKE_UP5 Handler
        DCD     WAKE_UP6_IRQHandler         ; WAKE_UP6 Handler
        DCD     WAKE_UP7_IRQHandler         ; WAKE_UP7 Handler
        DCD     WAKE_UP8_IRQHandler         ; WAKE_UP8 Handler
        DCD     WAKE_UP9_IRQHandler         ; WAKE_UP9 Handler
        DCD     WAKE_UP10_IRQHandler        ; WAKE_UP10 Handler
        DCD     WAKE_UP11_IRQHandler        ; WAKE_UP11 Handler
        DCD     WAKE_UP12_IRQHandler        ; WAKE_UP12 Handler
        DCD     WAKE_UP13_IRQHandler        ; WAKE_UP13 Handler
        DCD     WAKE_UP14_IRQHandler        ; WAKE_UP14 Handler
        DCD     WAKE_UP15_IRQHandler        ; WAKE_UP15 Handler
        DCD     WAKE_UP16_IRQHandler        ; WAKE_UP16 Handler
        DCD     WAKE_UP17_IRQHandler        ; WAKE_UP17 Handler
        DCD     WAKE_UP18_IRQHandler        ; WAKE_UP18 Handler
        DCD     WAKE_UP19_IRQHandler        ; WAKE_UP19 Handler
        DCD     WAKE_UP20_IRQHandler        ; WAKE_UP20 Handler
        DCD     WAKE_UP21_IRQHandler        ; WAKE_UP21 Handler
        DCD     WAKE_UP22_IRQHandler        ; WAKE_UP22 Handler
        DCD     WAKE_UP23_IRQHandler        ; WAKE_UP23 Handler
        DCD     WAKE_UP24_IRQHandler        ; WAKE_UP24 Handler
        DCD     WAKE_UP25_IRQHandler        ; WAKE_UP25 Handler
        DCD     WAKE_UP26_IRQHandler        ; WAKE_UP26 Handler
        DCD     WAKE_UP27_IRQHandler        ; WAKE_UP27 Handler
        DCD     WAKE_UP28_IRQHandler        ; WAKE_UP28 Handler
        DCD     WAKE_UP29_IRQHandler        ; WAKE_UP29 Handler
        DCD     WAKE_UP30_IRQHandler        ; WAKE_UP30 Handler
        DCD     WAKE_UP31_IRQHandler        ; WAKE_UP31 Handler
        DCD     WAKE_UP32_IRQHandler        ; WAKE_UP32 Handler
        DCD     WAKE_UP33_IRQHandler        ; WAKE_UP33 Handler
        DCD     WAKE_UP34_IRQHandler        ; WAKE_UP34 Handler
        DCD     WAKE_UP35_IRQHandler        ; WAKE_UP35 Handler
        DCD     WAKE_UP36_IRQHandler        ; WAKE_UP36 Handler
        DCD     WAKE_UP37_IRQHandler        ; WAKE_UP37 Handler
        DCD     WAKE_UP38_IRQHandler        ; WAKE_UP38 Handler
        DCD     WAKE_UP39_IRQHandler        ; WAKE_UP39 Handler
        DCD     I2C0_IRQHandler             ; I2C0 Handler        
        DCD     CT16B0_IRQHandler           ; CT16B0 Handler
        DCD     CT16B1_IRQHandler           ; CT16B1 Handler
        DCD     CT32B0_IRQHandler           ; CT32B0 Handler
        DCD     CT32B1_IRQHandler           ; CT32B1 Handler
        DCD     SSP0_IRQHandler             ; SSP0 Handler
        DCD     UART0_IRQHandler            ; UART0 Handler
        DCD     USBIRQ_IRQHandler           ; USB IRQ Handler
        DCD     USBFIQ_IRQHandler           ; USB FIQ Handler
        DCD     ADC_IRQHandler              ; ADC Handler
        DCD     WDT_IRQHandler              ; WDT Handler
        DCD     BOD_IRQHandler              ; BOD Handler
        DCD     0                           ; Reserved
        DCD     PIO3_IRQHandler             ; PIO3 Interupt Handler
        DCD     PIO2_IRQHandler             ; PIO2 Interupt Handler
        DCD     PIO1_IRQHandler             ; PIO1 Interupt Handler
        DCD     PIO0_IRQHandler             ; PIO0 Interupt Handler
        
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;
;; Default interrupt handlers.
;;
        
        PUBWEAK NMI_Handler
        PUBWEAK HardFault_Handler
        PUBWEAK MemManage_Handler
        PUBWEAK BusFault_Handler
        PUBWEAK UsageFault_Handler
        PUBWEAK SVC_Handler
        PUBWEAK DebugMon_Handler
        PUBWEAK PendSV_Handler
        PUBWEAK SysTick_Handler
        PUBWEAK WAKE_UP0_IRQHandler
        PUBWEAK WAKE_UP1_IRQHandler
        PUBWEAK WAKE_UP2_IRQHandler
        PUBWEAK WAKE_UP3_IRQHandler
        PUBWEAK WAKE_UP4_IRQHandler
        PUBWEAK WAKE_UP5_IRQHandler
        PUBWEAK WAKE_UP6_IRQHandler
        PUBWEAK WAKE_UP7_IRQHandler
        PUBWEAK WAKE_UP8_IRQHandler
        PUBWEAK WAKE_UP9_IRQHandler
        PUBWEAK WAKE_UP10_IRQHandler
        PUBWEAK WAKE_UP11_IRQHandler
        PUBWEAK WAKE_UP12_IRQHandler
        PUBWEAK WAKE_UP13_IRQHandler
        PUBWEAK WAKE_UP14_IRQHandler
        PUBWEAK WAKE_UP15_IRQHandler
        PUBWEAK WAKE_UP16_IRQHandler
        PUBWEAK WAKE_UP17_IRQHandler
        PUBWEAK WAKE_UP18_IRQHandler
        PUBWEAK WAKE_UP19_IRQHandler
        PUBWEAK WAKE_UP20_IRQHandler
        PUBWEAK WAKE_UP21_IRQHandler
        PUBWEAK WAKE_UP22_IRQHandler
        PUBWEAK WAKE_UP23_IRQHandler
        PUBWEAK WAKE_UP24_IRQHandler
        PUBWEAK WAKE_UP25_IRQHandler
        PUBWEAK WAKE_UP26_IRQHandler
        PUBWEAK WAKE_UP27_IRQHandler
        PUBWEAK WAKE_UP28_IRQHandler
        PUBWEAK WAKE_UP29_IRQHandler
        PUBWEAK WAKE_UP30_IRQHandler
        PUBWEAK WAKE_UP31_IRQHandler
        PUBWEAK WAKE_UP32_IRQHandler
        PUBWEAK WAKE_UP34_IRQHandler
        PUBWEAK WAKE_UP35_IRQHandler
        PUBWEAK WAKE_UP33_IRQHandler
        PUBWEAK WAKE_UP36_IRQHandler
        PUBWEAK WAKE_UP37_IRQHandler
        PUBWEAK WAKE_UP38_IRQHandler
        PUBWEAK WAKE_UP39_IRQHandler
        PUBWEAK I2C0_IRQHandler
        PUBWEAK CT16B0_IRQHandler
        PUBWEAK CT16B1_IRQHandler
        PUBWEAK CT32B0_IRQHandler
        PUBWEAK CT32B1_IRQHandler
        PUBWEAK SSP0_IRQHandler
        PUBWEAK UART0_IRQHandler
        PUBWEAK USBIRQ_IRQHandler
        PUBWEAK USBFIQ_IRQHandler
        PUBWEAK ADC_IRQHandler
        PUBWEAK WDT_IRQHandler
        PUBWEAK BOD_IRQHandler
        PUBWEAK PIO3_IRQHandler
        PUBWEAK PIO2_IRQHandler
        PUBWEAK PIO1_IRQHandler
        PUBWEAK PIO0_IRQHandler

        SECTION .text:CODE:REORDER(1)
        THUMB

NMI_Handler
HardFault_Handler
MemManage_Handler
BusFault_Handler
UsageFault_Handler
SVC_Handler
DebugMon_Handler
PendSV_Handler
SysTick_Handler
WAKE_UP0_IRQHandler
WAKE_UP1_IRQHandler
WAKE_UP2_IRQHandler
WAKE_UP3_IRQHandler
WAKE_UP4_IRQHandler
WAKE_UP5_IRQHandler
WAKE_UP6_IRQHandler
WAKE_UP7_IRQHandler
WAKE_UP8_IRQHandler
WAKE_UP9_IRQHandler
WAKE_UP10_IRQHandler
WAKE_UP11_IRQHandler
WAKE_UP12_IRQHandler
WAKE_UP13_IRQHandler
WAKE_UP14_IRQHandler
WAKE_UP15_IRQHandler
WAKE_UP16_IRQHandler
WAKE_UP17_IRQHandler
WAKE_UP18_IRQHandler
WAKE_UP19_IRQHandler
WAKE_UP20_IRQHandler
WAKE_UP21_IRQHandler
WAKE_UP22_IRQHandler
WAKE_UP23_IRQHandler
WAKE_UP24_IRQHandler
WAKE_UP25_IRQHandler
WAKE_UP26_IRQHandler
WAKE_UP27_IRQHandler
WAKE_UP28_IRQHandler
WAKE_UP29_IRQHandler
WAKE_UP30_IRQHandler
WAKE_UP31_IRQHandler
WAKE_UP32_IRQHandler
WAKE_UP33_IRQHandler
WAKE_UP34_IRQHandler
WAKE_UP35_IRQHandler
WAKE_UP36_IRQHandler
WAKE_UP37_IRQHandler
WAKE_UP38_IRQHandler
WAKE_UP39_IRQHandler
I2C0_IRQHandler
CT16B0_IRQHandler
CT16B1_IRQHandler
CT32B0_IRQHandler
CT32B1_IRQHandler
SSP0_IRQHandler
UART0_IRQHandler
USBIRQ_IRQHandler
USBFIQ_IRQHandler
ADC_IRQHandler
WDT_IRQHandler
BOD_IRQHandler
PIO3_IRQHandler
PIO2_IRQHandler
PIO1_IRQHandler
PIO0_IRQHandler
Default_Handler
       B Default_Handler

        END
