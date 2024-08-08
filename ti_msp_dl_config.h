/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)



#define CPUCLK_FREQ                                                     32000000



/* Defines for PWM_AB */
#define PWM_AB_INST                                                        TIMA1
#define PWM_AB_INST_IRQHandler                                  TIMA1_IRQHandler
#define PWM_AB_INST_INT_IRQN                                    (TIMA1_INT_IRQn)
#define PWM_AB_INST_CLK_FREQ                                              125000
/* GPIO defines for channel 0 */
#define GPIO_PWM_AB_C0_PORT                                                GPIOB
#define GPIO_PWM_AB_C0_PIN                                         DL_GPIO_PIN_2
#define GPIO_PWM_AB_C0_IOMUX                                     (IOMUX_PINCM15)
#define GPIO_PWM_AB_C0_IOMUX_FUNC                    IOMUX_PINCM15_PF_TIMA1_CCP0
#define GPIO_PWM_AB_C0_IDX                                   DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_AB_C1_PORT                                                GPIOB
#define GPIO_PWM_AB_C1_PIN                                         DL_GPIO_PIN_3
#define GPIO_PWM_AB_C1_IOMUX                                     (IOMUX_PINCM16)
#define GPIO_PWM_AB_C1_IOMUX_FUNC                    IOMUX_PINCM16_PF_TIMA1_CCP1
#define GPIO_PWM_AB_C1_IDX                                   DL_TIMER_CC_1_INDEX



/* Defines for TIMER_10ms */
#define TIMER_10ms_INST                                                  (TIMA0)
#define TIMER_10ms_INST_IRQHandler                              TIMA0_IRQHandler
#define TIMER_10ms_INST_INT_IRQN                                (TIMA0_INT_IRQn)
#define TIMER_10ms_INST_LOAD_VALUE                                        (499U)



/* Defines for UART_0 */
#define UART_0_INST                                                        UART0
#define UART_0_INST_IRQHandler                                  UART0_IRQHandler
#define UART_0_INST_INT_IRQN                                      UART0_INT_IRQn
#define GPIO_UART_0_RX_PORT                                                GPIOA
#define GPIO_UART_0_TX_PORT                                                GPIOA
#define GPIO_UART_0_RX_PIN                                        DL_GPIO_PIN_11
#define GPIO_UART_0_TX_PIN                                        DL_GPIO_PIN_10
#define GPIO_UART_0_IOMUX_RX                                     (IOMUX_PINCM22)
#define GPIO_UART_0_IOMUX_TX                                     (IOMUX_PINCM21)
#define GPIO_UART_0_IOMUX_RX_FUNC                      IOMUX_PINCM22_PF_UART0_RX
#define GPIO_UART_0_IOMUX_TX_FUNC                      IOMUX_PINCM21_PF_UART0_TX
#define UART_0_BAUD_RATE                                                  (9600)
#define UART_0_IBRD_4_MHZ_9600_BAUD                                         (26)
#define UART_0_FBRD_4_MHZ_9600_BAUD                                          (3)





/* Port definition for Pin Group MPU6050 */
#define MPU6050_PORT                                                     (GPIOA)

/* Defines for SCL: GPIOA.0 with pinCMx 1 on package pin 1 */
#define MPU6050_SCL_PIN                                          (DL_GPIO_PIN_0)
#define MPU6050_SCL_IOMUX                                         (IOMUX_PINCM1)
/* Defines for SDA: GPIOA.1 with pinCMx 2 on package pin 2 */
#define MPU6050_SDA_PIN                                          (DL_GPIO_PIN_1)
#define MPU6050_SDA_IOMUX                                         (IOMUX_PINCM2)
/* Port definition for Pin Group GPIO_GRP_A */
#define GPIO_GRP_A_PORT                                                  (GPIOA)

/* Defines for PIN_Ain1: GPIOA.8 with pinCMx 19 on package pin 16 */
#define GPIO_GRP_A_PIN_Ain1_PIN                                  (DL_GPIO_PIN_8)
#define GPIO_GRP_A_PIN_Ain1_IOMUX                                (IOMUX_PINCM19)
/* Defines for PIN_Ain2: GPIOA.9 with pinCMx 20 on package pin 17 */
#define GPIO_GRP_A_PIN_Ain2_PIN                                  (DL_GPIO_PIN_9)
#define GPIO_GRP_A_PIN_Ain2_IOMUX                                (IOMUX_PINCM20)
/* Port definition for Pin Group GPIO_GRP_B */
#define GPIO_GRP_B_PORT                                                  (GPIOB)

/* Defines for PIN_Bin1: GPIOB.6 with pinCMx 23 on package pin 20 */
#define GPIO_GRP_B_PIN_Bin1_PIN                                  (DL_GPIO_PIN_6)
#define GPIO_GRP_B_PIN_Bin1_IOMUX                                (IOMUX_PINCM23)
/* Defines for PIN_Bin2: GPIOB.7 with pinCMx 24 on package pin 21 */
#define GPIO_GRP_B_PIN_Bin2_PIN                                  (DL_GPIO_PIN_7)
#define GPIO_GRP_B_PIN_Bin2_IOMUX                                (IOMUX_PINCM24)
/* Defines for PIN_Buzzer: GPIOA.18 with pinCMx 40 on package pin 33 */
#define GPIO_GRP_0_PIN_Buzzer_PORT                                       (GPIOA)
#define GPIO_GRP_0_PIN_Buzzer_PIN                               (DL_GPIO_PIN_18)
#define GPIO_GRP_0_PIN_Buzzer_IOMUX                              (IOMUX_PINCM40)
/* Defines for PIN_KEY1: GPIOA.12 with pinCMx 34 on package pin 27 */
#define GPIO_GRP_0_PIN_KEY1_PORT                                         (GPIOA)
#define GPIO_GRP_0_PIN_KEY1_PIN                                 (DL_GPIO_PIN_12)
#define GPIO_GRP_0_PIN_KEY1_IOMUX                                (IOMUX_PINCM34)
/* Defines for PIN_KEY2: GPIOB.18 with pinCMx 44 on package pin 37 */
#define GPIO_GRP_0_PIN_KEY2_PORT                                         (GPIOB)
#define GPIO_GRP_0_PIN_KEY2_PIN                                 (DL_GPIO_PIN_18)
#define GPIO_GRP_0_PIN_KEY2_IOMUX                                (IOMUX_PINCM44)
/* Defines for PIN_X1: GPIOA.23 with pinCMx 53 on package pin 43 */
#define GPIO_GRP_Detect_PIN_X1_PORT                                      (GPIOA)
#define GPIO_GRP_Detect_PIN_X1_PIN                              (DL_GPIO_PIN_23)
#define GPIO_GRP_Detect_PIN_X1_IOMUX                             (IOMUX_PINCM53)
/* Defines for PIN_X2: GPIOA.24 with pinCMx 54 on package pin 44 */
#define GPIO_GRP_Detect_PIN_X2_PORT                                      (GPIOA)
#define GPIO_GRP_Detect_PIN_X2_PIN                              (DL_GPIO_PIN_24)
#define GPIO_GRP_Detect_PIN_X2_IOMUX                             (IOMUX_PINCM54)
/* Defines for PIN_X3: GPIOB.24 with pinCMx 52 on package pin 42 */
#define GPIO_GRP_Detect_PIN_X3_PORT                                      (GPIOB)
#define GPIO_GRP_Detect_PIN_X3_PIN                              (DL_GPIO_PIN_24)
#define GPIO_GRP_Detect_PIN_X3_IOMUX                             (IOMUX_PINCM52)
/* Defines for PIN_X4: GPIOB.20 with pinCMx 48 on package pin 41 */
#define GPIO_GRP_Detect_PIN_X4_PORT                                      (GPIOB)
#define GPIO_GRP_Detect_PIN_X4_PIN                              (DL_GPIO_PIN_20)
#define GPIO_GRP_Detect_PIN_X4_IOMUX                             (IOMUX_PINCM48)
/* Defines for GROUND: GPIOA.28 with pinCMx 3 on package pin 3 */
#define GPIO_GRP_0_GROUND_PORT                                           (GPIOA)
#define GPIO_GRP_0_GROUND_PIN                                   (DL_GPIO_PIN_28)
#define GPIO_GRP_0_GROUND_IOMUX                                   (IOMUX_PINCM3)



/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_PWM_AB_init(void);
void SYSCFG_DL_TIMER_10ms_init(void);
void SYSCFG_DL_UART_0_init(void);

void SYSCFG_DL_SYSTICK_init(void);

bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
