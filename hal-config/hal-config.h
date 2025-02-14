#ifndef HAL_CONFIG_H
#define HAL_CONFIG_H

#include "em_device.h"
#include "hal-config-types.h"

// This file is auto-generated by Hardware Configurator in Simplicity Studio.
// Any content between $[ and ]$ will be replaced whenever the file is regenerated.
// Content outside these regions will be preserved.

// $[ACMP0]
// [ACMP0]$

// $[ACMP1]
// [ACMP1]$

// $[ANTDIV]
// [ANTDIV]$

// $[BTL_BUTTON]
// [BTL_BUTTON]$

// $[BUTTON]
#define BSP_BUTTON_PRESENT                    (1)

#define BSP_BUTTON0_PIN                       (0U)
#define BSP_BUTTON0_PORT                      (gpioPortB)

#define BSP_BUTTON1_PIN                       (1U)
#define BSP_BUTTON1_PORT                      (gpioPortB)

#define BSP_BUTTON2_PIN                       (6U)
#define BSP_BUTTON2_PORT                      (gpioPortA)

#define BSP_BUTTON_COUNT                      (3U)
#define BSP_BUTTON_INIT                       { { BSP_BUTTON0_PORT, BSP_BUTTON0_PIN }, { BSP_BUTTON1_PORT, BSP_BUTTON1_PIN }, { BSP_BUTTON2_PORT, BSP_BUTTON2_PIN } }
#define BSP_BUTTON_GPIO_DOUT                  (HAL_GPIO_DOUT_LOW)
#define BSP_BUTTON_GPIO_MODE                  (HAL_GPIO_MODE_INPUT_PULL)
#define HAL_BUTTON_COUNT                      (3U)
#define HAL_BUTTON_ENABLE                     { 0, 1, 2 }
// [BUTTON]$

// $[CMU]
#define HAL_CLK_HFCLK_SOURCE                  (HAL_CLK_HFCLK_SOURCE_HFXO)
#define HAL_CLK_PLL_CONFIGURATION             (HAL_CLK_PLL_CONFIGURATION_40MHZ)
#define HAL_CLK_EM01CLK_SOURCE                (HAL_CLK_HFCLK_SOURCE_HFRCODPLL)
#define HAL_CLK_EM23CLK_SOURCE                (HAL_CLK_LFCLK_SOURCE_LFRCO)
#define HAL_CLK_EM4CLK_SOURCE                 (HAL_CLK_LFCLK_SOURCE_LFRCO)
#define HAL_CLK_RTCCCLK_SOURCE                (HAL_CLK_LFCLK_SOURCE_LFRCO)
#define HAL_CLK_WDOGCLK_SOURCE                (HAL_CLK_LFCLK_SOURCE_LFRCO)
#define BSP_CLK_HFXO_PRESENT                  (1)
#define BSP_CLK_HFXO_FREQ                     (38400000UL)
#define BSP_CLK_HFXO_INIT                      CMU_HFXOINIT_DEFAULT
#define BSP_CLK_HFXO_CTUNE                    (-1)
#define BSP_CLK_LFXO_PRESENT                  (0)
#define BSP_CLK_LFXO_INIT                      CMU_LFXOINIT_DEFAULT
#define BSP_CLK_LFXO_FREQ                     (32768U)
#define BSP_CLK_LFXO_CTUNE                    (0U)
#define HAL_CLK_LFXO_PRECISION                (500UL)
// [CMU]$

// $[COEX]
// [COEX]$

// $[EMU]
// [EMU]$

// $[EXTFLASH]
// [EXTFLASH]$

// $[EZRADIOPRO]
// [EZRADIOPRO]$

// $[FEM]
// [FEM]$

// $[GPIO]
// [GPIO]$

// $[I2C0]
// [I2C0]$

// $[I2C1]
// [I2C1]$

// $[I2CSENSOR]
// [I2CSENSOR]$

// $[IADC0]
// [IADC0]$

// $[IOEXP]
// [IOEXP]$

// $[LED]
#define BSP_LED_PRESENT                       (1)

#define BSP_LED0_PIN                          (4U)
#define BSP_LED0_PORT                         (gpioPortC)

#define BSP_LED1_PIN                          (1U)
#define BSP_LED1_PORT                         (gpioPortC)

#define BSP_LED2_PIN                          (2U)
#define BSP_LED2_PORT                         (gpioPortC)

#define BSP_LED3_PIN                          (3U)
#define BSP_LED3_PORT                         (gpioPortC)

#define BSP_LED_COUNT                         (4U)
#define BSP_LED_INIT                          { { BSP_LED0_PORT, BSP_LED0_PIN }, { BSP_LED1_PORT, BSP_LED1_PIN }, { BSP_LED2_PORT, BSP_LED2_PIN }, { BSP_LED3_PORT, BSP_LED3_PIN } }
#define HAL_LED_COUNT                         (4U)
#define HAL_LED_ENABLE                        { 0, 1, 2 ,3  }
#define BSP_LED_POLARITY                      (0)
// [LED]$

// $[LETIMER0]
// [LETIMER0]$

// $[LFXO]
// [LFXO]$

// $[MODEM]
// [MODEM]$

// $[PA]
#define HAL_PA_ENABLE                         (1)

#define HAL_PA_CURVE_HEADER                    "pa_curves_efr32.h"
#define HAL_PA_POWER                          (252U)
#define HAL_PA_RAMP                           (10UL)
#define BSP_PA_VOLTAGE                        (3300U)
#define HAL_PA_SELECTION                      (HAL_PA_SELECTION_2P4_HP)
// [PA]$

// $[PORTIO]
// [PORTIO]$

// $[PRS]
// [PRS]$

// $[PTI]
// [PTI]$

// $[RSSI]
// [RSSI]$

// $[SERIAL]
#define BSP_SERIAL_APP_PORT                   (HAL_SERIAL_PORT_USART0)
#define HAL_SERIAL_RXWAKE_ENABLE              (0)
#define HAL_SERIAL_IDLE_WAKE_ENABLE           (1)
#define HAL_SERIAL_USART0_ENABLE              (0)
#define HAL_SERIAL_USART1_ENABLE              (0)
#define HAL_SERIAL_USART2_ENABLE              (0)
#define BSP_SERIAL_APP_TX_PIN                 (4U)
#define BSP_SERIAL_APP_TX_PORT                (gpioPortA)

#define BSP_SERIAL_APP_RX_PIN                 (5U)
#define BSP_SERIAL_APP_RX_PORT                (gpioPortA)

#define HAL_SERIAL_APP_BAUD_RATE              (115200UL)
#define HAL_SERIAL_APP_FLOW_CONTROL           (HAL_USART_FLOW_CONTROL_NONE)
#define HAL_SERIAL_APP_RXSTOP                 (16UL)
#define HAL_SERIAL_APP_RXSTART                (16UL)
#define HAL_SERIAL_APP_TX_QUEUE_SIZE          (128UL)
#define HAL_SERIAL_APP_RX_QUEUE_SIZE          (128UL)
// [SERIAL]$

// $[SPIDISPLAY]
// [SPIDISPLAY]$

// $[SPINCP]
// [SPINCP]$

// $[TIMER0]
// [TIMER0]$

// $[TIMER1]
// [TIMER1]$

// $[TIMER2]
// [TIMER2]$

// $[TIMER3]
// [TIMER3]$

// $[UARTNCP]
// [UARTNCP]$

// $[USART0]
#define PORTIO_USART0_RX_PIN                  (5U)
#define PORTIO_USART0_RX_PORT                 (gpioPortA)

#define PORTIO_USART0_TX_PIN                  (4U)
#define PORTIO_USART0_TX_PORT                 (gpioPortA)

#define HAL_USART0_ENABLE                     (1)

#define BSP_USART0_TX_PIN                     (4U)
#define BSP_USART0_TX_PORT                    (gpioPortA)

#define BSP_USART0_RX_PIN                     (5U)
#define BSP_USART0_RX_PORT                    (gpioPortA)

#define HAL_USART0_BAUD_RATE                  (115200UL)
#define HAL_USART0_FLOW_CONTROL               (HAL_USART_FLOW_CONTROL_NONE)
#define HAL_USART0_RXSTOP                     (16UL)
#define HAL_USART0_RXSTART                    (16UL)
#define HAL_USART0_TX_QUEUE_SIZE              (128UL)
#define HAL_USART0_RX_QUEUE_SIZE              (128UL)
// [USART0]$

// $[USART1]
// [USART1]$

// $[USART2]
// [USART2]$

// $[VCOM]
// [VCOM]$

// $[VUART]
// [VUART]$

// $[WDOG]
#define HAL_WDOG_ENABLE                       (1)

// [WDOG]$

#if defined(_SILICON_LABS_MODULE)
#include "sl_module.h"
#endif

#endif /* HAL_CONFIG_H */

