#ifndef EXIT_BUTTON_MCU_CONFIG_H
#define EXIT_BUTTON_MCU_CONFIG_H

#include <ch32v00x.h>

#define MODBUS_UART USART1
#define MODBUS_SPEED 115200
#define MODBUS_SLAVE_ID 69

#define TX_PORT GPIOD
#define TX_PIN GPIO_Pin_5

#define RX_PORT GPIOD
#define RX_PIN GPIO_Pin_6

#define DIR_PORT GPIOA
#define DIR_PIN GPIO_Pin_2

#define BUTTON_PORT GPIOC
#define BUTTON_PIN GPIO_Pin_4
#define BUTTON_PORT_SOURCE GPIO_PortSourceGPIOC
#define BUTTON_PIN_SOURCE GPIO_PinSource4
#define BUTTON_EXTI_LINE EXTI_Line4

#define GREEN_PORT GPIOC
#define GREEN_PIN GPIO_Pin_1

#define RED_PORT GPIOC
#define RED_PIN GPIO_Pin_2


#endif  //EXIT_BUTTON_MCU_CONFIG_H
