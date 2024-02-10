/********************************** (C) COPYRIGHT
 ******************************** File Name          : ch32v00x_it.c Author :
 *WCH Version            : V1.0.0 Date               : 2022/08/08 Description :
 *Main Interrupt Service Routines.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#include <ch32v00x_it.h>

void NMI_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void HardFault_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

void NMI_Handler(void) {}

void HardFault_Handler(void) {
  while (1) {
  }
}
