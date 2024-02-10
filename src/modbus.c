#include "modbus.h"

#include <ch32v00x.h>

#include "mcu_config.h"

#define FRAME_DELAY (SystemCoreClock / 8000000 * 1750)
#define CHARACTER_DELAY (SystemCoreClock / 8000000 * 750)

void StartSysTimer(uint32_t cmp_value) {
  SysTick->CTLR &= ~1;
  SysTick->SR &= ~1;
  SysTick->CMP = cmp_value;
  SysTick->CNT = 0;
  SysTick->CTLR |= 1;
}

uint8_t IsSysTimerTriggered() {
  return SysTick->SR & 1;
}

uint16_t ReadRawModbusPacket(uint8_t* buffer, uint16_t length) {
  uint16_t read_length = 0;

  while (USART_GetFlagStatus(MODBUS_UART, USART_FLAG_RXNE) == RESET) {}

  while (read_length < length) {
    buffer[read_length++] = USART_ReceiveData(MODBUS_UART);
    StartSysTimer(CHARACTER_DELAY);
    while (USART_GetFlagStatus(MODBUS_UART, USART_FLAG_RXNE) == RESET &&
           !IsSysTimerTriggered()) {
    }
    if (IsSysTimerTriggered()) {
      break;
    }
  }

  return read_length;
}

uint16_t CalcCRC(const uint8_t* data, uint16_t length) {
  uint16_t crc = 0xFFFF;

  while (length--) {
    crc ^= *(data++);
    for (uint8_t i = 0; i < 8; i++) {
      if (crc & 1) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }

  return crc;
}

uint8_t ReadModbusPacket(ModbusPacket* packet) {
  uint8_t buffer[256 + 16];
  uint16_t length = ReadRawModbusPacket(buffer, sizeof(buffer));

  if (length < 1 + 1 + 2) {
    return MODBUS_READ_ERROR_WRONG_PACKET_SIZE;
  }

  uint16_t crc = (uint16_t)(buffer[length - 1] << 8) | buffer[length - 2];
  uint16_t calculated_crc = CalcCRC(buffer, length - 2);

  if (calculated_crc != crc) {
    return MODBUS_READ_ERROR_WRONG_CRC;
  }

  packet->slave_id = buffer[0];
  packet->function = buffer[1];

  switch (packet->function) {
    case MODBUS_PACKET_FUNCTION_CODE_READ_DISCRETE_INPUTS:
      if (length < 1 + 1 + 2 + 2 + 2) {
        return MODBUS_READ_ERROR_WRONG_PACKET_SIZE;
      }
      packet->read_discrete_inputs_data.address =
          (uint16_t)(buffer[2] << 8) | buffer[3];
      packet->read_discrete_inputs_data.number =
          (uint16_t)(buffer[4] << 8) | buffer[5];
      break;
    case MODBUS_PACKET_FUNCTION_CODE_WRITE_SINGLE_COIL:
      if (length < 1 + 1 + 2 + 2 + 2) {
        return MODBUS_READ_ERROR_WRONG_PACKET_SIZE;
      }
      packet->write_single_coil_data.address =
          (uint16_t)(buffer[2] << 8) | buffer[3];
      packet->write_single_coil_data.value =
          (uint16_t)(buffer[4] << 8) | buffer[5];
      break;
    case MODBUS_PACKET_FUNCTION_CODE_WRITE_MULTIPLE_COILS:
      if (length < 1 + 1 + 2 + 2 + 1 + 2) {
        return MODBUS_READ_ERROR_WRONG_PACKET_SIZE;
      }
      if (length < 1 + 1 + 2 + 2 + 1 + buffer[6] + 2) {
        return MODBUS_READ_ERROR_WRONG_PACKET_SIZE;
      }
      packet->write_multiple_coils_data.address =
          (uint16_t)(buffer[2] << 8) | buffer[3];
      packet->write_multiple_coils_data.number =
          (uint16_t)(buffer[4] << 8) | buffer[5];
      packet->write_multiple_coils_data.bytes = buffer[6];
      for (uint16_t i = 0; i < packet->write_multiple_coils_data.bytes; i++) {
        packet->write_multiple_coils_data.data[i] = buffer[7 + i];
      }
      break;
    default:
      return MODBUS_READ_ERROR_UNSUPPORTED_PACKET;
  }

  return MODBUS_READ_ERROR_SUCCESS;
}

void SendModbusPacket(const uint8_t* data, uint8_t length) {
  GPIO_WriteBit(DIR_PORT, DIR_PIN, Bit_SET);
  StartSysTimer(FRAME_DELAY);
  while (!IsSysTimerTriggered()) {}
  for (uint16_t i = 0; i < length; i++) {
    USART_SendData(MODBUS_UART, data[i]);
    while (USART_GetFlagStatus(MODBUS_UART, USART_FLAG_TXE) == RESET) {}
  }
  uint16_t crc = CalcCRC(data, length);
  USART_SendData(MODBUS_UART, crc & 0xFF);
  while (USART_GetFlagStatus(MODBUS_UART, USART_FLAG_TXE) == RESET) {}
  USART_SendData(MODBUS_UART, crc >> 8);
  while (USART_GetFlagStatus(MODBUS_UART, USART_FLAG_TXE) == RESET) {}
  StartSysTimer(FRAME_DELAY);
  while (!IsSysTimerTriggered()) {}
  GPIO_WriteBit(DIR_PORT, DIR_PIN, Bit_RESET);
}
