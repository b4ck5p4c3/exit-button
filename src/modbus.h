#ifndef EXIT_BUTTON_MODBUS_H
#define EXIT_BUTTON_MODBUS_H

#include <stdint.h>

enum ModbusReadError {
  MODBUS_READ_ERROR_SUCCESS,
  MODBUS_READ_ERROR_WRONG_CRC,
  MODBUS_READ_ERROR_WRONG_PACKET_SIZE,
  MODBUS_READ_ERROR_UNSUPPORTED_PACKET
};

enum ModbusPacketFunctionCode {
  MODBUS_PACKET_FUNCTION_CODE_READ_DISCRETE_INPUTS = 0x02,
  MODBUS_PACKET_FUNCTION_CODE_WRITE_SINGLE_COIL = 0x05,
  MODBUS_PACKET_FUNCTION_CODE_WRITE_MULTIPLE_COILS = 0x0F
};

enum ModbusExceptionCode {
  MODBUS_EXCEPTION_CODE_ILLEGAL_FUNCTION = 0x01,
  MODBUS_EXCEPTION_CODE_ILLEGAL_DATA_ADDRESS = 0x02
};

typedef struct {
  uint16_t address;
  uint16_t number;
} ModbusPacketReadDiscreteInputsData;

typedef struct {
  uint16_t address;
  uint16_t value;
} ModbusPacketWriteSingleCoilData;

typedef struct {
  uint16_t address;
  uint16_t number;
  uint8_t bytes;
  uint8_t data[256];
} ModbusPacketWriteMultipleCoilsData;

typedef struct {
  uint8_t slave_id;
  enum ModbusPacketFunctionCode function;
  union {
    ModbusPacketReadDiscreteInputsData read_discrete_inputs_data;
    ModbusPacketWriteSingleCoilData write_single_coil_data;
    ModbusPacketWriteMultipleCoilsData write_multiple_coils_data;
  };
} ModbusPacket;

uint8_t ReadModbusPacket(ModbusPacket* packet);
void SendModbusPacket(const uint8_t* data, uint8_t length);

#endif  //EXIT_BUTTON_MODBUS_H
