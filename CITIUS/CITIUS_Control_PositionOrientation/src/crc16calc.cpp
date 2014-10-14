
/** 
 * @file  crc16calc.h
 * @brief Declara una colección de funciones para la obtención del campo CRC16 
 * a partir de los datos en crudo de una trama
 * @date: 2013, 2014
 */

#include "crc16calc.h"

/**
 * Función auxiliar sin efecto necesaria para la implementacion de otras
 * @param[in] value Valor de entrada 
 * @return Valor de salida
 */
uint16_t straight_16(uint16_t value) {
  return value;
}

/**
 * Función auxiliar sin efecto necesaria para la implementacion de otras
 * @param[in] value Valor de entrada 
 * @return Valor de salida
 */
uint8_t straight_8(uint8_t value) {
  return value;
}

/**
 * Función que calcula el CRC16 en base al mensaje íntegro
 * @param[in] message Mensaje del que calcula el CRC16
 * @param[in] nBytes Longitud del mensaje en bytes
 * @param[in] data_order Sin importancia
 * @param[in] remainder_order Sin importancia
 * @param[in] remainder Sin importancia
 * @param[in] polynomial Sin importancia
 * @return Valor del CRC16 calculado
 */
uint16_t crc16(uint8_t *message, int nBytes, bit_order_8 data_order, bit_order_16 remainder_order, uint16_t remainder, uint16_t polynomial) {
  for (int byte = 0; byte < nBytes; ++byte) {
    remainder ^= (data_order(message[byte]) << 8);
    for (uint8_t bit = 8; bit > 0; --bit) {
      if (remainder & 0x8000) {
        remainder = (remainder << 1) ^ polynomial;
      } else {
        remainder = (remainder << 1);
      }
    }
  }
  return remainder_order(remainder);
}

/**
 * Función que calcula el CRC16 (CCITT Xmodem) en base al mensaje íntegro
 * @param[in] message Mensaje del que calcula el CRC16
 * @param[in] nBytes Longitud del mensaje en bytes
 * @return Valor del CRC16 calculado
 */
uint16_t crc16ccitt_xmodem(uint8_t *message, int nBytes) {
  return crc16(message, nBytes, straight_8, straight_16, 0x0000, 0x1021);
}

