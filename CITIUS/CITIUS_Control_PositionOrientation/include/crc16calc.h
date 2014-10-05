
/** 
 * @file  crc16calc.h
 * @brief Declara una coleccion de funciones para el calculo del CRC16 necesario
 * para verificar los mensajes obtenidos de los dispositivos
 * @date: 2013, 2014
 */

#ifndef CRC16CALC_H
#define	CRC16CALC_H

#include <stdint.h>

typedef uint16_t bit_order_16(uint16_t value);
typedef uint8_t bit_order_8(uint8_t value);

#endif	/* CRC16CALC_H */

uint16_t straight_16(uint16_t);
uint8_t straight_8(uint8_t);
uint16_t crc16(uint8_t  *message, int nBytes,bit_order_8 , bit_order_16 ,uint16_t , uint16_t );
uint16_t crc16ccitt_xmodem(uint8_t  *, int );

