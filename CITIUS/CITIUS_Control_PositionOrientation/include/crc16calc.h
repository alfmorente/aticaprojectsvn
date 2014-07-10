/* 
 * File:   crc16calc.h
 * Author: Carlos Amores
 *
 * Created on 8 de julio de 2014, 18:43
 */

#ifndef CRC16CALC_H
#define	CRC16CALC_H

#ifdef	__cplusplus
extern "C" {
#endif




#ifdef	__cplusplus
}
#endif

#endif	/* CRC16CALC_H */

#include <stdint.h>


typedef uint16_t bit_order_16(uint16_t value);
typedef uint8_t bit_order_8(uint8_t value);

uint16_t straight_16(uint16_t);

uint8_t straight_8(uint8_t);

uint16_t crc16(uint8_t  *message, int nBytes,bit_order_8 , bit_order_16 ,uint16_t , uint16_t );

uint16_t crc16ccitt_xmodem(uint8_t  *, int );

