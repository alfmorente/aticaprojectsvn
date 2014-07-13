
#include "crc16calc.h"



uint16_t straight_16(uint16_t value) {
    return value;
}

uint8_t straight_8(uint8_t value) {
    return value;
}

uint16_t crc16(uint8_t  *message, int nBytes,bit_order_8 data_order, bit_order_16 remainder_order,uint16_t remainder, uint16_t polynomial) {
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

uint16_t crc16ccitt_xmodem(uint8_t  *message, int nBytes) {
    return crc16(message, nBytes, straight_8, straight_16, 0x0000, 0x1021);
}

