/**
  @file CksFrame.cpp
  @brief Implementación de la colección de funciones que calculan CRC32
  @author Carlos Amores
  @date 2013,2014,2015
*/

#include "Modulo_GPS/CksFrame.h"

/**
 * Método auxiliar que calcula el valor de checksum CRC32 de un entero
 * @param[i] i Entero cuyo CRC32 se desea calcular
 * @return Valor del CRC32 calculado sobre un entero
 */
unsigned long CRC32Value(int i)
{
  int j;
  unsigned long ulCRC;
  ulCRC = i;
  for (j = 8; j > 0; j--)
  {
    if (ulCRC & 1)
      ulCRC = (ulCRC >> 1)^CRC32_POLYNOMIAL;
    else ulCRC >>= 1;
  }
  return ulCRC;
}

/**
 * Método principal que calcula el CRC32 de una cadena mediante la descomposición
 * estándar de la misma
 * @param ulCount Contador auxiliar de cálculo
 * @param ucBuffer Cadena cuyo CRC32 se desea calcular
 * @return Valor del CRC32 calculado sobre una cadena
 */
unsigned long CalculateBlockCRC32(unsigned long ulCount,unsigned char *ucBuffer)
{
  unsigned long ulTemp1;
  unsigned long ulTemp2;
  unsigned long ulCRC = 0;
  while (ulCount-- != 0)
  {
    ulTemp1 = (ulCRC >> 8) & 0x00FFFFFFL;
    ulTemp2 = CRC32Value(((int) ulCRC^*ucBuffer++)&0xff);
    ulCRC = ulTemp1^ulTemp2;
  }
  return (ulCRC);
}
