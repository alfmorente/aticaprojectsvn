
/** 
 * @file  conversionTypes.cpp
 * @brief Implementacion de coleccion de funciones para conversion de tipos 
 * utilizada por la clase TraxAHRSModuleDriver
 * @author: Carlos Amores
 * @date: 2013, 2014
 */

#include "conversionTypes.h"

/**
 * Funcion de conversion de tipos. Convierte un vector de bytes en un float
 * @param[in] buffer[in] Datos en crudo a convertir
 * @return Float resultado de la conversion
 */
float hexa2float(vector<char> buffer) {

  union {
    float value;
    unsigned char buffer[4];

  } floatUnion;

  floatUnion.buffer[0] = buffer[3];
  floatUnion.buffer[1] = buffer[2];
  floatUnion.buffer[2] = buffer[1];
  floatUnion.buffer[3] = buffer[0];

  return floatUnion.value;
}

/**
 * Funcion de conversion de tipos. Convierte un vector de bytes en un int
 * @param[in] buffer[in] Datos en crudo a convertir
 * @return Int resultado de la conversion
 */
int hexa2int(std::vector<unsigned char> buffer) {

  union {
    int value;
    unsigned char buffer[4];

  } intUnion;

  intUnion.buffer[0] = buffer[3];
  intUnion.buffer[1] = buffer[2];
  intUnion.buffer[2] = buffer[1];
  intUnion.buffer[3] = buffer[0];

  return intUnion.value;
}

/**
 * Funcion de conversion de tipos. Convierte un vector de bytes en un short
 * @param[in] buffer[in] Datos en crudo a convertir
 * @return Short resultado de la conversion
 */
short hexa2short(vector<char> buffer) {

  union {
    short value;
    unsigned char buffer[2];

  } shortUnion;

  shortUnion.buffer[0] = buffer[1];
  shortUnion.buffer[1] = buffer[0];

  return shortUnion.value;
}

/**
 * Funcion de conversion de tipos. Convierte un vector de bytes en un double
 * @param[in] buffer[in] Datos en crudo a convertir
 * @return Double resultado de la conversion
 */
double hexa2double(std::vector<unsigned char> buffer) {

  union {
    double value;
    unsigned char buffer[8];
  } doubleUnion;

  doubleUnion.buffer[0] = buffer[7];
  doubleUnion.buffer[1] = buffer[6];
  doubleUnion.buffer[2] = buffer[5];
  doubleUnion.buffer[3] = buffer[4];
  doubleUnion.buffer[4] = buffer[3];
  doubleUnion.buffer[5] = buffer[2];
  doubleUnion.buffer[6] = buffer[1];
  doubleUnion.buffer[7] = buffer[0];

  return doubleUnion.value;
}

/**
 * Funcion de conversion de tipos. Convierte un dato de tipo short a una cadena
 * de bytes
 * @param[in] buffer[in] Dato a convertir en bytes
 * @return Vector de bytes resultado de la conversion
 */
vector<char> shortToHexa(short s) {
  char *buf = (char *) malloc(2);
  vector<char> out;
  memcpy(buf, &s, 2);
  out.push_back(buf[1]);
  out.push_back(buf[0]);
  free(buf);
  return out;

}

