
/** 
 * @file  conversionTypes.h
 * @brief Declara una colección de funciones para el tratamiento de datos en
 * crudo obtenido de los dispositivos y almacenarlos en los tipos correctos
 * @author Carlos Amores
 * @date 2013, 2014
 * @addtogroup Control Subsistema de Control
 * @{
 */

#ifndef CONVERSIONTYPES_H
#define	CONVERSIONTYPES_H

#include <stdlib.h>
#include <string.h>
#include <vector>

using namespace std;

float hexa2float( std::vector<char> );
double hexa2double(std::vector<unsigned char> );
int hexa2int(std::vector<unsigned char> );
short hexa2short(std::vector<char> );
std::vector<char> shortToHexa(short);

#endif	/* CONVERSIONTYPES_H */

/**
 * @}
 */

