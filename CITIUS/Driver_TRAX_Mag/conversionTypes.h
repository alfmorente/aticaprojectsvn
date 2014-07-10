/* 
 * File:   conversionTypes.h
 * Author: Carlos Amores
 *
 * Created on 8 de julio de 2014, 18:45
 */

#ifndef CONVERSIONTYPES_H
#define	CONVERSIONTYPES_H

#ifdef	__cplusplus
extern "C" {
#endif




#ifdef	__cplusplus
}
#endif

#endif	/* CONVERSIONTYPES_H */

#include <stdlib.h>
#include <string.h>
#include <vector>

float hexa2float( std::vector<char> );
double hexa2double(std::vector<unsigned char> );
int hexa2int(std::vector<unsigned char> );
short hexa2short(std::vector<char> );
std::vector<char> shortToHexa(short);

