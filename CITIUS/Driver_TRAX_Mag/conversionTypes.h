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


float hexa2float( char * );
double hexa2double(unsigned char * );
int hexa2int(unsigned char * );
short hexa2short(char[2]);
char *shortToHexa(short);

