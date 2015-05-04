/**
 * @file   TypeConverter.h
 * @brief  Fichero Cabecera para la gestion de conversion de tipo de datos
 * @author David Jimenez 
 * @date   2013, 2014, 2015
 * @addtogroup RearLaser
 * @{
 */

#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>


using namespace std;


//Funciones auxiliares para convertir cualquier tipo de dato a string 
string convertTostring(bool dato);
string convertTostring(char dato,bool hex,bool addSign);
string convertTostring(short dato,bool hex,bool addSign);
string convertTostring(int dato,bool hex,bool addSign);
string convertTostring(float dato);

//Funcion que recibe un string donde cada 2 caracteres es un byte. Devuelve un nuevo string donde los bytes
//estan separados por un espacio situados en la forma little endian (byte menos significativo el primero)
string divideInTwoBytes(string dato, int nbytes);

//Funcion que recibe un caracter en hezadecimal y devuelve su valor en entero
int convertToInt(char c);

//Funciones auxiliares que reciben un string hexadecimal y lo convierten en un tipo numerico (según el tipo que se quiera devolver) tal como este esta representado en en memoria 
float convertToReal(string dato);
int convertToInteger(string dato);
unsigned int  convertToUnsignedInteger(string dato);

/**
 *@}
 */