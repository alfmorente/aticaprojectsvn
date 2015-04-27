/**
 * @file   TypeConverter.cpp
 * @brief  Fichero fuente de gestion de conversion de tipo de datos
 * @author David Jimenez 
 * @date   2013, 2014, 2015
 */
#include <Modulo_Laser2D_Rear/TypeConverter.h>
#include <stdio.h>
#include <stdlib.h>

/**
 * Metodo para convertir de booleano a string
 * @param[in] dato Booleano a convertir
 * @return String convertido
 */
/** convertTostring: 
    Parametros de entrada: 
	-dato: booleano para ser convertido a string
    Parametro de salida: string con el valor del booleano
**/
string convertTostring(bool dato)
{
	string dataString;
	stringstream auxiliar;
	auxiliar << dato;
	auxiliar >> dataString;

	return dataString;

}

/**
 * Metodo para convertir un short a string
 * @param[in] dato Short a convertir
 * @param[in] hex Indica si el short esta en hexadecimal
 * @param[in] addSign Indica si hace falta añadirle signo al string
 * @return String convertido
 */
/** convertTostring: 
    Parametros de entrada: 
	-dato: short para ser convertido a string
	-hex: Indica si el short esta en hexadecimal
	-addSign: Indica si hay que añadirle signo al string
    Parametro de salida: string con el valor del short
**/
string convertTostring(short dato,bool hex,bool addSign)
{
	string dataString;

	if(!hex)
	{
		stringstream auxiliar;
		if(addSign && dato >=0)
			auxiliar <<  '+';
		auxiliar << dato;
		auxiliar >> dataString;
	}
	else
	{
		char* aux=new char[4];
		sprintf(aux,"%X ",dato);		
		dataString.append(aux);
		delete aux;
	}
	return dataString;

}

/**
 * Metodo para convertir un float a string
 * @param[in] dato Float a convertir
 * @param[in] addSign Indica si hay que añadirle signo al string
 * @return String convertido
 */
/** convertTostring: 
    Parametros de entrada: 
	-dato: float para ser convertido a stringl
	-addSign: Indica si hay que añadirle signo al string
    Parametro de salida: string con el valor del float
**/
string convertTostring(float dato,bool addSign)
{
	string dataString;
	stringstream auxiliar;
	if(addSign && dato >=0)
		auxiliar <<  '+';
	auxiliar << dato;
	auxiliar >> dataString;
	
	
	return dataString;

}

/**
 * Metodo para convertir un int  a string
 * @param[in] dato Entero a convertir
 * @param[in] addSign Indica si hay que añadirle signo al string
 * @return String convertido
 */
/** convertTostring: 
    Parametros de entrada: 
	-dato: int para ser convertido a string
	-hex: Indica si el short esta en hexadecimal
	-addSign: Indica si hay que añadirle signo al string
    Parametro de salida: string con el valor del int
**/
string convertTostring(int dato,bool hex,bool addSign)
{
	
	string dataString;

	if(!hex)
	{
		stringstream auxiliar;
		if(addSign && dato >=0)
			auxiliar <<  '+';
		auxiliar << dato;
		auxiliar >> dataString;
	}
	else
	{
		char* aux=new char[4];
		sprintf(aux,"%X",dato);
		dataString.append(aux);
		delete aux;
	}
	
		
	return dataString;

}

/**
 * Metodo para convertir un char a string
 * @param[in] dato Char a convertir
 * @param[in] addSign Indica si hay que añadirle signo al string
 * @return String convertido
 */
/** convertTostring: 
    Parametros de entrada: 
	-dato: char para ser convertido a string
	-hex: Indica si el short esta en hexadecimal
	-addSign: Indica si hay que añadirle signo al string
    Parametro de salida: string con el valor del char
**/
string convertTostring(char dato,bool hex,bool addSign)
{
	string dataString;
	
	if(!hex)
	{
		stringstream auxiliar;
		if(addSign && dato >=0)
			auxiliar <<  '+';
		auxiliar << (short)dato;
		auxiliar >> dataString;
	}
	else
	{
		char* aux=new char[4];
		sprintf(aux,"%X",dato);
		dataString.append(aux);
		delete aux;
	}
	return dataString;

}

/**
 * Metodo para dividir un string en otro con distinto numero de bytes
 * @param[in] dato String a ser dividido
 * @param[in] nBytes Numero de bytes en los que quiere ser dividido
 * @return String dividido
 */
/** divideInTwoBytes: 
    Parametros de entrada: 
	-dato: string para ser dividido
	-nBytes: numero de bytes en los que se quiere dividir
    Parametro de salida: string con el valor recibido dividido en dicho numero de bytes.

   Ejemplo: si recibimos "0132" y queremos dividirlo en dos bytes el valor devuelto es "32 01" 
   Si quisieramos dividirlo en 4 bytes seria "32 01 00 00" 
**/
string divideInTwoBytes(string dato,int nBytes)
{
	char byte[2];
	int numBytes=0;
	string aux;
	int j=2;
	int i=dato.length()-1;


	while(i>=0)
	{
		if(j!=0)
		{
			byte[j-1]=dato[i];
			i--;
			j--;
		}

		else
		{
			numBytes++;
			aux +=byte[0];
			aux +=byte[1];
			aux +=" ";
			j=2;		
		}
	}

	//Si aun no he copiado el ultimo bytes y se ha terminado la cadena
	if(j!=2)
	{
		if(j==1)
		{	
			numBytes++;
			aux += "0";
			aux +=byte[1];
		}
		if(j==0)
		{	
			numBytes++;
			aux +=byte[0];
			aux +=byte[1];
		}
		
	}


	while(numBytes < nBytes)
	{
		aux += " ";
		aux += "0";
		aux += "0";
		numBytes++;
	}

	return aux;
		
}

/**
 * Metodo que convierte un char en hexadecimal en un entero
 * @param[in] c Char en hexadecimal a convertir
 * @return Entero en decimal
 */
/** convertToInt: 
    Parametros de entrada: 
	-c: caracter de un numero en hexadecimal 0..F
    Parametro de salida: Valor numérico de ese caracter en decimal.
	Ejemplo: Si recibimos F devolveriamos 15.
**/
int convertToInt(char c)
{
	switch(c)
	{
		case '0':
			return 0;
		case '1':
			return 1;
		case '2':
			return 2;
		case '3':
			return 3;
		case '4':
			return 4;
		case '5':
			return 5;
		case '6':
			return 6;
		case '7':
			return 7;
		case '8':
			return 8;
		case '9':
			return 9;
		case 'A':
			return 10;
		case 'B':
			return 11;
		case 'C':
			return 12;
		case 'D':
			return 13;
		case 'E':
			return 14;
		case 'F':
			return 15;
		default:
			return -1;
	}
}

/**
 * Metodo que convierte un string en un float
 * @param[in] dato String a convertir
 * @return Float convertido
 */
/** convertToReal: 
    Parametros de entrada: 
	-dato: string en hexadecimal para ser convertido
    Parametro de salida: Valor numerico del string si estuviera guardado como un tipo float
**/
float convertToReal(string dato)
{
		
	float res;
	
	if((dato.length()%2)!=0)
	{
		string aux;
		aux += "0";
		aux += dato;
		dato=aux;
	}

	unsigned char c[4]={0,0,0,0};
	for(unsigned int i=0;i<dato.length();i+=2)
	{
		c[i/2]=convertToInt(dato[dato.length()-i-2])*16+ convertToInt(dato[dato.length()-i-1]);
	}
	memcpy(&res,c,4);
	return res;

}

/**
 * Metodo que convierte un string en un entero
 * @param[in] dato String a convertir
 * @return Entero convertido
 */
/** convertToReal: 
    Parametros de entrada: 
	-dato: string en hexadecimal para ser convertido
    Parametro de salida: Valor numerico del string si estuviera guardado como un tipo int

    Nota: Esta funcion tambien sirve para convertirlo en tipo short o char ya que el
	  int es lo mismo pero rellenado con ceros
**/
int convertToInteger(string dato)
{
		
	int res;

	
	if((dato.length()%2)!=0)
	{
		string aux;
		aux += "0";
		aux += dato;
		dato=aux;
	}

	unsigned char c[4]={0,0,0,0};
	for(unsigned int i=0;i<dato.length();i+=2)
	{
		c[i/2]=convertToInt(dato[dato.length()-i-2])*16+ convertToInt(dato[dato.length()-i-1]);
	}
	memcpy(&res,c,4);
	return res;

}

/**
 * Metodo que convierte un string en un entero sin signo
 * @param[in] dato String a convertir
 * @return Entero sin signo convertido
 */
/** convertToReal: 
    Parametros de entrada: 
	-dato: string en hexadecimal para ser convertido
    Parametro de salida: Valor numerico del string si estuviera guardado como un tipo unsigned int

     Nota: Esta funcion tambien sirve para convertirlo en tipo unsigned short o unsigned char ya que el
	   unsigned int es lo mismo pero rellenado con ceros
**/
unsigned int convertToUnsignedInteger(string dato)
{


	unsigned int res;
	if((dato.length()%2)!=0)
	{
		string aux;
		aux += "0";
		aux += dato;
		dato=aux;
	}


	unsigned char c[4]={0,0,0,0};
	for(unsigned int i=0;i<dato.length();i+=2)
	{
		c[i/2]=convertToInt(dato[dato.length()-i-2])*16 + convertToInt(dato[dato.length()-i-1]);
	}
	

	memcpy(&res,c,4);
	return res;

}

