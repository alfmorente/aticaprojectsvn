/* 
 * File:   mastil.h
 * Author: atica
 *
 * Created on 4 de julio de 2014, 13:35
 */

#ifndef MASTIL_H
#define	MASTIL_H

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h> 
#include <string.h>



void datosFocoDerecho();
void datosFocoIzquierdo();
void datosTiltAbajo();
void datosTiltArriba();
void datosPanIzquierda();
void datosPanDerecha();
void datosOff();
void datosBajarMastil();
void datosSubirMastil();
void datosStop();


#endif	/* MASTIL_H */

