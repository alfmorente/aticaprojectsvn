/* 
 * File:   mastil.h
 * Author: atica
 *
 * Created on 4 de julio de 2014, 13:35
 */

#ifndef MASTIL_H
#define	MASTIL_H

//Envío Tramas puerto Serie
unsigned char focoDerecho [39] = {'\0'};
unsigned char focoIzquierdo [39] = {'\0'};
unsigned char tiltAbajo [19] = {'\0'};
unsigned char tiltArriba [19] = {'\0'};
unsigned char panIzquierda [19] = {'\0'};
unsigned char panDerecha [19] = {'\0'};
unsigned char off [19] = {'\0'};
unsigned char bajarMastil [39] = {'\0'};
unsigned char subirMastil [39] = {'\0'};
unsigned char stop [9] = {'\0'};

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

