
/** 
 * @file  constant.h
 * @brief Declara las constantes necesarias para el manejo de la comunicacion 
 * con la camara de apoyo a la conduccion del subsistema de control
 * @author: Carlos Amores
 * @date: 2013, 2014
 */

#ifndef CONSTANT_H
#define	CONSTANT_H

/*******************************************************************************
 * ESTADO DE NODO
 ******************************************************************************/

#define NODESTATUS_INIT 0
#define NODESTATUS_OK 1
#define NODESTATUS_CORRUPT 2
#define NODESTATUS_OFF 3

/*******************************************************************************
 * FRECUENCIA DE REQUERIMIENTO DE INFORMACION A DISPOSITIVO
 ******************************************************************************/

#define FREC_10HZ 0.1
#define FREC_5HZ 0.2
#define FREC_2HZ 0.5
#define FREC_1HZ 1

#endif	/* CONSTANT_H */


