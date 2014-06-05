/* 
 * File:   constant.h
 * Author: Carlos Amores
 *
 * Created on 5 de junio de 2014, 9:03
 */

#ifndef CONSTANT_H
#define	CONSTANT_H

#ifdef	__cplusplus
extern "C" {
#endif




#ifdef	__cplusplus
}
#endif

#endif	/* CONSTANT_H */

/*******************************************************************************
 *                              ESTADOS DEL NODO
*******************************************************************************/

#define NODESTATUS_INIT 0
#define NODESTATUS_OK 1
#define NODESTATUS_CORRUPT 2
#define NODESTATUS_OFF 3

/*******************************************************************************
 *                          MODOS DE OPERACION DEL VEHICULO
*******************************************************************************/

#define OPERATION_MODE_LOCAL 0
#define OPERATION_MODE_INICIANDO 1
#define OPERATION_MODE_CONDUCCION 2
#define OPERATION_MODE_OBSERVACION 3
#define OPERATION_MODE_APAGANDO 4