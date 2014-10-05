
/** 
 * @file  constant.h
 * @brief Declara las constantes necesarias para el manejo de la maquina de
 * estados (modos de operacion) del subsistema de control
 * @author: Carlos Amores
 * @date: 2013, 2014
 */

#ifndef CONSTANT_H
#define	CONSTANT_H

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

/*******************************************************************************
 *              POSICION CONMUTADOR LOCAL - TELEOPERADOR
*******************************************************************************/
#define SWITCHER_INIT -1
#define SWITCHER_LOCAL 0
#define SWITCHER_TELECONTROL 1


/*******************************************************************************
 *              REINTENTOS DE CONEXION CON DISPOSITIVOS
*******************************************************************************/

#define MAX_ATTEMPS 5

#endif	/* CONSTANT_H */

