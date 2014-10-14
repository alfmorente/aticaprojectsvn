
/** 
 * @file  constant.h
 * @brief Declara las constantes necesarias para el manejo de la máquina de
 * estados (modos de operación) del subsistema de control
 * @author Carlos Amores
 * @date 2013, 2014
 * @addtogroup Control Subsistema de Control
 * @{
 */

#ifndef CONSTANT_H
#define	CONSTANT_H

/*******************************************************************************
 *                              ESTADOS DEL NODO
 *******************************************************************************/

#define NODESTATUS_INIT 0 /// Identificador del estado INIT de la máquina de estados de nodo
#define NODESTATUS_OK 1 /// Identificador del estado OK de la máquina de estados de nodo
#define NODESTATUS_CORRUPT 2 /// Identificador del estado CORRUPT de la máquina de estados de nodo
#define NODESTATUS_OFF 3 /// Identificador del estado OFF de la máquina de estados de nodo

/*******************************************************************************
 *                          MODOS DE OPERACION DEL VEHICULO
 *******************************************************************************/

#define OPERATION_MODE_LOCAL 0 /// Identificador del modo de operación LOCAL de la máquina de estados del vehículo
#define OPERATION_MODE_INICIANDO 1 /// Identificador del modo de operación OPERATION_MODE_INICIANDO de la máquina de estados del vehículo
#define OPERATION_MODE_CONDUCCION 2 /// Identificador del modo de operación OPERATION_MODE_CONDUCCION de la máquina de estados del vehículo
#define OPERATION_MODE_OBSERVACION 3 /// Identificador del modo de operación OPERATION_MODE_OBSERVACION de la máquina de estados del vehículo
#define OPERATION_MODE_APAGANDO 4 /// Identificador del modo de operación OPERATION_MODE_APAGANDO de la máquina de estados del vehículo

/*******************************************************************************
 *           MAXIMO NUMERO DE INTENTOS DE CONEXION
 *******************************************************************************/

#define MAX_ATTEMPS 5 /// Náximo número de reintentos de conexión

/*******************************************************************************
 *              POSICION CONMUTADOR LOCAL - TELEOPERADOR
 *******************************************************************************/
#define SWITCHER_INIT -1 /// Valor para posición del conmutador previa lectura
#define SWITCHER_LOCAL 0 /// Valor para posición del conmutador LOCAL
#define SWITCHER_TELECONTROL 1 /// Valor para posición del conmutador TELEOPERADO

#endif	/* CONSTANT_H */

/**
 * @}
 */