
/** 
 * @file  constant.h
 * @brief Declara las constantes necesarias para el manejo de la comunicacion 
 * con la camara de apoyo a la conduccion del subsistema de control
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
 *                   SOCKET PAYLOAD DE CONDUCCION
 *******************************************************************************/

#define IP_CAMERA "192.168.24.120" /// Direccion IP de la cámara
#define PORT_CAMERA 80 /// Puerto de acceso a la cámara

/*******************************************************************************
 * FRECUENCIA -> PERIODO
 ******************************************************************************/

#define FREC_10HZ 0.1 /// Periodo para ejecución de rutinas con frecuencia 10Hz
#define FREC_5HZ 0.2 /// Periodo para ejecución de rutinas con frecuencia 5Hz
#define FREC_2HZ 0.5 /// Periodo para ejecución de rutinas con frecuencia 2Hz
#define FREC_1HZ 1 /// Periodo para ejecución de rutinas con frecuencia 1Hz

/*******************************************************************************
 * MONTAJE DE COMANDOS
 ******************************************************************************/

#define AUTH_CAM_USER "root" /// User de autenticación con cámara
#define AUTH_CAM_PASS "usv" /// Pass de autenticación con cámara
#define PTZ_ROUTE "/axis-cgi/com/ptz.cgi?" /// Ruta de acceso para comandos
#define ORDER_ZOOM 0 /// Identificador de orden ZOOM
#define ORDER_PAN 1 /// Identificador de orden PAN
#define ORDER_TILT 2 /// Identificador de orden TILT

/**
 * /struct FrameDriving
 * /brief Estructura de intercambio con payload de conducción
 */
typedef struct {
  bool state; /// Valor del estado (actualizado o no) de lectura de variables
  float zoom; /// Valor de zoom
  float pan; /// Valor de pan
  float tilt; /// Valor de tilt
} LensPosition;

#endif	/* CONSTANT_H */

/**
 * @}
 */