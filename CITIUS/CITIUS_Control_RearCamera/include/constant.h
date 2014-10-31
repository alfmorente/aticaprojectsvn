
/** 
 * @file  constant.h
 * @brief Declara las constantes necesarias para el manejo de la comunicacion 
 * con la camara de apoyo a la conduccion del subsistema de control
 * @author: Carlos Amores
 * @date: 2013, 2014
 * @addtogroup RearCameraDriver 
 * @{
 */

#ifndef CONSTANT_H
#define	CONSTANT_H

/*******************************************************************************
 *                              ESTADOS DEL NODO
 *******************************************************************************/

/**
 * \enum NodeStatus
 * \brief Tipos para estado local de nodo
 */
typedef enum {
  NODESTATUS_INIT = 0, ///<Identificador del estado INIT de la máquina de estados de nodo
  NODESTATUS_OK = 1, ///<Identificador del estado OK de la máquina de estados de nodo
  NODESTATUS_CORRUPT = 2, ///<Identificador del estado CORRUPT de la máquina de estados de nodo
  NODESTATUS_OFF = 3 ///<Identificador del estado OFF de la máquina de estados de nodo
} NodeStatus;

/*******************************************************************************
 * FICHERO DE CONFIGURACION
 ******************************************************************************/

#define CONFIG_FILE_IP_NAME "IP" ///<Identificador de búsqueda en fichero de IP
#define CONFIG_FILE_PORT_NAME "PORT" ///<Identificador de búsqueda en fichero de PORT

/*******************************************************************************
 * FRECUENCIA -> PERIODO
 ******************************************************************************/

#define FREC_10HZ 0.1 ///<Periodo para ejecución de rutinas con frecuencia 10Hz
#define FREC_5HZ 0.2 ///<Periodo para ejecución de rutinas con frecuencia 5Hz
#define FREC_2HZ 0.5 ///<Periodo para ejecución de rutinas con frecuencia 2Hz
#define FREC_1HZ 1 ///<Periodo para ejecución de rutinas con frecuencia 1Hz

/*******************************************************************************
 * MONTAJE DE COMANDOS
 ******************************************************************************/

#define AUTH_CAM_USER "root" ///<User de autenticación con cámara
#define AUTH_CAM_PASS "usv" ///<Pass de autenticación con cámara
#define PTZ_ROUTE "/axis-cgi/com/ptz.cgi?" ///<Ruta de acceso para comandos
#define ORDER_ZOOM 0 ///<Identificador de orden ZOOM
#define ORDER_PAN 1 ///<Identificador de orden PAN
#define ORDER_TILT 2 ///<Identificador de orden TILT

/*******************************************************************************
 * CONVERSION PTZ CAMARA
 ******************************************************************************/

#define CONV_TO_CAMERA 1/50 ///<Factor de conversion entre PTZ porcentual y valores de cámara
#define CONV_FROM_CAMERA 50 ///<Factor de conversion entre  valores de cámara y PTZ porcentual

/**
 * \struct FrameDriving
 * \brief Estructura de intercambio con payload de conducción
 */
typedef struct {
  bool state; ///<Valor del estado (actualizado o no) de lectura de variables
  float zoom; ///<Valor de zoom
  float pan; ///<Valor de pan
  float tilt; ///<Valor de tilt
} LensPosition;

#endif	/* CONSTANT_H */

/**
 * @}
 */