
/** 
 * @file  constant.h
 * @brief Declara las constantes necesarias para el manejo de la comunicación 
 * con los dispositivos encargados de obtener la posición y orientación del
 * vehículo
 * @author Carlos Amores
 * @date 2013, 2014
 * @addtogroup Control Subsistema de Control
 * @{
 */

#ifndef CONSTANT_H
#define	CONSTANT_H

#include <vector>

/*******************************************************************************
 * ESTADOS DEL NODO
 *******************************************************************************/

/**
 * \enum NodeStatus
 * \brief Tipos para estado local de nodo
 */
typedef enum{
  NODESTATUS_INIT = 0 , /// Identificador del estado INIT de la máquina de estados de nodo
  NODESTATUS_OK = 1 , /// Identificador del estado OK de la máquina de estados de nodo
  NODESTATUS_CORRUPT = 2 , /// Identificador del estado CORRUPT de la máquina de estados de nodo
  NODESTATUS_OFF = 3 /// Identificador del estado OFF de la máquina de estados de nodo
}NodeStatus;

/*******************************************************************************
 * FRECUENCIA -> PERIODO
 ******************************************************************************/

#define FREC_30HZ 0.03 /// Periodo para ejecución de rutinas con frecuencia 30Hz
#define FREC_25HZ 0.04 /// Periodo para ejecución de rutinas con frecuencia 25Hz
#define FREC_10HZ 0.1 /// Periodo para ejecución de rutinas con frecuencia 10Hz
#define FREC_5HZ 0.2 /// Periodo para ejecución de rutinas con frecuencia 5Hz
#define FREC_2HZ 0.5 /// Periodo para ejecución de rutinas con frecuencia 2Hz
#define FREC_1HZ 1 /// Periodo para ejecución de rutinas con frecuencia 1Hz

/*******************************************************************************
 * ID FRAME (TRAX)
 ******************************************************************************/

#define IDFRAME_KGETMODINFO 0x01 /// Identificador de trama (Trax) kGetModInfo
#define IDFRAME_KGETMODINFORESP 0x02  /// Identificador de trama (Trax) kGetModInfoResp
#define IDFRAME_KSETDATACOMPONENTS 0x03  /// Identificador de trama (Trax) kSetDataComponents
#define IDFRAME_KGETDATA 0x04 /// Identificador de trama (Trax) kGetData
#define IDFRAME_KGETDATARESP 0x05 /// Identificador de trama (Trax) kGetDataInfo

/*******************************************************************************
 * ID MEASURE (TRAX)
 ******************************************************************************/

#define IDMEASURE_HEADING 0x05 /// Identificador en trama (Trax) de Heading
#define IDMEASURE_PITCH 0x18 /// Identificador en trama (Trax) de Pitch
#define IDMEASURE_ROLL 0x19 /// Identificador en trama (Trax) de Roll
#define IDMEASURE_HEADING_STATUS 0x4F /// Identificador en trama (Trax) de Heading Status
#define IDMEASURE_ACCX 0x15 /// Identificador en trama (Trax) de AccX
#define IDMEASURE_ACCY 0x16 /// Identificador en trama (Trax) de AccY
#define IDMEASURE_ACCZ 0x17 /// Identificador en trama (Trax) de AccZ
#define IDMEASURE_GYRX 0x4A /// Identificador en trama (Trax) de GyrX
#define IDMEASURE_GYRY 0x4B /// Identificador en trama (Trax) de GyrY
#define IDMEASURE_GYRZ 0x4C /// Identificador en trama (Trax) de GyrZ

/*******************************************************************************
 * ESTRUCTURA MANEJO DE MAGNETOMETRO (TRAX)
 ******************************************************************************/

/**
 * \struct PacketFrame
 * \brief Estructura contenedor de payload's Trax en crudo
 */
typedef struct {
  char idFrame; /// Identificador de la trama
  std::vector<char> payload; /// Valor del campo payload de la trama en crudo
} PacketFrame;

/**
 * \struct TraxMsg
 * \brief Estructura contenedor de distintos campos de las tramas Trax
 */
typedef struct {
  std::vector<char> byteCount; /// Valor del campo byteCount de la trama en crudo
  PacketFrame packFrame; /// Trama semidesempaquetada
  std::vector<char> crc; /// Valor del CRC16 en crudo
  bool checked; /// Indicador de tratamiento de la trama finalizado
} TraxMsg;

/**
 * \struct TraxMeasurement
 * \brief Estructura contenedor de las medidas obtenidas de una trama Trax
 */
typedef struct {
  char heading_status; /// Valor del estado de la orientacion obtenida
  float heading; /// Valor de la orientacion en yaw (Z)
  float pitch; /// Valor de la orientacion en pitch (X)
  float roll; /// Valor de la orientacion en roll (Y)
  float accX; /// Aceleracion longitudinal componente X
  float accY; /// Aceleracion longitudinal componente Y
  float accZ; /// Aceleracion longitudinal componente Z
  float gyrX; /// Velocidad rotacional componente X
  float gyrY; /// Velocidad rotacional componente Y
  float gyrZ; /// Velocidad rotacional componente Z
} TraxMeasurement;

/*******************************************************************************
 * IDENTIFICACION DE TRAMAS (XSENS)
 ******************************************************************************/

#define COMMAND_PRE 0xFA /// Identificador de campo PRE
#define COMMAND_BID 0xFF /// Identificador de campo BID
#define COMMAND_MID_GOTOCONFIG 0x30 /// Identificador de campo MID GoToConfig
#define COMMAND_MID_GOTOMEASUREMENT 0x10 /// Identificador de campo MID GoToMeasurement
#define COMMAND_MID_SETOUTPUTMODE 0xD0 /// Identificador de campo MID SetOutPutMode
#define COMMAND_MID_SETOUTPUTSETTINGS 0xD2 /// Identificador de campo MID SetOutPutSettings
#define COMMAND_MID_REQDID 0x00 /// Identificador de campo MID ReqID
#define COMMAND_MID_SETOUTPUTSKIPFACTOR 0xD4 /// Identificador de campo MID SetOutPutSkipFactor
#define COMMAND_MID_SETPERIOD 0x04 /// Identificador de campo MID SetPeriod
#define COMMAND_MID_SETOUTPUTCONFIGURATION 0xC0 /// Identificador de campo MID SetOutPutConfiguration
#define COMMAND_MID_MTDATA2 0x36 /// Identificador de campo MID MTData2
#define COMMAND_LEN_0 0x00 /// Identificador de campo LEN
#define FREC_REQ_DATA 0x0A  /// Identificador para frecuencia de solicitud de datos

/*******************************************************************************
 * ESTRUCTURA MANEJO DE GPS+IMU (XSENS)
 ******************************************************************************/

/**
 * \struct GPSINSInfo
 * \brief Estructura contenedor de las medidas obtenidas de una trama de datos
 * XSens
 */
typedef struct{
  // Estado
  short positionStatus; /// Valor del estado de la posicion
  short orientationStatus; /// Valor del estado de la orientacion
  // Posicion
  double latitude; /// Valor de posicion en latitud
  double longitude; /// Valor de la posicion en longitud
  float altitude; /// Valor de la posicion en altitud
  // Orientacion
  float roll; /// Valor de la orientacion en roll (Y)
  float pitch; /// Valor de la orientacion en pitch (X)
  float yaw; /// Valor de la orientacion en yaw (Z)
  // Velocidad longitudinal
  float velX; /// Valor de la velocidad longitudinal componente X
  float velY; /// Valor de la velocidad longitudinal componente Y
  float velZ; /// Valor de la velocidad longitudinal componente Z
  // Acc longitudinal
  float accX; /// Valor de la aceleracion longitudinal componente X
  float accY; /// Valor de la aceleracion longitudinal componente Y
  float accZ; /// Valor de la aceleracion longitudinal componente Z
  // Acc rotacional
  float rateX; /// Valor de la aceleracion rotacional componente X
  float rateY; /// Valor de la aceleracion rotacional componente Y
  float rateZ; /// Valor de la aceleracion rotacional componente Z
} GPSINSInfo;

/**
 * \struct XsensMsg
 * \brief Estructura para descomposición de trama XSens
 */
typedef struct {
    unsigned char pre; /// Valor en crudo del campo PRE de la trama
    unsigned char bid; /// Valor en crudo del campo BID de la trama
    unsigned char mid; /// Valor en crudo del campo MID de la trama
    unsigned char len; /// Valor en crudo del campo LEN de la trama
    std::vector<unsigned char> data; /// Datos en crudo de la trama
    unsigned char cs; /// Valor en crudo del campo CS de la trama
} XsensMsg;

/**
 * \struct dataPacketMT2
 * \brief Estructura para descomposición de campo Data de las tramas de datos
 * XSens
 */
typedef struct {
  unsigned char idGroup; /// Valor del identificador del grupo de datos (MTData2)
  unsigned char idSignal; /// Valor del identificador de la señal (MTData2)
  unsigned char len; /// Valor del campo longitud (MTData2)
  std::vector<unsigned char> data; /// Valor de los datos en crudo (MTData2)
} dataPacketMT2;

#endif	/* CONSTANT_H */

/**
 * @}
 */