
/** 
 * @file  RosNode_PositionOrientation.cpp
 * @brief Implementación de la clase "RosNode_PositionOrientation"
 * @author Carlos Amores
 * @date 2013, 2014
 */

#include "RosNose_PositionOrientation.h"

/**
 * Constructor de la clase. Inicia la máquina de estados del nodo y crea la 
 * instancia que permite la gestión de los dispositivos
 */
RosNode_PositionOrientation::RosNode_PositionOrientation() {
  nodeStatus = NODESTATUS_INIT;
  gpsinsDriver = new XSensMTi700Driver();
  gpsinsOK = false;
}

/**
 * Destructor de la clase. Libera punteros con referencia a los objetos de las
 * clases driver de los dispositivos
 */
RosNode_PositionOrientation::~RosNode_PositionOrientation() {
  delete(gpsinsDriver);
}

/**
 * Método público inicializador de artefactos ROS de la clase
 */
void RosNode_PositionOrientation::initROS() {
  ros::NodeHandle nh;
  pubPosOriInfo = nh.advertise<CITIUS_Control_PositionOrientation::msg_posOriInfo>("posOriInfo", 1000);
  servNodeStatus = nh.advertiseService("poNodeStatus", &RosNode_PositionOrientation::fcn_serv_nodeStatus, this);
}

/**
 * Método público que solicita la configuración de los dispositivos en el 
 * supuesto de que se hayan iniciado correctamente
 */
void RosNode_PositionOrientation::configureDevices() {
  gpsinsDriver->configureDevice();
  ROS_INFO("[Control] Position / Orientation - Configuracion de GPS/INS establecida");
}

/**
 * Método privado que recepciona las peticionesde cambios de estado del nodo. 
 * Recibe las peticiones de transición y las ejecuta segun la lógica establecida
 * @param[in] rq Parámetros de requerimiento
 * @param[in] rsp Parámetros de respuesta
 * @return Booleano que indica si se ha realizado el correcto tratamiento de
 * la petición de servicio
 */
bool RosNode_PositionOrientation::fcn_serv_nodeStatus(CITIUS_Control_PositionOrientation::srv_nodeStatus::Request &rq, CITIUS_Control_PositionOrientation::srv_nodeStatus::Response &rsp) {
  if (rq.status == NODESTATUS_OK) {
    if (nodeStatus != NODESTATUS_OK) {
      nodeStatus = NODESTATUS_OK;
      rsp.confirmation = true;
    } else {
      rsp.confirmation = false;
    }
  } else if (rq.status == NODESTATUS_OFF) {
    nodeStatus = NODESTATUS_OFF;
    rsp.confirmation = true;
  } else {
    rsp.confirmation = false;
  }
  return true;
}

/**
 * Método público consultor del atributo "gpsinsDriver" de la clase con 
 * referencia al objeto que gestiona el dispositivo X-Sens MTi-G 700
 * @return Referencia al objeto de la clase "XSensMTi700Driver"
 */
XSensMTi700Driver *RosNode_PositionOrientation::getXSensManager() {
  return gpsinsDriver;
}

/**
 * Método público consultor del atributo "gpsinsOK" de la clase que indica si
 *  el dispositivo está disponible durante la ejecución del nodo
 * @return Atributo "gpsinsOK" de la clase
 */
bool RosNode_PositionOrientation::getGpsStatus() {
  return gpsinsOK;
}

/**
 * Método público modificador del atributo "gpsinsOK" de la clase 
 * @param[in] status Nuevo valor para el atributo "gpsinsOK"
 */
void RosNode_PositionOrientation::setGpsStatus(bool status) {
  gpsinsOK = status;
}

/**
 * Método público que obtiene el valor de la ultima lectura de los dispositivos 
 * y publica la información en el topic correspondiente
 */
void RosNode_PositionOrientation::publishInformation() {
    if (gpsinsDriver->getData()) {
      CITIUS_Control_PositionOrientation::msg_posOriInfo msgSnd;
      GPSINSInfo information = gpsinsDriver->getInfo();
      msgSnd.positionStatus = information.positionStatus;
      msgSnd.orientationStatus = information.orientationStatus;
      msgSnd.latitude = information.latitude;
      msgSnd.longitude = information.longitude;
      msgSnd.altitude = information.altitude;
      msgSnd.roll = information.roll;
      msgSnd.pitch = information.pitch;
      msgSnd.yaw = information.yaw;
      msgSnd.velX = information.velX;
      msgSnd.velY = information.velY;
      msgSnd.velZ = information.velZ;
      msgSnd.accX = information.accX;
      msgSnd.accY = information.accY;
      msgSnd.accZ = information.accZ;
      msgSnd.rateX = information.rateX;
      msgSnd.rateY = information.rateY;
      msgSnd.rateZ = information.rateZ;
      pubPosOriInfo.publish(msgSnd);
  }
}
