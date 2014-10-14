
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
  poNodeStatus = NODESTATUS_INIT;
  gpsinsDriver = new XSensMTi700Driver();
  magnetometerDriver = new TraxAHRSModuleDriver();
  magnOK = false;
  gpsinsOK = false;
}

/**
 * Destructor de la clase. Libera punteros con referencia a los objetos de las
 * clases driver de los dispositivos
 */
RosNode_PositionOrientation::~RosNode_PositionOrientation() {
  delete gpsinsDriver;
  delete magnetometerDriver;
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
  if (gpsinsOK) {
    gpsinsDriver->configureDevice();
    ROS_INFO("[Control] Position / Orientation - Configuracion de GPS/INS establecida");
  }

  if (magnOK) {
    magnetometerDriver->configureDevice();
    ROS_INFO("[Control] Position / Orientation - Configuracion de Magnetometro establecida");
  }
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
    if (poNodeStatus != NODESTATUS_OK) {
      poNodeStatus = NODESTATUS_OK;
      rsp.confirmation = true;
    } else {
      rsp.confirmation = false;
    }

  } else if (rq.status == NODESTATUS_OFF) {
    poNodeStatus = NODESTATUS_OFF;
    rsp.confirmation = true;
  } else {
    rsp.confirmation = false;
  }

  return true;
}

/**
 * Método público consultor del atributo "poNodeStatus" de la clase que almacena 
 * el valor actual de la máquina de estados del nodo
 * @return Atributo "poNodeStatus" de la clase
 */
short RosNode_PositionOrientation::getPONodeStatus() {
  return poNodeStatus;
}

/**
 * Método público modificador del atributo "poNodeStatus" de la clase para 
 * modificar el valor de la máquina de estados del nodo
 * @param[in] newPONodeStatus Nuevo valor para el atributo "poNodeStatus"
 */
void RosNode_PositionOrientation::setPONodeStatus(short newPONodeStatus) {
  poNodeStatus = newPONodeStatus;
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
 * Método público consultor del atributo "gpsinsDriver" de la clase con 
 * referencia al objeto que gestiona el dispositivo TRAX AHRS Module
 * @return Referencia al objeto de la clase "TraxAHRSModuleDriver"
 */
TraxAHRSModuleDriver *RosNode_PositionOrientation::getMagnetometerManager() {
  return magnetometerDriver;
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
 * Método público consultor del atributo "magnOK" de la clase que indica si
 *  el dispositivo está disponible durante la ejecución del nodo
 * @return Atributo "magnOK" de la clase
 */
bool RosNode_PositionOrientation::getMagnStatus() {
  return magnOK;
}

/**
 * Método público modificador del atributo "gpsinsOK" de la clase 
 * @param[in] status Nuevo valor para el atributo "gpsinsOK"
 */
void RosNode_PositionOrientation::setGpsStatus(bool status) {
  gpsinsOK = status;
}

/**
 * Método público modificador del atributo "magOK" de la clase
 * @param[in] status Nuevo valor para el atributo "magOK"
 */
void RosNode_PositionOrientation::setMagnStatus(bool status) {
  magnOK = status;
}

/**
 * Método público que obtiene el valor de la ultima lectura de los dispositivos 
 * y publica la información en el topic correspondiente
 */
void RosNode_PositionOrientation::publishInformation() {

  if (gpsinsOK && !magnOK) {
    if (gpsinsDriver->getData()) {
      // Conversion a mensaje ROS y publicacion

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

  } else if (!gpsinsOK && magnOK) {
    if (magnetometerDriver->getData()) {
      // Conversion a mensaje ROS y publicacion
      CITIUS_Control_PositionOrientation::msg_posOriInfo msgSnd;
      TraxMeasurement information = magnetometerDriver->getInfo();

      msgSnd.positionStatus = 0;
      msgSnd.orientationStatus = information.heading_status;
      msgSnd.latitude = 0;
      msgSnd.longitude = 0;
      msgSnd.altitude = 0;
      msgSnd.roll = information.roll;
      msgSnd.pitch = information.pitch;
      msgSnd.yaw = information.heading;
      msgSnd.velX = 0;
      msgSnd.velY = 0;
      msgSnd.velZ = 0;
      msgSnd.accX = information.accX;
      msgSnd.accY = information.accY;
      msgSnd.accZ = information.accZ;
      msgSnd.rateX = information.gyrX;
      msgSnd.rateY = information.gyrY;
      msgSnd.rateZ = information.gyrZ;

      pubPosOriInfo.publish(msgSnd);
    }

  } else if (gpsinsOK && magnOK) {
    if (gpsinsDriver->getData()) {
      // Conversion a mensaje ROS y publicacion

      CITIUS_Control_PositionOrientation::msg_posOriInfo msgSnd;

      GPSINSInfo informationG = gpsinsDriver->getInfo();

      msgSnd.positionStatus = informationG.positionStatus;
      msgSnd.latitude = informationG.latitude;
      msgSnd.longitude = informationG.longitude;
      msgSnd.altitude = informationG.altitude;
      msgSnd.velX = informationG.velX;
      msgSnd.velY = informationG.velY;
      msgSnd.velZ = informationG.velZ;

      if (magnetometerDriver->getData()) {

        TraxMeasurement informationM = magnetometerDriver->getInfo();

        if (informationM.heading_status == 1) {

          msgSnd.orientationStatus = informationM.heading_status;
          msgSnd.roll = informationM.roll;
          msgSnd.pitch = informationM.pitch;
          msgSnd.yaw = informationM.heading;
          msgSnd.accX = informationM.accX;
          msgSnd.accY = informationM.accY;
          msgSnd.accZ = informationM.accZ;
          msgSnd.rateX = informationM.gyrX;
          msgSnd.rateY = informationM.gyrY;
          msgSnd.rateZ = informationM.gyrZ;

        } else {

          msgSnd.orientationStatus = informationG.orientationStatus;
          msgSnd.roll = informationG.roll;
          msgSnd.pitch = informationG.pitch;
          msgSnd.yaw = informationG.yaw;
          msgSnd.accX = informationG.accX;
          msgSnd.accY = informationG.accY;
          msgSnd.accZ = informationG.accZ;
          msgSnd.rateX = informationG.rateX;
          msgSnd.rateY = informationG.rateY;
          msgSnd.rateZ = informationG.rateZ;
        }

      } else {

        msgSnd.orientationStatus = informationG.orientationStatus;
        msgSnd.roll = informationG.roll;
        msgSnd.pitch = informationG.pitch;
        msgSnd.yaw = informationG.yaw;
        msgSnd.accX = informationG.accX;
        msgSnd.accY = informationG.accY;
        msgSnd.accZ = informationG.accZ;
        msgSnd.rateX = informationG.rateX;
        msgSnd.rateY = informationG.rateY;
        msgSnd.rateZ = informationG.rateZ;

      }

      pubPosOriInfo.publish(msgSnd);
    }

  }
}
