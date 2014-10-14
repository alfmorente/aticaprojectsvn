
/** 
 * @file  RosNode_FrontCamera.cpp
 * @brief Implementación de la clase "RosNode_FrontCamera"
 * @author Carlos Amores
 * @date 2013, 2014
 */

#include "RosNode_FrontCamera.h"

/**
 * Constructor de la clase. Inicia la maquina de estados del nodo y crea la 
 * instancia que permite la gestion de la cámara
 */
RosNode_FrontCamera::RosNode_FrontCamera() {
  fcNodeStatus = NODESTATUS_INIT;
  dFrontCamera = new AxisP3364LveDriver();
}

/**
 * Método público inicializador de artefactos ROS de la clase
 */
void RosNode_FrontCamera::initROS() {
  ros::NodeHandle nh;
  pubFrontCameraInfo = nh.advertise<CITIUS_Control_FrontCamera::msg_frontCameraInfo>("frontCameraInfo", 1000);
  subsCtrlFrontCamera = nh.subscribe("ctrlFrontCamera", 1000, &RosNode_FrontCamera::fcn_sub_ctrlFrontCamera, this);
  servNodeStatus = nh.advertiseService("fcNodeStatus", &RosNode_FrontCamera::fcv_serv_nodeStatus, this);
}

/**
 * Método privado receptor de mensajes ROS para el control de la cámara. Recibe 
 * las órdenes y las transmite hacia la cámara
 * @param[in] msg Mensaje ROS con órdenes de actuacion sobre la cámara
 */
void RosNode_FrontCamera::fcn_sub_ctrlFrontCamera(CITIUS_Control_FrontCamera::msg_ctrlFrontCamera msg) {
  if (msg.isZoom) dFrontCamera->setZoom(msg.zoom);
  if (msg.isPan) dFrontCamera->setPan(msg.pan);
  if (msg.isZoom) dFrontCamera->setTilt(msg.tilt);
}

/**
 * Método privado que recepciona las peticionesde cambios de estado del nodo. 
 * Recibe las peticiones de transición y las ejecuta segun la lógica establecida
 * @param[in] rq Parámetros de requerimiento
 * @param[in] rsp Parámetros de respuesta
 * @return Booleano que indica si se ha realizado el correcto tratamiento de
 * la petición de servicio
 */
bool RosNode_FrontCamera::fcv_serv_nodeStatus(CITIUS_Control_FrontCamera::srv_nodeStatus::Request &rq, CITIUS_Control_FrontCamera::srv_nodeStatus::Response &rsp) {
  if (rq.status == NODESTATUS_OK) {
    fcNodeStatus = NODESTATUS_OK;
    rsp.confirmation = true;
  } else if (rq.status == NODESTATUS_OFF) {
    fcNodeStatus = NODESTATUS_OFF;
    rsp.confirmation = true;
  } else {
    rsp.confirmation = false;
  }
  return true;
}

/**
 * Método público consultor del atributo "fcNodeStatus" de la clase que registra 
 * el estado actual en el que se encuentra la máquina de estados
 * @return Atributo "fcNodeStatus" de la clase
 */
short RosNode_FrontCamera::getFcNodeStatus() {
  return fcNodeStatus;
}

/**
 * Método público modificador del atributo "fcNodeStatus" de la clase
 * @param[in] newFcNodeStatus Nuevo valor para el atributo
 */
void RosNode_FrontCamera::setFcNodeStatus(short newFcNodeStatus) {
  fcNodeStatus = newFcNodeStatus;
}

/**
 * Método público consultor del atributo "pubFrontCameraInfo" de la clase con el 
 * publicador de información del estado de la cámara
 * @return Atributo "pubFrontCameraInfo"
 */
ros::Publisher RosNode_FrontCamera::getPubFrontCameraInfo() {
  return pubFrontCameraInfo;
}

/**
 * Método público consultor del atributo "dFrontCamera" de la clase con una 
 * referencia a la instancia de la clase que gestiona la comunicación con la 
 * cámara
 * @return Atributo "dFrontCamera"
 */
AxisP3364LveDriver *RosNode_FrontCamera::getDriverMng() {
  return dFrontCamera;
}