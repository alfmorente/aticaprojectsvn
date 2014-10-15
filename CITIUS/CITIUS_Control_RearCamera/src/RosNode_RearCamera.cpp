
/** 
 * @file  RosNode_RearCamera.cpp
 * @brief Implementación de la clase "RosNode_RearCamera"
 * @author Carlos Amores
 * @date 2013, 2014
 */

#include "RosNode_RearCamera.h"

/**
 * Constructor de la clase. Inicia la maquina de estados del nodo y crea la 
 * instancia que permite la gestion de la cámara
 */
RosNode_RearCamera::RosNode_RearCamera() {
  nodeStatus = NODESTATUS_INIT;
  dRearCamera = new AxisP3364LveDriver();
}

/**
 * Método público inicializador de artefactos ROS de la clase
 */
void RosNode_RearCamera::initROS() {
  ros::NodeHandle nh;
  pubRearCameraInfo = nh.advertise<CITIUS_Control_RearCamera::msg_rearCameraInfo>("rearCameraInfo", 1000);
  subsCtrlRearCamera = nh.subscribe("ctrlRearCamera", 1000, &RosNode_RearCamera::fcn_sub_ctrlRearCamera, this);
  servNodeStatus = nh.advertiseService("fcNodeStatus", &RosNode_RearCamera::fcv_serv_nodeStatus, this);
}

/**
 * Método privado receptor de mensajes ROS para el control de la cámara. Recibe 
 * las órdenes y las transmite hacia la cámara
 * @param[in] msg Mensaje ROS con órdenes de actuacion sobre la cámara
 */
void RosNode_RearCamera::fcn_sub_ctrlRearCamera(CITIUS_Control_RearCamera::msg_ctrlRearCamera msg) {
  if (msg.isZoom) dRearCamera->setZoom(msg.zoom);
  if (msg.isPan) dRearCamera->setPan(msg.pan);
  if (msg.isZoom) dRearCamera->setTilt(msg.tilt);
}

/**
 * Método privado que recepciona las peticionesde cambios de estado del nodo. 
 * Recibe las peticiones de transición y las ejecuta segun la lógica establecida
 * @param[in] rq Parámetros de requerimiento
 * @param[in] rsp Parámetros de respuesta
 * @return Booleano que indica si se ha realizado el correcto tratamiento de
 * la petición de servicio
 */
bool RosNode_RearCamera::fcv_serv_nodeStatus(CITIUS_Control_RearCamera::srv_nodeStatus::Request &rq, CITIUS_Control_RearCamera::srv_nodeStatus::Response &rsp) {
  if (rq.status == NODESTATUS_OK) {
    nodeStatus = NODESTATUS_OK;
    rsp.confirmation = true;
  } else if (rq.status == NODESTATUS_OFF) {
    nodeStatus = NODESTATUS_OFF;
    rsp.confirmation = true;
  } else {
    rsp.confirmation = false;
  }
  return true;
}

/**
 * Método público consultor del atributo "pubRearCameraInfo" de la clase con el 
 * publicador de información del estado de la cámara
 * @return Atributo "pubRearCameraInfo"
 */
ros::Publisher RosNode_RearCamera::getPubRearCameraInfo() {
  return pubRearCameraInfo;
}

/**
 * Método público consultor del atributo "dRearCamera" de la clase con una 
 * referencia a la instancia de la clase que gestiona la comunicación con la 
 * cámara
 * @return Atributo "dRearCamera"
 */
AxisP3364LveDriver *RosNode_RearCamera::getDriverMng() {
  return dRearCamera;
}