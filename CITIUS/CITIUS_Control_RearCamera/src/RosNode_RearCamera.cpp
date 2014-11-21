
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
 * Destructor de la clase
 */
RosNode_RearCamera::~RosNode_RearCamera() {
  delete(dRearCamera);
}

/**
 * Método público inicializador de artefactos ROS de la clase
 */
void RosNode_RearCamera::initROS() {
  ros::NodeHandle nh;
  pubRearCameraInfo = nh.advertise<CITIUS_Control_RearCamera::msg_rearCameraInfo>("rearCameraInfo", 1000);
  subsCtrlRearCamera = nh.subscribe("ctrlRearCamera", 1000, &RosNode_RearCamera::fcn_sub_ctrlRearCamera, this);
  servNodeStatus = nh.advertiseService("rcNodeStatus", &RosNode_RearCamera::fcv_serv_nodeStatus, this);
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
 * Método público consultor del atributo "dRearCamera" de la clase con una 
 * referencia a la instancia de la clase que gestiona la comunicación con la 
 * cámara
 * @return Atributo "dRearCamera"
 */
AxisP3364LveDriver *RosNode_RearCamera::getDriverMng() {
  return dRearCamera;
}

/**
 * Método público que transmite el estado al dispositivo, solicita el estado 
 * real y lo publica mediante el topic correspondiente
 */
void RosNode_RearCamera::manageDevice() {
  dRearCamera->sendSetToDevice(ORDER_PAN, dRearCamera->getPan());
  dRearCamera->sendSetToDevice(ORDER_TILT, dRearCamera->getTilt());
  dRearCamera->sendSetToDevice(ORDER_ZOOM, dRearCamera->getZoom());
  LensPosition lensPos = dRearCamera->getPosition();
  if (lensPos.state) {
    CITIUS_Control_RearCamera::msg_rearCameraInfo rcMsg;
    rcMsg.pan = lensPos.pan * CONV_TO_CAMERA; // * (100/5000) Conversion de formato camara
    rcMsg.tilt = lensPos.tilt * CONV_TO_CAMERA; // * (100/5000) Conversion de formato camara
    rcMsg.zoom = lensPos.zoom * CONV_TO_CAMERA; // * (100/5000) Conversion de formato camara
    pubRearCameraInfo.publish(rcMsg);
    usleep(50000);
  }
}