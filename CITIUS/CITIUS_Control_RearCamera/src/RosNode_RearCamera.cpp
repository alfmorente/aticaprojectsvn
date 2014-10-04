
/** 
 * @file  RosNode_RearCamera.cpp
 * @brief Implementacion de la clase "RosNode_RearCamera"
 * @author: Carlos Amores
 * @date: 2013, 2014
 */

#include "RosNode_RearCamera.h"

/**
 * Constructor de la clase. Inicia la maquina de estados del nodo y crea la 
 * instancia que permite la gestion de la camara
 */
RosNode_RearCamera::RosNode_RearCamera() {
    rcNodeStatus = NODESTATUS_INIT;
    dRearCamera = new AxisP3364LveDriver();
}

/**
 * Inicializador de artefactos ROS de la clase
 */
void RosNode_RearCamera::initROS() {
    ros::NodeHandle nh;
    pubRearCameraInfo = nh.advertise<CITIUS_Control_RearCamera::msg_rearCameraInfo>("rearCameraInfo",1000);
    subsCtrlRearCamera = nh.subscribe("ctrlRearCamera",1000,&RosNode_RearCamera::fcn_sub_ctrlRearCamera,this);
    servNodeStatus = nh.advertiseService("fcNodeStatus",&RosNode_RearCamera::fcv_serv_nodeStatus,this);
}

/**
 * Receptor de mensajes ROS para el control de la camara. Recibe las ordenes
 * y las transmite hacia la camara
 * @param[in] msg Mensaje ROS con ordenes de actuacion sobre la camara
 */
void RosNode_RearCamera::fcn_sub_ctrlRearCamera(CITIUS_Control_RearCamera::msg_ctrlRearCamera msg){
    if(msg.isZoom) dRearCamera->setZoom(msg.zoom);
    if(msg.isPan) dRearCamera->setPan(msg.pan);
    if(msg.isZoom) dRearCamera->setTilt(msg.tilt);
}
    
/**
 * Rutina del servidor de estados del nodo. Recibe las peticiones de transicion
 * y las ejecuta segun la logica establecida
 * @param[in] rq Parametros de requerimiento
 * @param[in] rsp Parametros de respuesta
 * @return Booleano que indica si se ha realizado el correcto tratamiento de
 * la peticion de servicio
 */
bool RosNode_RearCamera::fcv_serv_nodeStatus(CITIUS_Control_RearCamera::srv_nodeStatus::Request &rq, CITIUS_Control_RearCamera::srv_nodeStatus::Response &rsp){
    if(rq.status == NODESTATUS_OK){
        rcNodeStatus = NODESTATUS_OK;
        rsp.confirmation = true;
    }else if(rq.status == NODESTATUS_OFF){
        rcNodeStatus = NODESTATUS_OFF;
        rsp.confirmation = true;
    }else{
        rsp.confirmation = false;
    }
    return true;
}

/**
 * Consultor del atributo "rcNodeStatus" de la clase que registra el estado
 * actual en el que se encuentra la maquina de estados
 * @return Atributo "fcNodeStatus" de la clase
 */
short RosNode_RearCamera::getRcNodeStatus(){
    return rcNodeStatus;
}

/**
 * Modificador del atributo "rcNodeStatus" de la clase
 * @param[in] newRcNodeStatus Nuevo valor para el atributo
 */
void RosNode_RearCamera::setRcNodeStatus(short newRcNodeStatus){
    rcNodeStatus = newRcNodeStatus;
}

/**
 * Consultor del atributo "pubRearCameraInfo" de la clase con el publicador 
 * de informacion del estado de la camara
 * @return Atributo "pubRearCameraInfo"
 */
ros::Publisher RosNode_RearCamera::getPubRearCameraInfo(){
    return pubRearCameraInfo;
}

/**
 * Consultor del atributo "dRearCamera" de la clase con una referencia a la 
 * instancia de la clase que gestiona la comunicacion con la camara
 * @return Atributo "dRearCamera"
 */
AxisP3364LveDriver *RosNode_RearCamera::getDriverMng(){
    return dRearCamera;
}