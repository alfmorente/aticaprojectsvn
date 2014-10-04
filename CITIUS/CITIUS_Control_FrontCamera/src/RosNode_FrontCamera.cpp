
/** 
 * @file  RosNode_FrontCamera.cpp
 * @brief Implementacion de la clase "RosNode_FrontCamera"
 * @author: Carlos Amores
 * @date: 2013, 2014
 */

#include "RosNode_FrontCamera.h"

/**
 * Constructor de la clase. Inicia la maquina de estados del nodo y crea la 
 * instancia que permite la gestion de la camara
 */
RosNode_FrontCamera::RosNode_FrontCamera() {
    fcNodeStatus = NODESTATUS_INIT;
    dFrontCamera = new AxisP3364LveDriver();
}

/**
 * Inicializador de artefactos ROS de la clase
 */
void RosNode_FrontCamera::initROS() {
    ros::NodeHandle nh;
    pubFrontCameraInfo = nh.advertise<CITIUS_Control_FrontCamera::msg_frontCameraInfo>("frontCameraInfo",1000);
    subsCtrlFrontCamera = nh.subscribe("ctrlFrontCamera",1000,&RosNode_FrontCamera::fcn_sub_ctrlFrontCamera,this);
    servNodeStatus = nh.advertiseService("fcNodeStatus",&RosNode_FrontCamera::fcv_serv_nodeStatus,this);
}

/**
 * Receptor de mensajes ROS para el control de la camara. Recibe las ordenes
 * y las transmite hacia la camara
 * @param[in] msg Mensaje ROS con ordenes de actuacion sobre la camara
 */
void RosNode_FrontCamera::fcn_sub_ctrlFrontCamera(CITIUS_Control_FrontCamera::msg_ctrlFrontCamera msg){
    if(msg.isZoom) dFrontCamera->setZoom(msg.zoom);
    if(msg.isPan) dFrontCamera->setPan(msg.pan);
    if(msg.isZoom) dFrontCamera->setTilt(msg.tilt);
}
    
/**
 * Rutina del servidor de estados del nodo. Recibe las peticiones de transicion
 * y las ejecuta segun la logica establecida
 * @param[in] rq Parametros de requerimiento
 * @param[in] rsp Parametros de respuesta
 * @return Booleano que indica si se ha realizado el correcto tratamiento de
 * la peticion de servicio
 */
bool RosNode_FrontCamera::fcv_serv_nodeStatus(CITIUS_Control_FrontCamera::srv_nodeStatus::Request &rq, CITIUS_Control_FrontCamera::srv_nodeStatus::Response &rsp){
    if(rq.status == NODESTATUS_OK){
        fcNodeStatus = NODESTATUS_OK;
        rsp.confirmation = true;
    }else if(rq.status == NODESTATUS_OFF){
        fcNodeStatus = NODESTATUS_OFF;
        rsp.confirmation = true;
    }else{
        rsp.confirmation = false;
    }
    return true;
}

/**
 * Consultor del atributo "fcNodeStatus" de la clase que registra el estado
 * actual en el que se encuentra la maquina de estados
 * @return Atributo "fcNodeStatus" de la clase
 */
short RosNode_FrontCamera::getFcNodeStatus(){
    return fcNodeStatus;
}

/**
 * Modificador del atributo "fcNodeStatus" de la clase
 * @param[in] newFcNodeStatus Nuevo valor para el atributo
 */
void RosNode_FrontCamera::setFcNodeStatus(short newFcNodeStatus){
    fcNodeStatus = newFcNodeStatus;
}

/**
 * Consultor del atributo "pubFrontCameraInfo" de la clase con el publicador 
 * de informacion del estado de la camara
 * @return Atributo "pubFrontCameraInfo"
 */
ros::Publisher RosNode_FrontCamera::getPubFrontCameraInfo(){
    return pubFrontCameraInfo;
}

/**
 * Consultor del atributo "dFrontCamera" de la clase con una referencia a la 
 * instancia de la clase que gestiona la comunicacion con la camara
 * @return Atributo "dFrontCamera"
 */
AxisP3364LveDriver *RosNode_FrontCamera::getDriverMng(){
    return dFrontCamera;
}