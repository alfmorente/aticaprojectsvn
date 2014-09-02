#include "RosNode_RearCamera.h"

/*******************************************************************************
 * CONSTRUCTOR DE LA CLASE
 ******************************************************************************/

RosNode_RearCamera::RosNode_RearCamera() {
    setRcNodeStatus(NODESTATUS_INIT);
    dRearCamera = new AxisP3364LveDriver();
}

/*******************************************************************************
 * INICIALIZADOR DE ARTEFACTOS ROS
 ******************************************************************************/

void RosNode_RearCamera::initROS() {
    ros::NodeHandle nh;
    pubRearCameraInfo = nh.advertise<CITIUS_Control_RearCamera::msg_rearCameraInfo>("rearCameraInfo",1000);
    subsCtrlRearCamera = nh.subscribe("ctrlRearCamera",1000,&RosNode_RearCamera::fcn_sub_ctrlRearCamera,this);
    servNodeStatus = nh.advertiseService("fcNodeStatus",&RosNode_RearCamera::fcv_serv_nodeStatus,this);
}

// Control de la camara
void RosNode_RearCamera::fcn_sub_ctrlRearCamera(CITIUS_Control_RearCamera::msg_ctrlRearCamera msg){
    if(msg.isZoom) dRearCamera->setZoom(msg.zoom);
    if(msg.isPan) dRearCamera->setPan(msg.pan);
    if(msg.isZoom) dRearCamera->setTilt(msg.tilt);
}
    
// Gestion del estado del nodo
bool RosNode_RearCamera::fcv_serv_nodeStatus(CITIUS_Control_RearCamera::srv_nodeStatus::Request &rq, CITIUS_Control_RearCamera::srv_nodeStatus::Response &rsp){
    if(rq.status == NODESTATUS_OK){
        setRcNodeStatus(NODESTATUS_OK);
        rsp.confirmation = true;
    }else if(rq.status == NODESTATUS_OFF){
        setRcNodeStatus(NODESTATUS_OFF);
        rsp.confirmation = true;
    }else{
        rsp.confirmation = false;
    }
    return true;
}

/*******************************************************************************
 * GETTER AND SETTER NECESARIOS
 ******************************************************************************/

// Get del estado del nodo
short RosNode_RearCamera::getRcNodeStatus(){
    return rcNodeStatus;
}

// Set del estado del nodo
void RosNode_RearCamera::setRcNodeStatus(short newRcNodeStatus){
    rcNodeStatus = newRcNodeStatus;
}

// Obtener el publicador de informacion del vehiculo
ros::Publisher RosNode_RearCamera::getPubRearCameraInfo(){
    return pubRearCameraInfo;
}

// Obtener el driver 
AxisP3364LveDriver *RosNode_RearCamera::getDriverMng(){
    return dRearCamera;
}