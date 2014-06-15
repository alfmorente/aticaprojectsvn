#include "RosNode_RearCamera.h"

/*******************************************************************************
 * CONSTRUCTOR DE LA CLASE
 ******************************************************************************/

RosNode_RearCamera::RosNode_RearCamera() {
    this->setFcNodeStatus(NODESTATUS_INIT);
    this->dRearCamera = new DrivingCameraManager();
}

/*******************************************************************************
 * INICIALIZADOR DE ARTEFACTOS ROS
 ******************************************************************************/

void RosNode_RearCamera::initROS() {
    ros::NodeHandle nh;
    this->pubRearCameraInfo = nh.advertise<CITIUS_Control_RearCamera::msg_rearCameraInfo>("rearCameraInfo",1000);
    this->subsCtrlRearCamera = nh.subscribe("ctrlRearCamera",1000,&RosNode_RearCamera::fcn_sub_ctrlRearCamera,this);
    this->servNodeStatus = nh.advertiseService("fcNodeStatus",&RosNode_RearCamera::fcv_serv_nodeStatus,this);
}

/*******************************************************************************
 * GETTER AND SETTER NECESARIOS
 ******************************************************************************/

// Control de la camara
void RosNode_RearCamera::fcn_sub_ctrlRearCamera(CITIUS_Control_RearCamera::msg_ctrlRearCamera msg){
    this->getDriverMng()->setParam(IDPARAM_PAN,msg.pan);
    this->getDriverMng()->setParam(IDPARAM_TILT,msg.tilt);
}
    
// Gestion del estado del nodo
bool RosNode_RearCamera::fcv_serv_nodeStatus(CITIUS_Control_RearCamera::srv_nodeStatus::Request &rq, CITIUS_Control_RearCamera::srv_nodeStatus::Response &rsp){
    if(rq.status == NODESTATUS_OK){
        this->setFcNodeStatus(NODESTATUS_OK);
        rsp.confirmation = true;
    }else if(rq.status == NODESTATUS_OFF){
        this->getDriverMng()->disconnect();
        this->setFcNodeStatus(NODESTATUS_OFF);
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
short RosNode_RearCamera::getFcNodeStatus(){
    return this->fcNodeStatus;
}

// Set del estado del nodo
void RosNode_RearCamera::setFcNodeStatus(short newFcNodeStatus){
    this->fcNodeStatus = newFcNodeStatus;
}

// Obtener el publicador de informacion del vehiculo
ros::Publisher RosNode_RearCamera::getPubRearCameraInfo(){
    return this->pubRearCameraInfo;
}

// Obtener el driver 
DrivingCameraManager *RosNode_RearCamera::getDriverMng(){
    return this->dRearCamera;
}