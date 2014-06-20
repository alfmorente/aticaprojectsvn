#include "RosNode_Driving.h"

/*******************************************************************************
 * CONSTRUCTOR DE LA CLASE
 ******************************************************************************/

RosNode_Driving::RosNode_Driving() {
    this->setVMNodeStatus(NODESTATUS_INIT);
    this->dVehicle = new DrivingConnectionManager();
}

/*******************************************************************************
 * INICIALIZADOR DE ARTEFACTOS ROS
 ******************************************************************************/

void RosNode_Driving::initROS() {
    ros::NodeHandle nh;
    this->pubVehicleInfo = nh.advertise<CITIUS_Control_Driving::msg_vehicleInfo>("vehicleInfo",1000);
    this->subsCommand = nh.subscribe("command",1000,&RosNode_Driving::fcn_sub_command,this);
    this->servNodeStatus = nh.advertiseService("vmNodeStatus",&RosNode_Driving::fcv_serv_nodeStatus,this);
}

/*******************************************************************************
 * GETTER AND SETTER NECESARIOS
 ******************************************************************************/

// Control de la camara

void RosNode_Driving::fcn_sub_command(CITIUS_Control_Driving::msg_command msg) {
    if (this->vmNodeStatus == NODESTATUS_OK) {
        ROS_INFO("[Control] Driving - Comando de telecontrol recibido");
        if (checkCommand(msg)) {
            this->getDriverMng()->setParam(msg.id_device, msg.value);
        } else {
            ROS_INFO("[Control] Driving - Descartado comando - Fuera de rango");
        }
    }else{
        ROS_INFO("[Control] Driving - Descartado comando - Nodo en estado %d",this->vmNodeStatus);
    }

}
    
// Gestion del estado del nodo
bool RosNode_Driving::fcv_serv_nodeStatus(CITIUS_Control_Driving::srv_nodeStatus::Request &rq, CITIUS_Control_Driving::srv_nodeStatus::Response &rsp){
    if(rq.status == NODESTATUS_OK){
        this->setVMNodeStatus(NODESTATUS_OK);
        rsp.confirmation = true;
    }else if(rq.status == NODESTATUS_OFF){
        this->getDriverMng()->disconnectVehicle();
        this->setVMNodeStatus(NODESTATUS_OFF);
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
short RosNode_Driving::getVMNodeStatus(){
    return this->vmNodeStatus;
}

// Set del estado del nodo
void RosNode_Driving::setVMNodeStatus(short newVMNodeStatus){
    this->vmNodeStatus = newVMNodeStatus;
}

// Obtener el publicador de informacion del vehiculo
ros::Publisher RosNode_Driving::getPubVehicleInfo(){
    return this->pubVehicleInfo;
}

// Obtener el driver 
DrivingConnectionManager *RosNode_Driving::getDriverMng(){
    return this->dVehicle;
}

/*******************************************************************************
 * CRIBA DE COMANDOS FUERA DE RANGO
 ******************************************************************************/

bool RosNode_Driving::checkCommand(CITIUS_Control_Driving::msg_command msg){
    bool ret = true;
    short element = msg.id_device;
    short value = msg.value;
    switch(element){
        case (RESET):
            if(value < 0 || value > 1) ret = false;
            break;
        case BLINKER_RIGHT:
            if(value < 0 || value > 1) ret = false;
            break;
        case BLINKER_LEFT:
            if(value < 0 || value > 1) ret = false;
            break;
        case BLINKER_EMERGENCY:
            if(value < 0 || value > 1) ret = false;
            break;
        case MT_BLINKERS:
            if(value < 0 || value > 1) ret = false;
            break;
        case DIPSP:
            if(value < 0 || value > 1) ret = false;
            break;
        case DIPSS:
            if(value < 0 || value > 1) ret = false;
            break;
        case DIPSR:
            if(value < 0 || value > 1) ret = false;
            break;
        case KLAXON:
            if(value < 0 || value > 1) ret = false;
            break;
        case MT_LIGHTS:
            if(value < 0 || value > 1) ret = false;
            break;
        case GEAR:
            if(value < 0 || value > 2) ret = false;
            break;
        case MT_GEAR:
            if(value < 0 || value > 1) ret = false;
            break;
        case THROTTLE:
            if(value < 0 || value > 100) ret = false;
            break;
        case CRUISING_SPEED:
            if(value < 0 || value > 1000) ret = false;
            break;
        case MT_THROTTLE:
            if(value < 0 || value > 1) ret = false;
            break;
        case HANDBRAKE:
            if(value < 0 || value > 1) ret = false;
            break;
        case MT_HANDBRAKE:
            if(value < 0 || value > 1) ret = false;
            break;
        case BRAKE:
            if(value < 0 || value > 100) ret = false;
            break;
        case MT_BRAKE:
            if(value < 0 || value > 1) ret = false;
            break;
        case STEERING:
            if(value < -100 || value > 100) ret = false;
            break;
        case MT_STEERING:
            if(value < 0 || value > 1) ret = false;
            break;
        default:
            break;
    };
    return ret;
}

/*******************************************************************************
 * GESTION DE MENSAJES RECIBIDOS DEL VEHICULO
 ******************************************************************************/
void RosNode_Driving::manageMessage(FrameDriving frame){
    // Se comprueba que es de tipo INFO
    if(frame.instruction == INFO){
        // Tratamiento alarmas / demas elementos
        if(frame.element == DRIVE_ALARMS){
            // TODO: Tratamientos de alarmas del modulo
            
        }else if(frame.element == STEERING_ALARMS){
            // TODO: Tratamiento de alarmas de direccion
            
        }else{
            CITIUS_Control_Driving::msg_vehicleInfo vMsg;
            vMsg.id_device = frame.element;
            vMsg.value = frame.value;
            this->getPubVehicleInfo().publish(vMsg);
        }
    }else{
        ROS_INFO("[Control] Driving - Trama invalida recibida");
    }
    
}