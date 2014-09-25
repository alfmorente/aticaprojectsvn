#include "RosNode_Driving.h"

/*******************************************************************************
 * CONSTRUCTOR DE LA CLASE
 ******************************************************************************/

RosNode_Driving::RosNode_Driving() {
    setVMNodeStatus(NODESTATUS_INIT);
    dVehicle = new DrivingConnectionManager();
}

/*******************************************************************************
 * INICIALIZADOR DE ARTEFACTOS ROS
 ******************************************************************************/

void RosNode_Driving::initROS() {
    ros::NodeHandle nh;
    pubVehicleInfo = nh.advertise<CITIUS_Control_Driving::msg_vehicleInfo>("vehicleInfo",1000);
    subsCommand = nh.subscribe("command",1000,&RosNode_Driving::fcn_sub_command,this);
    servNodeStatus = nh.advertiseService("vmNodeStatus",&RosNode_Driving::fcv_serv_nodeStatus,this);
}

/*******************************************************************************
 * GETTER AND SETTER NECESARIOS
 ******************************************************************************/

// Control de la camara

void RosNode_Driving::fcn_sub_command(CITIUS_Control_Driving::msg_command msg) {
    ros::NodeHandle nh;
    int currentVehicleStatus = OPERATION_MODE_INICIANDO;
    nh.getParam("vehicleStatus", currentVehicleStatus);
    if (currentVehicleStatus == OPERATION_MODE_CONDUCCION) {
        if (vmNodeStatus == NODESTATUS_OK) {
            ROS_INFO("[Control] Driving - Comando de telecontrol recibido");
            if (checkCommand(msg)) {
                // Envio de comando a vehiculo
                FrameDriving command;
                command.instruction = SET;
                command.element = msg.id_device;
                command.value = msg.value;
                if(dVehicle->isCriticalInstruction(msg.id_device)){
                    short cont = dVehicle->getCountCriticalMessages();
                    // Valor de ID_INSTRUCCION
                    command.id_instruccion = cont;
                    // Introduccion en la cola de mensajes criticos
                    dVehicle->addToQueue(command);
                    // Incremento del contador
                    dVehicle->setCountCriticalMessages(cont+1);
                }else{
                    command.id_instruccion = -1;
                }
                
                
                dVehicle->sendToVehicle(command);
                
                
            } else {
                ROS_INFO("[Control] Driving - Descartado comando - Fuera de rango");
            }
        } else {
            ROS_INFO("[Control] Driving - Descartado comando - Nodo en estado %d", vmNodeStatus);
        }
    }else{
        ROS_INFO("[Control Driving - Descartado comando - Vehiculo fuera del modo CONDUCCION]");
    }
}
    
// Gestion del estado del nodo
bool RosNode_Driving::fcv_serv_nodeStatus(CITIUS_Control_Driving::srv_nodeStatus::Request &rq, CITIUS_Control_Driving::srv_nodeStatus::Response &rsp){
    if(rq.status == NODESTATUS_OK){
        setVMNodeStatus(NODESTATUS_OK);
        rsp.confirmation = true;
    }else if(rq.status == NODESTATUS_OFF){
        getDriverMng()->disconnectVehicle();
        setVMNodeStatus(NODESTATUS_OFF);
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
    return vmNodeStatus;
}

// Set del estado del nodo
void RosNode_Driving::setVMNodeStatus(short newVMNodeStatus){
    vmNodeStatus = newVMNodeStatus;
}

// Obtener el publicador de informacion del vehiculo
ros::Publisher RosNode_Driving::getPubVehicleInfo(){
    return pubVehicleInfo;
}

// Obtener el driver 
DrivingConnectionManager *RosNode_Driving::getDriverMng(){
    return dVehicle;
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
void RosNode_Driving::manageAlarmsMessage(FrameDriving frame){
    // Se comprueba que es de tipo INFO
    if(frame.instruction == INFO){
        // Tratamiento alarmas / demas elementos
        if(frame.element == DRIVE_ALARMS){
            // TODO: Tratamientos de alarmas del modulo
            ROS_INFO("[Control] Driving - Alarma recibida");
            
        }else if(frame.element == STEERING_ALARMS){
            // TODO: Tratamiento de alarmas de direccion
            ROS_INFO("[Control] Driving - Alarma de direccion recibida");
            
        }else{
            ROS_INFO("[Control] Driving - Recibida trama como alarma. Elemento: %d",frame.element);
        }
    }else{
        ROS_INFO("[Control] Driving - Trama invalida recibida");
    }
    
}

/*******************************************************************************
 * PUBLICACION DE INFORMACION DEL VEHICULO
 ******************************************************************************/

void RosNode_Driving::publishDrivingInfo(DrivingInfo info){
    
    CITIUS_Control_Driving::msg_vehicleInfo msg;
    
    msg.steering = info.steering;
    msg.thottle = info.thottle;
    msg.brake = info.brake;
    msg.parkingBrake = info.parkingBrake;
    msg.gear = info.gear;
    msg.speed = info.speed;
    msg.motorRPM = info.motorRPM;
    msg.motorTemperature = info.motorTemperature;
    msg.lights = info.lights;
    msg.blinkerLeft = info.blinkerLeft;
    msg.blinkerRight = info.blinkerRight;
    msg.dipss = info.dipss;
    msg.dipsr = info.dipsr;
    msg.dipsp = info.dipsp;
    msg.klaxon = info.klaxon;
    
    pubVehicleInfo.publish(msg);

}