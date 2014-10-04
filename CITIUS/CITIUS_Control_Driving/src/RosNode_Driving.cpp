
/** 
 * @file  RosNode_Driving.cpp
 * @brief Implementacion de la clase "RosNode_Driving"
 * @author: Carlos Amores
 * @date: 2013, 2014
 */

#include "RosNode_Driving.h"

/**
 * Constructor de la clase. Inicia la maquina de estados del nodo y crea la 
 * instancia del driver de conexion con el vehiculo
 */
RosNode_Driving::RosNode_Driving() {
    vmNodeStatus = NODESTATUS_INIT;
    dVehicle = new DrivingConnectionManager();
}

/**
 * Inicia los artefactos ROS atributos de la clase
 */
void RosNode_Driving::initROS() {
    ros::NodeHandle nh;
    pubVehicleInfo = nh.advertise<CITIUS_Control_Driving::msg_vehicleInfo>("vehicleInfo",1000);
    subsCommand = nh.subscribe("command",1000,&RosNode_Driving::fcn_sub_command,this);
    servNodeStatus = nh.advertiseService("vmNodeStatus",&RosNode_Driving::fcv_serv_nodeStatus,this);
}

/**
 * Receptor de mensajes ROS con comandos de actuacion sobre el vehiculo. Lo 
 * transmite el vehiculo y lo encola si el elemento es considerado critico y
 * por tanto debe llevarse a cabo el mecanismo de integridad
 * @param[in] msg Mensaje ROS recibido
 */
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
    
/**
 * Rutina del tratamiento de servicios para la modificacion de la maquina de 
 * estados del nodo
 * @param[in] rq Parametros de requerimiento
 * @param[in] rsp Parametros de respuesta
 * @return Booleano que indica si se ha realizado el correcto tratamiento de
 * la peticion de servicio
 */
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

/**
 * Consultor del atributo "vmNodeStatus" de la clase que proporciona el estado
 * actual de la maquina de estados del nodo
 * @return Atributo "vmNodeStatus" de la clase
 */
short RosNode_Driving::getVMNodeStatus(){
    return vmNodeStatus;
}

/**
 * Modificador del atributo "vmNodeStatus" de la clase para realizar una 
 * transicion en la maquina de estados del nodo
 * @param[in] newVMNodeStatus Nuevo estado al que realizar la transicion
 */
void RosNode_Driving::setVMNodeStatus(short newVMNodeStatus){
    vmNodeStatus = newVMNodeStatus;
}

/**
 * Consultor del atributo "dVehicle" de la clase que proporciona la instancia 
 * del driver utilizado en la comunicacion con el vehiculo
 * @return Atributo "dVehicle" de la clase
 */
DrivingConnectionManager *RosNode_Driving::getDriverMng(){
    return dVehicle;
}

/**
 * Realiza la criba de comandos recibidos si el valor a imprimir sobre un
 * elemento esta fuera de los limites establecidos para ese elemento
 * @param[in] msg Mensaje ROS a comprobar
 * @return Booleano que indica si el comando es valido (valor dentro de los 
 * limites establecidos para dicho elemento) o no
 */
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

/**
 * Publica la informacion del vehiculo que recibe como parametro en el topic 
 * ROS correspondiente
 * @param[in] info Informacion del vehiculo a publicar
 */
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