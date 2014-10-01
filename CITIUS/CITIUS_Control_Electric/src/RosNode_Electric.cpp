#include <deque>

#include "RosNode_Electric.h"

/*******************************************************************************
 * CONSTRUCTOR DE LA CLASE
 ******************************************************************************/

RosNode_Electric::RosNode_Electric() {
    emNodeStatus = NODESTATUS_INIT;
    dElectric = new ElectricConnectionManager();
}

/*******************************************************************************
 * INICIALIZADOR DE ARTEFACTOS ROS
 ******************************************************************************/

void RosNode_Electric::initROS() {
    
    ros::NodeHandle nh;
    
    clientVehicleStatus = nh.serviceClient<CITIUS_Control_Electric::srv_vehicleStatus>("vehicleStatus");
   
    pubElectricInfo = nh.advertise<CITIUS_Control_Electric::msg_electricInfo>("electricInfo",1000);
    pubCommand = nh.advertise<CITIUS_Control_Electric::msg_command>("command",1000);
    pubSwitcher = nh.advertise<CITIUS_Control_Electric::msg_switcher>("switcher",1000);
        
    // Se solicita la activacion del resto de nodos del vehiculo
    ROS_INFO("[Control] Electric - Solicitando inicio de nodos del vehiculo");
    
    // Set up del suministro electrico
    FrameDriving frame;
    frame.instruction = SET;
    frame.id_instruction = dElectric->getCountCriticalMessages();
    dElectric->setCountCriticalMessages(dElectric->getCountCriticalMessages()+1);
    frame.element = SUPPLY_TURN_ON;
    frame.value = 1;
    dElectric->sendToVehicle(frame);
        
    CITIUS_Control_Electric::srv_vehicleStatus service;
    
    service.request.status = OPERATION_MODE_INICIANDO;
    // Solicitar a vehiculo posicion conmutador local/teleoperado
    // TODO
    service.request.posSwitcher = SWITCHER_LOCAL; // comentar

    while(!clientVehicleStatus.call(service)){
        ros::spinOnce();
    }
    
    if(service.response.confirmation) {
        ROS_INFO("[Control] Electric - Se ha iniciado el vehiculo");
        emNodeStatus = NODESTATUS_OK;
    } else {
        ROS_INFO("[Control] Electric - El vehiculo no se ha podido iniciar");
        emNodeStatus = NODESTATUS_OFF;
    }
    
}


/*******************************************************************************
 * GETTER AND SETTER NECESARIOS
 ******************************************************************************/

// Get del estado del nodo
short RosNode_Electric::getEMNodeStatus(){
    return emNodeStatus;
}

// Set del estado del nodo
void RosNode_Electric::setEMNodeStatus(short newStatus){
    emNodeStatus = newStatus;
}

// Obtener el publicador de informacion del vehiculo
ros::Publisher RosNode_Electric::getPubElectricInfo(){
    return pubElectricInfo;
}

// Obtener el driver 
ElectricConnectionManager *RosNode_Electric::getDriverMng(){
    return dElectric;
}

ros::ServiceClient RosNode_Electric::getClientVehicleStatus(){
    return clientVehicleStatus;
}

/*******************************************************************************
 * PUBLICACION DE INFORMACION DEL VEHICULO
 ******************************************************************************/

void RosNode_Electric::publishElectricInfo(ElectricInfo info){
    
    CITIUS_Control_Electric::msg_electricInfo msg;
    
    msg.battery_level = info.battery_level;
    msg.battery_voltage = info.battery_voltage;
    msg.battery_current = info.battery_current;
    msg.battery_temperature = info.battery_temperature;
    msg.supply_alarms = info.supply_alarms;

    pubElectricInfo.publish(msg);
    
}