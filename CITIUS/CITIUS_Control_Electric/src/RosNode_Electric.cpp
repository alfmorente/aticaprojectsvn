
/** 
 * @file  RosNode_Electric.cpp
 * @brief Implementacion de la clase "RosNode_Electric"
 * @author: Carlos Amores
 * @date: 2013, 2014
 */

#include "RosNode_Electric.h"

/**
 * Constructor de la clase. Inicia la maquina de estados del nodo y crea la 
 * instancia del driver de conexion con el vehiculo
 */
RosNode_Electric::RosNode_Electric() {
    emNodeStatus = NODESTATUS_INIT;
    dElectric = new ElectricConnectionManager();
}

/**
 * Inicia los artefactos ROS atributos de la clase. Consulta la position del 
 * conmutador local/teleoperado y solicita la inicializacion de la maquina de
 * estados de modos de operacion del vehiculo con la señal obtenida
 */
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
    // Cola de comandos criticos
    dElectric->addToQueue(frame);
    // Envio a vehiculo
    dElectric->sendToVehicle(frame);
    usleep(1000);
        
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

/**
 * Consultor del atributo "emNodeStatus" de la clase que proporciona el estado
 * actual de la maquina de estados del nodo
 * @return Atributo "emNodeStatus" de la clase
 */
short RosNode_Electric::getEMNodeStatus(){
    return emNodeStatus;
}

/**
 * Modificador del atributo "emNodeStatus" de la clase para realizar una 
 * transicion en la maquina de estados del nodo
 * @param[in] newStatus Nuevo estado al que realizar la transicion
 */
void RosNode_Electric::setEMNodeStatus(short newStatus){
    emNodeStatus = newStatus;
}

/**
 * Consultor del atributo "dElectric" de la clase que proporciona la instancia 
 * del driver utilizado en la comunicacion con el vehiculo
 * @return Atributo "dElectric" de la clase
 */
ElectricConnectionManager *RosNode_Electric::getDriverMng(){
    return dElectric;
}

/**
 * Consultor del atributo "clientVehicleStatus" de la clase que proporciona el
 * cliente que realiza peticiones para las transiciones en la maquina de 
 * estados de los modos de operacion del vehiculo
 * @return Atributo "clientVehicleStatus"
 */
ros::ServiceClient RosNode_Electric::getClientVehicleStatus(){
    return clientVehicleStatus;
}

/**
 * Publica la informacion del vehiculo que recibe como parametro en el topic 
 * ROS correspondiente
 * @param[in] info Informacion electrica del vehiculo a publicar
 */
void RosNode_Electric::publishElectricInfo(ElectricInfo info){
    
    CITIUS_Control_Electric::msg_electricInfo msg;
    
    msg.battery_level = info.battery_level;
    msg.battery_voltage = info.battery_voltage;
    msg.battery_current = info.battery_current;
    msg.battery_temperature = info.battery_temperature;
    msg.supply_alarms = info.supply_alarms;

    pubElectricInfo.publish(msg);
    
}

/**
 * Publica la informacion de un cambio en la posicion del conmutador local /
 * teleoperado que recibe como parametro en el topic correspondiente
 * @param[in] position Nueva posicion leida del conmutador local / teleoperado
 */
void RosNode_Electric::publishSwitcherInfo(short position){
    
    CITIUS_Control_Electric::msg_switcher msg;
    msg.switcher = position;
    pubElectricInfo.publish(msg);
    
}

/**
 * Publica una serie de comandos de activacion sobre diversos actuadores ante 
 * una lectura de cambio en el conmutador local / teleoperado
 * @param[in] on Nueva posicion del conmutador local / teleoperado
 */
void RosNode_Electric::publishSetupCommands(bool on){
    
    CITIUS_Control_Electric::msg_command msg;
    
    if(on)
        msg.value = 1;
    else
        msg.value = 0;
    
    // Activa/desactiva acelerador
    msg.id_device = MT_THROTTLE;
    pubCommand.publish(msg);
    
    // Activa/desactiva freno de servicio
    msg.id_device = MT_BRAKE;
    pubCommand.publish(msg);
    
    // Activa/desactiva freno de estacionamiento
    msg.id_device = MT_HANDBRAKE;
    pubCommand.publish(msg);
    
    // Activa/desactiva direccion
    msg.id_device = MT_STEERING;
    pubCommand.publish(msg);
    
    // Activa/desactiva marcha
    msg.id_device = MT_GEAR;
    pubCommand.publish(msg);
    
    // Activa/desactiva luces
    msg.id_device = MT_LIGHTS;
    pubCommand.publish(msg);
    
    // Activa/desactiva intermitentes
    msg.id_device = MT_BLINKERS;
    pubCommand.publish(msg);
        
}