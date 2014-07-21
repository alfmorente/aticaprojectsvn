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
    CITIUS_Control_Electric::srv_vehicleStatus service;
    
    service.request.status = OPERATION_MODE_INICIANDO;
    // Solicitar a vehiculo posicion conmutador local/teleoperado
    // TODO
    service.request.posSwitcher = SWITCHER_LOCAL;
    
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

// Obtener el publicador de informacion del vehiculo
ros::Publisher RosNode_Electric::getPubElectricInfo(){
    return this->pubElectricInfo;
}

// Obtener el driver 
ElectricConnectionManager *RosNode_Electric::getDriverMng(){
    return this->dElectric;
}

ros::ServiceClient RosNode_Electric::getClientVehicleStatus(){
    return this->clientVehicleStatus;
}

/*******************************************************************************
 * GESTION DE MENSAJES RECIBIDOS DEL VEHICULO
 ******************************************************************************/
void RosNode_Electric::manageMessage(FrameDriving frame){
    // Se comprueba que es de tipo INFO
    if(frame.instruction == INFO){
        // Tratamiento alarmas / demas elementos
        if(frame.element == SUPPLY_ALARMS){
            // TODO: Tratamientos de alarmas del modulo
            // Gestion del conmutador LOCAL / AUTOMATICO
            
        // Apagado del vehiculo
        }else if(frame.element == TURN_OFF){
            ROS_INFO("[Control] Electric - Preparando el apagado del sistema");
            CITIUS_Control_Electric::srv_vehicleStatus srvVS;
            srvVS.request.status = OPERATION_MODE_APAGANDO;
            while(!this->getClientVehicleStatus().call(srvVS));
            if(srvVS.response.confirmation){
                
                // Cambio de estado de nodo
                emNodeStatus = NODESTATUS_OFF;
                // Envio de confirmacion de apagado
                dElectric->setParam(TURN_OFF,1);
                // Desconexion del vehiculo
                dElectric->disconnectVehicle();
                // Cambio de estado vehiculo
                ros::NodeHandle nh;
                nh.setParam("vehicleStatus",OPERATION_MODE_APAGANDO);
                
            }
            
        }else if(frame.element == OPERATION_MODE_SWITCH){
            CITIUS_Control_Electric::msg_switcher vMsg;
            vMsg.switcher = frame.value;
            pubSwitcher.publish(vMsg);
        }else{
            CITIUS_Control_Electric::msg_electricInfo vMsg;
            vMsg.id_device = frame.element;
            vMsg.value = frame.value;
            pubElectricInfo.publish(vMsg);
        }
    }else{
        ROS_INFO("[Control] Electric - Trama invalida recibida");
    }
    
}