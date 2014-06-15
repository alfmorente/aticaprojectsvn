#include "RosNode_Electric.h"

/*******************************************************************************
 * CONSTRUCTOR DE LA CLASE
 ******************************************************************************/

RosNode_Electric::RosNode_Electric() {
    this->setEMNodeStatus(NODESTATUS_INIT);
    this->dElectric = new ElectricConnectionManager();
}

/*******************************************************************************
 * INICIALIZADOR DE ARTEFACTOS ROS
 ******************************************************************************/

void RosNode_Electric::initROS() {
    ros::NodeHandle nh;
    this->clientVehicleStatus = nh.serviceClient<CITIUS_Control_Electric::srv_vehicleStatus>("vehicleStatus");
    this->pubElectricInfo = nh.advertise<CITIUS_Control_Electric::msg_electricInfo>("electricInfo",1000);
    this->pubCommand = nh.advertise<CITIUS_Control_Electric::msg_command>("command",1000);
}


/*******************************************************************************
 * GETTER AND SETTER NECESARIOS
 ******************************************************************************/

// Get del estado del nodo
short RosNode_Electric::getEMNodeStatus(){
    return this->emNodeStatus;
}

// Set del estado del nodo
void RosNode_Electric::setEMNodeStatus(short newEMNodeStatus){
    this->emNodeStatus = newEMNodeStatus;
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
                // Envio de confirmacion de apagado
                this->getDriverMng()->setParam(TURN_OFF,1);
                // Desconexion del vehiculo
                this->getDriverMng()->disconnectVehicle();
                // Cambio de estado de nodo
                this->setEMNodeStatus(NODESTATUS_OFF);
                // Cambio de estado vehiculo
                ros::NodeHandle nh;
                nh.setParam("vehicleStatus",OPERATION_MODE_APAGANDO);
                
            }
            
        }else{
            CITIUS_Control_Electric::msg_electricInfo vMsg;
            vMsg.id_device = frame.element;
            vMsg.value = frame.value;
            this->getPubElectricInfo().publish(vMsg);
        }
    }else{
        ROS_INFO("[Control] Electric - Trama invalida recibida");
    }
    
}