

#include "CITIUS_Control_Driving/DrivingConnectionManager.h"
#include "CITIUS_Control_Driving/MenuHandler.h"

using namespace std;

// Rutina de suscripcion a msgcommandVehicle

void fnc_subs_command(CITIUS_Control_Driving::msg_command msg) {
    ROS_INFO("******************************************");
    ROS_INFO("Recibido mensaje de comando de telecontrol");
    ROS_INFO("Comando: %d - Valor: %d",msg.id_device,msg.value);
    ROS_INFO("******************************************");
    
    // TODO
    // Conversion de comando y envio por socket
}

// Rutina de servicio a nodeStatus

bool fcn_serv_nodeStatus(CITIUS_Control_Driving::srv_nodeStatus::Request &rq, CITIUS_Control_Driving::srv_nodeStatus::Response &rsp) {
    ROS_INFO("Recibida peticion de servicio de cambio de nodo");
    // TODO
    // Tratamiento del cambio de estado del nodo y generacion de respuesta
    return true;
}

/*******************************************************************************
 *                          IMPLEMENTACION DE LA CLASE                        *
 ******************************************************************************/

// Constructor de la clase

DrivingConnectionManager::DrivingConnectionManager() {
    
    // Inicializacion del estado del nodo;
    this->nodeStatus = NODESTATUS_INIT;
    ROS_INFO("[Control] Driving :: Cambio a estado INIT");

    // Creacion y apertura del socket
    // TODO

    // Inicializacion de publicadores
    this->publisher_vehicleInformation = this->nh.advertise<CITIUS_Control_Driving::msg_vehicleInformation>("vehicleInformation", 1000);
    // Inicializacion de suscriptores
    this->subscriber_command = this->nh.subscribe("command", 1000, fnc_subs_command);
    // Inicializacion de servidores
    this->server_nodeState = this->nh.advertiseService("nodeStateDriving", fcn_serv_nodeStatus);
    
    // Inicializacion del estado del nodo;
    this->nodeStatus = NODESTATUS_OK;
    ROS_INFO("[Control] Driving :: Cambio a estado OK");
}

// Actuacion sobre atributos
void DrivingConnectionManager::setNodeStatus(short newStatus) {
    this->nodeStatus = newStatus;
}

short DrivingConnectionManager::getNodeStatus() {
    return this->nodeStatus;
}

// Main del nodo
int main(int argc, char** argv) {
    
    // Iniciacion del middleware (ROS) para el nodo Driving
    ros::init(argc,argv,"ROSNODE_Control_Driving");
    
    // Con la creacion de dcm, se crean e inician suscriptores, publicadores,
    // y servicios. Tambien se abre el socket y se pone a disposicion del nodo.
    DrivingConnectionManager *dcm = new DrivingConnectionManager();
    
    // Bucle principal. Control C + Estado del nodo
    while(ros::ok() && (dcm->getNodeStatus()!=NODESTATUS_OFF)){    
        
        // Comprobacion mensajeria (topics/servicios)
        ros::spinOnce();
        
        // Comprobacion mensajeria (socket)
        //TODO
    }
    return 0;
}
