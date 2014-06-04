#include "CITIUS_Control_Driving/DrivingConnectionManager.h"
#include "CITIUS_Control_Driving/MenuHandler.h"

using namespace std;

// Rutina de suscripcion a msgcommandVehicle

void DrivingConnectionManager::fnc_subs_command(CITIUS_Control_Driving::msg_command msg) {
    // Criba y comprobacion de valores dentro de rango
    ROS_INFO("%d - %d", msg.id_device, msg.value);
    // Conversion de comando y envio por socket
    FrameDriving frame;
    frame.id_device = msg.id_device;
    frame.value = msg.value;
    
    if (send(this->socketDescriptor, &frame, sizeof(frame), 0) < 0) {
        ROS_INFO("[Control] Driving :: No se ha podido enviar una trama por el socket");
    }else{
         ROS_INFO("[Control] Driving :: Enviado comando a Payload de conduccion");
    }
}

// Rutina de servicio a nodeStatus

bool DrivingConnectionManager::fcn_serv_nodeStatus(CITIUS_Control_Driving::srv_nodeStatus::Request &rq, CITIUS_Control_Driving::srv_nodeStatus::Response &rsp) {
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
    this->setNodeStatus(NODESTATUS_INIT);
    ROS_INFO("[Control] Driving :: Cambio a estado de nodo INIT");

    // Creacion y apertura del socket
    this->socketDescriptor = socket(AF_INET, SOCK_STREAM, 0);
    
    if (this->socketDescriptor < 0) {
        
        ROS_INFO("[Control] Driving :: Imposible crear socket para comunicacion con Payload de Conducción");
        
    }else{
        
        ROS_INFO("[Control] Driving :: Socket creado satisfactoriamente");
        
        // Establecimiento de modo no bloqueante en operaciones de L/E
        if ( fcntl(this->socketDescriptor, F_SETFL, O_NONBLOCK) < 0 ){
            
            ROS_INFO("[Control] Driving :: Imposible establecer socket como no bloqueante en operaciones de L/E");
            
        }else{
            
            ROS_INFO("[Control] Driving :: Socket establecido como no bloqueante en operaciones de L/E");
            struct sockaddr_in socketAddr;

            socketAddr.sin_family = AF_INET;
            socketAddr.sin_addr.s_addr = inet_addr(IP_PAYLOAD_CONDUCCION);
            socketAddr.sin_port = htons(PORT_PAYLOAD_CONDUCCION);
            
            // Establecimiento de la conexion
            if (connect(this->socketDescriptor, (struct sockaddr *) &socketAddr, sizeof (socketAddr)) < 0) {
                
                ROS_INFO("[Control] Driving :: Imposible conectar con socket para comunicacion con Payload de Conduccion");
                shutdown(this->socketDescriptor, 2);
                close(this->socketDescriptor);
                
            } else {
                
                ROS_INFO("[Control] Driving :: Conexión con socket para comunicacion con Payload de Conduccion establecida");
                
                // Inicializacion de publicadores
                this->publisher_vehicleInformation = this->nh.advertise<CITIUS_Control_Driving::msg_vehicleInformation>("vehicleInformation", 1000);
                // Inicializacion de suscriptores
                this->subscriber_command = this->nh.subscribe("command", 1000, &DrivingConnectionManager::fnc_subs_command,this);
                // Inicializacion de servidores
                this->server_nodeState = this->nh.advertiseService("nodeStateDriving", &DrivingConnectionManager::fcn_serv_nodeStatus,this);
                
                ROS_INFO("[Control] Driving :: Publicadores, suscriptores, servidores y clientes inicializados");
                
                // Inicializacion del estado del nodo;
                this->setNodeStatus(NODESTATUS_OK);
                ROS_INFO("[Control] Driving :: Cambio a estado de nodo OK");
            }
        }
    }
}

// Actuacion sobre atributos
void DrivingConnectionManager::setNodeStatus(short newStatus) {
    this->nodeStatus = newStatus;
}

short DrivingConnectionManager::getNodeStatus() {
    return this->nodeStatus;
}

void DrivingConnectionManager::setSocketDescriptor(int newSocketDescriptor) {
    this->socketDescriptor = newSocketDescriptor;
}

int DrivingConnectionManager::getSocketDescriptor() {
    return this->socketDescriptor;
}

ros::Publisher DrivingConnectionManager::getPublisherVehicleInformation(){
    return this->publisher_vehicleInformation;
}


// Main del nodo
int main(int argc, char** argv) {
    
    // Iniciacion del middleware (ROS) para el nodo Driving
    ros::init(argc,argv,"ROSNODE_Control_Driving");
    
    // Con la creacion de dcm, se crean e inician suscriptores, publicadores,
    // y servicios. Tambien se abre el socket y se pone a disposicion del nodo.
    DrivingConnectionManager *dcm = new DrivingConnectionManager();
    
    // Compronbacion de una correcta inicializacion
    if(dcm->getNodeStatus()==NODESTATUS_OK){
        
        FrameDriving recvFrame;
        
        // Bucle principal. Control+C y Estado del nodo
        while(ros::ok() && (dcm->getNodeStatus()!=NODESTATUS_OFF)){    

            // Comprobacion mensajeria (topics/servicios)
            ros::spinOnce();
            // Comprobacion mensajeria (socket)
            if (recv(dcm->getSocketDescriptor(), &recvFrame, sizeof (recvFrame), 0) >= 0) {
                ROS_INFO("Recibida una trama del Payload de Conduccion");
            }
            
        }
        
    }else{
        
        ROS_INFO("[Control] Driving :: Fallo en la inicializacion del nodo");
        ROS_INFO("[Control] Driving :: Nodo finalizado");
        
    }
    
    return 0;
}