#include "CITIUS_Control_Electric/ElectricConnectionManager.h"

/*******************************************************************************
 *******************************************************************************
 *          IMPLEMENTACION DE LA CLASE QUE IMPLEMENTA AL NODO ELECTRI          *
 *******************************************************************************
 ******************************************************************************/

/*******************************************************************************
 *                  CALLBACKS DE SUSCRIPTORES Y SERVICIOS                     *
 ******************************************************************************/

// Rutina de servicio a srv_nodeStatus
bool ElectricConnectionManager::fcn_serv_nodeStatus(CITIUS_Control_Electric::srv_nodeStatus::Request &rq, CITIUS_Control_Electric::srv_nodeStatus::Response &rsp) {
    
    // Solo se contempla la posibilidad de recibir una peticion de apagado por parte del nodo Manager
    // Comprobacion de que la solicitud es de apagado
    if(rq.status == NODESTATUS_OFF){
    
        ROS_INFO("[Control] Driving :: Finalizando nodo ROS...");
        
        // Cierre del socket
        shutdown(this->socketDescriptor, 2);
        close(this->socketDescriptor);
        
        // Cambio del estado del nodo
        this->setNodeStatus(NODESTATUS_OFF);
        
        // Respuesta del servidor
        rsp.confirmation = true;
        
        
    }else{
        
        ROS_INFO("[Control] Driving :: Descartada la peticion de cambio en el estado del nodo");
        rsp.confirmation = false;
    }
    
    return true;
}

/*******************************************************************************
 *                       CONSTRUCTOR -> INICIALIZADOR                          *
 ******************************************************************************/

// Constructor de la clase
ElectricConnectionManager::ElectricConnectionManager() {
    
    // Inicializacion del estado del nodo;
    this->setNodeStatus(NODESTATUS_INIT);
    ROS_INFO("[Control] Driving :: Cambio a estado de nodo INIT");

    // Creacion y apertura del socket
    this->socketDescriptor = socket(AF_INET, SOCK_STREAM, 0);
    
    if (this->socketDescriptor < 0) {
        
        ROS_INFO("[Control] Electric :: Imposible crear socket para comunicacion con Payload de Conducción");
        
    }else{
        
        ROS_INFO("[Control] Electric :: Socket creado satisfactoriamente");
        
        // Establecimiento de modo no bloqueante en operaciones de L/E
        if ( fcntl(this->socketDescriptor, F_SETFL, O_NONBLOCK) < 0 ){
            
            ROS_INFO("[Control] Electric :: Imposible establecer socket como no bloqueante en operaciones de L/E");
            
        }else{
            
            ROS_INFO("[Control] Electric :: Socket establecido como no bloqueante en operaciones de L/E");
            
            struct sockaddr_in socketAddr;
            socketAddr.sin_family = AF_INET;
            socketAddr.sin_addr.s_addr = inet_addr(IP_PAYLOAD_CONDUCCION_DRIVING);
            socketAddr.sin_port = htons(PORT_PAYLOAD_CONDUCCION_DRIVING);
            
            // Establecimiento de la conexion
            if (connect(this->socketDescriptor, (struct sockaddr *) &socketAddr, sizeof (socketAddr)) < 0) {
                
                ROS_INFO("[Control] Electric :: Imposible conectar con socket para comunicacion con Payload de Conduccion");
                shutdown(this->socketDescriptor, 2);
                close(this->socketDescriptor);
                
            } else {
                
                ROS_INFO("[Control] Driving :: Conexión con socket para comunicacion con Payload de Conduccion establecida");
                
                // Inicializacion de publicadores
                this->publisher_electricInformation = this->nh.advertise<CITIUS_Control_Electric::msg_electricInfo>("electricInformation", 1000);
                // Inicializacion de servidores
                this->server_nodeState = this->nh.advertiseService("nodeStateElectric", &ElectricConnectionManager::fcn_serv_nodeStatus,this);
                // Inicialización de clientes
                
                ROS_INFO("[Control] Electric :: Publicadores, suscriptores, servidores y clientes inicializados");
                
                // Inicializacion del estado del nodo;
                this->setNodeStatus(NODESTATUS_OK);
                ROS_INFO("[Control] Electric :: Cambio a estado de nodo OK");
            }
        }
    }
}

/*******************************************************************************
 *                        GETTER & SETTER NECESARIOS                          *
 ******************************************************************************/

// Actuacion sobre atributos
void ElectricConnectionManager::setNodeStatus(short newStatus) {
    this->nodeStatus = newStatus;
}

short ElectricConnectionManager::getNodeStatus() {
    return this->nodeStatus;
}

void ElectricConnectionManager::setSocketDescriptor(int newSocketDescriptor) {
    this->socketDescriptor = newSocketDescriptor;
}

int ElectricConnectionManager::getSocketDescriptor() {
    return this->socketDescriptor;
}

ros::Publisher ElectricConnectionManager::getPublisherElectricInformation(){
    return this->publisher_electricInformation;
}

/*******************************************************************************
 *               GESTION DE MENSAJES RECIBIDOS VIA SOCKET                     *
 ******************************************************************************/

// Tratamiento de mensajes recibidos del Payload de Conduccion
void ElectricConnectionManager::messageManager(FrameDriving fd){
    
    // Comprobacion de que se trata de una instruccion INFO
    if(fd.instruction == INFO){
        
        // Comprobacion de que son alarmas
        if(fd.element == DRIVE_ALARMS){
            // Gestion de las alarmas
            // TODO
            
        }else{
            
            // Publicacion de informacion del vehiculo
            CITIUS_Control_Electric::msg_electricInfo vMsg;
            vMsg.id_device = fd.element;
            vMsg.value = fd.value;
            this->getPublisherElectricInformation().publish(vMsg);
        
        }
    
    }else{
    
        ROS_INFO("[Control] Electric :: Se ha recibido una instruccion distinta de INFO del Payload de conduccion");
    
    }
}

/*******************************************************************************
 *                   SOLICITUD DE INFORMACION DE VEHICULO                     *
 ******************************************************************************/

void ElectricConnectionManager::reqVehicleInformation() {
    
    // Montaje de instrucciones de solicitud y envio por socket
    FrameDriving frame;
    
    frame.instruction = GET;
    frame.element = GEAR;
    frame.value = 0;
    
    // TODO
    // Repetir proceso con todo lo que se desee consultar
    
    if (send(this->socketDescriptor, &frame, sizeof (frame), 0) < 0) {
        ROS_INFO("[Control] Electric :: No se ha podido enviar solicitud de informacion por el socket a Payload de conduccion");
    } else {
        ROS_INFO("[Control] Electric :: Enviada solicitud de informacion a Payload de conduccion");
    }
    
}
