#include "CITIUS_Control_Driving/DrivingConnectionManager.h"

using namespace std;

/*******************************************************************************
 *******************************************************************************
 *          IMPLEMENTACION DE LA CLASE QUE IMPLEMENTA AL NODO DRIVING          *
 *******************************************************************************
 ******************************************************************************/

/*******************************************************************************
 *                  CALLBACKS DE SUSCRIPTORES Y SERVICIOS                     *
 ******************************************************************************/

// Rutina de suscripcion a msg_commandVehicle
void DrivingConnectionManager::fnc_subs_command(CITIUS_Control_Driving::msg_command msg) {
    
    // Comprobacion de que el vehículo se encuentra en estado de operacion CONDUCCION
    int vehicleStatus;
    this->nh.getParam("vehicleStatus",vehicleStatus);
    if(vehicleStatus == OPERATION_MODE_CONDUCCION){
        
        // Comprobacion de valores fuera de rango
        if(this->checkCommandInterval(msg.id_device,msg.value)){

            // Conversion de comando y envio por socket
            FrameDriving frame;
            frame.instruction = SET;
            frame.element = msg.id_device;
            frame.value = msg.value;

            if (send(this->socketDescriptor, &frame, sizeof(frame), 0) < 0) {
                ROS_INFO("[Control] Driving :: No se ha podido enviar una trama por el socket");
            }else{
                 ROS_INFO("[Control] Driving :: Enviado comando a Payload de conduccion");
            }

        }else{
            ROS_INFO("[Control] Driving :: Se descarta comando por valor fuera de rango");
        }
    
    }else{
    
        ROS_INFO("[Control] Driving :: Se descarta comando por vehiculo en modo de nodo CONDUCCION");
    
    }
}

// Rutina de servicio a srv_nodeStatus
bool DrivingConnectionManager::fcn_serv_nodeStatus(CITIUS_Control_Driving::srv_nodeStatus::Request &rq, CITIUS_Control_Driving::srv_nodeStatus::Response &rsp) {
    
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
            socketAddr.sin_addr.s_addr = inet_addr(IP_PAYLOAD_CONDUCCION_DRIVING);
            socketAddr.sin_port = htons(PORT_PAYLOAD_CONDUCCION_DRIVING);
            
            // Establecimiento de la conexion
            if (connect(this->socketDescriptor, (struct sockaddr *) &socketAddr, sizeof (socketAddr)) < 0) {
                
                ROS_INFO("[Control] Driving :: Imposible conectar con socket para comunicacion con Payload de Conduccion");
                shutdown(this->socketDescriptor, 2);
                close(this->socketDescriptor);
                
            } else {
                
                ROS_INFO("[Control] Driving :: Conexión con socket para comunicacion con Payload de Conduccion establecida");
                
                // Inicializacion de publicadores
                this->publisher_vehicleInfo = this->nh.advertise<CITIUS_Control_Driving::msg_vehicleInfo>("vehicleInfo", 1000);
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

/*******************************************************************************
 *                        GETTER & SETTER NECESARIOS                          *
 ******************************************************************************/

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

ros::Publisher DrivingConnectionManager::getPublisherVehicleInfo(){
    return this->publisher_vehicleInfo;
}

/*******************************************************************************
 *               GESTION DE MENSAJES RECIBIDOS VIA SOCKET                     *
 ******************************************************************************/

// Tratamiento de mensajes recibidos del Payload de Conduccion
void DrivingConnectionManager::messageManager(FrameDriving fd){
    
    // Comprobacion de que se trata de una instruccion INFO
    if(fd.instruction == INFO){
        
        // Comprobacion de que son alarmas
        if(fd.element == DRIVE_ALARMS){
            // Gestion de las alarmas
            // TODO
            
        }else{
            
            // Publicacion de informacion del vehiculo
            CITIUS_Control_Driving::msg_vehicleInfo vMsg;
            vMsg.id_device = fd.element;
            vMsg.value = fd.value;
            this->getPublisherVehicleInfo().publish(vMsg);
        
        }
    
    }else{
    
        ROS_INFO("[Control] Driving :: Se ha recibido una instruccion distinta de INFO del Payload de conduccion ");
    
    }
}

// Comprobacion de comando con valores en rangos establecidos
bool DrivingConnectionManager::checkCommandInterval(short element, short value){
    bool ret = true;
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
    };
    return ret;
}

/*******************************************************************************
 *                   SOLICITUD DE INFORMACION DE VEHICULO                     *
 ******************************************************************************/

void DrivingConnectionManager::reqVehicleInformation() {
    
    // Montaje de instrucciones de solicitud y envio por socket
    FrameDriving frame;
    
    frame.instruction = GET;
    frame.element = GEAR;
    frame.value = 0;

    if (send(this->getSocketDescriptor(), &frame, sizeof (frame), 0) < 0) {
        ROS_INFO("[Control] Driving :: No se ha podido enviar solicitud de informacion por el socket a Payload de conduccion");
    } else {
        ROS_INFO("[Control] Driving :: Enviada solicitud de informacion a Payload de conduccion");
    }
    
    frame.element = THROTTLE;
    if (send(this->getSocketDescriptor(), &frame, sizeof (frame), 0) < 0) {
        ROS_INFO("[Control] Driving :: No se ha podido enviar solicitud de informacion por el socket a Payload de conduccion");
    } else {
        ROS_INFO("[Control] Driving :: Enviada solicitud de informacion a Payload de conduccion");
    }
    
    frame.element = CRUISING_SPEED;
    if (send(this->getSocketDescriptor(), &frame, sizeof (frame), 0) < 0) {
        ROS_INFO("[Control] Driving :: No se ha podido enviar solicitud de informacion por el socket a Payload de conduccion");
    } else {
        ROS_INFO("[Control] Driving :: Enviada solicitud de informacion a Payload de conduccion");
    }
    
    frame.element = HANDBRAKE;
    if (send(this->getSocketDescriptor(), &frame, sizeof (frame), 0) < 0) {
        ROS_INFO("[Control] Driving :: No se ha podido enviar solicitud de informacion por el socket a Payload de conduccion");
    } else {
        ROS_INFO("[Control] Driving :: Enviada solicitud de informacion a Payload de conduccion");
    }
    
    frame.element = BRAKE;
    if (send(this->getSocketDescriptor(), &frame, sizeof (frame), 0) < 0) {
        ROS_INFO("[Control] Driving :: No se ha podido enviar solicitud de informacion por el socket a Payload de conduccion");
    } else {
        ROS_INFO("[Control] Driving :: Enviada solicitud de informacion a Payload de conduccion");
    }
    
    frame.element = STEERING;
    if (send(this->getSocketDescriptor(), &frame, sizeof (frame), 0) < 0) {
        ROS_INFO("[Control] Driving :: No se ha podido enviar solicitud de informacion por el socket a Payload de conduccion");
    } else {
        ROS_INFO("[Control] Driving :: Enviada solicitud de informacion a Payload de conduccion");
    }
    
    frame.element = BLINKER_RIGHT;
    if (send(this->getSocketDescriptor(), &frame, sizeof (frame), 0) < 0) {
        ROS_INFO("[Control] Driving :: No se ha podido enviar solicitud de informacion por el socket a Payload de conduccion");
    } else {
        ROS_INFO("[Control] Driving :: Enviada solicitud de informacion a Payload de conduccion");
    }
    
    frame.element = BLINKER_LEFT;
    if (send(this->getSocketDescriptor(), &frame, sizeof (frame), 0) < 0) {
        ROS_INFO("[Control] Driving :: No se ha podido enviar solicitud de informacion por el socket a Payload de conduccion");
    } else {
        ROS_INFO("[Control] Driving :: Enviada solicitud de informacion a Payload de conduccion");
    }
    
    frame.element = BLINKER_EMERGENCY;
    if (send(this->getSocketDescriptor(), &frame, sizeof (frame), 0) < 0) {
        ROS_INFO("[Control] Driving :: No se ha podido enviar solicitud de informacion por el socket a Payload de conduccion");
    } else {
        ROS_INFO("[Control] Driving :: Enviada solicitud de informacion a Payload de conduccion");
    }
    
    frame.element = DIPSP;
    if (send(this->getSocketDescriptor(), &frame, sizeof (frame), 0) < 0) {
        ROS_INFO("[Control] Driving :: No se ha podido enviar solicitud de informacion por el socket a Payload de conduccion");
    } else {
        ROS_INFO("[Control] Driving :: Enviada solicitud de informacion a Payload de conduccion");
    }
    
    frame.element = DIPSS;
    if (send(this->getSocketDescriptor(), &frame, sizeof (frame), 0) < 0) {
        ROS_INFO("[Control] Driving :: No se ha podido enviar solicitud de informacion por el socket a Payload de conduccion");
    } else {
        ROS_INFO("[Control] Driving :: Enviada solicitud de informacion a Payload de conduccion");
    }
    
    frame.element = DIPSR;
    if (send(this->getSocketDescriptor(), &frame, sizeof (frame), 0) < 0) {
        ROS_INFO("[Control] Driving :: No se ha podido enviar solicitud de informacion por el socket a Payload de conduccion");
    } else {
        ROS_INFO("[Control] Driving :: Enviada solicitud de informacion a Payload de conduccion");
    }
    
    frame.element = KLAXON;
    if (send(this->getSocketDescriptor(), &frame, sizeof (frame), 0) < 0) {
        ROS_INFO("[Control] Driving :: No se ha podido enviar solicitud de informacion por el socket a Payload de conduccion");
    } else {
        ROS_INFO("[Control] Driving :: Enviada solicitud de informacion a Payload de conduccion");
    }
    
    frame.element = MOTOR_RPM;
    if (send(this->getSocketDescriptor(), &frame, sizeof (frame), 0) < 0) {
        ROS_INFO("[Control] Driving :: No se ha podido enviar solicitud de informacion por el socket a Payload de conduccion");
    } else {
        ROS_INFO("[Control] Driving :: Enviada solicitud de informacion a Payload de conduccion");
    }
    
    frame.element = MOTOR_TEMPERATURE;
    if (send(this->getSocketDescriptor(), &frame, sizeof (frame), 0) < 0) {
        ROS_INFO("[Control] Driving :: No se ha podido enviar solicitud de informacion por el socket a Payload de conduccion");
    } else {
        ROS_INFO("[Control] Driving :: Enviada solicitud de informacion a Payload de conduccion");
    }
    
    frame.element = DRIVE_ALARMS;
    if (send(this->getSocketDescriptor(), &frame, sizeof (frame), 0) < 0) {
        ROS_INFO("[Control] Driving :: No se ha podido enviar solicitud de informacion por el socket a Payload de conduccion");
    } else {
        ROS_INFO("[Control] Driving :: Enviada solicitud de informacion a Payload de conduccion");
    }
}