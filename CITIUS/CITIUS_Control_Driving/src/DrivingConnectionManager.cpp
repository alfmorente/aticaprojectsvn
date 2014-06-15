#include "DrivingConnectionManager.h"
#include "constant.h"

/*******************************************************************************
 * CONSTRUCTOR DE LA CLASE
 ******************************************************************************/

DrivingConnectionManager::DrivingConnectionManager() {
    this->setSocketDescriptor(-1);
}

/*******************************************************************************
 * MANEJO DEL VEHICULO
 ******************************************************************************/

// Conexión con dispositivo
bool DrivingConnectionManager::connectVehicle(){
    // Creacion y apertura del socket
    this->setSocketDescriptor(socket(AF_INET, SOCK_STREAM, 0));
    if (this->getSocketDescriptor() < 0) {
        ROS_INFO("[Control] Driving - Imposible crear socket para comunicacion con Payload de Conducción");
        return false;
    }else{
        // Establecimiento de modo no bloqueante en operaciones de L/E
        if ( fcntl(this->getSocketDescriptor(), F_SETFL, O_NONBLOCK) < 0 ){
            ROS_INFO("[Control] Driving - Imposible establecer socket como no bloqueante en operaciones de L/E");
            return false;
        }else{
            ROS_INFO("[Control] Driving - Socket establecido como no bloqueante en operaciones de L/E");
            
            struct sockaddr_in socketAddr;
            socketAddr.sin_family = AF_INET;
            socketAddr.sin_addr.s_addr = inet_addr(IP_PAYLOAD_CONDUCCION_DRIVING);
            socketAddr.sin_port = htons(PORT_PAYLOAD_CONDUCCION_DRIVING);
            // Conexión
            if (connect(this->getSocketDescriptor(), (struct sockaddr *) &socketAddr, sizeof (socketAddr)) < 0) {
                ROS_INFO("[Control] Driving :: Imposible conectar con socket para comunicacion con Payload de Conduccion");
                shutdown(this->getSocketDescriptor(), 2);
                close(this->getSocketDescriptor());
                return false;
            } else {   
                ROS_INFO("[Control] Driving :: Conexión con socket para comunicacion con Payload de Conduccion establecida");
            }
        }
    }
    return true;
}

// Desconexión con dispositivo

bool DrivingConnectionManager::disconnectVehicle() {
    // Cierre del socket
    shutdown(this->getSocketDescriptor(), 2);
    close(this->getSocketDescriptor());
    ROS_INFO("[Control] Driving :: Socket cerrado correctamente");
    return true;
}

// Mandar comando de control
void DrivingConnectionManager::setParam(short idParam, float value){
    FrameDriving fd;
    fd.instruction = SET;
    fd.element = idParam;
    fd.value = value;
    if (send(this->getSocketDescriptor(), &fd, sizeof (fd), 0) < 0) {
        ROS_INFO("[Control] Driving :: No se ha podido enviar una trama por el socket");
    } else {
        ROS_INFO("[Control] Driving :: Enviado comando a Payload de conduccion");
    }
}

// Solicitar informacion mediante comando
void DrivingConnectionManager::getParam(short idParam){
    FrameDriving fd;
    fd.instruction = GET;
    fd.element = idParam;
    if (send(this->getSocketDescriptor(), &fd, sizeof (fd), 0) < 0) {
        ROS_INFO("[Control] Driving :: No se ha podido enviar una trama por el socket");
    } else {
        ROS_INFO("[Control] Driving :: Enviado comando a Payload de conduccion");
    }
}

// Solicitar informacion completa de vehiculo
void DrivingConnectionManager::reqVehicleInfo(){
    this->getParam(THROTTLE);
    this->getParam(BRAKE);
    this->getParam(HANDBRAKE);
    this->getParam(STEERING);
    this->getParam(GEAR);
    this->getParam(DIPSP);
    this->getParam(DIPSR);
    this->getParam(DIPSS);
    this->getParam(KLAXON);
    this->getParam(BLINKER_RIGHT);
    this->getParam(BLINKER_LEFT);
    this->getParam(BLINKER_EMERGENCY);
    this->getParam(MOTOR_RPM);
    this->getParam(MOTOR_TEMPERATURE);
    this->getParam(CRUISING_SPEED);
    
}

/*******************************************************************************
 * GETTER Y SETTER NECESARIOS
 ******************************************************************************/

// Set del descriptor de socket
void DrivingConnectionManager::setSocketDescriptor(int newSocketDescriptor){
    this->socketDescriptor = newSocketDescriptor;
}

// Get del descriptor de socket
int DrivingConnectionManager::getSocketDescriptor(){
    return this->socketDescriptor;
}