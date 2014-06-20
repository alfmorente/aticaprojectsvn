#include "DrivingConnectionManager.h"
#include "constant.h"

/*******************************************************************************
 * CONSTRUCTOR DE LA CLASE
 ******************************************************************************/

DrivingConnectionManager::DrivingConnectionManager() {
    this->socketDescriptor = -1;
}

/*******************************************************************************
 * MANEJO DEL VEHICULO
 ******************************************************************************/

// Conexión con dispositivo

bool DrivingConnectionManager::connectVehicle() {
    // Creacion y apertura del socket
    this->socketDescriptor = socket(AF_INET, SOCK_STREAM, 0);
    if (this->socketDescriptor < 0) {
        ROS_INFO("[Control] Driving - Imposible crear socket para comunicacion con Payload de Conducción");
        return false;
    } else {
        struct hostent *he;
        /* estructura que recibirá información sobre el nodo remoto */

        struct sockaddr_in server;
        /* información sobre la dirección del servidor */

        if ((he = gethostbyname(IP_PAYLOAD_CONDUCCION_DRIVING)) == NULL) {
            /* llamada a gethostbyname() */
            ROS_INFO("[Control] Driving - Imposible obtener el nombre del servidor socket");
            exit(-1);
        }
        
        if ((this->socketDescriptor = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
            /* llamada a socket() */
            ROS_INFO("[Control] Driving - Imposible crear socket para comunicacion con Payload de Conducción");
            exit(-1);
        }

        server.sin_family = AF_INET;
        server.sin_port = htons(PORT_PAYLOAD_CONDUCCION_DRIVING);
        /* htons() es necesaria nuevamente ;-o */
        server.sin_addr = *((struct in_addr *) he->h_addr);
        /*he->h_addr pasa la información de ``*he'' a "h_addr" */
        bzero(&(server.sin_zero), 8);

        if (connect(this->socketDescriptor, (struct sockaddr *) &server, sizeof (struct sockaddr)) == -1) {
            /* llamada a connect() */
            ROS_INFO("[Control] Driving - Imposible conectar con socket socket para comunicacion con Payload de Conducción");
            exit(-1);

        }
        ROS_INFO("[Control] Driving - Socket con Payload de Conduccion creado con exito y conectado");
        // Test if the socket is in non-blocking mode:
        // Put the socket in non-blocking mode:
        if (fcntl(this->socketDescriptor, F_SETFL, fcntl(this->socketDescriptor, F_GETFL) | O_NONBLOCK) >= 0) {
            ROS_INFO("[Control] Driving - Socket establecido como no bloqueante en operaciones L/E");
        }else{
            ROS_INFO("[Control] Driving - Imposible establecer socket como no bloqueante en operaciones L/E");
        }
    }
    return true;
}

// Desconexión con dispositivo

bool DrivingConnectionManager::disconnectVehicle() {
    // Cierre del socket
    shutdown(this->socketDescriptor, 2);
    close(this->socketDescriptor);
    ROS_INFO("[Control] Driving - Socket cerrado correctamente");
    return true;
}

// Mandar comando de control

void DrivingConnectionManager::setParam(short idParam, float value) {
    // Estructura de envio
    FrameDriving fd;
    fd.instruction = SET;
    fd.element = idParam;
    fd.value = value;
    // Buffer de envio
    char bufData[6];
    // Rellenado del buffer
    memcpy(&bufData[0], &fd.instruction, sizeof (fd.instruction));
    memcpy(&bufData[2], &fd.element, sizeof (fd.element));
    memcpy(&bufData[4], &fd.value, sizeof (fd.value));
    send(this->socketDescriptor, bufData, sizeof(bufData), 0);
    usleep(100);
}

// Solicitar informacion mediante comando
void DrivingConnectionManager::getParam(short idParam){
    // Estructura de envio
    FrameDriving fd;
    fd.instruction = GET;
    fd.element = idParam;
    fd.value = 0;
    // Buffer de envio
    char bufData[6];
    // Rellenado del buffer
    memcpy(&bufData[0], &fd.instruction, sizeof (fd.instruction));
    memcpy(&bufData[2], &fd.element, sizeof (fd.element));
    memcpy(&bufData[4], &fd.value, sizeof (fd.value));
    send(this->socketDescriptor, bufData, sizeof(bufData), 0);
    usleep(100);
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

// Get del descriptor de socket
int DrivingConnectionManager::getSocketDescriptor(){
    return this->socketDescriptor;
}