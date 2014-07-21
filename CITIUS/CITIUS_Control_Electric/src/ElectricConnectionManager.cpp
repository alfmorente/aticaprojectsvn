#include "ElectricConnectionManager.h"
#include "constant.h"

/*******************************************************************************
 * CONSTRUCTOR DE LA CLASE
 ******************************************************************************/

ElectricConnectionManager::ElectricConnectionManager() {
    this->setSocketDescriptor(-1);
}

/*******************************************************************************
 * MANEJO DEL VEHICULO
 ******************************************************************************/

// Conexión con dispositivo
bool ElectricConnectionManager::connectVehicle(){
    // Creacion y apertura del socket
    this->socketDescriptor = socket(AF_INET, SOCK_STREAM, 0);
    if (this->socketDescriptor < 0) {
        ROS_INFO("[Control] Electric - Imposible crear socket para comunicacion con Payload de Conduccion");
        return false;
    } else {
        struct hostent *he;
        /* estructura que recibirá información sobre el nodo remoto */

        struct sockaddr_in server;
        /* información sobre la dirección del servidor */

        if ((he = gethostbyname(IP_PAYLOAD_CONDUCCION_DRIVING)) == NULL) {
            /* llamada a gethostbyname() */
            ROS_INFO("[Control] Electric - Imposible obtener el nombre del servidor socket");
            exit(-1);
        }
        
        if ((this->socketDescriptor = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
            /* llamada a socket() */
            ROS_INFO("[Control] Electric - Imposible crear socket para comunicacion con Payload de Conduccion");
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
            ROS_INFO("[Control] Electric - Imposible conectar con socket socket para comunicacion con Payload de Conduccion");
            exit(-1);

        }
        ROS_INFO("[Control] Electric - Socket con Payload de Conduccion creado con exito y conectado");
        // Test if the socket is in non-blocking mode:
        // Put the socket in non-blocking mode:
        if (fcntl(this->socketDescriptor, F_SETFL, fcntl(this->socketDescriptor, F_GETFL) | O_NONBLOCK) >= 0) {
            ROS_INFO("[Control] Electric - Socket establecido como no bloqueante en operaciones L/E");
        }else{
            ROS_INFO("[Control] Electric - Imposible establecer socket como no bloqueante en operaciones L/E");
        }
    }
    return true;
}

// Desconexión con dispositivo

bool ElectricConnectionManager::disconnectVehicle() {
    // Cierre del socket
    shutdown(this->getSocketDescriptor(), 2);
    close(this->getSocketDescriptor());
    ROS_INFO("[Control] Electric :: Socket cerrado correctamente");
    return true;
}

// Solicitar control por comando
void ElectricConnectionManager::setParam(short idParam, float value){
    FrameDriving fd;
    fd.instruction = SET;
    fd.element = idParam;
    fd.value = value;
    if (send(this->getSocketDescriptor(), &fd, sizeof (fd), 0) < 0) {
        ROS_INFO("[Control] Electric :: No se ha podido enviar una trama por el socket");
    } else {
        ROS_INFO("[Control] Electric :: Enviado comando a Payload de conduccion");
    }
}

// Solicitar informacion mediante comando
void ElectricConnectionManager::getParam(short idParam){
    FrameDriving fd;
    fd.instruction = GET;
    fd.element = idParam;
    if (send(this->getSocketDescriptor(), &fd, sizeof (fd), 0) < 0) {
        ROS_INFO("[Control] Electric :: No se ha podido enviar una trama por el socket");
    } else {
        ROS_INFO("[Control] Electric :: Enviado comando a Payload de conduccion");
    }
}

// Solicitar informacion completa de vehiculo
void ElectricConnectionManager::reqElectricInfo(){
    this->getParam(BATTERY_LEVEL);
    this->getParam(BATTERY_VOLTAGE);
    this->getParam(BATTERY_CURRENT);
    this->getParam(BATTERY_TEMPERATURE);
    this->getParam(SUPPLY_CHECK);
}

/*******************************************************************************
 * GETTER Y SETTER NECESARIOS
 ******************************************************************************/

// Set del descriptor de socket
void ElectricConnectionManager::setSocketDescriptor(int newSocketDescriptor){
    this->socketDescriptor = newSocketDescriptor;
}

// Get del descriptor de socket
int ElectricConnectionManager::getSocketDescriptor(){
    return this->socketDescriptor;
}