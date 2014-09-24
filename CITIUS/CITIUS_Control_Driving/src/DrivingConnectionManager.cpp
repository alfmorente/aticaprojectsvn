#include "DrivingConnectionManager.h"
#include "constant.h"

/*******************************************************************************
 * CONSTRUCTOR DE LA CLASE
 ******************************************************************************/

DrivingConnectionManager::DrivingConnectionManager() {
    socketDescriptor = -1;
    countMsg = 1;
}

/*******************************************************************************
 * MANEJO DEL VEHICULO
 ******************************************************************************/

// Conexión con dispositivo

bool DrivingConnectionManager::connectVehicle() {
    // Creacion y apertura del socket
    socketDescriptor = socket(AF_INET, SOCK_STREAM, 0);
    if (socketDescriptor < 0) {
        ROS_INFO("[Control] Driving - Imposible crear socket para comunicacion con Payload de Conduccion");
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
        
        if ((socketDescriptor = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
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

        if (connect(socketDescriptor, (struct sockaddr *) &server, sizeof (struct sockaddr)) == -1) {
            /* llamada a connect() */
            ROS_INFO("[Control] Driving - Imposible conectar con socket socket para comunicacion con Payload de Conducción");
            exit(-1);

        }
        ROS_INFO("[Control] Driving - Socket con Payload de Conduccion creado con exito y conectado");
        // Test if the socket is in non-blocking mode:
        // Put the socket in non-blocking mode:
        if (fcntl(socketDescriptor, F_SETFL, fcntl(socketDescriptor, F_GETFL) | O_NONBLOCK) >= 0) {
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
    shutdown(socketDescriptor, 2);
    close(socketDescriptor);
    ROS_INFO("[Control] Driving - Socket cerrado correctamente");
    return true;
}

// Mandar comando de control

void DrivingConnectionManager::setParam(short idParam, float value) {
   
    // Estructura de envio
    FrameDriving fd;
    fd.instruction = SET;
    fd.id_instruccion = countMsg;
    fd.element = idParam;
    fd.value = value;
    
    // Incremento de contador de mensajes
    countMsg++;
    
    // Buffer de envio
    char bufData[8];
    
    // Rellenado del buffer
    memcpy(&bufData[0], &fd.instruction, sizeof (fd.instruction));
    memcpy(&bufData[2], &fd.id_instruccion, sizeof (fd.id_instruccion));
    memcpy(&bufData[4], &fd.element, sizeof (fd.element));
    memcpy(&bufData[6], &fd.value, sizeof (fd.value));
    
    // Envio via socket
    send(socketDescriptor, bufData, sizeof(bufData), 0);
    usleep(100);
    
    // Envio a la cola de mensajes a la espera de ACK
    messageQueue.queueMsgdata.push(fd);

}

// Solicitar informacion mediante comando
short DrivingConnectionManager::getParam(short idParam){
    
    // Valor de devolucion
    short dev = -500;
    // Estructura de envio
    FrameDriving fd;
    fd.instruction = GET;
    fd.element = idParam;
    fd.value = 0;
    // Buffer de envio/recepcion
    char bufData[6];
    // Rellenado del buffer
    memcpy(&bufData[0], &fd.instruction, sizeof (fd.instruction));
    memcpy(&bufData[2], &fd.element, sizeof (fd.element));
    memcpy(&bufData[4], &fd.value, sizeof (fd.value));
    send(socketDescriptor, bufData, sizeof (bufData), 0);
    usleep(100);
    // Variables de control de la recepcion
    bool received = false;
    bool timeout = false;
    while (!received && !timeout) {
        if (recv(socketDescriptor, bufData, sizeof (bufData), 0) > 0) {
            // Estructura de recepcion
            FrameDriving fdr;
            // Rellenado del buffer
            memcpy(&fdr.instruction, &bufData[0], sizeof (fdr.instruction));
            memcpy(&fdr.element, &bufData[2], sizeof (fdr.element));
            memcpy(&fdr.value, &bufData[4], sizeof (fdr.value));
            if (fdr.instruction == INFO || fdr.element == idParam) {
                received = true;
                dev = fdr.value;
            } else {
                // Comprobacion y tratamiento de alarmas
                // TODO
            }
        }
    }
    return dev;

    
}

// Solicitar informacion basica de vehiculo
DrivingInfo DrivingConnectionManager::reqBasicVehicleInfo(){
    DrivingInfo ret;
    
    ret.thottle = getParam(THROTTLE);
    ret.brake = getParam(BRAKE);
    if(getParam(HANDBRAKE)==0) ret.parkingBrake = false; else ret.parkingBrake = true;
    ret.steering = getParam(STEERING);
    ret.gear = getParam(GEAR);
    ret.speed = getParam(CRUISING_SPEED);
    ret.motorRPM = getParam(MOTOR_RPM);
    ret.motorTemperature = getParam(MOTOR_TEMPERATURE);
    ret.lights = false;
    
    return ret;
    
}

// Solicitar informacion completa de vehiculo
DrivingInfo DrivingConnectionManager::reqFullVehicleInfo(){
    
    DrivingInfo ret;
    
    ret.thottle = getParam(THROTTLE);
    ret.brake = getParam(BRAKE);
    if(getParam(HANDBRAKE) == 0) ret.parkingBrake = false; else ret.parkingBrake = true;
    ret.steering = getParam(STEERING);
    ret.gear = getParam(GEAR);
    ret.speed = getParam(CRUISING_SPEED);
    ret.motorRPM = getParam(MOTOR_RPM);
    ret.motorTemperature = getParam(MOTOR_TEMPERATURE);
    ret.lights = true;
    if(getParam(DIPSP) == 0) ret.dipsp = false; else ret.dipsp = true;
    if(getParam(DIPSR) == 0) ret.dipsr = false; else ret.dipsr = true;
    if(getParam(DIPSS) == 0) ret.dipss = false; else ret.dipss = true;
    if(getParam(KLAXON) == 0) ret.klaxon = false; else ret.klaxon = true;
    if(getParam(BLINKER_RIGHT) == 0) ret.blinkerRight = false; else ret.blinkerRight = true;
    if(getParam(BLINKER_LEFT) == 0) ret.blinkerLeft = false; else ret.blinkerLeft = true;
    if(getParam(BLINKER_EMERGENCY) == 1){
        ret.blinkerLeft = true; 
        ret.blinkerRight = true;
    }

    return ret;
    
}

/*******************************************************************************
 * GETTER Y SETTER NECESARIOS
 ******************************************************************************/

// Get del descriptor de socket
int DrivingConnectionManager::getSocketDescriptor(){
    return socketDescriptor;
}