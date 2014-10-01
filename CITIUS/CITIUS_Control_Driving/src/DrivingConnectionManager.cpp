#include "DrivingConnectionManager.h"



/*******************************************************************************
 * CONSTRUCTOR DE LA CLASE
 ******************************************************************************/

DrivingConnectionManager::DrivingConnectionManager() {
    socketDescriptor = -1;
    countMsg = 1;
    vehicleInfo.lights = false;
    vehicleInfo.blinkerLeft = false;
    vehicleInfo.blinkerRight = false;
    vehicleInfo.dipsp = false;
    vehicleInfo.dipsr = false;
    vehicleInfo.dipss = false;
    vehicleInfo.klaxon = false;
    vehicleInfo.brake = 0;
    vehicleInfo.thottle = 0;
    vehicleInfo.steering = 0;
    vehicleInfo.parkingBrake = false;
    vehicleInfo.gear = 0;
    vehicleInfo.speed = 0;
    vehicleInfo.motorTemperature = 0;
    vehicleInfo.motorRPM = 0;
    driveAlarms = 0x0000;
    steeringAlarms = 0x0000;
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
            ROS_INFO("[Control] Driving - Imposible crear socket para comunicacion con Payload de Conduccion");
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
            ROS_INFO("[Control] Driving - Imposible conectar con socket socket para comunicacion con Payload de Conduccion");
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

// Mandar comando a vehiculo
void DrivingConnectionManager::sendToVehicle(FrameDriving frame){
    
    // Buffer de envio
    char bufData[8];
    
    // Rellenado del buffer
    memcpy(&bufData[0], &frame.instruction, sizeof (frame.instruction));
    memcpy(&bufData[2], &frame.id_instruccion, sizeof (frame.id_instruccion));
    memcpy(&bufData[4], &frame.element, sizeof (frame.element));
    memcpy(&bufData[6], &frame.value, sizeof (frame.value));
    
    // Envio via socket
    send(socketDescriptor, bufData, sizeof(bufData), 0);
    usleep(1000);  
}

// Solicitar informacion basica de vehiculo

void DrivingConnectionManager::reqVehicleInfo(bool full) {

    FrameDriving frame;

    // Comun 
    frame.instruction = GET;
    frame.id_instruccion = -1;
    frame.value = -1;

    // Acelerador
    frame.element = THROTTLE;
    sendToVehicle(frame);
    // Freno 
    frame.element = BRAKE;
    sendToVehicle(frame);
    // Freno de mano
    frame.element = HANDBRAKE;
    sendToVehicle(frame);
    // Direccion
    frame.element = STEERING;
    sendToVehicle(frame);
    // Marcha
    frame.element = GEAR;
    sendToVehicle(frame);
    // Velocidad de crucero
    frame.element = CRUISING_SPEED;
    sendToVehicle(frame);
    // Temperatura de motor
    frame.element = MOTOR_TEMPERATURE;
    sendToVehicle(frame);
    // rpm de motor
    frame.element = MOTOR_RPM;
    sendToVehicle(frame);

    // Completa la peticion con elementos de señalizacion

    if (full) {
        // Luces de posicion
        frame.element = DIPSP;
        sendToVehicle(frame);
        // Luces cortas
        frame.element = DIPSS;
        sendToVehicle(frame);
        // Luces largas
        frame.element = DIPSR;
        sendToVehicle(frame);
        // Intermitente derecha
        frame.element = BLINKER_RIGHT;
        sendToVehicle(frame);
        // Intermitente izquierda
        frame.element = BLINKER_LEFT;
        sendToVehicle(frame);
        // Claxon
        frame.element = KLAXON;
        sendToVehicle(frame);

    }

}

bool DrivingConnectionManager::checkForVehicleMessages() {
    
    char bufData[8];
    
    if (recv(socketDescriptor, bufData, sizeof (bufData), 0) > 0) {
        // Estructura de recepcion
        FrameDriving fdr;
        
        // Rellenado del buffer
        memcpy(&fdr.instruction, &bufData[0], sizeof (fdr.instruction));
        memcpy(&fdr.id_instruccion, &bufData[2], sizeof (fdr.id_instruccion));
        memcpy(&fdr.element, &bufData[4], sizeof (fdr.element));
        memcpy(&fdr.value, &bufData[6], sizeof (fdr.value));

        if (fdr.instruction == ACK) {
            
            printf("Recibido ACK\n");
            if(isCriticalInstruction(fdr.element))
                informResponse(true, fdr.id_instruccion);
            
        } else if (fdr.instruction == NACK) {
            
            printf("Recibido NACK\n");
            
            RtxStruct rtxList = informResponse(false, fdr.id_instruccion);
            
            for(int i = 0; i < rtxList.numOfMsgs; i++){
                sendToVehicle((FrameDriving)rtxList.msgs.at(i));
            }
            
        } else if (fdr.instruction == INFO) {
                        
            if(fdr.element == STEERING_ALARMS || fdr.element == DRIVE_ALARMS){
            
                setAlarmsInfo(fdr.element, fdr.value);
                
            }else{ // INFO corriente
                
                setVehicleInfo(fdr.element,fdr.value);
                
            }
            // TODO ALARMAS
                        
        }
        
        return true;
    }else{
        return false;
    }
}

/*******************************************************************************
 * GETTER Y SETTER NECESARIOS
 ******************************************************************************/

// Get del descriptor de socket

int DrivingConnectionManager::getSocketDescriptor(){
    return socketDescriptor;
}

// Get de la ultima informacion recibida del vehiculo

DrivingInfo DrivingConnectionManager::getVehicleInfo(bool full) {
    DrivingInfo ret = vehicleInfo;
    
    if(full)
        ret.lights = true;
    else
        ret.lights = false;
    
    return ret;
}

// Informacion de vehiculo

void DrivingConnectionManager::setVehicleInfo(short id_device, short value){
    switch(id_device){
        case THROTTLE:
            vehicleInfo.thottle = value;
            break;
        case BRAKE:
            vehicleInfo.brake = value;
            break;
        case HANDBRAKE:
            vehicleInfo.parkingBrake = (bool) value;
            break;
        case STEERING:
            vehicleInfo.steering = value;
            break;
        case GEAR:
            vehicleInfo.gear = value;
            break;
        case MOTOR_RPM:
            vehicleInfo.motorRPM = value;
            break;
        case CRUISING_SPEED:
            vehicleInfo.speed = value;
            break;
        case MOTOR_TEMPERATURE:
            vehicleInfo.motorTemperature = value;
            break;
        case BLINKER_LEFT:
            vehicleInfo.blinkerLeft = (bool) value;
            break;
        case BLINKER_RIGHT:
            vehicleInfo.blinkerRight = (bool) value;
            break;
        case KLAXON:
            vehicleInfo.klaxon = (bool) value;
            break;
        case DIPSP:
            vehicleInfo.dipsp = (bool) value;
            break;
        case DIPSS:
            vehicleInfo.dipss = (bool) value;
            break;
        case DIPSR:
            vehicleInfo.dipsr = (bool) value;
            break;
        default: 
            break;
    }
}

// Contador de mensajes criticos

short DrivingConnectionManager::getCountCriticalMessages(){
    return countMsg;
}

void DrivingConnectionManager::setCountCriticalMessages(short cont) {
    countMsg = cont;
    // Contador: 1 .. 1024
    if(countMsg == 1025)
        countMsg = 1;
}

// Alarmas

short DrivingConnectionManager::getDriveAlarms(){
    return driveAlarms;
}

short DrivingConnectionManager::getSteeringAlarms(){
    return steeringAlarms;
}

void DrivingConnectionManager::setAlarmsInfo(short element, short value) {
    if(element == DRIVE_ALARMS)
        driveAlarms = value;
    else if(element == STEERING_ALARMS)
        steeringAlarms = value;
}


/*******************************************************************************
 * METODOS PROPIOS
 *******************************************************************************/

bool DrivingConnectionManager::isCriticalInstruction(short element) {
    if (element == RESET
            || element == GEAR
            || element == MT_GEAR
            || element == THROTTLE
            || element == MT_THROTTLE
            || element == CRUISING_SPEED
            || element == HANDBRAKE
            || element == MT_HANDBRAKE
            || element == BRAKE
            || element == MT_BRAKE
            || element == STEERING
            || element == MT_STEERING) {
        return true;
    } else {
        return false;
    }
}

/*******************************************************************************
 * MANEJO DE LA COLA DE MENSAJES
 *******************************************************************************/

void DrivingConnectionManager::addToQueue(FrameDriving frame){
    messageQueue.push_back(frame);
}

RtxStruct DrivingConnectionManager::informResponse(bool ack, short id_instruction){
    
    RtxStruct ret;
    ret.numOfMsgs = 0;
    
    // Se situa el iterador al principio de la cola
    vector<FrameDriving>::iterator it = messageQueue.begin();
    
    if(ack){ // ACK
        
        if(id_instruction == (*it).id_instruccion){ // Primer elemento y requerido coinciden
            
            // Se elimina el primer elemento
            messageQueue.erase(it);
            
        }else if(id_instruction > (*it).id_instruccion){ // Confirmacion de varios elementos
            
            while(id_instruction >= (*it).id_instruccion){
                
                messageQueue.erase(it);
                
            }
        }
                
    } else { // NACK
           
        while(it != messageQueue.end()){
            
            // Se incluyen en la respuesta todos los  mensajes no confirmados
            // para retransmitir
            ret.numOfMsgs++;
            ret.msgs.push_back((*it));
            it++;
            
        }
        
    }
    return ret;
}