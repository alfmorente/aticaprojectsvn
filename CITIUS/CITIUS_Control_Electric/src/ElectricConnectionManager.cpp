#include "ElectricConnectionManager.h"

/*******************************************************************************
 * CONSTRUCTOR DE LA CLASE
 ******************************************************************************/

ElectricConnectionManager::ElectricConnectionManager() {
    socketDescriptor = -1;
    countMsg = 1;
    electricInfo.battery_level = 0;
    electricInfo.battery_voltage = 0;
    electricInfo.battery_current = 0;
    electricInfo.battery_temperature = 0;
    electricInfo.supply_alarms = 0;
    turnOff = false;
    swPosition.flag = false;
    swPosition.position = -1;
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

// Mandar comando a vehiculo
void ElectricConnectionManager::sendToVehicle(FrameDriving frame){
    
    // Buffer de envio
    char bufData[8];
    
    // Rellenado del buffer
    memcpy(&bufData[0], &frame.instruction, sizeof (frame.instruction));
    memcpy(&bufData[2], &frame.id_instruction, sizeof (frame.id_instruction));
    memcpy(&bufData[4], &frame.element, sizeof (frame.element));
    memcpy(&bufData[6], &frame.value, sizeof (frame.value));
    
    // Envio via socket
    send(socketDescriptor, bufData, sizeof(bufData), 0);
    usleep(1000);  
}

// Solicitar informacion basica de vehiculo

void ElectricConnectionManager::reqElectricInfo() {

    FrameDriving frame;

    // Comun 
    frame.instruction = GET;
    frame.id_instruction = -1;
    frame.value = -1;

    // Nivel de bateria
    frame.element = BATTERY_LEVEL;
    sendToVehicle(frame);
    // Tension de bateria 
    frame.element = BATTERY_VOLTAGE;
    sendToVehicle(frame);
    // Intensidad bateria
    frame.element = BATTERY_CURRENT;
    sendToVehicle(frame);
    // Temperatura bateria
    frame.element = BATTERY_TEMPERATURE;
    sendToVehicle(frame);
    // Alarmas de suministro
    frame.element = SUPPLY_ALARMS;
    sendToVehicle(frame);
    
}


bool ElectricConnectionManager::checkForVehicleMessages() {
    
    char bufData[8];
    
    if (recv(socketDescriptor, bufData, sizeof (bufData), 0) > 0) {
        // Estructura de recepcion
        FrameDriving fdr;
        
        // Rellenado del buffer
        memcpy(&fdr.instruction, &bufData[0], sizeof (fdr.instruction));
        memcpy(&fdr.id_instruction, &bufData[2], sizeof (fdr.id_instruction));
        memcpy(&fdr.element, &bufData[4], sizeof (fdr.element));
        memcpy(&fdr.value, &bufData[6], sizeof (fdr.value));

        if (fdr.instruction == ACK) {
            
            informResponse(true, fdr.id_instruction);
            
        } else if (fdr.instruction == NACK) {
            
            RtxStruct rtxList = informResponse(false, fdr.id_instruction);
            
            for(int i = 0; i < rtxList.numOfMsgs; i++){
                sendToVehicle((FrameDriving)rtxList.msgs.at(i));
            }
            
        } else if (fdr.instruction == INFO) {
            
            if(fdr.element == SUPPLY_ALARMS){
                 
                // TODO ALARMAS
                setVehicleInfo(fdr.element,fdr.value);
            
            }else if (fdr.element == TURN_OFF){ 
                ROS_INFO("[Control] Electric - Preparando el apagado del sistema");
                turnOff = true;
                
            }else if (fdr.element == OPERATION_MODE_SWITCH){
                ROS_INFO("[Control] Electric - Preparando el apagado del sistema");
                swPosition.flag = true;
                swPosition.position = fdr.value;
            
            }else{ // INFO corriente
                setVehicleInfo(fdr.element,fdr.value);
            }            
            
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
int ElectricConnectionManager::getSocketDescriptor(){
    return socketDescriptor;
}

// Get de la ultima informacion recibida del vehiculo
ElectricInfo ElectricConnectionManager::getVehicleInfo() {
    return electricInfo;
}

void ElectricConnectionManager::setVehicleInfo(short id_device, short value){
    switch(id_device){
        case BATTERY_LEVEL:
            electricInfo.battery_level = value;
            break;
        case BATTERY_VOLTAGE:
            electricInfo.battery_voltage = value;
            break;
        case BATTERY_CURRENT:
            electricInfo.battery_current = (bool) value;
            break;
        case BATTERY_TEMPERATURE:
            electricInfo.battery_temperature = value;
            break;
        case SUPPLY_ALARMS:
            electricInfo.supply_alarms = value;
            break;
        default: 
            break;
    }
}

short ElectricConnectionManager::getCountCriticalMessages(){
    return countMsg;
}

void ElectricConnectionManager::setCountCriticalMessages(short cont) {
    countMsg = cont;
    // Contador: 1 .. 1024
    if(countMsg == 1025)
        countMsg = 1;
}

bool ElectricConnectionManager::getTurnOffFlag(){
    return turnOff;
}

SwitcherStruct ElectricConnectionManager::getSwitcherStruct(){
    return swPosition;
}

void ElectricConnectionManager::setSwitcherStruct(bool flag){
    swPosition.flag = flag;
    swPosition.position = -1;
}

/*******************************************************************************
 * METODOS PROPIOS
 *******************************************************************************/

bool ElectricConnectionManager::isCriticalInstruction(short element) {
    return false;
}

/*******************************************************************************
 * MANEJO DE LA COLA DE MENSAJES
 *******************************************************************************/

void ElectricConnectionManager::addToQueue(FrameDriving frame){
    messageQueue.push_back(frame);
}

RtxStruct ElectricConnectionManager::informResponse(bool ack, short id_instruction){
    
    RtxStruct ret;
    ret.numOfMsgs = 0;
    
    // Se situa el iterador al principio de la cola
    vector<FrameDriving>::iterator it = messageQueue.begin();
    
    if(ack){ // ACK
        
        if(id_instruction == (*it).id_instruction){ // Primer elemento y requerido coinciden
            
            // Se elimina el primer elemento
            messageQueue.erase(it);
            
        }else if(id_instruction > (*it).id_instruction){ // Confirmacion de varios elementos
            
            while(id_instruction >= (*it).id_instruction){
                
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
