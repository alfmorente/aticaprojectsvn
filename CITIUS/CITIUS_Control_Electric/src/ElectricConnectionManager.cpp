
/** 
 * @file  ElectricConnectionManager.cpp
 * @brief Implementacion de la clase "ElectricConnectionManager"
 * @author: Carlos Amores
 * @date: 2013, 2014
 */

#include "ElectricConnectionManager.h"

/** Constructor de la clase*/
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
    supplyAlarms = 0x0000;
}

/**
 * Realiza la inicializacion y conexion del socket de comunicacion con el 
 * vehiculo. 
 * @return Booleano que indica si la conexion ha sido posible
 */
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

/**
 * Realiza la desconexion del vehiculo mediante la liberacion del socket de
 * comunicacion
 * @return Booleano que indica si la desconexion se ha realizado con exito
 */
bool ElectricConnectionManager::disconnectVehicle() {
    // Cierre del socket
    shutdown(this->getSocketDescriptor(), 2);
    close(this->getSocketDescriptor());
    ROS_INFO("[Control] Electric :: Socket cerrado correctamente");
    return true;
}

/**
 * Envia la informacion de una trama al vehiculo haciendo uso del socket de
 * comunicacion
 * @param[in] frame Trama a enviar via socket al vehiculo 
 */
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

/**
 * Envia una serie de tramas de tipo GET solicitando la informacion electrica 
 * del vehiculo
 */
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

/**
 * Envia mensaje de confirmacion (SET) para indicar el fin de un apagado 
 * ordenado de los distintos modulos del vehiculo
 */
void ElectricConnectionManager::setTurnOff() {
    FrameDriving frame;
    frame.instruction = SET;
    frame.id_instruction = -1;
    frame.element = TURN_OFF;
    frame.value = 1;
    sendToVehicle(frame);
}

/**
 * Realiza una lectura por el socket de comunicacion con el vehiculo y obtiene 
 * una trama en caso de que el propio vehiculo la haya enviado. La clasifica 
 * segun el elemento al que hace referencia y procede a su tratamiento
 * @return Booleano que indica si se ha llevado a cabo una lectura via socket
 */
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
            
            }else{ // INFO corriente o alarmas
                setVehicleInfo(fdr.element,fdr.value);
            }            
            
        }
        return true;
    }else{
        return false;
    }
}

/**
 * Consultor del atributo "socketDescriptor" de la clase 
 * @return Atributo "socketDescriptor" de la clase
 */
int ElectricConnectionManager::getSocketDescriptor(){
    return socketDescriptor;
}

/**
 * Consultor del atributo "electricInfo" de la clase que almacena la informacion
 * de la ultima lectura realizada del vehiculo
 * @return Atributo "electricInfo" de la clase
 */
ElectricInfo ElectricConnectionManager::getVehicleInfo() {
    return electricInfo;
}

/**
 * Modificador del atributo "electricInfo" de la clase con la informacion de un
 * dispositivo electrico concreto
 * @param[in] id_device Identificador del dispositivo a modificar
 * @param[in] value Valor de lectura del dispositivo a modificar
 */
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
            supplyAlarms = value;
            break;
        default: 
            break;
    }
}

/**
 * Consultor del atributo "countMsg" de la clase utilizado para llevar el 
 * conteo de los mensajes criticos (mecanismo de integridad)
 * @return Atributo "countMsg" de la clase
 */
short ElectricConnectionManager::getCountCriticalMessages(){
    return countMsg;
}

/**
 * Modificador del atributo "countMsg" de la clase utilizado para llevar el
 * conteo del los mensajes criticos (mecanismo de integridad). Contempla que se
 * lleve a cabo segun un contador incremental con intervalo 1..1024
 * @param[in] cont Nuevo valor para el atributo "countMsg"
 */
void ElectricConnectionManager::setCountCriticalMessages(short cont) {
    countMsg = cont;
    // Contador: 1 .. 1024
    if(countMsg == 1025)
        countMsg = 1;
}

/**
 * Consultor del atributo "turnOff" de la clase utilizado para indicar que se 
 * ha recibido una peticion de apagado ordenado por parte del vehiculo
 * @return Atributo "turnOff" de la clase
 */
bool ElectricConnectionManager::getTurnOffFlag(){
    return turnOff;
}

/**
 * Consultor del atributo "swPosition" de la clase utilizado para indicar que 
 * ha habido un cambio en la posicion del conmutador (switcher) local / 
 * teleoperado
 * @return Atributo "swPosition" de la clase
 */
SwitcherStruct ElectricConnectionManager::getSwitcherStruct(){
    return swPosition;
}

/**
 * Modificador del atributo "swPosition" de la clase que se actualiza cuando se
 * detecta un cambio de posicion del conmutador (switcher) local / teleoperado
 * del vehiculo o cuando se ha llevado a cabo el tratamiento tras su deteccion
 * @param[in] flag Nueva posicion del estado de la estructura de tratamiento
 */
void ElectricConnectionManager::setSwitcherStruct(bool flag){
    swPosition.flag = flag;
    swPosition.position = -1;
}

/**
 * Consultor del atributo "supplyAlarms" de la clase que indica la ultima
 * lectura realizada del vector de alarmas del modulo electrico del Payload de
 * conduccion del vehiculo
 * @return 
 */
short ElectricConnectionManager::getSupplyAlarms() {
    return supplyAlarms;
}

/**
 * Consulta si un elemento es critico y por tanto debe ser contemplado para
 * llevar a cabo el mecanismo de integridad
 * @param[in] element Elemento de consulta
 * @return Booleano que indica si el elemento es critico o  no
 */
bool ElectricConnectionManager::isCriticalInstruction(short element) {
    if (element == SUPPLY_TURN_ON) {
        return true;
    } else {
        return false;
    }
}

/**
 * Una vez que un mensaje es considerado critico, se utiliza este metodo para
 * encolarlo hasta que se reciba el ACK correspondiente o retransmitirlo en caso
 * de obtener un NACK
 * @param[in] frame Trama a incluir en la cola de mensajes criticos
 */
void ElectricConnectionManager::addToQueue(FrameDriving frame){
    messageQueue.push_back(frame);
}

/**
 * Tratamiento de la cola de mensajes tras la recepcion de un mensaje de 
 * confirmacion (ACK) o negacion (NACK). En el caso de recibir un ACK, se 
 * desencolan todos los mensajes cuyo campo "id_instruccion" es menor al que
 * se incluye en la trama del propio ACK. En el caso de recibir un NACK, se 
 * retransmiten todos los mensajes de la cola cuyo campo "id_instruccion" sea
 * menor o igual que el que se incluye en la trama del propio ACK y se elimina
 * el resto
 * @param[in] ack Indica si se ha recibido un ACK (true) o un NACK (false) 
 * @param[in] id_instruction Indica el campo "id_instruccion" (cuenta) del mensaje
 * recibido del vehiculo
 * @return Estructura con el numero de mensajes a retransmitir (en caso de
 * haberlos) y una lista de dichos mensajes.
 */
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