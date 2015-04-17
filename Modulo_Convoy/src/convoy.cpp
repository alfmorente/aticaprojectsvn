/**
  @file convoy.cpp
  @brief 

 * Archivo principal del Módulo Convoy. Se encarga de establecer la comunicación
 * entre los dos vehículos que forman el convoy. Es el mismo software para los 
 * dos vehículos.

  @author Alfonso Morente
  @date 19/03/2014

*/

#include "../include/Modulo_Convoy/convoy.h"

using namespace std;

/**
 * Método principal del nodo. 
 * @param[in] argc Número de argumentos
 * @param[in] argv Vector de argumentos
 * @return Entero distinto de 0 si ha habido problemas. 0 en caso contrario. 
 */
int main(int argc, char** argv) {
    // Obtencion del modo de operacion y comprobacion de que es correcto
    int operationMode;
    if ((operationMode = getOperationMode(argc, argv)) == 0) {
        return 1;
    }

    // Inicio de ROS
    ros::init(argc, argv, "convoy");

    // Manejador ROS
    ros::NodeHandle n;

    // Espera activa de inicio de modulo
    int estado_actual = STATE_OFF;
    while (estado_actual != STATE_CONF) {
        n.getParam("estado_modulo_convoy", estado_actual);
    }
    cout << "Atica CONVOY :: Iniciando configuración..." << endl;

    // Inicialización de variables
    initialize(n);
    usleep(50000);
    // Todo esta correcto, lo especificamos con el correspondiente parametro
    n.setParam("estado_modulo_convoy", STATE_OK);
    cout << "Atica CONVOY :: Configurado y funcionando" << endl;

    switch (operationMode) {
        case OPERATION_MODE_DEBUG:
            while (ros::ok() && !exitModule) {
                // Funcionamiento del modo debug
                n.getParam("estado_modulo_convoy", estado_actual);
                if (estado_actual == STATE_ERROR || estado_actual == STATE_OFF) {
                    exitModule = true;
                }

                if (enableModule == ENABLE_ON) {
                    sendDataFlag = true;    
                } 
                else if (enableModule == ENABLE_OFF) { // Desconexión - deshabilitación módulo
                    disconnectSocket();
                    enableModule = ENABLE_INIT;
                    sendDataFlag = false; // Se cortan todos los posibles envíos en los suscriptores
                }
                else
                    sendDataFlag = false;
                usleep(20000);
                ros::spinOnce();
            }
            break;
        case OPERATION_MODE_RELEASE:
            // Funcionamiento del modo release
            n.getParam("estado_modulo_rangeDataFusion", estado_actual);
            if (estado_actual == STATE_ERROR || estado_actual == STATE_OFF) {
                exitModule = true;
            }
            ros::spinOnce();
            break;
        case OPERATION_MODE_SIMULATION:
            // Funcionamiento del modo simulacion
            n.getParam("estado_modulo_rangeDataFusion", estado_actual);
            if (estado_actual == STATE_ERROR || estado_actual == STATE_OFF) {
                exitModule = true;
            }
            ros::spinOnce();
            break;
        default:
            break;
    }

    cout << "Atica CONVOY :: Módulo finalizado" << endl;
    pthread_exit(NULL);
    return 0;
}

/*******************************************************************************
 *******************************************************************************
 *                              SUSCRIPTORES
 * *****************************************************************************
 * ****************************************************************************/

/**
 * Suscriptor de habilitación de módulo. Al recibir la orden de habilitación 
 * también indica que el vehículo se comporta de LEADER.
 * @param msg Mensaje proveniente del publicador
 */
void fcn_sub_module_enable(const Common_files::msg_module_enablePtr& msg) {
    if ((msg->id_module == ID_MOD_CONVOY) && (msg->submode == SUBMODE_CONVOY) && (msg->status == MOD_ON)) {
        enableModule = ENABLE_ON;
        vehicleRole = LEADER;
    }
    if ((msg->id_module == ID_MOD_CONVOY) && (msg->id_module == SUBMODE_CONVOY) && (msg->status == MOD_OFF)) {
        enableModule = ENABLE_OFF;
        vehicleRole = UNDEFINED;
    }
}

/**
 * Suscriptor de los datos del GPS. Recibe los datos del GPS y los envía al 
 * vehículo FOLLOWER a través del socket
 * @param msg Mensaje proveniente del publicador
 */
void fcn_sub_gps(const Common_files::msg_gpsPtr& msg) {
    if ((sendDataFlag)) { // Envía msg_gps si el módulo está activado y la conexión inicial establecida
        stringstream tx_string;
        short type_msg = TYPE_GPS;
        tx_string << type_msg << "\n" << msg;
        sendto(fd_local, tx_string.str().c_str(), tx_string.str().length(), 0, (struct sockaddr *) &remote_addr, sizeof (remote_addr));
    }
}

/**
 * Suscriptor de errores. En caso de recibir un mensaje crítico en el LEADER, se 
 * manda por socket un error de parada al FOLLOWER
 * @param msg Mensaje proveniente del publicador
 */
void fcn_sub_error(const Common_files::msg_errorPtr& msg){
    /*Caso de error recibido siendo vehículo Leader*/
    if((sendDataFlag) && (vehicleRole==LEADER) && (msg->id_subsystem == SUBS_CONVOY) && (msg->id_error == LEADER_CRITICAL_ERROR_OUTPUT)){
        stringstream tx_string;
        short type_msg = TYPE_ERROR;
        tx_string << type_msg << "\n" << msg;
        sendto(fd_local, tx_string.str().c_str(), tx_string.str().length(), 0, (struct sockaddr *) &remote_addr, sizeof (remote_addr));
    }
    /*Caso de error recibido siendo vehículo Follower*/
    else if((sendDataFlag) && (vehicleRole==FOLLOWER)){
        /*PENDIENTE DE DEFINIR*/
    }
}

/* Suscriptor de errores en vehiculo follower
void fcn_sub_error_follower(const Common_files::msg_errorPtr& msg) {
    if ((sendDataFlag) && (msg->id_subsystem == SUBS_CONVOY) && (msg->id_error == FOLLOWER_ERROR) && (vehicleRole == FOLLOWER)) {
        stringstream tx_string;
        short type_msg = TYPE_ERROR;
        tx_string << type_msg << "\n" << msg;
        sendto(fd_local, tx_string.str().c_str(), tx_string.str().length(), 0, (struct sockaddr *) &remote_addr, sizeof (remote_addr));
        if (msg->type_error == TOE_CRITICAL) {
        }
    }
}*/

/**
 * Suscriptor de backup en vehiculo follower. La información recibida se 
 * transmite al LEADER.
 * @param msg Mensaje proveniente del publicador
 */
void fcn_sub_backup_follower(const Common_files::msg_backupPtr& msg) {
    if ((sendDataFlag) && (vehicleRole == FOLLOWER)) {
        stringstream tx_string;
        short type_msg = TYPE_BACKUP;
        tx_string << type_msg << "\n" << msg;
        sendto(fd_local, tx_string.str().c_str(), tx_string.str().length(), 0, (struct sockaddr *) &remote_addr, sizeof (remote_addr));
    }
}

/**
 * Suscriptor de modo en vehiculo follower.
 * @param msg Mensaje proveniente del publicador
 */
void fcn_sub_mode_follower(const Common_files::msg_modePtr& msg) {
    if ((msg->type_msg == INFO) && (msg->mode == MODE_CONVOY) && (msg->status == MODE_START))
        modeACK = true;
}


/*******************************************************************************
 *******************************************************************************
 *                              FUNCIONES PROPIAS
 * *****************************************************************************
 * ****************************************************************************/

// Inicialización de variables
void initialize(ros::NodeHandle n) {
    exitModule = false;
    enableModule = ENABLE_INIT;
    vehicleRole = UNDEFINED;
    initialConnection = false;
    sendDataFlag = false;
    modeACK = false;
    threadActive = 1;

    // Inicializacion de publicadores
    pub_mode = n.advertise<Common_files::msg_mode>("modeCNS", 1000);
    pub_error = n.advertise<Common_files::msg_error>("error", 1000);
    pub_error_follower = n.advertise<Common_files::msg_error>("errorF", 1000);
    pub_backup_follower = n.advertise<Common_files::msg_backup>("backupF", 1000);
    pub_gps_leader = n.advertise<Common_files::msg_gps>("gps", 1000);
    pub_gps_follower = n.advertise<Common_files::msg_gps>("gpsF", 1000);

    // Creacion de suscriptores
    sub_gps = n.subscribe("gps", 1000, fcn_sub_gps);
    sub_moduleEnable = n.subscribe("modEnable1", 1000, fcn_sub_module_enable);
    sub_error = n.subscribe("error", 1000, fcn_sub_error);
    sub_backup = n.subscribe("backup", 1000, fcn_sub_backup_follower);
    sub_mode_follower = n.subscribe("modeSCN", 1000, fcn_sub_mode_follower);

    // Variables para el envío a través de socket
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = inet_addr(IP_LOCAL);
    local_addr.sin_port = htons(1153);
    remote_addr.sin_family = AF_INET;
    remote_addr.sin_addr.s_addr = inet_addr(IP_REMOTE);
    remote_addr.sin_port = htons(1154);
    // Creación de hilo del servidor
    pthread_create(&s_Thread, NULL, socketThread, NULL);
}

// Hilo para la gestión del socket (enviar-recibir)
void* socketThread(void*) {
    /*Variables de la función*/
    stringstream tx_string;
    unsigned char buffer[BUFSIZE]; //receive buffer
    int data_read = 0;
    bool connectionOK;
    /*Creación del socket*/
    bool socketOK = createSocket(fd_local, local_addr);
    if (!socketOK)
        return NULL;
    // Bucle de espera para definir rol del vehículo
    wait_role:
    while (vehicleRole == UNDEFINED) {
        data_read = recvfrom(fd_local, buffer, BUFSIZE, 0, NULL, 0); // Recibo de remote
        if (data_read > 0) { // Si se recibe algo
            buffer[data_read] = '\0';
            convertToROS(buffer);
            if ((type_message == TYPE_MODE) && (msg_mode->mode == MODE_CONVOY) && (msg_mode->type_msg == SET) && (msg_mode->status == MODE_START)) {
                vehicleRole = FOLLOWER;
            }
        }
    }
    /*Ejecución de órdenes del vehículo Follower*/
    if (vehicleRole == FOLLOWER) {
        connectionOK = handshakeFollower();
        if (!connectionOK) {
            vehicleRole = UNDEFINED;
            goto wait_role;
        }
        while (vehicleRole == FOLLOWER) {
            /*RESTO DE MENSAJES RECIBIDOS POR SOCKET*/
            data_read = 0;
            data_read = recvfrom(fd_local, buffer, BUFSIZE, 0, NULL, 0); // Recibo del cliente
            if (data_read > 0) {
                buffer[data_read] = '\0';
                convertToROS(buffer);
                switch (type_message) {
                    case TYPE_MODE:             // Desconexión controlada
                        if ((msg_mode->mode == MODE_CONVOY) && (msg_mode->type_msg == SET) && (msg_mode->status == MODE_EXIT)) {
                            pub_mode.publish(msg_mode);
                            // Espera mensaje de asentimiento de Gestión de Sistema
                            t.Enable();
                            while ((t.GetTimed() < TIMER_SHORT) && (!modeACK)) {
                                ros::spinOnce();
                            }
                            // Envía asentimiento de vuelta porque se ha recibido la señal de todo OK por parte de GestSistema
                            if (modeACK) {
                                type_message = TYPE_MODE;
                                msg_mode->type_msg = INFO;
                                msg_mode->mode = MODE_CONVOY;
                                msg_mode->status = MODE_EXIT;
                                tx_string << type_message << "\n" << msg_mode;
                                sendto(fd_local, tx_string.str().c_str(), tx_string.str().length(), 0, (struct sockaddr *) &remote_addr, sizeof (remote_addr));
                            }
                        }
                        goto wait_role;
                        break;
                    case TYPE_ERROR:
                        pub_error_follower.publish(msg_error);
                        goto wait_role;
                        break;
                    case TYPE_GPS:
                        pub_gps_leader.publish(msg_gps);
                        break;
                    case TYPE_HEARTBEAT:
                        tx_string << type_message;
                        sendto(fd_local, tx_string.str().c_str(), tx_string.str().length(), 0, (struct sockaddr *) &remote_addr, sizeof (remote_addr));
                        break;
                    default:
                        ROS_INFO("Mensaje recibido por socket no válido");
                        break;
                }
            }
            if(!(heartbeat(fd_local,remote_addr))){
                vehicleRole=UNDEFINED;
                goto wait_role;
            }
        }
    }
    /*Ejecución de órdenes vehículo Leader*/
    else if (vehicleRole == LEADER) {
        connectionOK = handshakeLeader();
        if (!connectionOK) {
            vehicleRole = UNDEFINED;
            goto wait_role;
        }
        while (vehicleRole == LEADER) {
            data_read = 0; // Variable para comprobar si llega algo desde el socket
            unsigned char buffer[BUFSIZE]; // Buffer de recepción
            data_read = recvfrom(fd_local, buffer, BUFSIZE, 0, NULL, 0); // Recibo por el socket
            if (data_read > 0) { // Si se recibe algo
                buffer[data_read] = '\0';
                convertToROS(buffer);
            }
            //ROS_INFO("%d", type_message);
            switch (type_message) {
                case TYPE_ERROR:
                    pub_error_follower.publish(msg_error);
                    break;
                case TYPE_BACKUP:
                    pub_backup_follower.publish(msg_backup);
                    break;
                case TYPE_GPS:
                    pub_gps_follower.publish(msg_gps);
                    break;
                case TYPE_HEARTBEAT:
                    tx_string << type_message;
                    sendto(fd_local, tx_string.str().c_str(), tx_string.str().length(), 0, (struct sockaddr *) &remote_addr, sizeof (remote_addr));
                    break;
                default:
                    ROS_INFO("Mensaje recibido no determinado");
                    break;
            }
            if(!(heartbeat(fd_local,remote_addr))){
                vehicleRole=UNDEFINED;
                goto wait_role;
            }
        }
    }
    usleep(10000);
}

// Función para crear el socket UDP
bool createSocket(int fd, struct sockaddr_in address) {
    int count = 0;
    bool socketOK = true;
    msg_error->id_subsystem = SUBS_CONVOY;

    while (count < 3) {
        if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
            count++;
        else
            count = 99;
    }
    if (count == 3) {
        ROS_INFO("ERROR SOCKET 1");
        msg_error->id_error = UDP_SOCKET_FAIL;
        msg_error->type_error = TOE_UNDEFINED;
        pub_error.publish(msg_error);
        socketOK = false;
    }
    int flags = fcntl(fd, F_GETFL);
    flags |= O_NONBLOCK;
    fcntl(fd, F_SETFL, flags);
    // Identificar el socket cliente
    if ((bind(fd, (struct sockaddr *) &address, sizeof (address))) < 0) {
        ROS_INFO("ERROR SOCKET 2");
        msg_error->id_error = BIND_SOCKET_FAIL;
        msg_error->type_error = TOE_UNDEFINED;
        pub_error.publish(msg_error);
        socketOK = false;
    }

    return socketOK;
}

// Función de handshake en el follower
bool handshakeFollower() {
    bool connectionOK;
    stringstream tx_string;
    pub_mode.publish(msg_mode);
    // Espera mensaje de asentimiento de Gestión de Sistema
    t.Enable();
    while ((t.GetTimed() < TIMER_SHORT) && (!modeACK)) {
        ros::spinOnce();
    }
    // Envía asentimiento de vuelta porque se ha recibido la señal de todo OK por parte de GestSistema
    if (modeACK) {
        type_message = TYPE_MODE;
        msg_mode->type_msg = INFO;
        msg_mode->mode = MODE_CONVOY;
        msg_mode->status = MODE_START;
        tx_string << type_message << "\n" << msg_mode;
        sendto(fd_local, tx_string.str().c_str(), tx_string.str().length(), 0, (struct sockaddr *) &remote_addr, sizeof (remote_addr));
        connectionOK = true; // Establecida conexión inicial correctamente
    } else
        connectionOK = false;
    t.Disable();
    return connectionOK;
}

// Función de handshake en el Leader
bool handshakeLeader() {
    // Definición de variables de la función
    bool connectionOK = false;
    stringstream tx_string;
    unsigned char buf[BUFSIZE]; //receive buffer
    msg_error->id_subsystem = SUBS_CONVOY;
    // Envio mensaje de start
    type_message = TYPE_MODE;
    msg_mode->type_msg = SET;
    msg_mode->mode = MODE_CONVOY;
    msg_mode->status = MODE_START;
    tx_string << type_message << "\n" << msg_mode;
    sendto(fd_local, tx_string.str().c_str(), tx_string.str().length(), 0, (struct sockaddr *) &remote_addr, sizeof (remote_addr));

    // Temporizador de espera de ACK vehículo Follower
    t.Enable();
    int data_read = 0; // Variable para comprobar si llega algo desde el socket
    while (t.GetTimed() < TIMER_LONG) {
        data_read = recvfrom(fd_local, buf, BUFSIZE, 0, NULL, 0); // Recibo del server
        if (data_read > 0) { // Si se recibe algo
            buf[data_read] = '\0';
            convertToROS(buf);
            if ((type_message == TYPE_MODE) && (msg_mode->type_msg == INFO)) { // ACK recibido -> publico msg_mode
                pub_mode.publish(msg_mode);
                connectionOK = true; // Todo correcto en el handshake inicial
            }
        }
    }
    t.Disable();
    ROS_INFO("connectionOK en handshake=%d",connectionOK);
    return connectionOK;
}

// Función desconexión del cliente - cierre de socket
void disconnectSocket() {
    stringstream tx_string;
    unsigned char buf[BUFSIZE];         //receive buffer
    // Mensaje modo que se envía al follower
    msg_mode->type_msg = SET;
    msg_mode->mode = MODE_CONVOY;
    msg_mode->status = MODE_EXIT;
    type_message = TYPE_MODE;
    tx_string << type_message << "\n" << msg_mode;
    sendto(fd_local, tx_string.str().c_str(), tx_string.str().length(), 0, (struct sockaddr *) &remote_addr, sizeof (remote_addr));
    // Temporizador de espera de ACK vehículo Follower
    t.Enable();
    int data_read = 0; // Variable para comprobar si llega algo desde el socket
    while (t.GetTimed() < TIMER_LONG) {
        data_read = recvfrom(fd_local, buf, BUFSIZE, 0, NULL, 0); // Recibo del server
        if (data_read > 0) { // Si se recibe algo
            buf[data_read] = '\0';
            convertToROS(buf);
            if ((type_message == TYPE_MODE) && (msg_mode->type_msg == INFO)) { // ACK recibido -> publico msg_mode
                pub_mode.publish(msg_mode);
            }
        }
    }
    t.Disable();
    close(fd_local);
}

// Función conversión de cadenas del Socket a paquete ROS
void convertToROS(unsigned char* buffer) {
    string name_param;
    stringstream aux;
    aux << buffer;
    aux >> type_message;
    switch (type_message) {
        case TYPE_GPS:
            aux >> name_param;
            aux >> msg_gps->latitude;
            aux >> name_param;
            aux >> msg_gps->longitude;
            aux >> name_param;
            aux >> msg_gps->altitude;
            aux >> name_param;
            aux >> msg_gps->roll;
            aux >> name_param;
            aux >> msg_gps->pitch;
            aux >> name_param;
            aux >> msg_gps->yaw;
            break;
        case TYPE_ERROR:
            aux >> name_param;
            aux >> msg_error->id_subsystem;
            aux >> name_param;
            aux >> msg_error->id_error;
            aux >> name_param;
            aux >> msg_error->type_error;
            break;
        case TYPE_MODE:
            aux >> name_param;
            aux >> msg_mode->type_msg;
            aux >> name_param;
            aux >> msg_mode->mode;
            aux >> name_param;
            aux >> msg_mode->status;
            break;
        case TYPE_BACKUP:
            aux >> name_param;
            aux >> msg_backup->throttle;
            aux >> name_param;
            aux >> msg_backup->brake;
            aux >> name_param;
            aux >> msg_backup->steer;
            aux >> name_param;
            aux >> msg_backup->handbrake;
            aux >> name_param;
            aux >> msg_backup->gear;
            aux >> name_param;
            aux >> msg_backup->engine;
            aux >> name_param;
            aux >> msg_backup->speed;
            break;
        case TYPE_HEARTBEAT:
            break;
        default:
            cout << "Error en la transmisión" << endl;
            break;
    }
}

// Comprueba de manera periódica que el vehículo Follower está conectado mediante un método de envía-esperaACK
bool heartbeat(int fd, struct sockaddr_in address) {
    bool heartbeatOK = false;
    stringstream tx_string;
    type_message = TYPE_HEARTBEAT;
    tx_string << type_message;
    // Envío del mensaje tipo heartbeat desde el cliente(leader) hacia el servidor(follower)
    sendto(fd, tx_string.str().c_str(), tx_string.str().length(), 0, (struct sockaddr *) &address, sizeof (address));
    // Temporizador de espera de ACK vehículo Follower
    t.Enable();
    int data_read = 0; // Variable para comprobar si llega algo desde el socket
    unsigned char buffer[BUFSIZE]; // receive buffer
    while (t.GetTimed() < TIMER_SHORT) {
        data_read = recvfrom(fd, buffer, BUFSIZE, 0, NULL, 0); // Recibo del server
        if ((data_read > 0) && (buffer[0] == TYPE_HEARTBEAT)) // Si se recibe algo
            heartbeatOK = true;
    }
    t.Disable();
    // Publica mensaje de error si no se ha recibido ACK dentro de un tiempo establecido
    if (!heartbeatOK) {
        msg_error->id_subsystem = SUBS_CONVOY;
        msg_error->id_error = LINK_ERROR;
        msg_error->type_error = TOE_UNDEFINED;
        pub_error.publish(msg_error);
    }
    return heartbeatOK;
}