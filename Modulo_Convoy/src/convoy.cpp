/* 
 * File:   convoy.cpp
 * Author: atica
 *
 * Created on 19 de marzo de 2014, 9:51
 */

#include "../include/Modulo_Convoy/convoy.h"

ros::Publisher pub_error;
ros::Publisher pub_mode;
ros::Publisher pub_error_follower;
ros::Publisher pub_backup_follower;
ros::Publisher pub_gps_leader;
ros::Publisher pub_gps_follower;

using namespace std;

/*
 * 
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

    // Todo esta correcto, lo especificamos con el correspondiente parametro
    n.setParam("estado_modulo_convoy", STATE_OK);
    cout << "Atica CONVOY :: Configurado y funcionando" << endl;

    while (ros::ok() && !exitModule) {
        switch (operationMode) {
            case OPERATION_MODE_DEBUG:
                // Funcionamiento del modo debug
                n.getParam("estado_modulo_rangeDataFusion", estado_actual);
                if (estado_actual == STATE_ERROR || estado_actual == STATE_OFF) {
                    exitModule = true;
                }
                
                if (enableModule=ENABLE_ON){
                    if (!initialConnection){            // Comprueba que no es la primera conexión
                        if (handshake()==true){
                            sendDataFlag=true;
                            initialConnection=true;
                        }
                    }
                }
                else if(enableModule=ENABLE_OFF){       // Desconexión - deshabilitación módulo
                    disconnectClient();
                    enableModule=ENABLE_INIT;
                    sendDataFlag=false;                 // Se cortan todos los posibles envíos en los suscriptores
                    vehicleRole=FOLLOWER;               // Valor por defecto del módulo
                }
                else
                    sendDataFlag=false;
                /* *****PARTE DE RECEPCION DEL SOCKET EN EL CLIENTE (LEADER) INCLUIDO HEARTBEAT***** */
                ros::spinOnce();
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
    }
    cout << "Atica CONVOY :: Módulo finalizado" << endl;    
    
    return 0;
}

/*******************************************************************************
 *******************************************************************************
 *                              SUSCRIPTORES
 * *****************************************************************************
 * ****************************************************************************/
// Suscriptor de habilitación de módulo (module_enable)
void fcn_sub_module_enable(const Common_files::msg_module_enable msg)
{
    if ((msg.id_module==SUBS_CONVOY) && (msg.status==MOD_ON)){
        enableModule=ENABLE_ON;
        vehicleRole=LEADER;
    }
    if ((msg.id_module==SUBS_CONVOY) && (msg.status==MOD_OFF))
        enableModule=ENABLE_OFF;
}

// Suscriptor de los datos del GPS
void fcn_sub_gps(const Common_files::msg_gps msg)
{
    if ((sendDataFlag) && (vehicleRole==LEADER)){  // Aqui se envía msg_gps a través del socket UDP al server (follower)
        stringstream tx_string;
        short type_msg=TYPE_GPS;
        tx_string << type_msg << "\n" << msg;
        sendto(fd_client, tx_string.str().c_str(), tx_string.str().length(), 0, (struct sockaddr *)&server_addr, sizeof(server_addr));
    }
        
}

// Suscriptor de errores en vehiculo leader
void fcn_sub_error_leader(const Common_files::msg_error msg)
{
    if ((sendDataFlag) && (msg.id_error==LEADER_CRITICAL_ERROR_OUTPUT)){
        stringstream tx_string;
        short type_msg=TYPE_ERROR;
        tx_string << type_msg << "\n" << msg;
        sendto(fd_client, tx_string.str().c_str(), tx_string.str().length(), 0, (struct sockaddr *)&server_addr, sizeof(server_addr));
    }
}

// Suscriptor de errores en vehiculo follower
void fcn_sub_error_follower(const Common_files::msg_error msg)
{
    if ((sendDataFlag) && (msg.id_error==FOLLOWER_ERROR) && (vehicleRole==FOLLOWER)){
        stringstream tx_string;
        short type_msg=TYPE_ERROR;
        tx_string << type_msg << "\n" << msg;
        sendto(fd_server, tx_string.str().c_str(), tx_string.str().length(), 0, (struct sockaddr *)&client_addr, sizeof(client_addr));
        if (msg.type_error==TOE_CRITICAL){
            /* *****CIERRE DE LA COMUNICACION DEL SERVIDOR CON EL LIDER***** */
        }
    }
}

// Suscriptor de backup en vehiculo follower
void fcn_sub_backup_follower(const Common_files::msg_backup msg)
{
    if ((sendDataFlag) && (vehicleRole==FOLLOWER)){
        stringstream tx_string;
        short type_msg=TYPE_BACKUP;
        tx_string << type_msg << "\n" << msg;
        sendto(fd_server, tx_string.str().c_str(), tx_string.str().length(), 0, (struct sockaddr *)&client_addr, sizeof(client_addr));
    }
}

// Suscriptor de modo en vehiculo follower
void fcn_sub_mode_follower(const Common_files::msg_mode msg)
{
    if ((sendDataFlag) && (vehicleRole==FOLLOWER)){
        stringstream tx_string;
        short type_msg=TYPE_MODE;
        tx_string << type_msg << "\n" << msg;
        sendto(fd_server, tx_string.str().c_str(), tx_string.str().length(), 0, (struct sockaddr *)&client_addr, sizeof(client_addr));
    }
}


/*******************************************************************************
 *******************************************************************************
 *                              FUNCIONES PROPIAS
 * *****************************************************************************
 * ****************************************************************************/
// Inicialización de variables
void initialize(ros::NodeHandle n)
{
    exitModule=false;
    enableModule=ENABLE_INIT;
    vehicleRole=FOLLOWER;
    initialConnection=false;
    sendDataFlag=false;
    // Datos cliente del socket
    client_addr.sin_family = AF_INET;
    client_addr.sin_addr.s_addr = inet_addr(CLIENT_IP);
    client_addr.sin_port = htons(0);
    // Datos servidor del socket
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = inet_addr(SERVER_IP);
    server_addr.sin_port = htons(1153);
    
    // Inicializacion de publicadores
    pub_mode = n.advertise<Common_files::msg_mode>("modeCNS", 1000);
    pub_error = n.advertise<Common_files::msg_error>("error", 1000);
    pub_error_follower = n.advertise<Common_files::msg_error>("errorF", 1000);
    pub_backup_follower = n.advertise<Common_files::msg_backup>("backupF", 1000);
    pub_gps_leader = n.advertise<Common_files::msg_gps>("gps", 1000);
    pub_gps_follower = n.advertise<Common_files::msg_gps>("gpsF", 1000);

    // Creacion de suscriptores
    ros::Subscriber sub_gps = n.subscribe("gps", 1000, fcn_sub_gps);
    ros::Subscriber sub_moduleEnable = n.subscribe("modEnable", 1000, fcn_sub_module_enable);
    ros::Subscriber sub_error_leader = n.subscribe("error", 1000, fcn_sub_error_leader);
    ros::Subscriber sub_error_follower = n.subscribe("error", 1000, fcn_sub_error_follower);
    ros::Subscriber sub_backup = n.subscribe("backup", 1000, fcn_sub_backup_follower);
    ros::Subscriber sub_mode_follower = n.subscribe("modeSCN", 1000, fcn_sub_mode_follower);    
}


// 
bool handshake ()
{
    // Definición de variables de la función
    bool connectionOK=false;
    int count=0;
    ROSmessage tx_msg, rcv_msg;
    stringstream tx_string;
    unsigned char buf[BUFSIZE];         //receive buffer
    Common_files::msg_error msg_err;
    msg_err.id_subsystem=SUBS_CONVOY;
    // Creacion del socket UDP en el cliente
    while (count < 3){
        if ((fd_client = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
            count++;
        else
            count=99;
    }
    if (count==3){
        msg_err.id_error=UDP_SOCKET_FAIL;
        msg_err.type_error=TOE_UNDEFINED;
        pub_error.publish(msg_err);
        return connectionOK;
    }
    // Identificar el socket cliente
    if((bind(fd_client, (struct sockaddr *)&client_addr, sizeof(client_addr)))<0){
        msg_err.id_error=BIND_SOCKET_FAIL;
        msg_err.type_error=TOE_UNDEFINED;
        pub_error.publish(msg_err);
        return connectionOK;
    }
    // Envio mensaje de start
    tx_msg.type_message=TYPE_MODE;
    tx_msg.mode.type_msg=SET;
    tx_msg.mode.mode=MODE_CONVOY;
    tx_msg.mode.status=MODE_START;
    tx_string << tx_msg.type_message << "\n" << tx_msg.mode;
    sendto(fd_client, tx_string.str().c_str(), tx_string.str().length(), 0, (struct sockaddr *)&server_addr, sizeof(server_addr));
    
    // Temporizador de espera de ACK vehículo Follower
    t.Enable();
    int read = 0;       // Variable para comprobar si llega algo desde el socket
    while (t.GetTimed()<TIMER){ 
        read = recvfrom(fd_server, buf, BUFSIZE, 0, NULL, 0);   // Recibo del server
        if (read>0){                                            // Si se recibe algo
            buf[read] = '\0';
            rcv_msg = convertToROS(buf);
            if ((rcv_msg.type_message==TYPE_MODE) && (rcv_msg.mode.type_msg==INFO)){       // ACK recibido -> publico msg_mode
                pub_mode.publish(rcv_msg.mode);
                connectionOK = true;            // Todo correcto en el handshake inicial
            }
        }
    }
    
    return connectionOK;
}

// Función desconexión del cliente - cierre de socket
void disconnectClient ()
{
    Common_files::msg_mode mode;
    stringstream tx_string;
    unsigned char buf[BUFSIZE];         //receive buffer
    ROSmessage rcv_msg;
    // Mensaje modo que se envía al follower
    mode.type_msg=SET;
    mode.mode=MODE_CONVOY;
    mode.status=MODE_EXIT;
    short type_msg=TYPE_MODE;
    tx_string << type_msg << "\n" << mode;
    sendto(fd_client, tx_string.str().c_str(), tx_string.str().length(), 0, (struct sockaddr *)&server_addr, sizeof(server_addr));
    // Temporizador de espera de ACK vehículo Follower
    t.Enable();
    int read = 0;       // Variable para comprobar si llega algo desde el socket
    while (t.GetTimed()<TIMER){ 
        read = recvfrom(fd_server, buf, BUFSIZE, 0, NULL, 0);   // Recibo del server
        if (read>0){                                            // Si se recibe algo
            buf[read] = '\0';
            rcv_msg = convertToROS(buf);
            if ((rcv_msg.type_message==TYPE_MODE) && (rcv_msg.mode.type_msg==INFO)){       // ACK recibido -> publico msg_mode
                pub_mode.publish(rcv_msg.mode);
            }
        }
    }
    close (fd_client);
}

// Función conversión de cadenas del Socket a paquete ROS
ROSmessage convertToROS (unsigned char* buffer)
{
    ROSmessage msg;
    string name_param;
    stringstream aux;
    short type_msg;
    aux << buffer;
    aux >> type_msg;
    switch (type_msg){
        case TYPE_GPS:
            aux >> name_param;
            aux >> msg.gps.latitude;
            aux >> name_param;
            aux >> msg.gps.longitude;
            aux >> name_param;
            aux >> msg.gps.altitude;
            aux >> name_param;
            aux >> msg.gps.roll;
            aux >> name_param;
            aux >> msg.gps.pitch;
            aux >> name_param;
            aux >> msg.gps.yaw;
            break;
        case TYPE_ERROR:
            aux >> name_param;
            aux >> msg.error.id_subsystem;
            aux >> name_param;
            aux >> msg.error.id_error;
            aux >> name_param;
            aux >> msg.error.type_error;
            break;
        case TYPE_MODE:
            aux >> name_param;
            aux >> msg.mode.type_msg;
            aux >> name_param;
            aux >> msg.mode.mode;
            aux >> name_param;
            aux >> msg.mode.status;
            break;
        case TYPE_BACKUP:
            aux >> name_param;
            aux >> msg.backup.throttle;
            aux >> name_param;
            aux >> msg.backup.brake;
            aux >> name_param;
            aux >> msg.backup.steer;
            aux >> name_param;
            aux >> msg.backup.handbrake;
            aux >> name_param;
            aux >> msg.backup.gear;
            aux >> name_param;
            aux >> msg.backup.engine;
            aux >> name_param;
            aux >> msg.backup.speed;
            break;
        default:
            cout << "Error en la transmisión" << endl;
            break;
    }
    return msg;
}