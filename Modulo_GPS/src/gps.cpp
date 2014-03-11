#include "../include/Modulo_GPS/gps.h"

ros::Publisher pub_gps;
ros::Publisher pub_errores;
ros::Publisher pub_stream;


using namespace std;

int main(int argc, char **argv) {

    // Obtencion del modo de operacion y comprobacion de que es correcto
    int operationMode;
    if ((operationMode = getOperationMode(argc, argv)) == 0) {
        return 1;
    }

    // Inicialización de la estructura que contiene los datos del GPS y variables globales
    initModuleVariables();
    cout << "Altitude: " << insdata.latitude << " Roll: " << insdata.roll << endl;

    // Orden para la parada manual con CTtrl+C
    init_signals();
    // Inicio de ROS
    ros::init(argc, argv, "GPS");
    // Manejador ROS
    ros::NodeHandle n;

    cout << "ATICA GPS :: Esperando señal de inicio..." << endl;
    // Espera activa de inicio de modulo
    int estado_actual = STATE_OFF;
    while (estado_actual != STATE_CONF) {
        n.getParam("estado_modulo_GPS", estado_actual);
    }
    cout << "ATICA GPS :: Iniciando configuración..." << endl;
    
    // Variable donde guardar datos de INS
    //insData insdata;
    // Inicialización de la estructura que contiene los datos del GPS y variables globales
    //initInsValue(insdata);
    // Generación de publicadores
    pub_gps = n.advertise<Modulo_GPS::msg_gps>("gps", 1000);
    pub_errores = n.advertise<Modulo_GPS::msg_error>("error", 1000);
    pub_errores = n.advertise<Modulo_GPS::msg_stream>("teachfile",1000);
    // Inicialización de suscriptores
    ros::Subscriber sub_moduleEnable = n.subscribe("modEnable", 1000, fcn_sub_enableModule);
    ros::Subscriber sub_backup = n.subscribe("backup", 1000, fcn_sub_backup);    
    // Creación de mensaje de publicacion de datos
    Modulo_GPS::msg_gps insMessage;
    
    // Todo esta correcto, lo especificamos con el correspondiente parametro
    n.setParam("estado_modulo_GPS",STATE_OK);
    cout << "ATICA GPS :: Configurado y funcionando" << endl;

    switch (operationMode) {
        case OPERATION_MODE_DEBUG:
            // Funcionamiento del modo debug
            break;
        case OPERATION_MODE_RELEASE:
            // Funcionamiento del modo release
            break;
        case OPERATION_MODE_SIMULATION:
            
            // Funcionamiento del modo simulacion
            
            while (ros::ok() && !exitModule) {
                n.getParam("estado_modulo_GPS",estado_actual);
                if(estado_actual== STATE_ERROR || estado_actual==STATE_OFF){
                    exitModule=true;
                }else{
                    if(readyToPublish){
                        // Obtiene posicion y genera el mensaje
                        insMessage.latitude=insdata.latitude;
                        insMessage.longitude=insdata.longitude;
                        insMessage.altitude=insdata.altitude;
                        insMessage.roll=insdata.roll;
                        insMessage.pitch=insdata.pitch;
                        insMessage.yaw=insdata.yaw;
                        // Publica posicion
                        pub_gps.publish(insMessage);
                        readyToPublish=false;
                    }
                    // Prepara la recepcion de mensajes
                    ros::spinOnce();
                }                
            }       
                break;
        default:
                break;
    }

    return 0;
}

/*******************************************************************************
 *******************************************************************************
 *                              SUSCRIPTORES
 * *****************************************************************************
 * ****************************************************************************/

void fcn_sub_enableModule(const Modulo_GPS::msg_module_enable msg) {
    // TODO
}

void fcn_sub_backup(const Modulo_GPS::msg_backup msg) {
    // TODO
    // Actualizar insdata con lo de backup
    readyToPublish=true;
}
/*******************************************************************************
 *******************************************************************************
 *                              FUNCIONES PROPIAS
 * *****************************************************************************
 * ****************************************************************************/

//Funciones propias

bool connect() {
    return true;
}

bool disconnect() {
    return true;
}

bool configure() {
    return true;
}

bool sendData() {
    return true;
}

bool recvData() {
    return true;
}

bool isAlive() {
    return true;
}

bool checkStateGPS() {
    return true;
}

void initModuleVariables(){
    insdata.latitude=5;
    insdata.longitude=0;
    insdata.altitude=0;
    insdata.roll=10;
    insdata.pitch=0;
    insdata.yaw=0;
    exitModule=false;
    readyToPublish=false;
}