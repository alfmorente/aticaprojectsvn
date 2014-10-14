#include "Modulo_GPS_IMU/gps.h"

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
    

    // Orden para la parada manual con CTtrl+C
    init_signals();
    // Inicio de ROS
    ros::init(argc, argv, "GPS");
    // Manejador ROS
    ros::NodeHandle n;

    cout << "ATICA GPS :: Esperando señal de inicio..." << endl;
    // Espera activa de inicio de modulo
    //int estado_actual = STATE_OK;
    int estado_actual = STATE_OFF;
    while (estado_actual != STATE_CONF) {
        n.getParam("state_module_gps", estado_actual);
    }
    cout << "ATICA GPS :: Iniciando configuración..." << endl;

    // Generación de publicadores
    pub_gps = n.advertise<Common_files::msg_gps>("gps", 1000);
    pub_errores = n.advertise<Common_files::msg_error>("error", 1000);
    
    // Creación de mensaje de publicacion de datos
    Common_files::msg_gps insMessage;
    // Creacion de mensaje de errores
    Common_files::msg_error errMessage;
    
    // Todo esta correcto, lo especificamos con el correspondiente parametro
    n.setParam("estado_modulo_GPS",STATE_OK);
    cout << "ATICA GPS :: Configurado y funcionando" << endl;
    
    // Espera la señal de OK proviniento de Gestion del sistema
    int system_status=STATE_SYSTEM_OFF;
    while(system_status!=STATE_SYSTEM_ON && estado_actual != STATE_OFF){
        n.getParam("state_system", system_status);
        n.getParam("state_module_gps", estado_actual);
        usleep(50000);
    }
    
    GPS_Management *gps;

    switch (operationMode) {
        case OPERATION_MODE_DEBUG:

            // Funcionamiento del modo debug
            gps = new GPS_Management();
                                    
            if (gps->configurePuertoSerie()) {
                while (ros::ok() && !exitModule) {
                    
                    n.getParam("estado_modulo_GPS",estado_actual);
                    if(estado_actual== STATE_ERROR || estado_actual==STATE_OFF){
                        exitModule=true;
                    }else {
                        //cout << "Recepcion de comandos por puerto serie" << endl;
                        string s = gps->rcvData();
                        gps->getParamsCarly(s);
                                                 
                        // Creacion del mensaje de datos
                        if (gps->gpsFix == 1) {
                            if (gps->sat >= 6) {
                                if ((contSat == 1) || (contFix == 1)) {
                                    cout <<"Envío de Fin de Error" << endl;
                                    errMessage.id_subsystem = SUBS_GPS;
                                    errMessage.id_error = GPS_GLOBAL_ERROR;
                                    errMessage.type_error = TOE_END_ERROR;
                                    pub_errores.publish(errMessage);
                                    contSat = 0;
                                    contFix = 0;
                                }
                                
                                insMessage.roll=gps->roll;
                                insMessage.pitch=gps->pitch;    
                                insMessage.yaw=gps->yaw;                                                         
                                insMessage.latitude=(gps->latitude)/pow(10, 7);
                                insMessage.longitude=(gps->longitude)/pow(10, 7);
                                insMessage.altitude=(gps->altitude)/10;

                                pub_gps.publish(insMessage);
                            }
                            else {
                                if (contSat == 0) {
                                    cout << "Satelites insuficientes" << endl;
                                    errMessage.id_subsystem = SUBS_GPS;
                                    errMessage.id_error = INSUFFICIENT_OBS;
                                    errMessage.type_error = TOE_UNDEFINED;
                                    pub_errores.publish(errMessage);
                                    contSat++;
                                }
                            }
                        }
                        else {
                           if (contFix == 0) {
                                cout << "Insuficiente FIX" << endl;
                                errMessage.id_subsystem = SUBS_GPS;
                                errMessage.id_error = INVALID_FIX;
                                errMessage.type_error = TOE_UNDEFINED;
                                pub_errores.publish(errMessage);
                                contFix++;
                           }
                        }
                    }
                    ros::spinOnce();
                    usleep(25000);
                }
            } else {
                cout << "El puerto no se ha abierto" << endl;
                errMessage.id_subsystem = SUBS_GPS;
                errMessage.id_error = CONFIG_CONFIG_SERIALPORT_ERROR;
                errMessage.type_error = TOE_UNDEFINED;
                pub_errores.publish(errMessage);
            }
            gps->closePuertoSerie();
            break;
                
        case OPERATION_MODE_RELEASE:
            // Funcionamiento del modo release
            break;
        case OPERATION_MODE_SIMULATION:
            // Funcionamiento del modo simulacion            
            break;
        default:
                break;
    }
    
    return 0;
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
            
    exitModule=false;
    readyToPublish=false;
    contSat = 0;
    contFix = 0;
        
}