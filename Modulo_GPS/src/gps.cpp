#include "../include/Modulo_GPS/gps.h"

ros::Publisher pub_gps;
ros::Publisher pub_errores;

using namespace std;

int main(int argc, char **argv) {

    // Obtencion del modo de operacion y comprobacion de que es correcto
    int operationMode;
    if ((operationMode = getOperationMode(argc, argv)) == 0) {
        return 1;
    }

    // Orden para la parada manual con CTtrl+C
    init_signals();

    // Inicio de ROS
    ros::init(argc, argv, "GPS");

    // Manejador ROS
    ros::NodeHandle n;

    // Espera activa de inicio de modulo
    int estado_actual = STATE_OFF;
    while (estado_actual != STATE_CONF) {
        n.getParam("estado_modulo_GPS", estado_actual);
    }
    cout << "ATICA GPS :: Iniciando configuración..." << endl;

    // Generación de publicadores
    pub_gps = n.advertise<Modulo_GPS::msg_gps>("gps", 1000);
    pub_errores = n.advertise<Modulo_GPS::msg_error>("error", 1000);
    // Inicialización de suscriptores
    ros::Subscriber sub_moduleEnable = n.subscribe("modEnable", 1000, fcn_sub_enableModule);
    ros::Subscriber sub_backup = n.subscribe("backup", 1000, fcn_sub_backup);

    switch (operationMode) {
        case OPERATION_MODE_DEBUG:
            // Funcionamiento del modo debug
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

    /*
    // Inicializacion de variable global de fin de modulo
    exitModule=false;

    // Conexion y configuracion del dispositivo
    connect();
    configure();

    // Todo esta correcto, lo especificamos con el correspondiente parametro
    n.setParam("estado_modulo_GPS",STATE_OK);
    cout << "Atica GPS :: Configurado y funcionando" << endl;

    while (ros::ok() && !exitModule)
    {
        if(isAlive()){
            if(checkStateGPS()){
                // Genera la informacion en msg_gps y se publica

                n.getParam("estado_modulo_GPS",estado_actual);
                if(estado_actual== STATE_ERROR || estado_actual==STATE_OFF){
                    exitModule=true;
                }
            }else{
                Modulo_GPS::msg_error msg_err;
                msg_err.id_subsistema = SUBS_GPS;
                msg_err.id_error=0; // TODO por definir
                pub_errores.publish(msg_err);

                n.getParam("estado_modulo_GPS",estado_actual);
                if(estado_actual== STATE_ERROR || estado_actual==STATE_OFF){
                    exitModule=true;
                }
            }
        }else{
            Modulo_GPS::msg_error msg_err;
            msg_err.id_subsistema = SUBS_GPS;
            msg_err.id_error=0; // TODO por definir
            pub_errores.publish(msg_err);
            exitModule=true;
        }
    }
     */
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

