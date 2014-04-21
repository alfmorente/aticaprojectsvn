#include "Modulo_GPS/gps.h"

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

    // Generación de publicadores
    pub_gps = n.advertise<Common_files::msg_gps>("gps", 1000);
    pub_errores = n.advertise<Common_files::msg_error>("error", 1000);
    pub_stream = n.advertise<Common_files::msg_stream>("teachfile",1000);
    // Inicialización de suscriptores
    ros::Subscriber sub_moduleEnable = n.subscribe("modEnable", 1000, fcn_sub_enableModule);
    ros::Subscriber sub_backup = n.subscribe("backup", 1000, fcn_sub_backup);    
    // Creación de mensaje de publicacion de datos
    Common_files::msg_gps insMessage;
    
    // Todo esta correcto, lo especificamos con el correspondiente parametro
    n.setParam("estado_modulo_GPS",STATE_OK);
    cout << "ATICA GPS :: Configurado y funcionando" << endl;
    GPS_Management *gps;

    switch (operationMode) {
        case OPERATION_MODE_DEBUG:

            // Funcionamiento del modo debug
            gps = new GPS_Management();
            
            if (gps->isPortOpened()) {

                //gps->gps_log_general("bestgpsposa", "");
                //gps->rcvData();
                
                cout << "Configurando (Modo de alineamiento inicial)..." << endl;
                while (!gps->gps_conf_alignmentmode(ALIGNMENTMODE_UNAIDED));
                cout << "Establecido modo de alineamiento - KINEMATIC" << endl;

                cout << "Configuracion (Azimuth inicial)..." << endl;
                while (!gps->gps_conf_setinitazimuth(-90, 5));
                cout << "Azimuth OK" << endl;
                /*
                cout << "Configurando offset de la antena..." << endl;
                while (!gps->gps_conf_setimutoantoffset(-35,15,62,0,0,0));
                cout << "Offset antena OK" << endl;
                 */
                gps->gps_log_general("bestgpsposa", "ontime 0.1");
                while (gps->getGPSPos().sol_status != "SOL_COMPUTED") {
                    gps->rcvData();
                    cout << "Esperando Alineamiento de GPS: " << gps->getGPSPos().sol_status << endl;
                }
                cout << "GPS Alineado" << endl;

                //cout << "Comenzar movimiento 4km/h para alinear IMU" << endl;

                gps->gps_log_general("inspvaa", "ontime 0.1");
                while ((gps->getInspVa().status != "INS_SOLUTION_GOOD") && (gps->getInspVa().status != "INS_ALIGNMENT_COMPLETE")) {
                    gps->rcvData();
                    cout << "Esperando Alineamiento de IMU: " << gps->getInspVa().status << endl;
                }
                cout << "IMU Alineado" << endl;
                
                bool flagInspva = false,flagBestGPSPosa = false;
                int typeFrame;
                
                while (ros::ok() && !exitModule) {
                    n.getParam("estado_modulo_GPS",estado_actual);
                    if(estado_actual== STATE_ERROR || estado_actual==STATE_OFF){
                        exitModule=true;
                    }else {
                        typeFrame = gps->rcvData();
                        
                        switch (typeFrame) {
                            case TT_BESTGPSPOSA:
                                flagBestGPSPosa = true;
                                break;
                            case TT_INSPVAA:
                                flagInspva = true;
                                break;
                            default:
                                break;
                        }

                        if (flagBestGPSPosa && flagInspva) {
                            
                            // Creacion del mensaje
                            insMessage.latitude=gps->getGPSPos().lat;
                            insMessage.longitude=gps->getGPSPos().lon;
                            insMessage.altitude=gps->getGPSPos().hgt;
                            insMessage.roll=gps->getInspVa().roll;
                            insMessage.pitch=gps->getInspVa().pitch;
                            insMessage.yaw=gps->getInspVa().azimuth;
                            // Publica posicion
                            pub_gps.publish(insMessage);
                        }
                    }
                    ros::spinOnce();
                }
            } else {
                cout << "El puerto no se ha abierto" << endl;
            }
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
                }else {
                    if (launchTeach) { // LANZAR TEACH
                        // Lanzar el hilo
                        teachThread.Run();
                        launchTeach = false;
                    }

                    if(readyToPublish){
                        readyToPublish=false;
                        // Obtiene posicion y genera el mensaje
                        insMessage.latitude=insdata.latitude;
                        insMessage.longitude=insdata.longitude;
                        insMessage.altitude=insdata.altitude;
                        insMessage.roll=insdata.roll;
                        insMessage.pitch=insdata.pitch;
                        insMessage.yaw=insdata.yaw;
                        // Publica posicion
                        pub_gps.publish(insMessage);
                        if(teachActive){
                            teachThread.setMode(true);
                            teachData.latitude=insdata.latitude;
                            teachData.longitude=insdata.longitude;
                            teachThread.queueGPSdata.push(teachData);
                        }else{
                            teachThread.setMode(false);
                        }
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

void fcn_sub_enableModule(const Common_files::msg_module_enable msg) {
    // TODO
    if(msg.id_module==ID_MOD_TEACH){
        if(msg.status==ON){
            if(!teachActive){
                teachActive=true;
                launchTeach=true;
            }
        }else if(msg.status==OFF){
            if(teachActive){
                teachActive=false;
            }
        }
    }
}

void fcn_sub_backup(const Common_files::msg_backup msg) {
    // TODO
    // Actualizar insdata con lo de backup
    // PROVISIONAL :: Hasta obtener la función se rellena con random
    srand(time(NULL));
    insdata.altitude = ((rand()%60000000)/1000)-10000;
    insdata.latitude = ((rand()%18100000)/100000)-90;
    insdata.longitude = ((rand()%36100000)/100000)-180;
    insdata.roll = ((rand()%36100000)/100000)-180;
    insdata.pitch = ((rand()%36100000)/100000)-180;
    insdata.yaw = ((rand()%36100000)/100000);
    
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
    teachActive=false;
    launchTeach=false;
}