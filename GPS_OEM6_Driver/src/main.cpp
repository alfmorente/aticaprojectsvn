#include "GPS_OEM6_Driver/driverNovatelFlexPakG2V1.h"
#include "GPS_OEM6_Driver/gps.h" // Variables globales y publicadores

using namespace std;

driverNovatelFlexPakG2V1 gps(&res, serial);

void getData(void); // Posicion

int main(int argc, char **argv) {
    
    int operationMode;
    // Obtencion del modo de operacion y comprobacion de que es correcto
    if ((operationMode = getOperationMode(argc, argv)) == 0) {
        return 1;
    }

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
    
        // Generación de publicadores
    pub_gps = n.advertise<Common_files::msg_gps>("gps", 1000);
    pub_errores = n.advertise<Common_files::msg_error>("error", 1000);
    pub_errores = n.advertise<Common_files::msg_stream>("teachfile",1000);
    // Inicialización de suscriptores
    ros::Subscriber sub_moduleEnable = n.subscribe("modEnable", 1000, fcn_sub_enableModule);
    ros::Subscriber sub_backup = n.subscribe("backup", 1000, fcn_sub_backup);    
    // Creación de mensaje de publicacion de datos
    Common_files::msg_gps insMessage;
    
    // Todo esta correcto, lo especificamos con el correspondiente parametro
    n.setParam("estado_modulo_GPS",STATE_OK);
    cout << "ATICA GPS :: Configurado y funcionando" << endl;
    
    initModuleVariables();
    
    switch (operationMode) {
        case OPERATION_MODE_DEBUG:
            // Funcionamiento del modo debug
            //  Set de la frecuencia de recepcion de tramas
            
            if (gps.setFrequency(freq) != 0) {
                cout << "Error en el cambio de frecuencia" << endl;
                return -1;
            }
            cout << "Frecuencia cambiada a " << freq << "Hz" << endl;

            // Recoleccion de datos continua
            while (ros::ok() && !exitModule) {
                getData();
            }
            gps.apagarGPS();
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

void getData(void) {

    char *ptr;
    //char* leido=new char[255];
    char* frame = gps.getFrame();
    cout << "Trama con datos " << frame << endl;
    ptr = gps.getHeader(frame);
    if (strcmp(ptr, GPGGA) == 0) {
        cout << "Se ha obtenido una trama " << GPGGA << endl;
        if(!gps.GPGGAdata(&hora, &minutos, &segundos, &decsegundos, &latitude, &dirlatitude, &longitude, &dirlongitude, &gpsquality, &numsatellites, &precision, &altitude, &unit, checksum)){
            cout << "La trama viene vacia" << endl;
            cout << "===============================================" << endl;
        }else{
            // Exposicion de los datos
            cout << "Tiempo de muestreo: " << hora << ":" << minutos << ":" << segundos << "." << decsegundos << endl;
            cout << "Latitud: " << latitude << " con direccion " << dirlatitude  << endl;
            cout << "Longitud: " << longitude << " con direccion " << dirlongitude << endl;
            cout << "Calidad GPS: " << gpsquality << endl;
            cout << "Num. Satelites: " << numsatellites << endl;
            cout << "Precision: " << precision << endl;
            cout << "Altitud de la antena: " << altitude;
            cout << "Unidades: " << endl;
            cout << "===============================================" << endl;
        }
            
    } else if (strcmp(ptr, GPVTG) == 0) {
        cout << "Se ha obtenido una trama " << GPVTG << endl;
        if(!gps.GPVTGdata(&track_true, &t_indicator, &track_mag, &m_indicator, &knot_speed, &k_indicator, &km_speed, &km_indicator, checksum)){
            cout << "La trama viene vacia" << endl;
            cout << "===============================================" << endl;
        }else{
            // Exposicion de los datos
            cout << "Track: Grados (true): " << track_true << " Indicador: " << t_indicator << " Grados (magnéticos): " << track_mag << endl; 
            cout << "Velocidad (KNOTS): " << knot_speed << " Indicador: " << k_indicator << endl;
            cout << "Velocidad (km/hr): " << km_speed << " Indicador: " << km_indicator << endl;
            cout << "===============================================" << endl;
        }
    }
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
    readyToPublish=true;
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
    freq=1;
}