#include "../include/Modulo_Gest_Errores/gest_errores.h"

  // Publicadores

  ros::Publisher pub_modo;
  ros::Publisher pub_errores;
  ros::Publisher pub_com_teleop;
  ros::Publisher pub_avail_mode;

  using namespace std;

// what has to be done at program exit
void do_exit(int error)
{
  printf("finished (%d).\n\n", error);
  exit(error);
}

// the signal handler for manual break Ctrl-C
void signal_handler(int signal)
{
  do_exit(0);
}

// what has to be done at program start
void init()
{
  /* install signal handlers */
  signal(SIGTERM, signal_handler);
  signal(SIGINT, signal_handler);
  
}

int main(int argc, char **argv)
{
  // Orden para la parada manual con CTtrl+C
  init();

  // Inicio de ROS
  ros::init(argc, argv, "gest_errores");

  // Manejador ROS
  ros::NodeHandle n;

  // Espera activa de inicio de modulo
  int estado_actual=STATE_OK;       // Para poder probar
 /* int estado_actual=STATE_OFF;
  while(estado_actual!=STATE_CONF){
          n.getParam("estado_modulo_gestErrores",estado_actual);
  }
  cout << "Atica GEST. ERRORES :: Iniciando configuración..." << endl;
*/
  // Inicializacion de publicadores
  pub_modo = n.advertise<Modulo_Gest_Errores::msg_modo>("mode", 1000);
  pub_errores = n.advertise<Modulo_Gest_Errores::msg_errores>("error", 1000);
  pub_com_teleop = n.advertise<Modulo_Gest_Errores::msg_com_teleoperado>("teleop", 1000);
  pub_avail_mode = n.advertise<Modulo_Gest_Errores::msg_available_mode>("avail_mode", 1000);

  // Creacion de suscriptores
  ros::Subscriber sub_errores = n.subscribe("error", 1000, fcn_sub_errores);
  ros::Subscriber sub_modo = n.subscribe("mode", 1000, fcn_sub_modo);
  ros::Subscriber sub_confirm = n.subscribe("confirm", 1000, fcn_sub_confirm);

  // Inicialización de variables globales
  modoActual=MODE_NEUTRAL;
  exitModule=false;
  confirm_flag=false;

  // Todo esta correcto, lo especificamos con el correspondiente parametro
  n.setParam("estado_modulo_gestErrores",STATE_OK);
  cout << "Atica GEST. ERRORES :: Configurado y funcionando" << endl;

    // Inicialización de mensaje available_mode todo a true
    int i = 0;
    for (i=MODE_TELEOP;i<=MODE_CONVOY_AUTO;i++){
        avail_mode.available_mode[i]=true;
    }

  while (ros::ok() && !exitModule)
  {
      n.getParam("estado_modulo_gestErrores",estado_actual);
      if(estado_actual==STATE_ERROR || estado_actual== STATE_OFF){
          exitModule=true;
      }
      ros::spinOnce();
  }
  cout << "Atica GEST. ERRORES :: Módulo finalizado" << endl;
  return 0;
}

/*******************************************************************************
 *******************************************************************************
 *                              SUSCRIPTORES
 * *****************************************************************************
 * ****************************************************************************/

// Suscriptor de modo
void fcn_sub_modo(const Modulo_Gest_Errores::msg_modo msg)
{
    if (msg.status == MODE_START)
        modoActual= msg.mode;

    else if (msg.status == MODE_EXIT)
        modoActual=MODE_NEUTRAL;
}

// Suscriptor de confirmacion de modo neutro
void fcn_sub_confirm(const Modulo_Gest_Errores::msg_confirm msg){
    if (msg.conf_flag)
        confirm_flag=true;
}

// Suscriptor de errores
void fcn_sub_errores(const Modulo_Gest_Errores::msg_errores msg)
{
    int i=0,j=0;        // Variables para bucles for
    short end_error;    // Variable puente usada para generar el msg

    if(msg.type_error==TOE_UNDEFINED){                // Evita leer mensajes que el mismo haya enviado
        short type_error = isWarningOrCritical(msg,modoActual);     // Comprueba gravedad del error (CRIT o WARN)
        Modulo_Gest_Errores::msg_errores msg_err;
        switch(type_error){
            case TOE_CRITICAL:
                    // Generacion de mensaje de error
                    msg_err.id_subsystem=msg.id_subsystem;
                    msg_err.id_error=msg.id_error;
                    msg_err.type_error=TOE_CRITICAL;
                    cout << "Error critico\n";
                    pub_errores.publish(msg_err);
                    // Al ser un error critico se pasa a modo neutro
                    switchNeutral();
                    // Actualizo las tablas de gestion de modulos disponibles
                    num_err_mode[modoActual]++;
                    avail_mode.available_mode[modoActual]=false;
                    break;
            case TOE_WARNING:
                    // Generacion de mensaje de error
                    msg_err.id_subsystem=msg.id_subsystem;
                    msg_err.id_error=msg.id_error;
                    msg_err.type_error=TOE_WARNING;
                    cout << "Error no critico\n";
                    pub_errores.publish(msg_err);
                    break;
            default:
                    cout << "Atica Gest. Errores :: Se ha recibido un error no clasificado" << endl;
        }
        // Actualización del registro de modos disponibles
        short type_error_mode;
        for (i=MODE_TELEOP;i<=MODE_CONVOY_AUTO;i++)
            {
            if (i != modoActual){
                type_error_mode = isWarningOrCritical(msg,i);   // Comprueba si el error recibido es CRIT o WARN para cada modo
                if (type_error_mode == TOE_CRITICAL){           // independientemente del modo actual
                    num_err_mode[i]++;
                    avail_mode.available_mode[i]=false;         //ACTUALIZA CAMPO AVAILABLE_MODE
                    }
                }
            cout << "Numeros de errores " << i << " es igual a " << num_err_mode[i] << endl;
            cout << "Modo disponible de " << i << " es " << (short) avail_mode.available_mode[i] << endl;
            }
        pub_avail_mode.publish(avail_mode);
    }
    // Ordenes para actualizar msg de modos disponibles al recibir un mensaje fin de error
    else if (msg.type_error==TOE_END_ERROR){
        for (j=MODE_TELEOP;j<=MODE_CONVOY_AUTO;j++){
            end_error=isWarningOrCritical(msg,j);       // Comprueba para cada modo si el error finalizado es critico o warning
            if ((end_error == TOE_CRITICAL) && (num_err_mode[j]>0)){
                num_err_mode[j]--;
                // Comprueba si el num de errores CRIT acumulados en un modo es cero
                // Si no es cero, el modo sigue sin estar disponible
                if (num_err_mode[j] == 0){
                    avail_mode.available_mode[j]=true;
                    pub_avail_mode.publish(avail_mode);
                }
            }
            cout << "Numeros de errores " << j << " es igual a " << num_err_mode[j] << endl;
            cout << "Modo disponible de " << j << " es " << (short) avail_mode.available_mode[j] << endl;
        }
    }
}

/*******************************************************************************
 *******************************************************************************
 *                              FUNCIONES PROPIAS
 * *****************************************************************************
 * ****************************************************************************/
// Especifica que tipo de error se ha recibido y rellena el correspondiente
// campo del mensaje. Luego lo envia relleno.
short isWarningOrCritical(Modulo_Gest_Errores::msg_errores msg, short modo){
    short error=0;
    switch (modo){
        case (MODE_TELEOP):             // Modo REMOTE
            error = mode_remote_error(msg);
            break;

        case (MODE_START_ENGINE):       // Modo START ENGINE
            error = mode_startengine_error (msg);
            break;

        case (MODE_STOP_ENGINE):         // Modo STOP ENGINE
            error = mode_stopengine_error (msg);
            break;
            
        case (MODE_ENGAGE_BRAKE):       // Modo ENGAGE BRAKE
            error = mode_engagebrake_error(msg);
            break;
            
        case (MODE_PLAN):               // Modo PLAN
            error = mode_plan_error(msg);
            break;
            
        case (MODE_COME_TO_ME):         // Modo COME TO ME
            error = mode_cometome_error(msg);
            break;

        case (MODE_FOLLOW_ME):          // Modo FOLLOW ME
            error = mode_followme_error(msg);
            break;

        case (MODE_MAPPING):            // Modo MAPPING
            error = mode_mapping_error(msg);
            break;

        case (MODE_TEACH):              // Modo TEACH
            error = mode_teach_error(msg);
            break;

        case (MODE_CONVOY):             // Modo CONVOY
            error = mode_convoy_error(msg);
            break;

        case (MODE_CONVOY_TELEOP):      // Modo CONVOY+TELEOP
            error = mode_conv_teleop_error(msg);
            break;

        case(MODE_CONVOY_AUTO):         // Modo CONVOY+AUTO
            error = mode_conv_auto_error(msg);
            break;

        case (MODE_NEUTRAL):            // Modo Neutro > Todo Warning
            msg.type_error=TOE_WARNING;
            error = msg.type_error;
            break;

        default:                        // Modo incorrecto
            msg.type_error=TOE_UNDEFINED;
            error = msg.type_error;
            break;
    }
    return error;
}

// Método que define la gravedad del error para el modo REMOTE
short mode_remote_error(Modulo_Gest_Errores::msg_errores msg)
{
     switch (msg.id_subsystem){
                case SUBS_COMUNICACIONES:
                    switch (msg.id_error){
                        case (LINK_ERROR_900):
                            msg.type_error=TOE_CRITICAL;
                            break;
                        case (LINK_ERROR_2400):
                            msg.type_error=TOE_CRITICAL;
                            break;
                        case (JAUS_CONFIG_ERROR):
                            msg.type_error=TOE_CRITICAL;
                            break;
                        case (CREATE_COMPONENT_ERROR):
                            msg.type_error=TOE_CRITICAL;
                            break;
                        case (RUN_COMPONENT_ERROR):
                            msg.type_error=TOE_CRITICAL;
                            break;
                        case (COMM_LOG_FILE_ERROR):
                            msg.type_error=TOE_CRITICAL;
                            break;
                        case (COMM_CONFIG_FILE_ERROR):
                            msg.type_error=TOE_CRITICAL;
                            break;
                        case (COMM_CONFIG_FILE_STRUCTURE_ERROR):
                            msg.type_error=TOE_CRITICAL;
                            break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_GEST_SISTEMA:
                    switch (msg.id_error){
                        case (MODE_NOT_AVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (COMM_MODULE_NA):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (REMOTE_MODULE_NA):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (DRIVING_MODULE_NA):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (NAVIGATION_MODULE_NA):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (CAMERA_MODULE_NA):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (GPS_MODULE_NA):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (FRONT_LASER_MODULE_NA):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (REAR_LASER_MODULE_NA):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (LASER3D_MODULE_NA):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (BEACON_MODULE_NA):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (RDF_MODULE_NA):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (HL_MODULE_NA):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (CONVOY_MODULE_NA):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_TELEOP:
                    switch (msg.id_error){
                        case REMOTE_PARAMETER_OUTRANGE:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case NEAR_OBSTACLE_DETECTION:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case FAR_OBSTABLE_DETECTION:
                            msg.type_error=TOE_WARNING;
                           // cout << msg.type_error << endl;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_CONDUCCION:
                    switch (msg.id_error){
                        case CONNECTION_CAN_FAIL:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case COMMUNICATION_CAN_FAIL:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case START_STOP_FAILURE:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case THROTTLE_FAILURE:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case HANDBRAKE_FAILURE:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case BRAKE_FAILURE:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case GEAR_SHIFT_FAILURE:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case STEER_FAILURE:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case DIFFERENTIAL_LOCK_FAILURE:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_NAVEGACION:
                    switch (msg.id_error){
                        case (ERROR_NAVIGATION_OBSTACLE_UNAVOIDABLE):
                            msg.type_error=TOE_WARNING;
                        break;
                        case (ERROR_NAVIGATION_WAYPOINTS_GETTING_TIMEOUT):
                            msg.type_error=TOE_WARNING;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_CAMARA:
                    switch (msg.id_error){
                        break;
                        case (ERROR_CAMERA_COMMUNICATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_CAMERA_CONFIGURATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_GPS:
                    switch (msg.id_error){
                        case GPS_GLOBAL_ERROR:
                            msg.type_error=TOE_END_ERROR;
                        break;
                        case DATA_RCV_FAILED:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case CONFIG_OPEN_SERIALPORT_ERROR:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case CONFIG_CONFIG_SERIALPORT_ERROR:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case CONFIG_ALIGNMENT_MODE_ERROR:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case CONFIG_INITIAL_AZIMUT_ERROR:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case CONFIG_ANTOFFSET_ERROR:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case INS_INACTIVE:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case INS_ALIGNING:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case INS_SOLUTION_NOT_GOOD:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case INS_BAD_GPS_AGREEMENT:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case INS_ALIGNMENT_COMPLETE:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case INSUFFICIENT_OBS:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case NO_CONVERGENCE:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case SINGULARITY:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case COV_TRACE:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case TEST_DIST:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case COLD_START:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case V_H_LIMIT:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case VARIANCE:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case RESIDUALS:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case DELTA_POS:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case NEGATIVE_VAR:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case INTEGRITY_WARNING:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case IMU_UNPLUGGED:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case PENDING:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case INVALID_FIX:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case UNAUTHORIZED_STATE:
                            msg.type_error=TOE_CRITICAL;
                        break;

                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_LASER_DELANTERO:
                    switch (msg.id_error){
                        case LASER_SOCKET_FAIL:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case CONNECTION_ERROR:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case COMM_ERROR:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case INVALID_SAMPLE_FREQ:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case INVALID_ANGLE_RESOLUTION:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case INVALID_ANGLE_SAMPLE_RESOLUTION:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case INVALID_SCAN_AREA:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case UNKNOWN_ERROR:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case START_MEASURE_NA:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case STOP_MEASURE_NA:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case SELECT_USER_LEVEL_NA:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case SET_LMS_OUTPUT_NA:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case SAVE_DATA_NA:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case START_DEVICE_NA:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case INCORRECT_ANSWER:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case FRAME_OVERFLOW:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case LASER_LOG_FILE_ERROR:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case LASER_CONFIG_FILE_ERROR:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case LASER_CONFIG_FILE_STRUCTURE_ERROR:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_LASER_TRAS_IZQ:
                    switch (msg.id_error){
                        case LASER_SOCKET_FAIL:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case CONNECTION_ERROR:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case COMM_ERROR:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case INVALID_SAMPLE_FREQ:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case INVALID_ANGLE_RESOLUTION:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case INVALID_ANGLE_SAMPLE_RESOLUTION:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case INVALID_SCAN_AREA:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case UNKNOWN_ERROR:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case START_MEASURE_NA:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case STOP_MEASURE_NA:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case SELECT_USER_LEVEL_NA:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case SET_LMS_OUTPUT_NA:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case SAVE_DATA_NA:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case START_DEVICE_NA:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case INCORRECT_ANSWER:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case FRAME_OVERFLOW:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case LASER_LOG_FILE_ERROR:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case LASER_CONFIG_FILE_ERROR:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case LASER_CONFIG_FILE_STRUCTURE_ERROR:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_LASER_TRAS_DER:
                    switch (msg.id_error){
                        case LASER_SOCKET_FAIL:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case CONNECTION_ERROR:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case COMM_ERROR:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case INVALID_SAMPLE_FREQ:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case INVALID_ANGLE_RESOLUTION:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case INVALID_ANGLE_SAMPLE_RESOLUTION:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case INVALID_SCAN_AREA:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case UNKNOWN_ERROR:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case START_MEASURE_NA:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case STOP_MEASURE_NA:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case SELECT_USER_LEVEL_NA:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case SET_LMS_OUTPUT_NA:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case SAVE_DATA_NA:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case START_DEVICE_NA:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case INCORRECT_ANSWER:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case FRAME_OVERFLOW:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case LASER_LOG_FILE_ERROR:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case LASER_CONFIG_FILE_ERROR:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case LASER_CONFIG_FILE_STRUCTURE_ERROR:
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_LASER_3D:
                    switch (msg.id_error){
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_BEACON:
                    switch(msg.id_error){
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;
                    }
                    break;
                case SUBS_RDF:
                    switch(msg.id_error){
                        case RDF_INSUFFICIENT_DATA:
                            msg.type_error
                    }
                default:
                    msg.type_error=TOE_UNDEFINED;
                    break;     // id_subsystem incorrecto
            }
     return msg.type_error;
}

// Método que define la gravedad del error para el modo START ENGINE
short mode_startengine_error(Modulo_Gest_Errores::msg_errores msg)
{
    switch (msg.id_subsystem){
                case (SUBS_COMUNICACIONES):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_COMM_CONNECTION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_COMM_CONFIGURATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case (SUBS_GEST_SISTEMA):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_MODE_GPS):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_MODE_LASER):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_MODE_DRIVING):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_MODE_NAVIGATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_TELEOP):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_TELEOP_NEAR_OBSTACLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_TELEOP_FAR_OBSTABLE):
                            msg.type_error=TOE_WARNING;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_CONDUCCION):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_DRIVING_HANDBRAKE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_DRIVING_GEAR):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_DRIVING_ENGINE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_DRIVING_ROBOT):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_NAVEGACION):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_WARNING;
                        break;
                        case (ERROR_NAVIGATION_OBSTACLE_UNAVOIDABLE):
                            msg.type_error=TOE_WARNING;
                        break;
                        case (ERROR_NAVIGATION_WAYPOINTS_GETTING_TIMEOUT):
                            msg.type_error=TOE_WARNING;
                        break;
                        default: break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_CAMARA):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_CAMERA_COMMUNICATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_CAMERA_CONFIGURATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default: break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_GPS):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_GPS_NOT_SATELLITE_SIGNAL):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_GPS_SOLUTION_NOT_GOOD):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_GPS_IMU_BAD_ALIGNMENT):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_GPS_MEMORY_OVERFLOW_TEACH):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_LASER_DELANTERO):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_COMMUNICATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_CONFIGURATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_LASER_TRAS_IZQ):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_COMMUNICATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_CONFIGURATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_LASER_TRAS_DER):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_COMMUNICATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_CONFIGURATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_LASER_3D):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_COMMUNICATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_CONFIGURATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                default:
                    msg.type_error=TOE_UNDEFINED;
                    break;     // id_subsystem incorrecto
            }
    return msg.type_error;
}

// Método que define la gravedad del error para el modo STOP ENGINE
short mode_stopengine_error(Modulo_Gest_Errores::msg_errores msg)
{
    switch (msg.id_subsystem){
                case (SUBS_COMUNICACIONES):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_COMM_CONNECTION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_COMM_CONFIGURATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case (SUBS_GEST_SISTEMA):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_MODE_GPS):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_MODE_LASER):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_MODE_DRIVING):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_MODE_NAVIGATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_TELEOP):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_TELEOP_NEAR_OBSTACLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_TELEOP_FAR_OBSTABLE):
                            msg.type_error=TOE_WARNING;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_CONDUCCION):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_DRIVING_HANDBRAKE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_DRIVING_GEAR):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_DRIVING_ENGINE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_DRIVING_ROBOT):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_NAVEGACION):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_WARNING;
                        break;
                        case (ERROR_NAVIGATION_OBSTACLE_UNAVOIDABLE):
                            msg.type_error=TOE_WARNING;
                        break;
                        case (ERROR_NAVIGATION_WAYPOINTS_GETTING_TIMEOUT):
                            msg.type_error=TOE_WARNING;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_CAMARA):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_CAMERA_COMMUNICATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_CAMERA_CONFIGURATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_GPS):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_GPS_NOT_SATELLITE_SIGNAL):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_GPS_SOLUTION_NOT_GOOD):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_GPS_IMU_BAD_ALIGNMENT):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_GPS_MEMORY_OVERFLOW_TEACH):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_LASER_DELANTERO):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_COMMUNICATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_CONFIGURATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_LASER_TRAS_IZQ):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_COMMUNICATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_CONFIGURATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_LASER_TRAS_DER):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_COMMUNICATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_CONFIGURATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_LASER_3D):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_COMMUNICATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_CONFIGURATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                default:
                    msg.type_error=TOE_UNDEFINED;
                    break;     // id_subsystem incorrecto
            }
    return msg.type_error;
}

// Método que define la gravedad del error para el modo ENGAGE BRAKE
short mode_engagebrake_error(Modulo_Gest_Errores::msg_errores msg)
{
    switch (msg.id_subsystem){
                case (SUBS_COMUNICACIONES):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_COMM_CONNECTION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_COMM_CONFIGURATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case (SUBS_GEST_SISTEMA):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_MODE_GPS):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_MODE_LASER):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_MODE_DRIVING):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_MODE_NAVIGATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_TELEOP):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_TELEOP_NEAR_OBSTACLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_TELEOP_FAR_OBSTABLE):
                            msg.type_error=TOE_WARNING;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_CONDUCCION):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_DRIVING_HANDBRAKE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_DRIVING_GEAR):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_DRIVING_ENGINE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_DRIVING_ROBOT):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_NAVEGACION):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_WARNING;
                        break;
                        case (ERROR_NAVIGATION_OBSTACLE_UNAVOIDABLE):
                            msg.type_error=TOE_WARNING;
                        break;
                        case (ERROR_NAVIGATION_WAYPOINTS_GETTING_TIMEOUT):
                            msg.type_error=TOE_WARNING;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_CAMARA):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_CAMERA_COMMUNICATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_CAMERA_CONFIGURATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_GPS):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_GPS_NOT_SATELLITE_SIGNAL):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_GPS_SOLUTION_NOT_GOOD):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_GPS_IMU_BAD_ALIGNMENT):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_GPS_MEMORY_OVERFLOW_TEACH):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_LASER_DELANTERO):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_COMMUNICATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_CONFIGURATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_LASER_TRAS_IZQ):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_COMMUNICATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_CONFIGURATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_LASER_TRAS_DER):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_COMMUNICATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_CONFIGURATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_LASER_3D):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_COMMUNICATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_CONFIGURATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                default:
                    msg.type_error=TOE_UNDEFINED;
                    break;     // id_subsystem incorrecto
            }
    return msg.type_error;
}

// Método que define la gravedad del error para el modo PLAN
short mode_plan_error(Modulo_Gest_Errores::msg_errores msg)
{
    switch (msg.id_subsystem){
                case (SUBS_COMUNICACIONES):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_COMM_CONNECTION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_COMM_CONFIGURATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case (SUBS_GEST_SISTEMA):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_MODE_GPS):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_MODE_LASER):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_MODE_DRIVING):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_MODE_NAVIGATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_TELEOP):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_WARNING;
                        break;
                        case (ERROR_TELEOP_NEAR_OBSTACLE):
                            msg.type_error=TOE_WARNING;
                        break;
                        case (ERROR_TELEOP_FAR_OBSTABLE):
                            msg.type_error=TOE_WARNING;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_CONDUCCION):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_DRIVING_HANDBRAKE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_DRIVING_GEAR):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_DRIVING_ENGINE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_DRIVING_ROBOT):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_NAVEGACION):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_NAVIGATION_OBSTACLE_UNAVOIDABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_NAVIGATION_WAYPOINTS_GETTING_TIMEOUT):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_CAMARA):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_CAMERA_COMMUNICATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_CAMERA_CONFIGURATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_GPS):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_GPS_NOT_SATELLITE_SIGNAL):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_GPS_SOLUTION_NOT_GOOD):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_GPS_IMU_BAD_ALIGNMENT):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_GPS_MEMORY_OVERFLOW_TEACH):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_LASER_DELANTERO):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_COMMUNICATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_CONFIGURATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_LASER_TRAS_IZQ):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_COMMUNICATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_CONFIGURATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_LASER_TRAS_DER):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_COMMUNICATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_CONFIGURATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_LASER_3D):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_COMMUNICATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_CONFIGURATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                default:
                    msg.type_error=TOE_UNDEFINED;
                    break;     // id_subsystem incorrecto
            }
    return msg.type_error;
}

// Método que define la gravedad del error para el modo COME TO ME
short mode_cometome_error(Modulo_Gest_Errores::msg_errores msg)
{
    switch (msg.id_subsystem){
                case (SUBS_COMUNICACIONES):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_COMM_CONNECTION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_COMM_CONFIGURATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case (SUBS_GEST_SISTEMA):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_MODE_GPS):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_MODE_LASER):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_MODE_DRIVING):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_MODE_NAVIGATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_TELEOP):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_WARNING;
                        break;
                        case (ERROR_TELEOP_NEAR_OBSTACLE):
                            msg.type_error=TOE_WARNING;
                        break;
                        case (ERROR_TELEOP_FAR_OBSTABLE):
                            msg.type_error=TOE_WARNING;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_CONDUCCION):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_DRIVING_HANDBRAKE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_DRIVING_GEAR):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_DRIVING_ENGINE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_DRIVING_ROBOT):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_NAVEGACION):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_NAVIGATION_OBSTACLE_UNAVOIDABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_NAVIGATION_WAYPOINTS_GETTING_TIMEOUT):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_CAMARA):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_CAMERA_COMMUNICATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_CAMERA_CONFIGURATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_GPS):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_GPS_NOT_SATELLITE_SIGNAL):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_GPS_SOLUTION_NOT_GOOD):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_GPS_IMU_BAD_ALIGNMENT):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_GPS_MEMORY_OVERFLOW_TEACH):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_LASER_DELANTERO):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_COMMUNICATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_CONFIGURATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_LASER_TRAS_IZQ):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_COMMUNICATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_CONFIGURATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_LASER_TRAS_DER):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_COMMUNICATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_CONFIGURATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_LASER_3D):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_COMMUNICATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_CONFIGURATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                default:
                    msg.type_error=TOE_UNDEFINED;
                    break;     // id_subsystem incorrecto
            }
    return msg.type_error;
}

// Método que define la gravedad del error para el modo FOLLOW ME
short mode_followme_error(Modulo_Gest_Errores::msg_errores msg)
{
    switch (msg.id_subsystem){
                case (SUBS_COMUNICACIONES):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_COMM_CONNECTION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_COMM_CONFIGURATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case (SUBS_GEST_SISTEMA):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_MODE_GPS):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_MODE_LASER):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_MODE_DRIVING):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_MODE_NAVIGATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_TELEOP):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_WARNING;
                        break;
                        case (ERROR_TELEOP_NEAR_OBSTACLE):
                            msg.type_error=TOE_WARNING;
                        break;
                        case (ERROR_TELEOP_FAR_OBSTABLE):
                            msg.type_error=TOE_WARNING;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_CONDUCCION):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_DRIVING_HANDBRAKE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_DRIVING_GEAR):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_DRIVING_ENGINE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_DRIVING_ROBOT):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_NAVEGACION):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_NAVIGATION_OBSTACLE_UNAVOIDABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_NAVIGATION_WAYPOINTS_GETTING_TIMEOUT):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_CAMARA):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_CAMERA_COMMUNICATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_CAMERA_CONFIGURATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_GPS):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_GPS_NOT_SATELLITE_SIGNAL):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_GPS_SOLUTION_NOT_GOOD):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_GPS_IMU_BAD_ALIGNMENT):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_GPS_MEMORY_OVERFLOW_TEACH):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_LASER_DELANTERO):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_COMMUNICATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_CONFIGURATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_LASER_TRAS_IZQ):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_COMMUNICATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_CONFIGURATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_LASER_TRAS_DER):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_COMMUNICATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_CONFIGURATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_LASER_3D):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_COMMUNICATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_CONFIGURATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                default:
                    msg.type_error=TOE_UNDEFINED;
                    break;     // id_subsystem incorrecto
            }
    return msg.type_error;
}

// Método que define la gravedad del error para el modo TEACH
short mode_teach_error(Modulo_Gest_Errores::msg_errores msg)
{
    switch (msg.id_subsystem){
                case (SUBS_COMUNICACIONES):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_COMM_CONNECTION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_COMM_CONFIGURATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case (SUBS_GEST_SISTEMA):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_MODE_GPS):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_MODE_LASER):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_MODE_DRIVING):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_MODE_NAVIGATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_TELEOP):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_WARNING;
                        break;
                        case (ERROR_TELEOP_NEAR_OBSTACLE):
                            msg.type_error=TOE_WARNING;
                        break;
                        case (ERROR_TELEOP_FAR_OBSTABLE):
                            msg.type_error=TOE_WARNING;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_CONDUCCION):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_DRIVING_HANDBRAKE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_DRIVING_GEAR):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_DRIVING_ENGINE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_DRIVING_ROBOT):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_NAVEGACION):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_NAVIGATION_OBSTACLE_UNAVOIDABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_NAVIGATION_WAYPOINTS_GETTING_TIMEOUT):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_CAMARA):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_CAMERA_COMMUNICATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_CAMERA_CONFIGURATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_GPS):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_GPS_NOT_SATELLITE_SIGNAL):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_GPS_SOLUTION_NOT_GOOD):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_GPS_IMU_BAD_ALIGNMENT):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_GPS_MEMORY_OVERFLOW_TEACH):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_LASER_DELANTERO):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_COMMUNICATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_CONFIGURATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_LASER_TRAS_IZQ):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_COMMUNICATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_CONFIGURATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_LASER_TRAS_DER):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_COMMUNICATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_CONFIGURATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_LASER_3D):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_COMMUNICATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_CONFIGURATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                default:
                    msg.type_error=TOE_UNDEFINED;
                    break;     // id_subsystem incorrecto
            }
    return msg.type_error;
}

// Método que define la gravedad del error para el modo MAPPING
short mode_mapping_error(Modulo_Gest_Errores::msg_errores msg)
{
    switch (msg.id_subsystem){
                case (SUBS_COMUNICACIONES):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_COMM_CONNECTION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_COMM_CONFIGURATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case (SUBS_GEST_SISTEMA):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_MODE_GPS):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_MODE_LASER):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_MODE_DRIVING):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_MODE_NAVIGATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_TELEOP):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_WARNING;
                        break;
                        case (ERROR_TELEOP_NEAR_OBSTACLE):
                            msg.type_error=TOE_WARNING;
                        break;
                        case (ERROR_TELEOP_FAR_OBSTABLE):
                            msg.type_error=TOE_WARNING;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_CONDUCCION):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_DRIVING_HANDBRAKE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_DRIVING_GEAR):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_DRIVING_ENGINE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_DRIVING_ROBOT):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_NAVEGACION):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_NAVIGATION_OBSTACLE_UNAVOIDABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_NAVIGATION_WAYPOINTS_GETTING_TIMEOUT):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_CAMARA):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_CAMERA_COMMUNICATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_CAMERA_CONFIGURATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_GPS):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_GPS_NOT_SATELLITE_SIGNAL):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_GPS_SOLUTION_NOT_GOOD):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_GPS_IMU_BAD_ALIGNMENT):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_GPS_MEMORY_OVERFLOW_TEACH):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_LASER_DELANTERO):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_COMMUNICATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_CONFIGURATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_LASER_TRAS_IZQ):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_COMMUNICATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_CONFIGURATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_LASER_TRAS_DER):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_COMMUNICATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_CONFIGURATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_LASER_3D):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_COMMUNICATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_CONFIGURATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                default:
                    msg.type_error=TOE_UNDEFINED;
                    break;     // id_subsystem incorrecto
            }
    return msg.type_error;
}

// Método que define la gravedad del error para el modo CONVOY
short mode_convoy_error(Modulo_Gest_Errores::msg_errores msg){
   switch (msg.id_subsystem){
                case (SUBS_COMUNICACIONES):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_COMM_CONNECTION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_COMM_CONFIGURATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case (SUBS_GEST_SISTEMA):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_MODE_GPS):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_MODE_LASER):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_MODE_DRIVING):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_MODE_NAVIGATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_TELEOP):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_WARNING;
                        break;
                        case (ERROR_TELEOP_NEAR_OBSTACLE):
                            msg.type_error=TOE_WARNING;
                        break;
                        case (ERROR_TELEOP_FAR_OBSTABLE):
                            msg.type_error=TOE_WARNING;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_CONDUCCION):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_DRIVING_HANDBRAKE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_DRIVING_GEAR):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_DRIVING_ENGINE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_DRIVING_ROBOT):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_NAVEGACION):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_NAVIGATION_OBSTACLE_UNAVOIDABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_NAVIGATION_WAYPOINTS_GETTING_TIMEOUT):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_CAMARA):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_CAMERA_COMMUNICATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_CAMERA_CONFIGURATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_GPS):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_GPS_NOT_SATELLITE_SIGNAL):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_GPS_SOLUTION_NOT_GOOD):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_GPS_IMU_BAD_ALIGNMENT):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_GPS_MEMORY_OVERFLOW_TEACH):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_LASER_DELANTERO):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_COMMUNICATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_CONFIGURATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_LASER_TRAS_IZQ):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_COMMUNICATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_CONFIGURATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_LASER_TRAS_DER):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_COMMUNICATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_CONFIGURATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_LASER_3D):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_COMMUNICATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_CONFIGURATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                default:
                    msg.type_error=TOE_UNDEFINED;
                    break;     // id_subsystem incorrecto
            }
   return msg.type_error;
}

// Método que define la gravedad del error para el modo CONVOY+TELEOP
short mode_conv_teleop_error(Modulo_Gest_Errores::msg_errores msg){
    switch (msg.id_subsystem){
                case (SUBS_COMUNICACIONES):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_COMM_CONNECTION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_COMM_CONFIGURATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case (SUBS_GEST_SISTEMA):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_MODE_GPS):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_MODE_LASER):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_MODE_DRIVING):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_MODE_NAVIGATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_TELEOP):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_WARNING;
                        break;
                        case (ERROR_TELEOP_NEAR_OBSTACLE):
                            msg.type_error=TOE_WARNING;
                        break;
                        case (ERROR_TELEOP_FAR_OBSTABLE):
                            msg.type_error=TOE_WARNING;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_CONDUCCION):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_DRIVING_HANDBRAKE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_DRIVING_GEAR):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_DRIVING_ENGINE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_DRIVING_ROBOT):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_NAVEGACION):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_NAVIGATION_OBSTACLE_UNAVOIDABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_NAVIGATION_WAYPOINTS_GETTING_TIMEOUT):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_CAMARA):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_CAMERA_COMMUNICATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_CAMERA_CONFIGURATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_GPS):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_GPS_NOT_SATELLITE_SIGNAL):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_GPS_SOLUTION_NOT_GOOD):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_GPS_IMU_BAD_ALIGNMENT):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_GPS_MEMORY_OVERFLOW_TEACH):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_LASER_DELANTERO):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_COMMUNICATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_CONFIGURATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_LASER_TRAS_IZQ):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_COMMUNICATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_CONFIGURATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_LASER_TRAS_DER):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_COMMUNICATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_CONFIGURATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_LASER_3D):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_WARNING;
                        break;
                        case (ERROR_LASER_COMMUNICATION):
                            msg.type_error=TOE_WARNING;
                        break;
                        case (ERROR_LASER_CONFIGURATION):
                            msg.type_error=TOE_WARNING;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                default:
                    msg.type_error=TOE_UNDEFINED;
                    break;     // id_subsystem incorrecto
            }
    return msg.type_error;
}

// Método que define la gravedad del error para el modo CONVOY+AUTO
short mode_conv_auto_error(Modulo_Gest_Errores::msg_errores msg){
    switch (msg.id_subsystem){
                case (SUBS_COMUNICACIONES):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_COMM_CONNECTION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_COMM_CONFIGURATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case (SUBS_GEST_SISTEMA):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_MODE_GPS):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_MODE_LASER):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_MODE_DRIVING):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_MODE_NAVIGATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_TELEOP):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_WARNING;
                        break;
                        case (ERROR_TELEOP_NEAR_OBSTACLE):
                            msg.type_error=TOE_WARNING;
                        break;
                        case (ERROR_TELEOP_FAR_OBSTABLE):
                            msg.type_error=TOE_WARNING;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_CONDUCCION):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_DRIVING_HANDBRAKE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_DRIVING_GEAR):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_DRIVING_ENGINE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_DRIVING_ROBOT):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_NAVEGACION):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_NAVIGATION_OBSTACLE_UNAVOIDABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_NAVIGATION_WAYPOINTS_GETTING_TIMEOUT):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_CAMARA):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_WARNING;
                        break;
                        case (ERROR_CAMERA_COMMUNICATION):
                            msg.type_error=TOE_WARNING;
                        break;
                        case (ERROR_CAMERA_CONFIGURATION):
                            msg.type_error=TOE_WARNING;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_GPS):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_GPS_NOT_SATELLITE_SIGNAL):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_GPS_SOLUTION_NOT_GOOD):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_GPS_IMU_BAD_ALIGNMENT):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_GPS_MEMORY_OVERFLOW_TEACH):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_LASER_DELANTERO):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_COMMUNICATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_CONFIGURATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_LASER_TRAS_IZQ):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_COMMUNICATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_CONFIGURATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_LASER_TRAS_DER):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_COMMUNICATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        case (ERROR_LASER_CONFIGURATION):
                            msg.type_error=TOE_CRITICAL;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case(SUBS_LASER_3D):
                    switch (msg.id_error){
                        case (ERROR_MODULE_UNAVAILABLE):
                            msg.type_error=TOE_WARNING;
                        break;
                        case (ERROR_LASER_COMMUNICATION):
                            msg.type_error=TOE_WARNING;
                        break;
                        case (ERROR_LASER_CONFIGURATION):
                            msg.type_error=TOE_WARNING;
                        break;
                        default:
                            msg.type_error=TOE_UNDEFINED;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                default:
                    msg.type_error=TOE_UNDEFINED;
                    break;     // id_subsystem incorrecto
            }
    return msg.type_error;
}

// Cambia a modo neutro
void switchNeutral(){
    // Ordenes a módulo conducción para parar el vehiculo
    Modulo_Gest_Errores::msg_com_teleoperado msg_teleop;
    // Acelerador=0
    msg_teleop.id_element=ID_TELEOP_THROTTLE;
    msg_teleop.value=0;
    pub_com_teleop.publish(msg_teleop);
    // Freno máximo
    msg_teleop.id_element=ID_TELEOP_BRAKE;
    msg_teleop.value=100;   // Valor TBD
    pub_com_teleop.publish(msg_teleop);
    // Freno estacionamiento activo
    msg_teleop.id_element=ID_TELEOP_HANDBRAKE;
    msg_teleop.value=1;   // Valor TBD
    pub_com_teleop.publish(msg_teleop);
    // Giro = 0
    msg_teleop.id_element=ID_TELEOP_STEER;
    msg_teleop.value=0;   // Valor sin girar
    pub_com_teleop.publish(msg_teleop);
    // Marcha
    msg_teleop.id_element=ID_TELEOP_GEAR;
    msg_teleop.value=0;   // Valor Neutro
    pub_com_teleop.publish(msg_teleop);

    // Envia msg_mode con estado EXIT cuando llega confirmacion de DRIVING
    while (!confirm_flag);
    Modulo_Gest_Errores::msg_modo msg_ch_neutral;
    msg_ch_neutral.mode=modoActual;
    msg_ch_neutral.status=MODE_EXIT;
    pub_modo.publish(msg_ch_neutral);
}
