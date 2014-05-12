#include "../include/Modulo_Gest_Errores/gest_errores.h"

using namespace std;
ofstream flog;

int main(int argc, char **argv)
{
  // Obtencion del modo de operacion y comprobacion de que es correcto
  int operationMode;
  if ((operationMode = getOperationMode(argc, argv)) == 0) {
      return 1;
  }
    
  // Inicio de ROS
  ros::init(argc, argv, "gest_errores");

  // Manejador ROS
  ros::NodeHandle n;
  

  // Espera activa de inicio de modulo
  //int estado_actual=STATE_OK;       // Para poder probar
  int estado_actual=STATE_OFF;
  while(estado_actual!=STATE_CONF){
          n.getParam("state_module_error_management",estado_actual);
  }
  cout << "Atica GEST. ERRORES :: Iniciando configuración..." << endl;

  initialize(n);
  // Todo esta correcto, lo especificamos con el correspondiente parametro
  n.setParam("state_module_error_management",STATE_OK);
  cout << "Atica GEST. ERRORES :: Configurado y funcionando" << endl;

  while (ros::ok() && !exitModule)
  {
       switch (operationMode) {
        case OPERATION_MODE_DEBUG:
            // Funcionamiento del modo debug
            n.getParam("state_module_error_management",estado_actual);
            if(estado_actual==STATE_ERROR || estado_actual== STATE_OFF){
                exitModule=true;
            } 
            ros::spinOnce();
            break;
        case OPERATION_MODE_RELEASE:
            // Funcionamiento del modo release
            n.getParam("state_module_error_management",estado_actual);
            if(estado_actual==STATE_ERROR || estado_actual== STATE_OFF){
                exitModule=true;
            } 
            ros::spinOnce();
            break;
        case OPERATION_MODE_SIMULATION:
            // Funcionamiento del modo simulacion
            n.getParam("state_module_error_management",estado_actual);
            if(estado_actual==STATE_ERROR || estado_actual== STATE_OFF){
                exitModule=true;
            } 
            ros::spinOnce();
            break;
        default:
            break;
    }
      
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
void fcn_sub_mode(const Common_files::msg_mode msg)
{
    ROS_INFO("Cambio de modo");
    if ((msg.status == MODE_START) && (msg.type_msg == INFO))
        currentMode= msg.mode;

    else if ((msg.status == MODE_EXIT) && (msg.type_msg == INFO)){
        if ((currentMode == MODE_CONVOY_TELEOP) || (currentMode == MODE_CONVOY_AUTO))
            currentMode=MODE_CONVOY;
        else
            currentMode=MODE_NEUTRAL;
    }
        
}

// Suscriptor de errores
void fcn_sub_error(const Common_files::msg_error msg)
{
    short type_error = 0;
    bool av_mode_aux[13];
    if(msg.type_error==TOE_UNDEFINED){                              // Evita leer mensajes que el mismo haya enviado
        cout << "Me ha llegado el mensaje de error: " << msg;
        type_error = isWarningOrCritical(msg,currentMode);           // Comprueba gravedad del error (CRIT o WARN)
        Common_files::msg_error msg_err;
        switch(type_error){
            case TOE_CRITICAL:
                    // Generacion de mensaje de error
                    msg_err.id_subsystem=msg.id_subsystem;
                    msg_err.id_error=convertOutputError(msg);
                    msg_err.type_error=TOE_CRITICAL;
                    pub_error.publish(msg_err);
                    writeToLog(msg_err);
                    switchNeutral();            // Al ser un error critico se pasa a modo neutro
                    break;
            case TOE_WARNING:
                    // Generacion de mensaje de error
                    msg_err.id_subsystem=msg.id_subsystem;
                    msg_err.id_error=convertOutputError(msg);
                    msg_err.type_error=TOE_WARNING;
                    pub_error.publish(msg_err);
                    writeToLog(msg_err);
                    break;
            default:
                    cout << "Atica Gest. Errores :: Se ha recibido un error no clasificado" << endl;
        }
        // Actualización del registro de modos disponibles
        for(int i=MODE_REMOTE;i<=MODE_TEACH;i++){
            av_mode_aux[i]=avail_mode.available[i];
            //cout << av_mode_aux[i] << endl;
        }
        if (updateModeAvailable(msg,av_mode_aux)==false)
            pub_avail_mode.publish(avail_mode);
    }
    // Ordenes para actualizar msg de modos disponibles al recibir un mensaje fin de error
    else if (msg.type_error==TOE_END_ERROR){
        ROS_INFO("Fin de error recibido");
        for(int i=MODE_REMOTE;i<=MODE_TEACH;i++)
            av_mode_aux[i]=avail_mode.available[i];
        if (updateEndError(msg,av_mode_aux)==false)
            pub_avail_mode.publish(avail_mode);
    }
}

// Servicio de heartbeat para Gestion de sistema
bool fcn_heartbeat(Common_files::srv_data::Request &req, Common_files::srv_data::Response &resp)
{
    if (req.param == PARAM_ALIVE) {
        resp.value = 0;
        return true;
    } else
        return false;
}

/*******************************************************************************
 *******************************************************************************
 *                              FUNCIONES PROPIAS
 * *****************************************************************************
 * ****************************************************************************/
// Inicialización de variables
void initialize(ros::NodeHandle n) {
    // Creacion de suscriptores
    sub_error = n.subscribe("error", 1000, fcn_sub_error);
    sub_mode = n.subscribe("modeSE", 1000, fcn_sub_mode);
    server = n.advertiseService("module_alive_15",fcn_heartbeat);
    // Inicializacion de publicadores
    pub_mode = n.advertise<Common_files::msg_mode>("modeES", 1000);
    pub_error = n.advertise<Common_files::msg_error>("errorToUCR", 1000);
    pub_avail_mode = n.advertise<Common_files::msg_available>("avail", 1000);
    
    // Inicialización de variables globales
    currentMode = MODE_NEUTRAL;
    exitModule = false;
    flog.open("ficheroLog.txt");

    // Inicialización de mensaje available_mode todo a true
    int i,j,k;
    for (i = AVAILABLE_POS_REMOTE; i <= AVAILABLE_POS_TEACH; i++)
        avail_mode.available[i] = true;
    for (i=0;i<MAX_MODES;i++){
        for(j=0;j<MAX_MODULES;j++){
            for(k=0;k<MAX_ERRORS;k++){
                numErrorMode[i][j][k]=0;
            }
        }
    }
}

// Especifica que tipo de error se ha recibido y rellena el correspondiente
// campo del mensaje. Luego lo envia relleno.
short isWarningOrCritical(Common_files::msg_error msg, short mode){
    short error=0;
    switch (mode){
        case MODE_REMOTE:             // Modo REMOTE
            error = mode_remote_error(msg);
            break;
        case MODE_START_ENGINE:       // Modo START ENGINE
            error = mode_startengine_error (msg);
            break;
        case MODE_STOP_ENGINE:         // Modo STOP ENGINE
            error = mode_stopengine_error (msg);
            break;            
        case MODE_ENGAGE_BRAKE:       // Modo ENGAGE BRAKE
            error = mode_engagebrake_error(msg);
            break;            
        case MODE_PLAN:               // Modo PLAN
            error = mode_plan_error(msg);
            break;            
        case MODE_COME_TO_ME:         // Modo COME TO ME
            error = mode_cometome_error(msg);
            break;
        case MODE_FOLLOW_ME:          // Modo FOLLOW ME
            error = mode_followme_error(msg);
            break;
        case MODE_MAPPING:            // Modo MAPPING
            error = mode_mapping_error(msg);
            break;
        case MODE_TEACH:              // Modo TEACH
            error = mode_teach_error(msg);
            break;
        case MODE_CONVOY:             // Modo CONVOY
            error = mode_convoy_error(msg);
            break;
        case MODE_CONVOY_TELEOP:      // Modo CONVOY+TELEOP
            error = mode_conv_teleop_error(msg);
            break;
        case MODE_CONVOY_AUTO:         // Modo CONVOY+AUTO
            error = mode_conv_auto_error(msg);
            break;
        case MODE_NEUTRAL:            // Modo Neutro > Todo Warning
            error = TOE_WARNING;
            break;
        case MODE_MANUAL:            // Modo Manual > Todo Warning
            error = TOE_WARNING;
            break;
        default:                        // Modo incorrecto
            error = TOE_UNAVAILABLE;
            break;
    }
    return error;
}

// Método que define la gravedad del error para el modo REMOTE
short mode_remote_error(Common_files::msg_error msg)
{
     short type_error=0;
     switch (msg.id_subsystem){
                case SUBS_COMMUNICATION:
                    switch (msg.id_error){
                        case (JAUS_CONFIG_ERROR):
                            type_error=TOE_CRITICAL;
                            break;
                        case (CREATE_COMPONENT_ERROR):
                            type_error=TOE_CRITICAL;
                            break;
                        case (RUN_COMPONENT_ERROR):
                            type_error=TOE_CRITICAL;
                            break;
                        case (COMM_LOG_FILE_ERROR):
                            type_error=TOE_WARNING;
                            break;
                        case (COMM_CONFIG_FILE_ERROR):
                            type_error=TOE_CRITICAL;
                            break;
                        case (COMM_CONFIG_FILE_STRUCTURE_ERROR):
                            type_error=TOE_CRITICAL;
                            break;
                        case COMMUNICATION_UCR_FAIL:
                            type_error=TOE_WARNING;
                            break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_SYSTEM_MGMNT:
                    switch (msg.id_error){
                        case (MODE_NOT_AVAILABLE):
                            type_error=TOE_WARNING;
                        break;
                        case FUNCTION_NOT_AVAILABLE:
                            type_error=TOE_WARNING;
                            break;
                        case (COMM_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (REMOTE_MODULE_NA):
                            type_error=TOE_CRITICAL;
                        break;
                        case (DRIVING_MODULE_NA):
                            type_error=TOE_CRITICAL;
                        break;
                        case (NAVIGATION_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (CAMERA_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (GPS_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (FRONT_LASER_1_MODULE_NA):
                            type_error=TOE_CRITICAL;
                        break;
                        case (REAR_LASER_MODULE_NA):
                            type_error=TOE_CRITICAL;
                        break;
                        case (LASER3D_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (BEACON_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (RDF_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (HL_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (CONVOY_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_REMOTE:
                    switch (msg.id_error){
                        case REMOTE_PARAMETER_OUTRANGE:
                            type_error=TOE_WARNING;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_DRIVING:
                    switch (msg.id_error){
                        case CONNECTION_CAN_FAIL:
                            type_error=TOE_CRITICAL;
                        break;
                        case COMMUNICATION_CAN_FAIL:
                            type_error=TOE_CRITICAL;
                        break;
                        case START_STOP_FAILURE:
                            type_error=TOE_CRITICAL;
                        break;
                        case THROTTLE_FAILURE:
                            type_error=TOE_CRITICAL;
                        break;
                        case HANDBRAKE_FAILURE:
                            type_error=TOE_CRITICAL;
                        break;
                        case BRAKE_FAILURE:
                            type_error=TOE_CRITICAL;
                        break;
                        case GEAR_SHIFT_FAILURE:
                            type_error=TOE_CRITICAL;
                        break;
                        case STEER_FAILURE:
                            type_error=TOE_CRITICAL;
                        break;
                        case DIFFERENTIAL_LOCK_FAILURE:
                            type_error=TOE_CRITICAL;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_NAVIGATION:
                    switch (msg.id_error){
                        case (ERROR_NAVIGATION_OBSTACLE_UNAVOIDABLE):
                            type_error=TOE_WARNING;
                        break;
                        case (ERROR_NAVIGATION_WAYPOINTS_GETTING_TIMEOUT):
                            type_error=TOE_WARNING;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_CAMERA:
                    switch (msg.id_error){
                        break;
                        case (ERROR_CAMERA_COMMUNICATION):
                            type_error=TOE_WARNING;
                        break;
                        case (ERROR_CAMERA_CONFIGURATION):
                            type_error=TOE_WARNING;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_GPS:
                    switch (msg.id_error){
                        case GPS_GLOBAL_ERROR:
                            type_error=TOE_END_ERROR;
                        break;
                        case DATA_RCV_FAILED:
                            type_error=TOE_WARNING;
                        break;
                        case CONFIG_OPEN_SERIALPORT_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case CONFIG_CONFIG_SERIALPORT_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case CONFIG_ALIGNMENT_MODE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case CONFIG_INITIAL_AZIMUT_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case CONFIG_ANTOFFSET_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case INS_INACTIVE:
                            type_error=TOE_WARNING;
                        break;
                        case INS_ALIGNING:
                            type_error=TOE_WARNING;
                        break;
                        case INS_SOLUTION_NOT_GOOD:
                            type_error=TOE_WARNING;
                        break;
                        case INS_BAD_GPS_AGREEMENT:
                            type_error=TOE_WARNING;
                        break;
                        case INS_ALIGNMENT_COMPLETE:
                            type_error=TOE_WARNING;
                        break;
                        case INSUFFICIENT_OBS:
                            type_error=TOE_WARNING;
                        break;
                        case NO_CONVERGENCE:
                            type_error=TOE_WARNING;
                        break;
                        case SINGULARITY:
                            type_error=TOE_WARNING;
                        break;
                        case COV_TRACE:
                            type_error=TOE_WARNING;
                        break;
                        case TEST_DIST:
                            type_error=TOE_WARNING;
                        break;
                        case COLD_START:
                            type_error=TOE_WARNING;
                        break;
                        case V_H_LIMIT:
                            type_error=TOE_WARNING;
                        break;
                        case VARIANCE:
                            type_error=TOE_WARNING;
                        break;
                        case RESIDUALS:
                            type_error=TOE_WARNING;
                        break;
                        case DELTA_POS:
                            type_error=TOE_WARNING;
                        break;
                        case NEGATIVE_VAR:
                            type_error=TOE_WARNING;
                        break;
                        case INTEGRITY_WARNING:
                            type_error=TOE_WARNING;
                        break;
                        case IMU_UNPLUGGED:
                            type_error=TOE_WARNING;
                        break;
                        case PENDING:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_FIX:
                            type_error=TOE_WARNING;
                        break;
                        case UNAUTHORIZED_STATE:
                            type_error=TOE_WARNING;
                        break;

                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_FRONT_LASER_1:
                    switch (msg.id_error){
                        case LASER_SOCKET_FAIL:
                            type_error=TOE_CRITICAL;
                        break;
                        case CONNECTION_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case COMM_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case INVALID_SAMPLE_FREQ:
                            type_error=TOE_CRITICAL;
                        break;
                        case INVALID_ANGLE_RESOLUTION:
                            type_error=TOE_CRITICAL;
                        break;
                        case INVALID_ANGLE_SAMPLE_RESOLUTION:
                            type_error=TOE_CRITICAL;
                        break;
                        case INVALID_SCAN_AREA:
                            type_error=TOE_CRITICAL;
                        break;
                        case UNKNOWN_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case START_MEASURE_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case STOP_MEASURE_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case SELECT_USER_LEVEL_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case SET_LMS_OUTPUT_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case SAVE_DATA_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case START_DEVICE_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case INCORRECT_ANSWER:
                            type_error=TOE_CRITICAL;
                        break;
                        case FRAME_OVERFLOW:
                            type_error=TOE_CRITICAL;
                        break;
                        case LASER_LOG_FILE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_CONFIG_FILE_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case LASER_CONFIG_FILE_STRUCTURE_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_FRONT_LASER_2:
                    switch (msg.id_error){
                        case LASER_SOCKET_FAIL:
                            type_error=TOE_CRITICAL;
                        break;
                        case CONNECTION_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case COMM_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case INVALID_SAMPLE_FREQ:
                            type_error=TOE_CRITICAL;
                        break;
                        case INVALID_ANGLE_RESOLUTION:
                            type_error=TOE_CRITICAL;
                        break;
                        case INVALID_ANGLE_SAMPLE_RESOLUTION:
                            type_error=TOE_CRITICAL;
                        break;
                        case INVALID_SCAN_AREA:
                            type_error=TOE_CRITICAL;
                        break;
                        case UNKNOWN_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case START_MEASURE_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case STOP_MEASURE_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case SELECT_USER_LEVEL_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case SET_LMS_OUTPUT_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case SAVE_DATA_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case START_DEVICE_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case INCORRECT_ANSWER:
                            type_error=TOE_CRITICAL;
                        break;
                        case FRAME_OVERFLOW:
                            type_error=TOE_CRITICAL;
                        break;
                        case LASER_LOG_FILE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_CONFIG_FILE_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case LASER_CONFIG_FILE_STRUCTURE_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_REAR_LASER:
                    switch (msg.id_error){
                        case LASER_SOCKET_FAIL:
                            type_error=TOE_CRITICAL;
                        break;
                        case CONNECTION_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case COMM_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case INVALID_SAMPLE_FREQ:
                            type_error=TOE_CRITICAL;
                        break;
                        case INVALID_ANGLE_RESOLUTION:
                            type_error=TOE_CRITICAL;
                        break;
                        case INVALID_ANGLE_SAMPLE_RESOLUTION:
                            type_error=TOE_CRITICAL;
                        break;
                        case INVALID_SCAN_AREA:
                            type_error=TOE_CRITICAL;
                        break;
                        case UNKNOWN_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case START_MEASURE_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case STOP_MEASURE_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case SELECT_USER_LEVEL_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case SET_LMS_OUTPUT_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case SAVE_DATA_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case START_DEVICE_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case INCORRECT_ANSWER:
                            type_error=TOE_CRITICAL;
                        break;
                        case FRAME_OVERFLOW:
                            type_error=TOE_CRITICAL;
                        break;
                        case LASER_LOG_FILE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_CONFIG_FILE_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case LASER_CONFIG_FILE_STRUCTURE_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_LASER_3D:
                    switch (msg.id_error){
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_BEACON:
                    switch(msg.id_error){
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;
                    }
                    break;
                case SUBS_RANGE_DATA_FUSION:
                    switch(msg.id_error){
                        case RDF_INSUFFICIENT_DATA:
                            type_error=TOE_WARNING;
                            break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;
                    }
                    break;
                case SUBS_HUMAN_LOCALIZATION:
                    switch(msg.id_error){
                        case HL_INSUFFICIENT_DATA:
                            type_error=TOE_WARNING;
                            break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;
                    }
                    break;
                case SUBS_CONVOY:
                    switch (msg.id_error){
                        case UDP_SOCKET_FAIL:
                            type_error=TOE_WARNING;
                            break;
                        case BIND_SOCKET_FAIL:
                            type_error=TOE_WARNING;
                            break;
                        case SOCKET_RECEIVE_ERROR:
                            type_error=TOE_WARNING;
                            break;
                        case LEADER_CRITICAL_ERROR:
                            type_error=TOE_WARNING;
                            break;
                        case FOLLOWER_GPS_ERROR:
                            type_error=TOE_WARNING;
                            break;
                        case FOLLOWER_LASER_ERROR:
                            type_error=TOE_WARNING;
                            break;
                        case FOLLOWER_NAVIGATION_ERROR:
                            type_error=TOE_WARNING;
                            break;
                        case FOLLOWER_CAMERA_ERROR:
                            type_error=TOE_WARNING;
                            break;
                        default:
                            type_error=TOE_UNAVAILABLE;         // Ha llegado un id_error no contemplado
                            break;
                        }
                    break;
                default:
                    type_error=TOE_UNAVAILABLE;
                    break;     // id_subsystem incorrecto      
            }
     return type_error;
}

short mode_startengine_error(Common_files::msg_error msg)
{
     short type_error=0;    
     switch (msg.id_subsystem){
                case SUBS_COMMUNICATION:
                    switch (msg.id_error){
                        case (JAUS_CONFIG_ERROR):
                            type_error=TOE_CRITICAL;
                            break;
                        case (CREATE_COMPONENT_ERROR):
                            type_error=TOE_CRITICAL;
                            break;
                        case (RUN_COMPONENT_ERROR):
                            type_error=TOE_CRITICAL;
                            break;
                        case (COMM_LOG_FILE_ERROR):
                            type_error=TOE_WARNING;
                            break;
                        case (COMM_CONFIG_FILE_ERROR):
                            type_error=TOE_CRITICAL;
                            break;
                        case (COMM_CONFIG_FILE_STRUCTURE_ERROR):
                            type_error=TOE_CRITICAL;
                            break;
                        case COMMUNICATION_UCR_FAIL:
                            type_error=TOE_WARNING;
                            break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_SYSTEM_MGMNT:
                    switch (msg.id_error){
                        case (MODE_NOT_AVAILABLE):
                            type_error=TOE_WARNING;
                        break;
                        case FUNCTION_NOT_AVAILABLE:
                            type_error=TOE_WARNING;
                            break;
                        case (COMM_MODULE_NA):
                            type_error=TOE_CRITICAL;
                        break;
                        case (REMOTE_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (DRIVING_MODULE_NA):
                            type_error=TOE_CRITICAL;
                        break;
                        case (NAVIGATION_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (CAMERA_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (GPS_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (FRONT_LASER_1_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (REAR_LASER_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (LASER3D_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (BEACON_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (RDF_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (HL_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (CONVOY_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_REMOTE:
                    switch (msg.id_error){
                        case REMOTE_PARAMETER_OUTRANGE:
                            type_error=TOE_WARNING;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_DRIVING:
                    switch (msg.id_error){
                        case CONNECTION_CAN_FAIL:
                            type_error=TOE_CRITICAL;
                        break;
                        case COMMUNICATION_CAN_FAIL:
                            type_error=TOE_CRITICAL;
                        break;
                        case START_STOP_FAILURE:
                            type_error=TOE_CRITICAL;
                        break;
                        case THROTTLE_FAILURE:
                            type_error=TOE_WARNING;
                        break;
                        case HANDBRAKE_FAILURE:
                            type_error=TOE_WARNING;
                        break;
                        case BRAKE_FAILURE:
                            type_error=TOE_WARNING;
                        break;
                        case GEAR_SHIFT_FAILURE:
                            type_error=TOE_WARNING;
                        break;
                        case STEER_FAILURE:
                            type_error=TOE_WARNING;
                        break;
                        case DIFFERENTIAL_LOCK_FAILURE:
                            type_error=TOE_WARNING;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_NAVIGATION:
                    switch (msg.id_error){
                        case (ERROR_NAVIGATION_OBSTACLE_UNAVOIDABLE):
                            type_error=TOE_WARNING;
                        break;
                        case (ERROR_NAVIGATION_WAYPOINTS_GETTING_TIMEOUT):
                            type_error=TOE_WARNING;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_CAMERA:
                    switch (msg.id_error){
                        break;
                        case (ERROR_CAMERA_COMMUNICATION):
                            type_error=TOE_WARNING;
                        break;
                        case (ERROR_CAMERA_CONFIGURATION):
                            type_error=TOE_WARNING;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_GPS:
                    switch (msg.id_error){
                        case GPS_GLOBAL_ERROR:
                            type_error=TOE_END_ERROR;
                        break;
                        case DATA_RCV_FAILED:
                            type_error=TOE_WARNING;
                        break;
                        case CONFIG_OPEN_SERIALPORT_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case CONFIG_CONFIG_SERIALPORT_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case CONFIG_ALIGNMENT_MODE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case CONFIG_INITIAL_AZIMUT_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case CONFIG_ANTOFFSET_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case INS_INACTIVE:
                            type_error=TOE_WARNING;
                        break;
                        case INS_ALIGNING:
                            type_error=TOE_WARNING;
                        break;
                        case INS_SOLUTION_NOT_GOOD:
                            type_error=TOE_WARNING;
                        break;
                        case INS_BAD_GPS_AGREEMENT:
                            type_error=TOE_WARNING;
                        break;
                        case INS_ALIGNMENT_COMPLETE:
                            type_error=TOE_WARNING;
                        break;
                        case INSUFFICIENT_OBS:
                            type_error=TOE_WARNING;
                        break;
                        case NO_CONVERGENCE:
                            type_error=TOE_WARNING;
                        break;
                        case SINGULARITY:
                            type_error=TOE_WARNING;
                        break;
                        case COV_TRACE:
                            type_error=TOE_WARNING;
                        break;
                        case TEST_DIST:
                            type_error=TOE_WARNING;
                        break;
                        case COLD_START:
                            type_error=TOE_WARNING;
                        break;
                        case V_H_LIMIT:
                            type_error=TOE_WARNING;
                        break;
                        case VARIANCE:
                            type_error=TOE_WARNING;
                        break;
                        case RESIDUALS:
                            type_error=TOE_WARNING;
                        break;
                        case DELTA_POS:
                            type_error=TOE_WARNING;
                        break;
                        case NEGATIVE_VAR:
                            type_error=TOE_WARNING;
                        break;
                        case INTEGRITY_WARNING:
                            type_error=TOE_WARNING;
                        break;
                        case IMU_UNPLUGGED:
                            type_error=TOE_WARNING;
                        break;
                        case PENDING:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_FIX:
                            type_error=TOE_WARNING;
                        break;
                        case UNAUTHORIZED_STATE:
                            type_error=TOE_WARNING;
                        break;

                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_FRONT_LASER_1:
                    switch (msg.id_error){
                        case LASER_SOCKET_FAIL:
                            type_error=TOE_WARNING;
                        break;
                        case CONNECTION_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case COMM_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_SAMPLE_FREQ:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_ANGLE_RESOLUTION:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_ANGLE_SAMPLE_RESOLUTION:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_SCAN_AREA:
                            type_error=TOE_WARNING;
                        break;
                        case UNKNOWN_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case START_MEASURE_NA:
                            type_error=TOE_WARNING;
                        break;
                        case STOP_MEASURE_NA:
                            type_error=TOE_WARNING;
                        break;
                        case SELECT_USER_LEVEL_NA:
                            type_error=TOE_WARNING;
                        break;
                        case SET_LMS_OUTPUT_NA:
                            type_error=TOE_WARNING;
                        break;
                        case SAVE_DATA_NA:
                            type_error=TOE_WARNING;
                        break;
                        case START_DEVICE_NA:
                            type_error=TOE_WARNING;
                        break;
                        case INCORRECT_ANSWER:
                            type_error=TOE_WARNING;
                        break;
                        case FRAME_OVERFLOW:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_LOG_FILE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_CONFIG_FILE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_CONFIG_FILE_STRUCTURE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_FRONT_LASER_2:
                    switch (msg.id_error){
                        case LASER_SOCKET_FAIL:
                            type_error=TOE_WARNING;
                        break;
                        case CONNECTION_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case COMM_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_SAMPLE_FREQ:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_ANGLE_RESOLUTION:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_ANGLE_SAMPLE_RESOLUTION:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_SCAN_AREA:
                            type_error=TOE_WARNING;
                        break;
                        case UNKNOWN_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case START_MEASURE_NA:
                            type_error=TOE_WARNING;
                        break;
                        case STOP_MEASURE_NA:
                            type_error=TOE_WARNING;
                        break;
                        case SELECT_USER_LEVEL_NA:
                            type_error=TOE_WARNING;
                        break;
                        case SET_LMS_OUTPUT_NA:
                            type_error=TOE_WARNING;
                        break;
                        case SAVE_DATA_NA:
                            type_error=TOE_WARNING;
                        break;
                        case START_DEVICE_NA:
                            type_error=TOE_WARNING;
                        break;
                        case INCORRECT_ANSWER:
                            type_error=TOE_WARNING;
                        break;
                        case FRAME_OVERFLOW:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_LOG_FILE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_CONFIG_FILE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_CONFIG_FILE_STRUCTURE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_REAR_LASER:
                    switch (msg.id_error){
                        case LASER_SOCKET_FAIL:
                            type_error=TOE_WARNING;
                        break;
                        case CONNECTION_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case COMM_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_SAMPLE_FREQ:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_ANGLE_RESOLUTION:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_ANGLE_SAMPLE_RESOLUTION:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_SCAN_AREA:
                            type_error=TOE_WARNING;
                        break;
                        case UNKNOWN_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case START_MEASURE_NA:
                            type_error=TOE_WARNING;
                        break;
                        case STOP_MEASURE_NA:
                            type_error=TOE_WARNING;
                        break;
                        case SELECT_USER_LEVEL_NA:
                            type_error=TOE_WARNING;
                        break;
                        case SET_LMS_OUTPUT_NA:
                            type_error=TOE_WARNING;
                        break;
                        case SAVE_DATA_NA:
                            type_error=TOE_WARNING;
                        break;
                        case START_DEVICE_NA:
                            type_error=TOE_WARNING;
                        break;
                        case INCORRECT_ANSWER:
                            type_error=TOE_WARNING;
                        break;
                        case FRAME_OVERFLOW:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_LOG_FILE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_CONFIG_FILE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_CONFIG_FILE_STRUCTURE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_LASER_3D:
                    switch (msg.id_error){
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_BEACON:
                    switch(msg.id_error){
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;
                    }
                    break;
                case SUBS_RANGE_DATA_FUSION:
                    switch(msg.id_error){
                        case RDF_INSUFFICIENT_DATA:
                            type_error=TOE_UNAVAILABLE;
                            break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;
                    }
                    break;
                case SUBS_HUMAN_LOCALIZATION:
                    switch(msg.id_error){
                        case HL_INSUFFICIENT_DATA:
                            type_error=TOE_WARNING;
                            break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;
                    }
                    break;
                case SUBS_CONVOY:
                    switch (msg.id_error){
                        case UDP_SOCKET_FAIL:
                            type_error=TOE_WARNING;
                            break;
                        case BIND_SOCKET_FAIL:
                            type_error=TOE_WARNING;
                            break;
                        case SOCKET_RECEIVE_ERROR:
                            type_error=TOE_WARNING;
                            break;
                        case LEADER_CRITICAL_ERROR:
                            type_error=TOE_WARNING;
                            break;
                        case FOLLOWER_GPS_ERROR:
                            type_error=TOE_WARNING;
                            break;
                        case FOLLOWER_LASER_ERROR:
                            type_error=TOE_WARNING;
                            break;
                        case FOLLOWER_NAVIGATION_ERROR:
                            type_error=TOE_WARNING;
                            break;
                        case FOLLOWER_CAMERA_ERROR:
                            type_error=TOE_WARNING;
                            break;
                        default:
                            type_error=TOE_UNAVAILABLE;         // Ha llegado un id_error no contemplado
                            break;
                        }
                    break;
                default:
                    type_error=TOE_UNAVAILABLE;
                    break;     // id_subsystem incorrecto
            }
     return type_error;
}

short mode_stopengine_error(Common_files::msg_error msg)
{
     short type_error=0;    
     switch (msg.id_subsystem){
                case SUBS_COMMUNICATION:
                    switch (msg.id_error){
                        case (JAUS_CONFIG_ERROR):
                            type_error=TOE_CRITICAL;
                            break;
                        case (CREATE_COMPONENT_ERROR):
                            type_error=TOE_CRITICAL;
                            break;
                        case (RUN_COMPONENT_ERROR):
                            type_error=TOE_CRITICAL;
                            break;
                        case (COMM_LOG_FILE_ERROR):
                            type_error=TOE_WARNING;
                            break;
                        case (COMM_CONFIG_FILE_ERROR):
                            type_error=TOE_CRITICAL;
                            break;
                        case (COMM_CONFIG_FILE_STRUCTURE_ERROR):
                            type_error=TOE_CRITICAL;
                            break;
                        case COMMUNICATION_UCR_FAIL:
                            type_error=TOE_WARNING;
                            break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_SYSTEM_MGMNT:
                    switch (msg.id_error){
                        case (MODE_NOT_AVAILABLE):
                            type_error=TOE_WARNING;
                        break;
                        case FUNCTION_NOT_AVAILABLE:
                            type_error=TOE_WARNING;
                            break;
                        case (COMM_MODULE_NA):
                            type_error=TOE_CRITICAL;
                        break;
                        case (REMOTE_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (DRIVING_MODULE_NA):
                            type_error=TOE_CRITICAL;
                        break;
                        case (NAVIGATION_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (CAMERA_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (GPS_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (FRONT_LASER_1_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (REAR_LASER_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (LASER3D_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (BEACON_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (RDF_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (HL_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (CONVOY_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_REMOTE:
                    switch (msg.id_error){
                        case REMOTE_PARAMETER_OUTRANGE:
                            type_error=TOE_WARNING;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_DRIVING:
                    switch (msg.id_error){
                        case CONNECTION_CAN_FAIL:
                            type_error=TOE_CRITICAL;
                        break;
                        case COMMUNICATION_CAN_FAIL:
                            type_error=TOE_CRITICAL;
                        break;
                        case START_STOP_FAILURE:
                            type_error=TOE_CRITICAL;
                        break;
                        case THROTTLE_FAILURE:
                            type_error=TOE_WARNING;
                        break;
                        case HANDBRAKE_FAILURE:
                            type_error=TOE_WARNING;
                        break;
                        case BRAKE_FAILURE:
                            type_error=TOE_WARNING;
                        break;
                        case GEAR_SHIFT_FAILURE:
                            type_error=TOE_WARNING;
                        break;
                        case STEER_FAILURE:
                            type_error=TOE_WARNING;
                        break;
                        case DIFFERENTIAL_LOCK_FAILURE:
                            type_error=TOE_WARNING;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_NAVIGATION:
                    switch (msg.id_error){
                        case (ERROR_NAVIGATION_OBSTACLE_UNAVOIDABLE):
                            type_error=TOE_WARNING;
                        break;
                        case (ERROR_NAVIGATION_WAYPOINTS_GETTING_TIMEOUT):
                            type_error=TOE_WARNING;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_CAMERA:
                    switch (msg.id_error){
                        break;
                        case (ERROR_CAMERA_COMMUNICATION):
                            type_error=TOE_WARNING;
                        break;
                        case (ERROR_CAMERA_CONFIGURATION):
                            type_error=TOE_WARNING;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_GPS:
                    switch (msg.id_error){
                        case GPS_GLOBAL_ERROR:
                            type_error=TOE_END_ERROR;
                        break;
                        case DATA_RCV_FAILED:
                            type_error=TOE_WARNING;
                        break;
                        case CONFIG_OPEN_SERIALPORT_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case CONFIG_CONFIG_SERIALPORT_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case CONFIG_ALIGNMENT_MODE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case CONFIG_INITIAL_AZIMUT_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case CONFIG_ANTOFFSET_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case INS_INACTIVE:
                            type_error=TOE_WARNING;
                        break;
                        case INS_ALIGNING:
                            type_error=TOE_WARNING;
                        break;
                        case INS_SOLUTION_NOT_GOOD:
                            type_error=TOE_WARNING;
                        break;
                        case INS_BAD_GPS_AGREEMENT:
                            type_error=TOE_WARNING;
                        break;
                        case INS_ALIGNMENT_COMPLETE:
                            type_error=TOE_WARNING;
                        break;
                        case INSUFFICIENT_OBS:
                            type_error=TOE_WARNING;
                        break;
                        case NO_CONVERGENCE:
                            type_error=TOE_WARNING;
                        break;
                        case SINGULARITY:
                            type_error=TOE_WARNING;
                        break;
                        case COV_TRACE:
                            type_error=TOE_WARNING;
                        break;
                        case TEST_DIST:
                            type_error=TOE_WARNING;
                        break;
                        case COLD_START:
                            type_error=TOE_WARNING;
                        break;
                        case V_H_LIMIT:
                            type_error=TOE_WARNING;
                        break;
                        case VARIANCE:
                            type_error=TOE_WARNING;
                        break;
                        case RESIDUALS:
                            type_error=TOE_WARNING;
                        break;
                        case DELTA_POS:
                            type_error=TOE_WARNING;
                        break;
                        case NEGATIVE_VAR:
                            type_error=TOE_WARNING;
                        break;
                        case INTEGRITY_WARNING:
                            type_error=TOE_WARNING;
                        break;
                        case IMU_UNPLUGGED:
                            type_error=TOE_WARNING;
                        break;
                        case PENDING:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_FIX:
                            type_error=TOE_WARNING;
                        break;
                        case UNAUTHORIZED_STATE:
                            type_error=TOE_WARNING;
                        break;

                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_FRONT_LASER_1:
                    switch (msg.id_error){
                        case LASER_SOCKET_FAIL:
                            type_error=TOE_WARNING;
                        break;
                        case CONNECTION_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case COMM_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_SAMPLE_FREQ:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_ANGLE_RESOLUTION:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_ANGLE_SAMPLE_RESOLUTION:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_SCAN_AREA:
                            type_error=TOE_WARNING;
                        break;
                        case UNKNOWN_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case START_MEASURE_NA:
                            type_error=TOE_WARNING;
                        break;
                        case STOP_MEASURE_NA:
                            type_error=TOE_WARNING;
                        break;
                        case SELECT_USER_LEVEL_NA:
                            type_error=TOE_WARNING;
                        break;
                        case SET_LMS_OUTPUT_NA:
                            type_error=TOE_WARNING;
                        break;
                        case SAVE_DATA_NA:
                            type_error=TOE_WARNING;
                        break;
                        case START_DEVICE_NA:
                            type_error=TOE_WARNING;
                        break;
                        case INCORRECT_ANSWER:
                            type_error=TOE_WARNING;
                        break;
                        case FRAME_OVERFLOW:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_LOG_FILE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_CONFIG_FILE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_CONFIG_FILE_STRUCTURE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_FRONT_LASER_2:
                    switch (msg.id_error){
                        case LASER_SOCKET_FAIL:
                            type_error=TOE_WARNING;
                        break;
                        case CONNECTION_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case COMM_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_SAMPLE_FREQ:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_ANGLE_RESOLUTION:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_ANGLE_SAMPLE_RESOLUTION:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_SCAN_AREA:
                            type_error=TOE_WARNING;
                        break;
                        case UNKNOWN_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case START_MEASURE_NA:
                            type_error=TOE_WARNING;
                        break;
                        case STOP_MEASURE_NA:
                            type_error=TOE_WARNING;
                        break;
                        case SELECT_USER_LEVEL_NA:
                            type_error=TOE_WARNING;
                        break;
                        case SET_LMS_OUTPUT_NA:
                            type_error=TOE_WARNING;
                        break;
                        case SAVE_DATA_NA:
                            type_error=TOE_WARNING;
                        break;
                        case START_DEVICE_NA:
                            type_error=TOE_WARNING;
                        break;
                        case INCORRECT_ANSWER:
                            type_error=TOE_WARNING;
                        break;
                        case FRAME_OVERFLOW:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_LOG_FILE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_CONFIG_FILE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_CONFIG_FILE_STRUCTURE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_REAR_LASER:
                    switch (msg.id_error){
                        case LASER_SOCKET_FAIL:
                            type_error=TOE_WARNING;
                        break;
                        case CONNECTION_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case COMM_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_SAMPLE_FREQ:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_ANGLE_RESOLUTION:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_ANGLE_SAMPLE_RESOLUTION:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_SCAN_AREA:
                            type_error=TOE_WARNING;
                        break;
                        case UNKNOWN_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case START_MEASURE_NA:
                            type_error=TOE_WARNING;
                        break;
                        case STOP_MEASURE_NA:
                            type_error=TOE_WARNING;
                        break;
                        case SELECT_USER_LEVEL_NA:
                            type_error=TOE_WARNING;
                        break;
                        case SET_LMS_OUTPUT_NA:
                            type_error=TOE_WARNING;
                        break;
                        case SAVE_DATA_NA:
                            type_error=TOE_WARNING;
                        break;
                        case START_DEVICE_NA:
                            type_error=TOE_WARNING;
                        break;
                        case INCORRECT_ANSWER:
                            type_error=TOE_WARNING;
                        break;
                        case FRAME_OVERFLOW:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_LOG_FILE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_CONFIG_FILE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_CONFIG_FILE_STRUCTURE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_LASER_3D:
                    switch (msg.id_error){
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_BEACON:
                    switch(msg.id_error){
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;
                    }
                    break;
                case SUBS_RANGE_DATA_FUSION:
                    switch(msg.id_error){
                        case RDF_INSUFFICIENT_DATA:
                            type_error=TOE_UNAVAILABLE;
                            break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;
                    }
                    break;
                case SUBS_HUMAN_LOCALIZATION:
                    switch(msg.id_error){
                        case HL_INSUFFICIENT_DATA:
                            type_error=TOE_WARNING;
                            break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;
                    }
                    break;
                case SUBS_CONVOY:
                    switch (msg.id_error){
                        case UDP_SOCKET_FAIL:
                            type_error=TOE_WARNING;
                            break;
                        case BIND_SOCKET_FAIL:
                            type_error=TOE_WARNING;
                            break;
                        case SOCKET_RECEIVE_ERROR:
                            type_error=TOE_WARNING;
                            break;
                        case LEADER_CRITICAL_ERROR:
                            type_error=TOE_WARNING;
                            break;
                        case FOLLOWER_GPS_ERROR:
                            type_error=TOE_WARNING;
                            break;
                        case FOLLOWER_LASER_ERROR:
                            type_error=TOE_WARNING;
                            break;
                        case FOLLOWER_NAVIGATION_ERROR:
                            type_error=TOE_WARNING;
                            break;
                        case FOLLOWER_CAMERA_ERROR:
                            type_error=TOE_WARNING;
                            break;
                        default:
                            type_error=TOE_UNAVAILABLE;         // Ha llegado un id_error no contemplado
                            break;
                        }
                    break;
                default:
                    type_error=TOE_UNAVAILABLE;
                    break;     // id_subsystem incorrecto
            }
     return type_error;
}

short mode_engagebrake_error(Common_files::msg_error msg)
{
     short type_error=0;    
     switch (msg.id_subsystem){
                case SUBS_COMMUNICATION:
                    switch (msg.id_error){
                        case (JAUS_CONFIG_ERROR):
                            type_error=TOE_CRITICAL;
                            break;
                        case (CREATE_COMPONENT_ERROR):
                            type_error=TOE_CRITICAL;
                            break;
                        case (RUN_COMPONENT_ERROR):
                            type_error=TOE_CRITICAL;
                            break;
                        case (COMM_LOG_FILE_ERROR):
                            type_error=TOE_WARNING;
                            break;
                        case (COMM_CONFIG_FILE_ERROR):
                            type_error=TOE_CRITICAL;
                            break;
                        case (COMM_CONFIG_FILE_STRUCTURE_ERROR):
                            type_error=TOE_CRITICAL;
                            break;
                        case COMMUNICATION_UCR_FAIL:
                            type_error=TOE_WARNING;
                            break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_SYSTEM_MGMNT:
                    switch (msg.id_error){
                        case (MODE_NOT_AVAILABLE):
                            type_error=TOE_WARNING;
                        break;
                        case FUNCTION_NOT_AVAILABLE:
                            type_error=TOE_WARNING;
                            break;
                        case (COMM_MODULE_NA):
                            type_error=TOE_CRITICAL;
                        break;
                        case (REMOTE_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (DRIVING_MODULE_NA):
                            type_error=TOE_CRITICAL;
                        break;
                        case (NAVIGATION_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (CAMERA_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (GPS_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (FRONT_LASER_1_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (REAR_LASER_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (LASER3D_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (BEACON_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (RDF_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (HL_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (CONVOY_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_REMOTE:
                    switch (msg.id_error){
                        case REMOTE_PARAMETER_OUTRANGE:
                            type_error=TOE_WARNING;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_DRIVING:
                    switch (msg.id_error){
                        case CONNECTION_CAN_FAIL:
                            type_error=TOE_CRITICAL;
                        break;
                        case COMMUNICATION_CAN_FAIL:
                            type_error=TOE_CRITICAL;
                        break;
                        case START_STOP_FAILURE:
                            type_error=TOE_WARNING;
                        break;
                        case THROTTLE_FAILURE:
                            type_error=TOE_WARNING;
                        break;
                        case HANDBRAKE_FAILURE:
                            type_error=TOE_CRITICAL;
                        break;
                        case BRAKE_FAILURE:
                            type_error=TOE_WARNING;
                        break;
                        case GEAR_SHIFT_FAILURE:
                            type_error=TOE_WARNING;
                        break;
                        case STEER_FAILURE:
                            type_error=TOE_WARNING;
                        break;
                        case DIFFERENTIAL_LOCK_FAILURE:
                            type_error=TOE_WARNING;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_NAVIGATION:
                    switch (msg.id_error){
                        case (ERROR_NAVIGATION_OBSTACLE_UNAVOIDABLE):
                            type_error=TOE_WARNING;
                        break;
                        case (ERROR_NAVIGATION_WAYPOINTS_GETTING_TIMEOUT):
                            type_error=TOE_WARNING;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_CAMERA:
                    switch (msg.id_error){
                        break;
                        case (ERROR_CAMERA_COMMUNICATION):
                            type_error=TOE_WARNING;
                        break;
                        case (ERROR_CAMERA_CONFIGURATION):
                            type_error=TOE_WARNING;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_GPS:
                    switch (msg.id_error){
                        case GPS_GLOBAL_ERROR:
                            type_error=TOE_END_ERROR;
                        break;
                        case DATA_RCV_FAILED:
                            type_error=TOE_WARNING;
                        break;
                        case CONFIG_OPEN_SERIALPORT_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case CONFIG_CONFIG_SERIALPORT_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case CONFIG_ALIGNMENT_MODE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case CONFIG_INITIAL_AZIMUT_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case CONFIG_ANTOFFSET_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case INS_INACTIVE:
                            type_error=TOE_WARNING;
                        break;
                        case INS_ALIGNING:
                            type_error=TOE_WARNING;
                        break;
                        case INS_SOLUTION_NOT_GOOD:
                            type_error=TOE_WARNING;
                        break;
                        case INS_BAD_GPS_AGREEMENT:
                            type_error=TOE_WARNING;
                        break;
                        case INS_ALIGNMENT_COMPLETE:
                            type_error=TOE_WARNING;
                        break;
                        case INSUFFICIENT_OBS:
                            type_error=TOE_WARNING;
                        break;
                        case NO_CONVERGENCE:
                            type_error=TOE_WARNING;
                        break;
                        case SINGULARITY:
                            type_error=TOE_WARNING;
                        break;
                        case COV_TRACE:
                            type_error=TOE_WARNING;
                        break;
                        case TEST_DIST:
                            type_error=TOE_WARNING;
                        break;
                        case COLD_START:
                            type_error=TOE_WARNING;
                        break;
                        case V_H_LIMIT:
                            type_error=TOE_WARNING;
                        break;
                        case VARIANCE:
                            type_error=TOE_WARNING;
                        break;
                        case RESIDUALS:
                            type_error=TOE_WARNING;
                        break;
                        case DELTA_POS:
                            type_error=TOE_WARNING;
                        break;
                        case NEGATIVE_VAR:
                            type_error=TOE_WARNING;
                        break;
                        case INTEGRITY_WARNING:
                            type_error=TOE_WARNING;
                        break;
                        case IMU_UNPLUGGED:
                            type_error=TOE_WARNING;
                        break;
                        case PENDING:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_FIX:
                            type_error=TOE_WARNING;
                        break;
                        case UNAUTHORIZED_STATE:
                            type_error=TOE_WARNING;
                        break;

                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_FRONT_LASER_1:
                    switch (msg.id_error){
                        case LASER_SOCKET_FAIL:
                            type_error=TOE_WARNING;
                        break;
                        case CONNECTION_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case COMM_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_SAMPLE_FREQ:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_ANGLE_RESOLUTION:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_ANGLE_SAMPLE_RESOLUTION:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_SCAN_AREA:
                            type_error=TOE_WARNING;
                        break;
                        case UNKNOWN_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case START_MEASURE_NA:
                            type_error=TOE_WARNING;
                        break;
                        case STOP_MEASURE_NA:
                            type_error=TOE_WARNING;
                        break;
                        case SELECT_USER_LEVEL_NA:
                            type_error=TOE_WARNING;
                        break;
                        case SET_LMS_OUTPUT_NA:
                            type_error=TOE_WARNING;
                        break;
                        case SAVE_DATA_NA:
                            type_error=TOE_WARNING;
                        break;
                        case START_DEVICE_NA:
                            type_error=TOE_WARNING;
                        break;
                        case INCORRECT_ANSWER:
                            type_error=TOE_WARNING;
                        break;
                        case FRAME_OVERFLOW:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_LOG_FILE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_CONFIG_FILE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_CONFIG_FILE_STRUCTURE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_FRONT_LASER_2:
                    switch (msg.id_error){
                        case LASER_SOCKET_FAIL:
                            type_error=TOE_WARNING;
                        break;
                        case CONNECTION_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case COMM_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_SAMPLE_FREQ:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_ANGLE_RESOLUTION:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_ANGLE_SAMPLE_RESOLUTION:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_SCAN_AREA:
                            type_error=TOE_WARNING;
                        break;
                        case UNKNOWN_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case START_MEASURE_NA:
                            type_error=TOE_WARNING;
                        break;
                        case STOP_MEASURE_NA:
                            type_error=TOE_WARNING;
                        break;
                        case SELECT_USER_LEVEL_NA:
                            type_error=TOE_WARNING;
                        break;
                        case SET_LMS_OUTPUT_NA:
                            type_error=TOE_WARNING;
                        break;
                        case SAVE_DATA_NA:
                            type_error=TOE_WARNING;
                        break;
                        case START_DEVICE_NA:
                            type_error=TOE_WARNING;
                        break;
                        case INCORRECT_ANSWER:
                            type_error=TOE_WARNING;
                        break;
                        case FRAME_OVERFLOW:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_LOG_FILE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_CONFIG_FILE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_CONFIG_FILE_STRUCTURE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_REAR_LASER:
                    switch (msg.id_error){
                        case LASER_SOCKET_FAIL:
                            type_error=TOE_WARNING;
                        break;
                        case CONNECTION_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case COMM_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_SAMPLE_FREQ:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_ANGLE_RESOLUTION:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_ANGLE_SAMPLE_RESOLUTION:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_SCAN_AREA:
                            type_error=TOE_WARNING;
                        break;
                        case UNKNOWN_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case START_MEASURE_NA:
                            type_error=TOE_WARNING;
                        break;
                        case STOP_MEASURE_NA:
                            type_error=TOE_WARNING;
                        break;
                        case SELECT_USER_LEVEL_NA:
                            type_error=TOE_WARNING;
                        break;
                        case SET_LMS_OUTPUT_NA:
                            type_error=TOE_WARNING;
                        break;
                        case SAVE_DATA_NA:
                            type_error=TOE_WARNING;
                        break;
                        case START_DEVICE_NA:
                            type_error=TOE_WARNING;
                        break;
                        case INCORRECT_ANSWER:
                            type_error=TOE_WARNING;
                        break;
                        case FRAME_OVERFLOW:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_LOG_FILE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_CONFIG_FILE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_CONFIG_FILE_STRUCTURE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_LASER_3D:
                    switch (msg.id_error){
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_BEACON:
                    switch(msg.id_error){
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;
                    }
                    break;
                case SUBS_RANGE_DATA_FUSION:
                    switch(msg.id_error){
                        case RDF_INSUFFICIENT_DATA:
                            type_error=TOE_UNAVAILABLE;
                            break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;
                    }
                    break;
                case SUBS_HUMAN_LOCALIZATION:
                    switch(msg.id_error){
                        case HL_INSUFFICIENT_DATA:
                            type_error=TOE_WARNING;
                            break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;
                    }
                    break;
                case SUBS_CONVOY:
                    switch (msg.id_error){
                        case UDP_SOCKET_FAIL:
                            type_error=TOE_WARNING;
                            break;
                        case BIND_SOCKET_FAIL:
                            type_error=TOE_WARNING;
                            break;
                        case SOCKET_RECEIVE_ERROR:
                            type_error=TOE_WARNING;
                            break;
                        case LEADER_CRITICAL_ERROR:
                            type_error=TOE_WARNING;
                            break;
                        case FOLLOWER_GPS_ERROR:
                            type_error=TOE_WARNING;
                            break;
                        case FOLLOWER_LASER_ERROR:
                            type_error=TOE_WARNING;
                            break;
                        case FOLLOWER_NAVIGATION_ERROR:
                            type_error=TOE_WARNING;
                            break;
                        case FOLLOWER_CAMERA_ERROR:
                            type_error=TOE_WARNING;
                            break;
                        default:
                            type_error=TOE_UNAVAILABLE;         // Ha llegado un id_error no contemplado
                            break;
                        }
                    break;
                default:
                    type_error=TOE_UNAVAILABLE;
                    break;     // id_subsystem incorrecto
            }
     return type_error;
}

// Método que define la gravedad del error para el modo PLAN
short mode_plan_error(Common_files::msg_error msg) {
    short type_error = 0;
    switch (msg.id_subsystem) {
        case SUBS_COMMUNICATION:
            switch (msg.id_error) {
                case (JAUS_CONFIG_ERROR):
                    type_error = TOE_CRITICAL;
                    break;
                case (CREATE_COMPONENT_ERROR):
                    type_error = TOE_CRITICAL;
                    break;
                case (RUN_COMPONENT_ERROR):
                    type_error = TOE_CRITICAL;
                    break;
                case (COMM_LOG_FILE_ERROR):
                    type_error = TOE_WARNING;
                    break;
                case (COMM_CONFIG_FILE_ERROR):
                    type_error = TOE_CRITICAL;
                    break;
                case (COMM_CONFIG_FILE_STRUCTURE_ERROR):
                    type_error = TOE_CRITICAL;
                    break;
                case COMMUNICATION_UCR_FAIL:
                    type_error = TOE_WARNING;
                    break;
                default:
                    type_error = TOE_UNAVAILABLE;
                    break; // Ha llegado un id_error no contemplado
            }
            break;
        case SUBS_SYSTEM_MGMNT:
            switch (msg.id_error) {
                case (MODE_NOT_AVAILABLE):
                    type_error = TOE_WARNING;
                    break;
                case FUNCTION_NOT_AVAILABLE:
                    type_error = TOE_WARNING;
                    break;
                case (COMM_MODULE_NA):
                    type_error = TOE_CRITICAL;
                    break;
                case (REMOTE_MODULE_NA):
                    type_error = TOE_WARNING;
                    break;
                case (DRIVING_MODULE_NA):
                    type_error = TOE_CRITICAL;
                    break;
                case (NAVIGATION_MODULE_NA):
                    type_error = TOE_CRITICAL;
                    break;
                case (CAMERA_MODULE_NA):
                    type_error = TOE_WARNING;
                    break;
                case (GPS_MODULE_NA):
                    type_error = TOE_CRITICAL;
                    break;
                case (FRONT_LASER_1_MODULE_NA):
                    type_error = TOE_CRITICAL;
                    break;
                case (REAR_LASER_MODULE_NA):
                    type_error = TOE_CRITICAL;
                    break;
                case (LASER3D_MODULE_NA):
                    type_error = TOE_WARNING;
                    break;
                case (BEACON_MODULE_NA):
                    type_error = TOE_WARNING;
                    break;
                case (RDF_MODULE_NA):
                    type_error = TOE_WARNING;
                    break;
                case (HL_MODULE_NA):
                    type_error = TOE_WARNING;
                    break;
                case (CONVOY_MODULE_NA):
                    type_error = TOE_WARNING;
                    break;
                default:
                    type_error = TOE_UNAVAILABLE;
                    break; // Ha llegado un id_error no contemplado
            }
            break;
        case SUBS_REMOTE:
            switch (msg.id_error) {
                case REMOTE_PARAMETER_OUTRANGE:
                    type_error = TOE_WARNING;
                    break;
                default:
                    type_error = TOE_UNAVAILABLE;
                    break; // Ha llegado un id_error no contemplado
            }
            break;
        case SUBS_DRIVING:
            switch (msg.id_error) {
                case CONNECTION_CAN_FAIL:
                    type_error = TOE_CRITICAL;
                    break;
                case COMMUNICATION_CAN_FAIL:
                    type_error = TOE_CRITICAL;
                    break;
                case START_STOP_FAILURE:
                    type_error = TOE_CRITICAL;
                    break;
                case THROTTLE_FAILURE:
                    type_error = TOE_CRITICAL;
                    break;
                case HANDBRAKE_FAILURE:
                    type_error = TOE_CRITICAL;
                    break;
                case BRAKE_FAILURE:
                    type_error = TOE_CRITICAL;
                    break;
                case GEAR_SHIFT_FAILURE:
                    type_error = TOE_CRITICAL;
                    break;
                case STEER_FAILURE:
                    type_error = TOE_CRITICAL;
                    break;
                case DIFFERENTIAL_LOCK_FAILURE:
                    type_error = TOE_CRITICAL;
                    break;
                default:
                    type_error = TOE_UNAVAILABLE;
                    break; // Ha llegado un id_error no contemplado
            }
            break;
        case SUBS_NAVIGATION:
            switch (msg.id_error) {
                case (ERROR_NAVIGATION_OBSTACLE_UNAVOIDABLE):
                    type_error = TOE_WARNING;
                    break;
                case (ERROR_NAVIGATION_WAYPOINTS_GETTING_TIMEOUT):
                    type_error = TOE_WARNING;
                    break;
                default:
                    type_error = TOE_UNAVAILABLE;
                    break; // Ha llegado un id_error no contemplado
            }
            break;
        case SUBS_CAMERA:
            switch (msg.id_error) {
                    break;
                case (ERROR_CAMERA_COMMUNICATION):
                    type_error = TOE_WARNING;
                    break;
                case (ERROR_CAMERA_CONFIGURATION):
                    type_error = TOE_WARNING;
                    break;
                default:
                    type_error = TOE_UNAVAILABLE;
                    break; // Ha llegado un id_error no contemplado
            }
            break;
        case SUBS_GPS:
            switch (msg.id_error) {
                case GPS_GLOBAL_ERROR:
                    type_error = TOE_END_ERROR;
                    break;
                case DATA_RCV_FAILED:
                    type_error = TOE_CRITICAL;
                    break;
                case CONFIG_OPEN_SERIALPORT_ERROR:
                    type_error = TOE_CRITICAL;
                    break;
                case CONFIG_CONFIG_SERIALPORT_ERROR:
                    type_error = TOE_CRITICAL;
                    break;
                case CONFIG_ALIGNMENT_MODE_ERROR:
                    type_error = TOE_CRITICAL;
                    break;
                case CONFIG_INITIAL_AZIMUT_ERROR:
                    type_error = TOE_CRITICAL;
                    break;
                case CONFIG_ANTOFFSET_ERROR:
                    type_error = TOE_CRITICAL;
                    break;
                case INS_INACTIVE:
                    type_error = TOE_CRITICAL;
                    break;
                case INS_ALIGNING:
                    type_error = TOE_CRITICAL;
                    break;
                case INS_SOLUTION_NOT_GOOD:
                    type_error = TOE_CRITICAL;
                    break;
                case INS_BAD_GPS_AGREEMENT:
                    type_error = TOE_CRITICAL;
                    break;
                case INS_ALIGNMENT_COMPLETE:
                    type_error = TOE_CRITICAL;
                    break;
                case INSUFFICIENT_OBS:
                    type_error = TOE_CRITICAL;
                    break;
                case NO_CONVERGENCE:
                    type_error = TOE_CRITICAL;
                    break;
                case SINGULARITY:
                    type_error = TOE_CRITICAL;
                    break;
                case COV_TRACE:
                    type_error = TOE_CRITICAL;
                    break;
                case TEST_DIST:
                    type_error = TOE_CRITICAL;
                    break;
                case COLD_START:
                    type_error = TOE_CRITICAL;
                    break;
                case V_H_LIMIT:
                    type_error = TOE_CRITICAL;
                    break;
                case VARIANCE:
                    type_error = TOE_CRITICAL;
                    break;
                case RESIDUALS:
                    type_error = TOE_CRITICAL;
                    break;
                case DELTA_POS:
                    type_error = TOE_CRITICAL;
                    break;
                case NEGATIVE_VAR:
                    type_error = TOE_CRITICAL;
                    break;
                case INTEGRITY_WARNING:
                    type_error = TOE_CRITICAL;
                    break;
                case IMU_UNPLUGGED:
                    type_error = TOE_CRITICAL;
                    break;
                case PENDING:
                    type_error = TOE_CRITICAL;
                    break;
                case INVALID_FIX:
                    type_error = TOE_CRITICAL;
                    break;
                case UNAUTHORIZED_STATE:
                    type_error = TOE_CRITICAL;
                    break;

                default:
                    type_error = TOE_UNAVAILABLE;
                    break; // Ha llegado un id_error no contemplado
            }
            break;
        case SUBS_FRONT_LASER_1:
            switch (msg.id_error) {
                case LASER_SOCKET_FAIL:
                    type_error = TOE_CRITICAL;
                    break;
                case CONNECTION_ERROR:
                    type_error = TOE_CRITICAL;
                    break;
                case COMM_ERROR:
                    type_error = TOE_CRITICAL;
                    break;
                case INVALID_SAMPLE_FREQ:
                    type_error = TOE_CRITICAL;
                    break;
                case INVALID_ANGLE_RESOLUTION:
                    type_error = TOE_CRITICAL;
                    break;
                case INVALID_ANGLE_SAMPLE_RESOLUTION:
                    type_error = TOE_CRITICAL;
                    break;
                case INVALID_SCAN_AREA:
                    type_error = TOE_CRITICAL;
                    break;
                case UNKNOWN_ERROR:
                    type_error = TOE_CRITICAL;
                    break;
                case START_MEASURE_NA:
                    type_error = TOE_CRITICAL;
                    break;
                case STOP_MEASURE_NA:
                    type_error = TOE_CRITICAL;
                    break;
                case SELECT_USER_LEVEL_NA:
                    type_error = TOE_CRITICAL;
                    break;
                case SET_LMS_OUTPUT_NA:
                    type_error = TOE_CRITICAL;
                    break;
                case SAVE_DATA_NA:
                    type_error = TOE_CRITICAL;
                    break;
                case START_DEVICE_NA:
                    type_error = TOE_CRITICAL;
                    break;
                case INCORRECT_ANSWER:
                    type_error = TOE_CRITICAL;
                    break;
                case FRAME_OVERFLOW:
                    type_error = TOE_CRITICAL;
                    break;
                case LASER_LOG_FILE_ERROR:
                    type_error = TOE_WARNING;
                    break;
                case LASER_CONFIG_FILE_ERROR:
                    type_error = TOE_CRITICAL;
                    break;
                case LASER_CONFIG_FILE_STRUCTURE_ERROR:
                    type_error = TOE_CRITICAL;
                    break;
                default:
                    type_error = TOE_UNAVAILABLE;
                    break; // Ha llegado un id_error no contemplado
            }
            break;
        case SUBS_FRONT_LASER_2:
            switch (msg.id_error) {
                case LASER_SOCKET_FAIL:
                    type_error = TOE_CRITICAL;
                    break;
                case CONNECTION_ERROR:
                    type_error = TOE_CRITICAL;
                    break;
                case COMM_ERROR:
                    type_error = TOE_CRITICAL;
                    break;
                case INVALID_SAMPLE_FREQ:
                    type_error = TOE_CRITICAL;
                    break;
                case INVALID_ANGLE_RESOLUTION:
                    type_error = TOE_CRITICAL;
                    break;
                case INVALID_ANGLE_SAMPLE_RESOLUTION:
                    type_error = TOE_CRITICAL;
                    break;
                case INVALID_SCAN_AREA:
                    type_error = TOE_CRITICAL;
                    break;
                case UNKNOWN_ERROR:
                    type_error = TOE_CRITICAL;
                    break;
                case START_MEASURE_NA:
                    type_error = TOE_CRITICAL;
                    break;
                case STOP_MEASURE_NA:
                    type_error = TOE_CRITICAL;
                    break;
                case SELECT_USER_LEVEL_NA:
                    type_error = TOE_CRITICAL;
                    break;
                case SET_LMS_OUTPUT_NA:
                    type_error = TOE_CRITICAL;
                    break;
                case SAVE_DATA_NA:
                    type_error = TOE_CRITICAL;
                    break;
                case START_DEVICE_NA:
                    type_error = TOE_CRITICAL;
                    break;
                case INCORRECT_ANSWER:
                    type_error = TOE_CRITICAL;
                    break;
                case FRAME_OVERFLOW:
                    type_error = TOE_CRITICAL;
                    break;
                case LASER_LOG_FILE_ERROR:
                    type_error = TOE_WARNING;
                    break;
                case LASER_CONFIG_FILE_ERROR:
                    type_error = TOE_CRITICAL;
                    break;
                case LASER_CONFIG_FILE_STRUCTURE_ERROR:
                    type_error = TOE_CRITICAL;
                    break;
                default:
                    type_error = TOE_UNAVAILABLE;
                    break; // Ha llegado un id_error no contemplado
            }
            break;
        case SUBS_REAR_LASER:
            switch (msg.id_error) {
                case LASER_SOCKET_FAIL:
                    type_error = TOE_CRITICAL;
                    break;
                case CONNECTION_ERROR:
                    type_error = TOE_CRITICAL;
                    break;
                case COMM_ERROR:
                    type_error = TOE_CRITICAL;
                    break;
                case INVALID_SAMPLE_FREQ:
                    type_error = TOE_CRITICAL;
                    break;
                case INVALID_ANGLE_RESOLUTION:
                    type_error = TOE_CRITICAL;
                    break;
                case INVALID_ANGLE_SAMPLE_RESOLUTION:
                    type_error = TOE_CRITICAL;
                    break;
                case INVALID_SCAN_AREA:
                    type_error = TOE_CRITICAL;
                    break;
                case UNKNOWN_ERROR:
                    type_error = TOE_CRITICAL;
                    break;
                case START_MEASURE_NA:
                    type_error = TOE_CRITICAL;
                    break;
                case STOP_MEASURE_NA:
                    type_error = TOE_CRITICAL;
                    break;
                case SELECT_USER_LEVEL_NA:
                    type_error = TOE_CRITICAL;
                    break;
                case SET_LMS_OUTPUT_NA:
                    type_error = TOE_CRITICAL;
                    break;
                case SAVE_DATA_NA:
                    type_error = TOE_CRITICAL;
                    break;
                case START_DEVICE_NA:
                    type_error = TOE_CRITICAL;
                    break;
                case INCORRECT_ANSWER:
                    type_error = TOE_CRITICAL;
                    break;
                case FRAME_OVERFLOW:
                    type_error = TOE_CRITICAL;
                    break;
                case LASER_LOG_FILE_ERROR:
                    type_error = TOE_WARNING;
                    break;
                case LASER_CONFIG_FILE_ERROR:
                    type_error = TOE_CRITICAL;
                    break;
                case LASER_CONFIG_FILE_STRUCTURE_ERROR:
                    type_error = TOE_CRITICAL;
                    break;
                default:
                    type_error = TOE_UNAVAILABLE;
                    break; // Ha llegado un id_error no contemplado
            }
            break;
        case SUBS_LASER_3D:
            switch (msg.id_error) {
                default:
                    type_error = TOE_UNAVAILABLE;
                    break; // Ha llegado un id_error no contemplado
            }
            break;
        case SUBS_BEACON:
            switch (msg.id_error) {
                default:
                    type_error = TOE_UNAVAILABLE;
                    break;
            }
            break;
        case SUBS_RANGE_DATA_FUSION:
            switch (msg.id_error) {
                case RDF_INSUFFICIENT_DATA:
                    type_error = TOE_UNAVAILABLE;
                    break;
                default:
                    type_error = TOE_UNAVAILABLE;
                    break;
            }
            break;
        case SUBS_HUMAN_LOCALIZATION:
            switch (msg.id_error) {
                case HL_INSUFFICIENT_DATA:
                    type_error = TOE_WARNING;
                    break;
                default:
                    type_error = TOE_UNAVAILABLE;
                    break;
            }
            break;
        case SUBS_CONVOY:
            switch (msg.id_error) {
                case UDP_SOCKET_FAIL:
                    type_error = TOE_WARNING;
                    break;
                case BIND_SOCKET_FAIL:
                    type_error = TOE_WARNING;
                    break;
                case SOCKET_RECEIVE_ERROR:
                    type_error = TOE_WARNING;
                    break;
                case LEADER_CRITICAL_ERROR:
                    type_error = TOE_WARNING;
                    break;
                case FOLLOWER_GPS_ERROR:
                    type_error = TOE_WARNING;
                    break;
                case FOLLOWER_LASER_ERROR:
                    type_error = TOE_WARNING;
                    break;
                case FOLLOWER_NAVIGATION_ERROR:
                    type_error = TOE_WARNING;
                    break;
                case FOLLOWER_CAMERA_ERROR:
                    type_error = TOE_WARNING;
                    break;
                default:
                    type_error = TOE_UNAVAILABLE; // Ha llegado un id_error no contemplado
                    break;
            }
            break;
        default:
            type_error = TOE_UNAVAILABLE;
            break; // id_subsystem incorrecto
    }
    return (type_error);
}

// Método que define la gravedad del error para el modo COME TO ME
short mode_cometome_error(Common_files::msg_error msg)
{
     short type_error=0;    
     switch (msg.id_subsystem){
                case SUBS_COMMUNICATION:
                    switch (msg.id_error){
                        case (JAUS_CONFIG_ERROR):
                            type_error=TOE_CRITICAL;
                            break;
                        case (CREATE_COMPONENT_ERROR):
                            type_error=TOE_CRITICAL;
                            break;
                        case (RUN_COMPONENT_ERROR):
                            type_error=TOE_CRITICAL;
                            break;
                        case (COMM_LOG_FILE_ERROR):
                            type_error=TOE_WARNING;
                            break;
                        case (COMM_CONFIG_FILE_ERROR):
                            type_error=TOE_CRITICAL;
                            break;
                        case (COMM_CONFIG_FILE_STRUCTURE_ERROR):
                            type_error=TOE_CRITICAL;
                            break;
                        case COMMUNICATION_UCR_FAIL:
                            type_error=TOE_WARNING;
                            break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_SYSTEM_MGMNT:
                    switch (msg.id_error){
                        case (MODE_NOT_AVAILABLE):
                            type_error=TOE_WARNING;
                        break;
                        case FUNCTION_NOT_AVAILABLE:
                            type_error=TOE_WARNING;
                            break;
                        case (COMM_MODULE_NA):
                            type_error=TOE_CRITICAL;
                        break;
                        case (REMOTE_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (DRIVING_MODULE_NA):
                            type_error=TOE_CRITICAL;
                        break;
                        case (NAVIGATION_MODULE_NA):
                            type_error=TOE_CRITICAL;
                        break;
                        case (CAMERA_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (GPS_MODULE_NA):
                            type_error=TOE_CRITICAL;
                        break;
                        case (FRONT_LASER_1_MODULE_NA):
                            type_error=TOE_CRITICAL;
                        break;
                        case (REAR_LASER_MODULE_NA):
                            type_error=TOE_CRITICAL;
                        break;
                        case (LASER3D_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (BEACON_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (RDF_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (HL_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (CONVOY_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_REMOTE:
                    switch (msg.id_error){
                        case REMOTE_PARAMETER_OUTRANGE:
                            type_error=TOE_WARNING;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_DRIVING:
                    switch (msg.id_error){
                        case CONNECTION_CAN_FAIL:
                            type_error=TOE_CRITICAL;
                        break;
                        case COMMUNICATION_CAN_FAIL:
                            type_error=TOE_CRITICAL;
                        break;
                        case START_STOP_FAILURE:
                            type_error=TOE_CRITICAL;
                        break;
                        case THROTTLE_FAILURE:
                            type_error=TOE_CRITICAL;
                        break;
                        case HANDBRAKE_FAILURE:
                            type_error=TOE_CRITICAL;
                        break;
                        case BRAKE_FAILURE:
                            type_error=TOE_CRITICAL;
                        break;
                        case GEAR_SHIFT_FAILURE:
                            type_error=TOE_CRITICAL;
                        break;
                        case STEER_FAILURE:
                            type_error=TOE_CRITICAL;
                        break;
                        case DIFFERENTIAL_LOCK_FAILURE:
                            type_error=TOE_CRITICAL;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_NAVIGATION:
                    switch (msg.id_error){
                        case (ERROR_NAVIGATION_OBSTACLE_UNAVOIDABLE):
                            type_error=TOE_WARNING;
                        break;
                        case (ERROR_NAVIGATION_WAYPOINTS_GETTING_TIMEOUT):
                            type_error=TOE_WARNING;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_CAMERA:
                    switch (msg.id_error){
                        break;
                        case (ERROR_CAMERA_COMMUNICATION):
                            type_error=TOE_WARNING;
                        break;
                        case (ERROR_CAMERA_CONFIGURATION):
                            type_error=TOE_WARNING;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_GPS:
                    switch (msg.id_error){
                        case GPS_GLOBAL_ERROR:
                            type_error=TOE_END_ERROR;
                        break;
                        case DATA_RCV_FAILED:
                            type_error=TOE_CRITICAL;
                        break;
                        case CONFIG_OPEN_SERIALPORT_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case CONFIG_CONFIG_SERIALPORT_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case CONFIG_ALIGNMENT_MODE_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case CONFIG_INITIAL_AZIMUT_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case CONFIG_ANTOFFSET_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case INS_INACTIVE:
                            type_error=TOE_CRITICAL;
                        break;
                        case INS_ALIGNING:
                            type_error=TOE_CRITICAL;
                        break;
                        case INS_SOLUTION_NOT_GOOD:
                            type_error=TOE_CRITICAL;
                        break;
                        case INS_BAD_GPS_AGREEMENT:
                            type_error=TOE_CRITICAL;
                        break;
                        case INS_ALIGNMENT_COMPLETE:
                            type_error=TOE_CRITICAL;
                        break;
                        case INSUFFICIENT_OBS:
                            type_error=TOE_CRITICAL;
                        break;
                        case NO_CONVERGENCE:
                            type_error=TOE_CRITICAL;
                        break;
                        case SINGULARITY:
                            type_error=TOE_CRITICAL;
                        break;
                        case COV_TRACE:
                            type_error=TOE_CRITICAL;
                        break;
                        case TEST_DIST:
                            type_error=TOE_CRITICAL;
                        break;
                        case COLD_START:
                            type_error=TOE_CRITICAL;
                        break;
                        case V_H_LIMIT:
                            type_error=TOE_CRITICAL;
                        break;
                        case VARIANCE:
                            type_error=TOE_CRITICAL;
                        break;
                        case RESIDUALS:
                            type_error=TOE_CRITICAL;
                        break;
                        case DELTA_POS:
                            type_error=TOE_CRITICAL;
                        break;
                        case NEGATIVE_VAR:
                            type_error=TOE_CRITICAL;
                        break;
                        case INTEGRITY_WARNING:
                            type_error=TOE_CRITICAL;
                        break;
                        case IMU_UNPLUGGED:
                            type_error=TOE_CRITICAL;
                        break;
                        case PENDING:
                            type_error=TOE_CRITICAL;
                        break;
                        case INVALID_FIX:
                            type_error=TOE_CRITICAL;
                        break;
                        case UNAUTHORIZED_STATE:
                            type_error=TOE_CRITICAL;
                        break;

                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_FRONT_LASER_1:
                    switch (msg.id_error){
                        case LASER_SOCKET_FAIL:
                            type_error=TOE_CRITICAL;
                        break;
                        case CONNECTION_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case COMM_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case INVALID_SAMPLE_FREQ:
                            type_error=TOE_CRITICAL;
                        break;
                        case INVALID_ANGLE_RESOLUTION:
                            type_error=TOE_CRITICAL;
                        break;
                        case INVALID_ANGLE_SAMPLE_RESOLUTION:
                            type_error=TOE_CRITICAL;
                        break;
                        case INVALID_SCAN_AREA:
                            type_error=TOE_CRITICAL;
                        break;
                        case UNKNOWN_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case START_MEASURE_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case STOP_MEASURE_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case SELECT_USER_LEVEL_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case SET_LMS_OUTPUT_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case SAVE_DATA_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case START_DEVICE_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case INCORRECT_ANSWER:
                            type_error=TOE_CRITICAL;
                        break;
                        case FRAME_OVERFLOW:
                            type_error=TOE_CRITICAL;
                        break;
                        case LASER_LOG_FILE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_CONFIG_FILE_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case LASER_CONFIG_FILE_STRUCTURE_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_FRONT_LASER_2:
                    switch (msg.id_error){
                        case LASER_SOCKET_FAIL:
                            type_error=TOE_CRITICAL;
                        break;
                        case CONNECTION_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case COMM_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case INVALID_SAMPLE_FREQ:
                            type_error=TOE_CRITICAL;
                        break;
                        case INVALID_ANGLE_RESOLUTION:
                            type_error=TOE_CRITICAL;
                        break;
                        case INVALID_ANGLE_SAMPLE_RESOLUTION:
                            type_error=TOE_CRITICAL;
                        break;
                        case INVALID_SCAN_AREA:
                            type_error=TOE_CRITICAL;
                        break;
                        case UNKNOWN_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case START_MEASURE_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case STOP_MEASURE_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case SELECT_USER_LEVEL_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case SET_LMS_OUTPUT_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case SAVE_DATA_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case START_DEVICE_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case INCORRECT_ANSWER:
                            type_error=TOE_CRITICAL;
                        break;
                        case FRAME_OVERFLOW:
                            type_error=TOE_CRITICAL;
                        break;
                        case LASER_LOG_FILE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_CONFIG_FILE_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case LASER_CONFIG_FILE_STRUCTURE_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_REAR_LASER:
                    switch (msg.id_error){
                        case LASER_SOCKET_FAIL:
                            type_error=TOE_CRITICAL;
                        break;
                        case CONNECTION_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case COMM_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case INVALID_SAMPLE_FREQ:
                            type_error=TOE_CRITICAL;
                        break;
                        case INVALID_ANGLE_RESOLUTION:
                            type_error=TOE_CRITICAL;
                        break;
                        case INVALID_ANGLE_SAMPLE_RESOLUTION:
                            type_error=TOE_CRITICAL;
                        break;
                        case INVALID_SCAN_AREA:
                            type_error=TOE_CRITICAL;
                        break;
                        case UNKNOWN_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case START_MEASURE_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case STOP_MEASURE_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case SELECT_USER_LEVEL_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case SET_LMS_OUTPUT_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case SAVE_DATA_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case START_DEVICE_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case INCORRECT_ANSWER:
                            type_error=TOE_CRITICAL;
                        break;
                        case FRAME_OVERFLOW:
                            type_error=TOE_CRITICAL;
                        break;
                        case LASER_LOG_FILE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_CONFIG_FILE_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case LASER_CONFIG_FILE_STRUCTURE_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_LASER_3D:
                    switch (msg.id_error){
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_BEACON:
                    switch(msg.id_error){
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;
                    }
                    break;
                case SUBS_RANGE_DATA_FUSION:
                    switch(msg.id_error){
                        case RDF_INSUFFICIENT_DATA:
                            type_error=TOE_WARNING;
                            break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;
                    }
                    break;
                case SUBS_HUMAN_LOCALIZATION:
                    switch(msg.id_error){
                        case HL_INSUFFICIENT_DATA:
                            type_error=TOE_WARNING;
                            break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;
                    }
                    break;
                case SUBS_CONVOY:
                    switch (msg.id_error){
                        case UDP_SOCKET_FAIL:
                            type_error=TOE_WARNING;
                            break;
                        case BIND_SOCKET_FAIL:
                            type_error=TOE_WARNING;
                            break;
                        case SOCKET_RECEIVE_ERROR:
                            type_error=TOE_WARNING;
                            break;
                        case LEADER_CRITICAL_ERROR:
                            type_error=TOE_WARNING;
                            break;
                        case FOLLOWER_GPS_ERROR:
                            type_error=TOE_WARNING;
                            break;
                        case FOLLOWER_LASER_ERROR:
                            type_error=TOE_WARNING;
                            break;
                        case FOLLOWER_NAVIGATION_ERROR:
                            type_error=TOE_WARNING;
                            break;
                        case FOLLOWER_CAMERA_ERROR:
                            type_error=TOE_WARNING;
                            break;
                        default:
                            type_error=TOE_UNAVAILABLE;         // Ha llegado un id_error no contemplado
                            break;
                        }
                    break;
                default:
                    type_error=TOE_UNAVAILABLE;
                    break;     // id_subsystem incorrecto
            }
     return type_error;
}

// Método que define la gravedad del error para el modo FOLLOW ME
short mode_followme_error(Common_files::msg_error msg)
{
     short type_error=0;    
     switch (msg.id_subsystem){
                case SUBS_COMMUNICATION:
                    switch (msg.id_error){
                        case (JAUS_CONFIG_ERROR):
                            type_error=TOE_CRITICAL;
                            break;
                        case (CREATE_COMPONENT_ERROR):
                            type_error=TOE_CRITICAL;
                            break;
                        case (RUN_COMPONENT_ERROR):
                            type_error=TOE_CRITICAL;
                            break;
                        case (COMM_LOG_FILE_ERROR):
                            type_error=TOE_WARNING;
                            break;
                        case (COMM_CONFIG_FILE_ERROR):
                            type_error=TOE_CRITICAL;
                            break;
                        case (COMM_CONFIG_FILE_STRUCTURE_ERROR):
                            type_error=TOE_CRITICAL;
                            break;
                        case COMMUNICATION_UCR_FAIL:
                            type_error=TOE_WARNING;
                            break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_SYSTEM_MGMNT:
                    switch (msg.id_error){
                        case (MODE_NOT_AVAILABLE):
                            type_error=TOE_WARNING;
                        break;
                        case FUNCTION_NOT_AVAILABLE:
                            type_error=TOE_WARNING;
                            break;
                        case (COMM_MODULE_NA):
                            type_error=TOE_CRITICAL;
                        break;
                        case (REMOTE_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (DRIVING_MODULE_NA):
                            type_error=TOE_CRITICAL;
                        break;
                        case (NAVIGATION_MODULE_NA):
                            type_error=TOE_CRITICAL;
                        break;
                        case (CAMERA_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (GPS_MODULE_NA):
                            type_error=TOE_CRITICAL;
                        break;
                        case (FRONT_LASER_1_MODULE_NA):
                            type_error=TOE_CRITICAL;
                        break;
                        case (REAR_LASER_MODULE_NA):
                            type_error=TOE_CRITICAL;
                        break;
                        case (LASER3D_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (BEACON_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (RDF_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (HL_MODULE_NA):
                            type_error=TOE_CRITICAL;
                        break;
                        case (CONVOY_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_REMOTE:
                    switch (msg.id_error){
                        case REMOTE_PARAMETER_OUTRANGE:
                            type_error=TOE_WARNING;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_DRIVING:
                    switch (msg.id_error){
                        case CONNECTION_CAN_FAIL:
                            type_error=TOE_CRITICAL;
                        break;
                        case COMMUNICATION_CAN_FAIL:
                            type_error=TOE_CRITICAL;
                        break;
                        case START_STOP_FAILURE:
                            type_error=TOE_CRITICAL;
                        break;
                        case THROTTLE_FAILURE:
                            type_error=TOE_CRITICAL;
                        break;
                        case HANDBRAKE_FAILURE:
                            type_error=TOE_CRITICAL;
                        break;
                        case BRAKE_FAILURE:
                            type_error=TOE_CRITICAL;
                        break;
                        case GEAR_SHIFT_FAILURE:
                            type_error=TOE_CRITICAL;
                        break;
                        case STEER_FAILURE:
                            type_error=TOE_CRITICAL;
                        break;
                        case DIFFERENTIAL_LOCK_FAILURE:
                            type_error=TOE_CRITICAL;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_NAVIGATION:
                    switch (msg.id_error){
                        case (ERROR_NAVIGATION_OBSTACLE_UNAVOIDABLE):
                            type_error=TOE_WARNING;
                        break;
                        case (ERROR_NAVIGATION_WAYPOINTS_GETTING_TIMEOUT):
                            type_error=TOE_WARNING;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_CAMERA:
                    switch (msg.id_error){
                        break;
                        case (ERROR_CAMERA_COMMUNICATION):
                            type_error=TOE_WARNING;
                        break;
                        case (ERROR_CAMERA_CONFIGURATION):
                            type_error=TOE_WARNING;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_GPS:
                    switch (msg.id_error){
                        case GPS_GLOBAL_ERROR:
                            type_error=TOE_END_ERROR;
                        break;
                        case DATA_RCV_FAILED:
                            type_error=TOE_CRITICAL;
                        break;
                        case CONFIG_OPEN_SERIALPORT_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case CONFIG_CONFIG_SERIALPORT_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case CONFIG_ALIGNMENT_MODE_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case CONFIG_INITIAL_AZIMUT_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case CONFIG_ANTOFFSET_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case INS_INACTIVE:
                            type_error=TOE_CRITICAL;
                        break;
                        case INS_ALIGNING:
                            type_error=TOE_CRITICAL;
                        break;
                        case INS_SOLUTION_NOT_GOOD:
                            type_error=TOE_CRITICAL;
                        break;
                        case INS_BAD_GPS_AGREEMENT:
                            type_error=TOE_CRITICAL;
                        break;
                        case INS_ALIGNMENT_COMPLETE:
                            type_error=TOE_CRITICAL;
                        break;
                        case INSUFFICIENT_OBS:
                            type_error=TOE_CRITICAL;
                        break;
                        case NO_CONVERGENCE:
                            type_error=TOE_CRITICAL;
                        break;
                        case SINGULARITY:
                            type_error=TOE_CRITICAL;
                        break;
                        case COV_TRACE:
                            type_error=TOE_CRITICAL;
                        break;
                        case TEST_DIST:
                            type_error=TOE_CRITICAL;
                        break;
                        case COLD_START:
                            type_error=TOE_CRITICAL;
                        break;
                        case V_H_LIMIT:
                            type_error=TOE_CRITICAL;
                        break;
                        case VARIANCE:
                            type_error=TOE_CRITICAL;
                        break;
                        case RESIDUALS:
                            type_error=TOE_CRITICAL;
                        break;
                        case DELTA_POS:
                            type_error=TOE_CRITICAL;
                        break;
                        case NEGATIVE_VAR:
                            type_error=TOE_CRITICAL;
                        break;
                        case INTEGRITY_WARNING:
                            type_error=TOE_CRITICAL;
                        break;
                        case IMU_UNPLUGGED:
                            type_error=TOE_CRITICAL;
                        break;
                        case PENDING:
                            type_error=TOE_CRITICAL;
                        break;
                        case INVALID_FIX:
                            type_error=TOE_CRITICAL;
                        break;
                        case UNAUTHORIZED_STATE:
                            type_error=TOE_CRITICAL;
                        break;

                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_FRONT_LASER_1:
                    switch (msg.id_error){
                        case LASER_SOCKET_FAIL:
                            type_error=TOE_CRITICAL;
                        break;
                        case CONNECTION_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case COMM_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case INVALID_SAMPLE_FREQ:
                            type_error=TOE_CRITICAL;
                        break;
                        case INVALID_ANGLE_RESOLUTION:
                            type_error=TOE_CRITICAL;
                        break;
                        case INVALID_ANGLE_SAMPLE_RESOLUTION:
                            type_error=TOE_CRITICAL;
                        break;
                        case INVALID_SCAN_AREA:
                            type_error=TOE_CRITICAL;
                        break;
                        case UNKNOWN_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case START_MEASURE_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case STOP_MEASURE_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case SELECT_USER_LEVEL_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case SET_LMS_OUTPUT_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case SAVE_DATA_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case START_DEVICE_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case INCORRECT_ANSWER:
                            type_error=TOE_CRITICAL;
                        break;
                        case FRAME_OVERFLOW:
                            type_error=TOE_CRITICAL;
                        break;
                        case LASER_LOG_FILE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_CONFIG_FILE_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case LASER_CONFIG_FILE_STRUCTURE_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_FRONT_LASER_2:
                    switch (msg.id_error){
                        case LASER_SOCKET_FAIL:
                            type_error=TOE_CRITICAL;
                        break;
                        case CONNECTION_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case COMM_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case INVALID_SAMPLE_FREQ:
                            type_error=TOE_CRITICAL;
                        break;
                        case INVALID_ANGLE_RESOLUTION:
                            type_error=TOE_CRITICAL;
                        break;
                        case INVALID_ANGLE_SAMPLE_RESOLUTION:
                            type_error=TOE_CRITICAL;
                        break;
                        case INVALID_SCAN_AREA:
                            type_error=TOE_CRITICAL;
                        break;
                        case UNKNOWN_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case START_MEASURE_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case STOP_MEASURE_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case SELECT_USER_LEVEL_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case SET_LMS_OUTPUT_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case SAVE_DATA_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case START_DEVICE_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case INCORRECT_ANSWER:
                            type_error=TOE_CRITICAL;
                        break;
                        case FRAME_OVERFLOW:
                            type_error=TOE_CRITICAL;
                        break;
                        case LASER_LOG_FILE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_CONFIG_FILE_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case LASER_CONFIG_FILE_STRUCTURE_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_REAR_LASER:
                    switch (msg.id_error){
                        case LASER_SOCKET_FAIL:
                            type_error=TOE_CRITICAL;
                        break;
                        case CONNECTION_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case COMM_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case INVALID_SAMPLE_FREQ:
                            type_error=TOE_CRITICAL;
                        break;
                        case INVALID_ANGLE_RESOLUTION:
                            type_error=TOE_CRITICAL;
                        break;
                        case INVALID_ANGLE_SAMPLE_RESOLUTION:
                            type_error=TOE_CRITICAL;
                        break;
                        case INVALID_SCAN_AREA:
                            type_error=TOE_CRITICAL;
                        break;
                        case UNKNOWN_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case START_MEASURE_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case STOP_MEASURE_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case SELECT_USER_LEVEL_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case SET_LMS_OUTPUT_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case SAVE_DATA_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case START_DEVICE_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case INCORRECT_ANSWER:
                            type_error=TOE_CRITICAL;
                        break;
                        case FRAME_OVERFLOW:
                            type_error=TOE_CRITICAL;
                        break;
                        case LASER_LOG_FILE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_CONFIG_FILE_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case LASER_CONFIG_FILE_STRUCTURE_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_LASER_3D:
                    switch (msg.id_error){
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_BEACON:
                    switch(msg.id_error){
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;
                    }
                    break;
                case SUBS_RANGE_DATA_FUSION:
                    switch(msg.id_error){
                        case RDF_INSUFFICIENT_DATA:
                            type_error=TOE_WARNING;
                            break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;
                    }
                    break;
                case SUBS_HUMAN_LOCALIZATION:
                    switch(msg.id_error){
                        case HL_INSUFFICIENT_DATA:
                            type_error=TOE_WARNING;
                            break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;
                    }
                    break;
                case SUBS_CONVOY:
                    switch (msg.id_error){
                        case UDP_SOCKET_FAIL:
                            type_error=TOE_WARNING;
                            break;
                        case BIND_SOCKET_FAIL:
                            type_error=TOE_WARNING;
                            break;
                        case SOCKET_RECEIVE_ERROR:
                            type_error=TOE_WARNING;
                            break;
                        case LEADER_CRITICAL_ERROR:
                            type_error=TOE_WARNING;
                            break;
                        case FOLLOWER_GPS_ERROR:
                            type_error=TOE_WARNING;
                            break;
                        case FOLLOWER_LASER_ERROR:
                            type_error=TOE_WARNING;
                            break;
                        case FOLLOWER_NAVIGATION_ERROR:
                            type_error=TOE_WARNING;
                            break;
                        case FOLLOWER_CAMERA_ERROR:
                            type_error=TOE_WARNING;
                            break;
                        default:
                            type_error=TOE_UNAVAILABLE;         // Ha llegado un id_error no contemplado
                            break;
                        }
                    break;
                default:
                    type_error=TOE_UNAVAILABLE;
                    break;     // id_subsystem incorrecto
            }
     return type_error;
}

// Método que define la gravedad del error para el modo TEACH
short mode_teach_error(Common_files::msg_error msg)
{
     short type_error=0;    
     switch (msg.id_subsystem){
                case SUBS_COMMUNICATION:
                    switch (msg.id_error){
                        case (JAUS_CONFIG_ERROR):
                            type_error=TOE_CRITICAL;
                            break;
                        case (CREATE_COMPONENT_ERROR):
                            type_error=TOE_CRITICAL;
                            break;
                        case (RUN_COMPONENT_ERROR):
                            type_error=TOE_CRITICAL;
                            break;
                        case (COMM_LOG_FILE_ERROR):
                            type_error=TOE_WARNING;
                            break;
                        case (COMM_CONFIG_FILE_ERROR):
                            type_error=TOE_CRITICAL;
                            break;
                        case (COMM_CONFIG_FILE_STRUCTURE_ERROR):
                            type_error=TOE_CRITICAL;
                            break;
                        case COMMUNICATION_UCR_FAIL:
                            type_error=TOE_WARNING;
                            break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_SYSTEM_MGMNT:
                    switch (msg.id_error){
                        case (MODE_NOT_AVAILABLE):
                            type_error=TOE_WARNING;
                        break;
                        case FUNCTION_NOT_AVAILABLE:
                            type_error=TOE_WARNING;
                            break;
                        case (COMM_MODULE_NA):
                            type_error=TOE_CRITICAL;
                        break;
                        case (REMOTE_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (DRIVING_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (NAVIGATION_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (CAMERA_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (GPS_MODULE_NA):
                            type_error=TOE_CRITICAL;
                        break;
                        case (FRONT_LASER_1_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (REAR_LASER_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (LASER3D_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (BEACON_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (RDF_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (HL_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (CONVOY_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_REMOTE:
                    switch (msg.id_error){
                        case REMOTE_PARAMETER_OUTRANGE:
                            type_error=TOE_WARNING;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_DRIVING:
                    switch (msg.id_error){
                        case CONNECTION_CAN_FAIL:
                            type_error=TOE_WARNING;
                        break;
                        case COMMUNICATION_CAN_FAIL:
                            type_error=TOE_WARNING;
                        break;
                        case START_STOP_FAILURE:
                            type_error=TOE_WARNING;
                        break;
                        case THROTTLE_FAILURE:
                            type_error=TOE_WARNING;
                        break;
                        case HANDBRAKE_FAILURE:
                            type_error=TOE_WARNING;
                        break;
                        case BRAKE_FAILURE:
                            type_error=TOE_WARNING;
                        break;
                        case GEAR_SHIFT_FAILURE:
                            type_error=TOE_WARNING;
                        break;
                        case STEER_FAILURE:
                            type_error=TOE_WARNING;
                        break;
                        case DIFFERENTIAL_LOCK_FAILURE:
                            type_error=TOE_WARNING;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_NAVIGATION:
                    switch (msg.id_error){
                        case (ERROR_NAVIGATION_OBSTACLE_UNAVOIDABLE):
                            type_error=TOE_WARNING;
                        break;
                        case (ERROR_NAVIGATION_WAYPOINTS_GETTING_TIMEOUT):
                            type_error=TOE_WARNING;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_CAMERA:
                    switch (msg.id_error){
                        break;
                        case (ERROR_CAMERA_COMMUNICATION):
                            type_error=TOE_WARNING;
                        break;
                        case (ERROR_CAMERA_CONFIGURATION):
                            type_error=TOE_WARNING;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_GPS:
                    switch (msg.id_error){
                        case GPS_GLOBAL_ERROR:
                            type_error=TOE_END_ERROR;
                        break;
                        case DATA_RCV_FAILED:
                            type_error=TOE_CRITICAL;
                        break;
                        case CONFIG_OPEN_SERIALPORT_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case CONFIG_CONFIG_SERIALPORT_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case CONFIG_ALIGNMENT_MODE_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case CONFIG_INITIAL_AZIMUT_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case CONFIG_ANTOFFSET_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case INS_INACTIVE:
                            type_error=TOE_CRITICAL;
                        break;
                        case INS_ALIGNING:
                            type_error=TOE_CRITICAL;
                        break;
                        case INS_SOLUTION_NOT_GOOD:
                            type_error=TOE_CRITICAL;
                        break;
                        case INS_BAD_GPS_AGREEMENT:
                            type_error=TOE_CRITICAL;
                        break;
                        case INS_ALIGNMENT_COMPLETE:
                            type_error=TOE_CRITICAL;
                        break;
                        case INSUFFICIENT_OBS:
                            type_error=TOE_CRITICAL;
                        break;
                        case NO_CONVERGENCE:
                            type_error=TOE_CRITICAL;
                        break;
                        case SINGULARITY:
                            type_error=TOE_CRITICAL;
                        break;
                        case COV_TRACE:
                            type_error=TOE_CRITICAL;
                        break;
                        case TEST_DIST:
                            type_error=TOE_CRITICAL;
                        break;
                        case COLD_START:
                            type_error=TOE_CRITICAL;
                        break;
                        case V_H_LIMIT:
                            type_error=TOE_CRITICAL;
                        break;
                        case VARIANCE:
                            type_error=TOE_CRITICAL;
                        break;
                        case RESIDUALS:
                            type_error=TOE_CRITICAL;
                        break;
                        case DELTA_POS:
                            type_error=TOE_CRITICAL;
                        break;
                        case NEGATIVE_VAR:
                            type_error=TOE_CRITICAL;
                        break;
                        case INTEGRITY_WARNING:
                            type_error=TOE_CRITICAL;
                        break;
                        case IMU_UNPLUGGED:
                            type_error=TOE_CRITICAL;
                        break;
                        case PENDING:
                            type_error=TOE_CRITICAL;
                        break;
                        case INVALID_FIX:
                            type_error=TOE_CRITICAL;
                        break;
                        case UNAUTHORIZED_STATE:
                            type_error=TOE_CRITICAL;
                        break;

                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_FRONT_LASER_1:
                    switch (msg.id_error){
                        case LASER_SOCKET_FAIL:
                            type_error=TOE_WARNING;
                        break;
                        case CONNECTION_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case COMM_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_SAMPLE_FREQ:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_ANGLE_RESOLUTION:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_ANGLE_SAMPLE_RESOLUTION:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_SCAN_AREA:
                            type_error=TOE_WARNING;
                        break;
                        case UNKNOWN_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case START_MEASURE_NA:
                            type_error=TOE_WARNING;
                        break;
                        case STOP_MEASURE_NA:
                            type_error=TOE_WARNING;
                        break;
                        case SELECT_USER_LEVEL_NA:
                            type_error=TOE_WARNING;
                        break;
                        case SET_LMS_OUTPUT_NA:
                            type_error=TOE_WARNING;
                        break;
                        case SAVE_DATA_NA:
                            type_error=TOE_WARNING;
                        break;
                        case START_DEVICE_NA:
                            type_error=TOE_WARNING;
                        break;
                        case INCORRECT_ANSWER:
                            type_error=TOE_WARNING;
                        break;
                        case FRAME_OVERFLOW:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_LOG_FILE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_CONFIG_FILE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_CONFIG_FILE_STRUCTURE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_FRONT_LASER_2:
                    switch (msg.id_error){
                        case LASER_SOCKET_FAIL:
                            type_error=TOE_WARNING;
                        break;
                        case CONNECTION_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case COMM_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_SAMPLE_FREQ:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_ANGLE_RESOLUTION:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_ANGLE_SAMPLE_RESOLUTION:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_SCAN_AREA:
                            type_error=TOE_WARNING;
                        break;
                        case UNKNOWN_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case START_MEASURE_NA:
                            type_error=TOE_WARNING;
                        break;
                        case STOP_MEASURE_NA:
                            type_error=TOE_WARNING;
                        break;
                        case SELECT_USER_LEVEL_NA:
                            type_error=TOE_WARNING;
                        break;
                        case SET_LMS_OUTPUT_NA:
                            type_error=TOE_WARNING;
                        break;
                        case SAVE_DATA_NA:
                            type_error=TOE_WARNING;
                        break;
                        case START_DEVICE_NA:
                            type_error=TOE_WARNING;
                        break;
                        case INCORRECT_ANSWER:
                            type_error=TOE_WARNING;
                        break;
                        case FRAME_OVERFLOW:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_LOG_FILE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_CONFIG_FILE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_CONFIG_FILE_STRUCTURE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_REAR_LASER:
                    switch (msg.id_error){
                        case LASER_SOCKET_FAIL:
                            type_error=TOE_WARNING;
                        break;
                        case CONNECTION_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case COMM_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_SAMPLE_FREQ:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_ANGLE_RESOLUTION:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_ANGLE_SAMPLE_RESOLUTION:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_SCAN_AREA:
                            type_error=TOE_WARNING;
                        break;
                        case UNKNOWN_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case START_MEASURE_NA:
                            type_error=TOE_WARNING;
                        break;
                        case STOP_MEASURE_NA:
                            type_error=TOE_WARNING;
                        break;
                        case SELECT_USER_LEVEL_NA:
                            type_error=TOE_WARNING;
                        break;
                        case SET_LMS_OUTPUT_NA:
                            type_error=TOE_WARNING;
                        break;
                        case SAVE_DATA_NA:
                            type_error=TOE_WARNING;
                        break;
                        case START_DEVICE_NA:
                            type_error=TOE_WARNING;
                        break;
                        case INCORRECT_ANSWER:
                            type_error=TOE_WARNING;
                        break;
                        case FRAME_OVERFLOW:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_LOG_FILE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_CONFIG_FILE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_CONFIG_FILE_STRUCTURE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_LASER_3D:
                    switch (msg.id_error){
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_BEACON:
                    switch(msg.id_error){
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;
                    }
                    break;
                case SUBS_RANGE_DATA_FUSION:
                    switch(msg.id_error){
                        case RDF_INSUFFICIENT_DATA:
                            type_error=TOE_WARNING;
                            break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;
                    }
                    break;
                case SUBS_HUMAN_LOCALIZATION:
                    switch(msg.id_error){
                        case HL_INSUFFICIENT_DATA:
                            type_error=TOE_WARNING;
                            break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;
                    }
                    break;
                case SUBS_CONVOY:
                    switch (msg.id_error){
                        case UDP_SOCKET_FAIL:
                            type_error=TOE_WARNING;
                            break;
                        case BIND_SOCKET_FAIL:
                            type_error=TOE_WARNING;
                            break;
                        case SOCKET_RECEIVE_ERROR:
                            type_error=TOE_WARNING;
                            break;
                        case LEADER_CRITICAL_ERROR:
                            type_error=TOE_WARNING;
                            break;
                        case FOLLOWER_GPS_ERROR:
                            type_error=TOE_WARNING;
                            break;
                        case FOLLOWER_LASER_ERROR:
                            type_error=TOE_WARNING;
                            break;
                        case FOLLOWER_NAVIGATION_ERROR:
                            type_error=TOE_WARNING;
                            break;
                        case FOLLOWER_CAMERA_ERROR:
                            type_error=TOE_WARNING;
                            break;
                        default:
                            type_error=TOE_UNAVAILABLE;         // Ha llegado un id_error no contemplado
                            break;
                        }
                    break;
                default:
                    type_error=TOE_UNAVAILABLE;
                    break;     // id_subsystem incorrecto
            }
     return type_error;
}

// Método que define la gravedad del error para el modo MAPPING
short mode_mapping_error(Common_files::msg_error msg)
{
     short type_error=0;    
     switch (msg.id_subsystem){
                case SUBS_COMMUNICATION:
                    switch (msg.id_error){
                        case (JAUS_CONFIG_ERROR):
                            type_error=TOE_CRITICAL;
                            break;
                        case (CREATE_COMPONENT_ERROR):
                            type_error=TOE_CRITICAL;
                            break;
                        case (RUN_COMPONENT_ERROR):
                            type_error=TOE_CRITICAL;
                            break;
                        case (COMM_LOG_FILE_ERROR):
                            type_error=TOE_WARNING;
                            break;
                        case (COMM_CONFIG_FILE_ERROR):
                            type_error=TOE_CRITICAL;
                            break;
                        case (COMM_CONFIG_FILE_STRUCTURE_ERROR):
                            type_error=TOE_CRITICAL;
                            break;
                        case COMMUNICATION_UCR_FAIL:
                            type_error=TOE_WARNING;
                            break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_SYSTEM_MGMNT:
                    switch (msg.id_error){
                        case (MODE_NOT_AVAILABLE):
                            type_error=TOE_WARNING;
                        break;
                        case FUNCTION_NOT_AVAILABLE:
                            type_error=TOE_WARNING;
                            break;
                        case (COMM_MODULE_NA):
                            type_error=TOE_CRITICAL;
                        break;
                        case (REMOTE_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (DRIVING_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (NAVIGATION_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (CAMERA_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (GPS_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (FRONT_LASER_1_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (REAR_LASER_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (LASER3D_MODULE_NA):
                            type_error=TOE_CRITICAL;
                        break;
                        case (BEACON_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (RDF_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (HL_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (CONVOY_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_REMOTE:
                    switch (msg.id_error){
                        case REMOTE_PARAMETER_OUTRANGE:
                            type_error=TOE_WARNING;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_DRIVING:
                    switch (msg.id_error){
                        case CONNECTION_CAN_FAIL:
                            type_error=TOE_WARNING;
                        break;
                        case COMMUNICATION_CAN_FAIL:
                            type_error=TOE_WARNING;
                        break;
                        case START_STOP_FAILURE:
                            type_error=TOE_WARNING;
                        break;
                        case THROTTLE_FAILURE:
                            type_error=TOE_WARNING;
                        break;
                        case HANDBRAKE_FAILURE:
                            type_error=TOE_WARNING;
                        break;
                        case BRAKE_FAILURE:
                            type_error=TOE_WARNING;
                        break;
                        case GEAR_SHIFT_FAILURE:
                            type_error=TOE_WARNING;
                        break;
                        case STEER_FAILURE:
                            type_error=TOE_WARNING;
                        break;
                        case DIFFERENTIAL_LOCK_FAILURE:
                            type_error=TOE_WARNING;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_NAVIGATION:
                    switch (msg.id_error){
                        case (ERROR_NAVIGATION_OBSTACLE_UNAVOIDABLE):
                            type_error=TOE_WARNING;
                        break;
                        case (ERROR_NAVIGATION_WAYPOINTS_GETTING_TIMEOUT):
                            type_error=TOE_WARNING;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_CAMERA:
                    switch (msg.id_error){
                        break;
                        case (ERROR_CAMERA_COMMUNICATION):
                            type_error=TOE_WARNING;
                        break;
                        case (ERROR_CAMERA_CONFIGURATION):
                            type_error=TOE_WARNING;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_GPS:
                    switch (msg.id_error){
                        case GPS_GLOBAL_ERROR:
                            type_error=TOE_END_ERROR;
                        break;
                        case DATA_RCV_FAILED:
                            type_error=TOE_WARNING;
                        break;
                        case CONFIG_OPEN_SERIALPORT_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case CONFIG_CONFIG_SERIALPORT_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case CONFIG_ALIGNMENT_MODE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case CONFIG_INITIAL_AZIMUT_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case CONFIG_ANTOFFSET_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case INS_INACTIVE:
                            type_error=TOE_WARNING;
                        break;
                        case INS_ALIGNING:
                            type_error=TOE_WARNING;
                        break;
                        case INS_SOLUTION_NOT_GOOD:
                            type_error=TOE_WARNING;
                        break;
                        case INS_BAD_GPS_AGREEMENT:
                            type_error=TOE_WARNING;
                        break;
                        case INS_ALIGNMENT_COMPLETE:
                            type_error=TOE_WARNING;
                        break;
                        case INSUFFICIENT_OBS:
                            type_error=TOE_WARNING;
                        break;
                        case NO_CONVERGENCE:
                            type_error=TOE_WARNING;
                        break;
                        case SINGULARITY:
                            type_error=TOE_WARNING;
                        break;
                        case COV_TRACE:
                            type_error=TOE_WARNING;
                        break;
                        case TEST_DIST:
                            type_error=TOE_WARNING;
                        break;
                        case COLD_START:
                            type_error=TOE_WARNING;
                        break;
                        case V_H_LIMIT:
                            type_error=TOE_WARNING;
                        break;
                        case VARIANCE:
                            type_error=TOE_WARNING;
                        break;
                        case RESIDUALS:
                            type_error=TOE_WARNING;
                        break;
                        case DELTA_POS:
                            type_error=TOE_WARNING;
                        break;
                        case NEGATIVE_VAR:
                            type_error=TOE_WARNING;
                        break;
                        case INTEGRITY_WARNING:
                            type_error=TOE_WARNING;
                        break;
                        case IMU_UNPLUGGED:
                            type_error=TOE_WARNING;
                        break;
                        case PENDING:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_FIX:
                            type_error=TOE_WARNING;
                        break;
                        case UNAUTHORIZED_STATE:
                            type_error=TOE_WARNING;
                        break;

                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_FRONT_LASER_1:
                    switch (msg.id_error){
                        case LASER_SOCKET_FAIL:
                            type_error=TOE_WARNING;
                        break;
                        case CONNECTION_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case COMM_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_SAMPLE_FREQ:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_ANGLE_RESOLUTION:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_ANGLE_SAMPLE_RESOLUTION:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_SCAN_AREA:
                            type_error=TOE_WARNING;
                        break;
                        case UNKNOWN_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case START_MEASURE_NA:
                            type_error=TOE_WARNING;
                        break;
                        case STOP_MEASURE_NA:
                            type_error=TOE_WARNING;
                        break;
                        case SELECT_USER_LEVEL_NA:
                            type_error=TOE_WARNING;
                        break;
                        case SET_LMS_OUTPUT_NA:
                            type_error=TOE_WARNING;
                        break;
                        case SAVE_DATA_NA:
                            type_error=TOE_WARNING;
                        break;
                        case START_DEVICE_NA:
                            type_error=TOE_WARNING;
                        break;
                        case INCORRECT_ANSWER:
                            type_error=TOE_WARNING;
                        break;
                        case FRAME_OVERFLOW:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_LOG_FILE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_CONFIG_FILE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_CONFIG_FILE_STRUCTURE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_FRONT_LASER_2:
                    switch (msg.id_error){
                        case LASER_SOCKET_FAIL:
                            type_error=TOE_WARNING;
                        break;
                        case CONNECTION_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case COMM_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_SAMPLE_FREQ:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_ANGLE_RESOLUTION:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_ANGLE_SAMPLE_RESOLUTION:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_SCAN_AREA:
                            type_error=TOE_WARNING;
                        break;
                        case UNKNOWN_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case START_MEASURE_NA:
                            type_error=TOE_WARNING;
                        break;
                        case STOP_MEASURE_NA:
                            type_error=TOE_WARNING;
                        break;
                        case SELECT_USER_LEVEL_NA:
                            type_error=TOE_WARNING;
                        break;
                        case SET_LMS_OUTPUT_NA:
                            type_error=TOE_WARNING;
                        break;
                        case SAVE_DATA_NA:
                            type_error=TOE_WARNING;
                        break;
                        case START_DEVICE_NA:
                            type_error=TOE_WARNING;
                        break;
                        case INCORRECT_ANSWER:
                            type_error=TOE_WARNING;
                        break;
                        case FRAME_OVERFLOW:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_LOG_FILE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_CONFIG_FILE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_CONFIG_FILE_STRUCTURE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_REAR_LASER:
                    switch (msg.id_error){
                        case LASER_SOCKET_FAIL:
                            type_error=TOE_WARNING;
                        break;
                        case CONNECTION_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case COMM_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_SAMPLE_FREQ:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_ANGLE_RESOLUTION:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_ANGLE_SAMPLE_RESOLUTION:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_SCAN_AREA:
                            type_error=TOE_WARNING;
                        break;
                        case UNKNOWN_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case START_MEASURE_NA:
                            type_error=TOE_WARNING;
                        break;
                        case STOP_MEASURE_NA:
                            type_error=TOE_WARNING;
                        break;
                        case SELECT_USER_LEVEL_NA:
                            type_error=TOE_WARNING;
                        break;
                        case SET_LMS_OUTPUT_NA:
                            type_error=TOE_WARNING;
                        break;
                        case SAVE_DATA_NA:
                            type_error=TOE_WARNING;
                        break;
                        case START_DEVICE_NA:
                            type_error=TOE_WARNING;
                        break;
                        case INCORRECT_ANSWER:
                            type_error=TOE_WARNING;
                        break;
                        case FRAME_OVERFLOW:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_LOG_FILE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_CONFIG_FILE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_CONFIG_FILE_STRUCTURE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_LASER_3D:
                    switch (msg.id_error){
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_BEACON:
                    switch(msg.id_error){
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;
                    }
                    break;
                case SUBS_RANGE_DATA_FUSION:
                    switch(msg.id_error){
                        case RDF_INSUFFICIENT_DATA:
                            type_error=TOE_WARNING;
                            break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;
                    }
                    break;
                case SUBS_HUMAN_LOCALIZATION:
                    switch(msg.id_error){
                        case HL_INSUFFICIENT_DATA:
                            type_error=TOE_WARNING;
                            break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;
                    }
                    break;
                case SUBS_CONVOY:
                    switch (msg.id_error){
                        case UDP_SOCKET_FAIL:
                            type_error=TOE_WARNING;
                            break;
                        case BIND_SOCKET_FAIL:
                            type_error=TOE_WARNING;
                            break;
                        case SOCKET_RECEIVE_ERROR:
                            type_error=TOE_WARNING;
                            break;
                        case LEADER_CRITICAL_ERROR:
                            type_error=TOE_WARNING;
                            break;
                        case FOLLOWER_GPS_ERROR:
                            type_error=TOE_WARNING;
                            break;
                        case FOLLOWER_LASER_ERROR:
                            type_error=TOE_WARNING;
                            break;
                        case FOLLOWER_NAVIGATION_ERROR:
                            type_error=TOE_WARNING;
                            break;
                        case FOLLOWER_CAMERA_ERROR:
                            type_error=TOE_WARNING;
                            break;
                        default:
                            type_error=TOE_UNAVAILABLE;         // Ha llegado un id_error no contemplado
                            break;
                        }
                    break;
                default:
                    type_error=TOE_UNAVAILABLE;
                    break;     // id_subsystem incorrecto
            }
     return type_error;
}

// Método que define la gravedad del error para el modo CONVOY
short mode_convoy_error(Common_files::msg_error msg)
{
     short type_error=0;    
     switch (msg.id_subsystem){
                case SUBS_COMMUNICATION:
                    switch (msg.id_error){
                        case (JAUS_CONFIG_ERROR):
                            type_error=TOE_CRITICAL;
                            break;
                        case (CREATE_COMPONENT_ERROR):
                            type_error=TOE_CRITICAL;
                            break;
                        case (RUN_COMPONENT_ERROR):
                            type_error=TOE_CRITICAL;
                            break;
                        case (COMM_LOG_FILE_ERROR):
                            type_error=TOE_WARNING;
                            break;
                        case (COMM_CONFIG_FILE_ERROR):
                            type_error=TOE_CRITICAL;
                            break;
                        case (COMM_CONFIG_FILE_STRUCTURE_ERROR):
                            type_error=TOE_CRITICAL;
                            break;
                        case COMMUNICATION_UCR_FAIL:
                            type_error=TOE_WARNING;
                            break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_SYSTEM_MGMNT:
                    switch (msg.id_error){
                        case (MODE_NOT_AVAILABLE):
                            type_error=TOE_WARNING;
                        break;
                        case FUNCTION_NOT_AVAILABLE:
                            type_error=TOE_WARNING;
                            break;
                        case (COMM_MODULE_NA):
                            type_error=TOE_CRITICAL;
                        break;
                        case (REMOTE_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (DRIVING_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (NAVIGATION_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (CAMERA_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (GPS_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (FRONT_LASER_1_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (REAR_LASER_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (LASER3D_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (BEACON_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (RDF_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (HL_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (CONVOY_MODULE_NA):
                            type_error=TOE_CRITICAL;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_REMOTE:
                    switch (msg.id_error){
                        case REMOTE_PARAMETER_OUTRANGE:
                            type_error=TOE_WARNING;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_DRIVING:
                    switch (msg.id_error){
                        case CONNECTION_CAN_FAIL:
                            type_error=TOE_WARNING;
                        break;
                        case COMMUNICATION_CAN_FAIL:
                            type_error=TOE_WARNING;
                        break;
                        case START_STOP_FAILURE:
                            type_error=TOE_WARNING;
                        break;
                        case THROTTLE_FAILURE:
                            type_error=TOE_WARNING;
                        break;
                        case HANDBRAKE_FAILURE:
                            type_error=TOE_WARNING;
                        break;
                        case BRAKE_FAILURE:
                            type_error=TOE_WARNING;
                        break;
                        case GEAR_SHIFT_FAILURE:
                            type_error=TOE_WARNING;
                        break;
                        case STEER_FAILURE:
                            type_error=TOE_WARNING;
                        break;
                        case DIFFERENTIAL_LOCK_FAILURE:
                            type_error=TOE_WARNING;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_NAVIGATION:
                    switch (msg.id_error){
                        case (ERROR_NAVIGATION_OBSTACLE_UNAVOIDABLE):
                            type_error=TOE_WARNING;
                        break;
                        case (ERROR_NAVIGATION_WAYPOINTS_GETTING_TIMEOUT):
                            type_error=TOE_WARNING;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_CAMERA:
                    switch (msg.id_error){
                        break;
                        case (ERROR_CAMERA_COMMUNICATION):
                            type_error=TOE_WARNING;
                        break;
                        case (ERROR_CAMERA_CONFIGURATION):
                            type_error=TOE_WARNING;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_GPS:
                    switch (msg.id_error){
                        case GPS_GLOBAL_ERROR:
                            type_error=TOE_END_ERROR;
                        break;
                        case DATA_RCV_FAILED:
                            type_error=TOE_WARNING;
                        break;
                        case CONFIG_OPEN_SERIALPORT_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case CONFIG_CONFIG_SERIALPORT_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case CONFIG_ALIGNMENT_MODE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case CONFIG_INITIAL_AZIMUT_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case CONFIG_ANTOFFSET_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case INS_INACTIVE:
                            type_error=TOE_WARNING;
                        break;
                        case INS_ALIGNING:
                            type_error=TOE_WARNING;
                        break;
                        case INS_SOLUTION_NOT_GOOD:
                            type_error=TOE_WARNING;
                        break;
                        case INS_BAD_GPS_AGREEMENT:
                            type_error=TOE_WARNING;
                        break;
                        case INS_ALIGNMENT_COMPLETE:
                            type_error=TOE_WARNING;
                        break;
                        case INSUFFICIENT_OBS:
                            type_error=TOE_WARNING;
                        break;
                        case NO_CONVERGENCE:
                            type_error=TOE_WARNING;
                        break;
                        case SINGULARITY:
                            type_error=TOE_WARNING;
                        break;
                        case COV_TRACE:
                            type_error=TOE_WARNING;
                        break;
                        case TEST_DIST:
                            type_error=TOE_WARNING;
                        break;
                        case COLD_START:
                            type_error=TOE_WARNING;
                        break;
                        case V_H_LIMIT:
                            type_error=TOE_WARNING;
                        break;
                        case VARIANCE:
                            type_error=TOE_WARNING;
                        break;
                        case RESIDUALS:
                            type_error=TOE_WARNING;
                        break;
                        case DELTA_POS:
                            type_error=TOE_WARNING;
                        break;
                        case NEGATIVE_VAR:
                            type_error=TOE_WARNING;
                        break;
                        case INTEGRITY_WARNING:
                            type_error=TOE_WARNING;
                        break;
                        case IMU_UNPLUGGED:
                            type_error=TOE_WARNING;
                        break;
                        case PENDING:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_FIX:
                            type_error=TOE_WARNING;
                        break;
                        case UNAUTHORIZED_STATE:
                            type_error=TOE_WARNING;
                        break;

                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_FRONT_LASER_1:
                    switch (msg.id_error){
                        case LASER_SOCKET_FAIL:
                            type_error=TOE_WARNING;
                        break;
                        case CONNECTION_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case COMM_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_SAMPLE_FREQ:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_ANGLE_RESOLUTION:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_ANGLE_SAMPLE_RESOLUTION:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_SCAN_AREA:
                            type_error=TOE_WARNING;
                        break;
                        case UNKNOWN_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case START_MEASURE_NA:
                            type_error=TOE_WARNING;
                        break;
                        case STOP_MEASURE_NA:
                            type_error=TOE_WARNING;
                        break;
                        case SELECT_USER_LEVEL_NA:
                            type_error=TOE_WARNING;
                        break;
                        case SET_LMS_OUTPUT_NA:
                            type_error=TOE_WARNING;
                        break;
                        case SAVE_DATA_NA:
                            type_error=TOE_WARNING;
                        break;
                        case START_DEVICE_NA:
                            type_error=TOE_WARNING;
                        break;
                        case INCORRECT_ANSWER:
                            type_error=TOE_WARNING;
                        break;
                        case FRAME_OVERFLOW:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_LOG_FILE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_CONFIG_FILE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_CONFIG_FILE_STRUCTURE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_FRONT_LASER_2:
                    switch (msg.id_error){
                        case LASER_SOCKET_FAIL:
                            type_error=TOE_WARNING;
                        break;
                        case CONNECTION_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case COMM_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_SAMPLE_FREQ:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_ANGLE_RESOLUTION:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_ANGLE_SAMPLE_RESOLUTION:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_SCAN_AREA:
                            type_error=TOE_WARNING;
                        break;
                        case UNKNOWN_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case START_MEASURE_NA:
                            type_error=TOE_WARNING;
                        break;
                        case STOP_MEASURE_NA:
                            type_error=TOE_WARNING;
                        break;
                        case SELECT_USER_LEVEL_NA:
                            type_error=TOE_WARNING;
                        break;
                        case SET_LMS_OUTPUT_NA:
                            type_error=TOE_WARNING;
                        break;
                        case SAVE_DATA_NA:
                            type_error=TOE_WARNING;
                        break;
                        case START_DEVICE_NA:
                            type_error=TOE_WARNING;
                        break;
                        case INCORRECT_ANSWER:
                            type_error=TOE_WARNING;
                        break;
                        case FRAME_OVERFLOW:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_LOG_FILE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_CONFIG_FILE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_CONFIG_FILE_STRUCTURE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_REAR_LASER:
                    switch (msg.id_error){
                        case LASER_SOCKET_FAIL:
                            type_error=TOE_WARNING;
                        break;
                        case CONNECTION_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case COMM_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_SAMPLE_FREQ:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_ANGLE_RESOLUTION:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_ANGLE_SAMPLE_RESOLUTION:
                            type_error=TOE_WARNING;
                        break;
                        case INVALID_SCAN_AREA:
                            type_error=TOE_WARNING;
                        break;
                        case UNKNOWN_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case START_MEASURE_NA:
                            type_error=TOE_WARNING;
                        break;
                        case STOP_MEASURE_NA:
                            type_error=TOE_WARNING;
                        break;
                        case SELECT_USER_LEVEL_NA:
                            type_error=TOE_WARNING;
                        break;
                        case SET_LMS_OUTPUT_NA:
                            type_error=TOE_WARNING;
                        break;
                        case SAVE_DATA_NA:
                            type_error=TOE_WARNING;
                        break;
                        case START_DEVICE_NA:
                            type_error=TOE_WARNING;
                        break;
                        case INCORRECT_ANSWER:
                            type_error=TOE_WARNING;
                        break;
                        case FRAME_OVERFLOW:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_LOG_FILE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_CONFIG_FILE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_CONFIG_FILE_STRUCTURE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_LASER_3D:
                    switch (msg.id_error){
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_BEACON:
                    switch(msg.id_error){
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;
                    }
                    break;
                case SUBS_RANGE_DATA_FUSION:
                    switch(msg.id_error){
                        case RDF_INSUFFICIENT_DATA:
                            type_error=TOE_WARNING;
                            break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;
                    }
                    break;
                case SUBS_HUMAN_LOCALIZATION:
                    switch(msg.id_error){
                        case HL_INSUFFICIENT_DATA:
                            type_error=TOE_WARNING;
                            break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;
                    }
                    break;
                case SUBS_CONVOY:
                    switch (msg.id_error){
                        case UDP_SOCKET_FAIL:
                            type_error=TOE_CRITICAL;
                            break;
                        case BIND_SOCKET_FAIL:
                            type_error=TOE_CRITICAL;
                            break;
                        case SOCKET_RECEIVE_ERROR:
                            type_error=TOE_CRITICAL;
                            break;
                        case LEADER_CRITICAL_ERROR:
                            type_error=TOE_CRITICAL;
                            break;
                        case FOLLOWER_GPS_ERROR:
                            type_error=TOE_CRITICAL;
                            break;
                        case FOLLOWER_LASER_ERROR:
                            type_error=TOE_CRITICAL;
                            break;
                        case FOLLOWER_NAVIGATION_ERROR:
                            type_error=TOE_CRITICAL;
                            break;
                        case FOLLOWER_CAMERA_ERROR:
                            type_error=TOE_WARNING;
                            break;
                        default:
                            type_error=TOE_UNAVAILABLE;         // Ha llegado un id_error no contemplado
                            break;
                        }
                    break;
                default:
                    type_error=TOE_UNAVAILABLE;
                    break;     // id_subsystem incorrecto
            }
     return type_error;
}

// Método que define la gravedad del error para el modo CONVOY+TELEOP
short mode_conv_teleop_error(Common_files::msg_error msg)
{
     short type_error=0;    
     switch (msg.id_subsystem){
                case SUBS_COMMUNICATION:
                    switch (msg.id_error){
                        case (JAUS_CONFIG_ERROR):
                            type_error=TOE_CRITICAL;
                            break;
                        case (CREATE_COMPONENT_ERROR):
                            type_error=TOE_CRITICAL;
                            break;
                        case (RUN_COMPONENT_ERROR):
                            type_error=TOE_CRITICAL;
                            break;
                        case (COMM_LOG_FILE_ERROR):
                            type_error=TOE_WARNING;
                            break;
                        case (COMM_CONFIG_FILE_ERROR):
                            type_error=TOE_CRITICAL;
                            break;
                        case (COMM_CONFIG_FILE_STRUCTURE_ERROR):
                            type_error=TOE_CRITICAL;
                            break;
                        case COMMUNICATION_UCR_FAIL:
                            type_error=TOE_WARNING;
                            break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_SYSTEM_MGMNT:
                    switch (msg.id_error){
                        case (MODE_NOT_AVAILABLE):
                            type_error=TOE_WARNING;
                        break;
                        case FUNCTION_NOT_AVAILABLE:
                            type_error=TOE_WARNING;
                            break;
                        case (COMM_MODULE_NA):
                            type_error=TOE_CRITICAL;
                        break;
                        case (REMOTE_MODULE_NA):
                            type_error=TOE_CRITICAL;
                        break;
                        case (DRIVING_MODULE_NA):
                            type_error=TOE_CRITICAL;
                        break;
                        case (NAVIGATION_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (CAMERA_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (GPS_MODULE_NA):
                            type_error=TOE_CRITICAL;
                        break;
                        case (FRONT_LASER_1_MODULE_NA):
                            type_error=TOE_CRITICAL;
                        break;
                        case (REAR_LASER_MODULE_NA):
                            type_error=TOE_CRITICAL;
                        break;
                        case (LASER3D_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (BEACON_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (RDF_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (HL_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (CONVOY_MODULE_NA):
                            type_error=TOE_CRITICAL;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_REMOTE:
                    switch (msg.id_error){
                        case REMOTE_PARAMETER_OUTRANGE:
                            type_error=TOE_WARNING;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_DRIVING:
                    switch (msg.id_error){
                        case CONNECTION_CAN_FAIL:
                            type_error=TOE_CRITICAL;
                        break;
                        case COMMUNICATION_CAN_FAIL:
                            type_error=TOE_CRITICAL;
                        break;
                        case START_STOP_FAILURE:
                            type_error=TOE_CRITICAL;
                        break;
                        case THROTTLE_FAILURE:
                            type_error=TOE_CRITICAL;
                        break;
                        case HANDBRAKE_FAILURE:
                            type_error=TOE_CRITICAL;
                        break;
                        case BRAKE_FAILURE:
                            type_error=TOE_CRITICAL;
                        break;
                        case GEAR_SHIFT_FAILURE:
                            type_error=TOE_CRITICAL;
                        break;
                        case STEER_FAILURE:
                            type_error=TOE_CRITICAL;
                        break;
                        case DIFFERENTIAL_LOCK_FAILURE:
                            type_error=TOE_CRITICAL;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_NAVIGATION:
                    switch (msg.id_error){
                        case (ERROR_NAVIGATION_OBSTACLE_UNAVOIDABLE):
                            type_error=TOE_WARNING;
                        break;
                        case (ERROR_NAVIGATION_WAYPOINTS_GETTING_TIMEOUT):
                            type_error=TOE_WARNING;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_CAMERA:
                    switch (msg.id_error){
                        break;
                        case (ERROR_CAMERA_COMMUNICATION):
                            type_error=TOE_WARNING;
                        break;
                        case (ERROR_CAMERA_CONFIGURATION):
                            type_error=TOE_WARNING;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_GPS:
                    switch (msg.id_error){
                        case GPS_GLOBAL_ERROR:
                            type_error=TOE_END_ERROR;
                        break;
                        case DATA_RCV_FAILED:
                            type_error=TOE_CRITICAL;
                        break;
                        case CONFIG_OPEN_SERIALPORT_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case CONFIG_CONFIG_SERIALPORT_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case CONFIG_ALIGNMENT_MODE_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case CONFIG_INITIAL_AZIMUT_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case CONFIG_ANTOFFSET_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case INS_INACTIVE:
                            type_error=TOE_CRITICAL;
                        break;
                        case INS_ALIGNING:
                            type_error=TOE_CRITICAL;
                        break;
                        case INS_SOLUTION_NOT_GOOD:
                            type_error=TOE_CRITICAL;
                        break;
                        case INS_BAD_GPS_AGREEMENT:
                            type_error=TOE_CRITICAL;
                        break;
                        case INS_ALIGNMENT_COMPLETE:
                            type_error=TOE_CRITICAL;
                        break;
                        case INSUFFICIENT_OBS:
                            type_error=TOE_CRITICAL;
                        break;
                        case NO_CONVERGENCE:
                            type_error=TOE_CRITICAL;
                        break;
                        case SINGULARITY:
                            type_error=TOE_CRITICAL;
                        break;
                        case COV_TRACE:
                            type_error=TOE_CRITICAL;
                        break;
                        case TEST_DIST:
                            type_error=TOE_CRITICAL;
                        break;
                        case COLD_START:
                            type_error=TOE_CRITICAL;
                        break;
                        case V_H_LIMIT:
                            type_error=TOE_CRITICAL;
                        break;
                        case VARIANCE:
                            type_error=TOE_CRITICAL;
                        break;
                        case RESIDUALS:
                            type_error=TOE_CRITICAL;
                        break;
                        case DELTA_POS:
                            type_error=TOE_CRITICAL;
                        break;
                        case NEGATIVE_VAR:
                            type_error=TOE_CRITICAL;
                        break;
                        case INTEGRITY_WARNING:
                            type_error=TOE_CRITICAL;
                        break;
                        case IMU_UNPLUGGED:
                            type_error=TOE_CRITICAL;
                        break;
                        case PENDING:
                            type_error=TOE_CRITICAL;
                        break;
                        case INVALID_FIX:
                            type_error=TOE_CRITICAL;
                        break;
                        case UNAUTHORIZED_STATE:
                            type_error=TOE_CRITICAL;
                        break;

                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_FRONT_LASER_1:
                    switch (msg.id_error){
                        case LASER_SOCKET_FAIL:
                            type_error=TOE_CRITICAL;
                        break;
                        case CONNECTION_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case COMM_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case INVALID_SAMPLE_FREQ:
                            type_error=TOE_CRITICAL;
                        break;
                        case INVALID_ANGLE_RESOLUTION:
                            type_error=TOE_CRITICAL;
                        break;
                        case INVALID_ANGLE_SAMPLE_RESOLUTION:
                            type_error=TOE_CRITICAL;
                        break;
                        case INVALID_SCAN_AREA:
                            type_error=TOE_CRITICAL;
                        break;
                        case UNKNOWN_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case START_MEASURE_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case STOP_MEASURE_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case SELECT_USER_LEVEL_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case SET_LMS_OUTPUT_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case SAVE_DATA_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case START_DEVICE_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case INCORRECT_ANSWER:
                            type_error=TOE_CRITICAL;
                        break;
                        case FRAME_OVERFLOW:
                            type_error=TOE_CRITICAL;
                        break;
                        case LASER_LOG_FILE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_CONFIG_FILE_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case LASER_CONFIG_FILE_STRUCTURE_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_FRONT_LASER_2:
                    switch (msg.id_error){
                        case LASER_SOCKET_FAIL:
                            type_error=TOE_CRITICAL;
                        break;
                        case CONNECTION_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case COMM_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case INVALID_SAMPLE_FREQ:
                            type_error=TOE_CRITICAL;
                        break;
                        case INVALID_ANGLE_RESOLUTION:
                            type_error=TOE_CRITICAL;
                        break;
                        case INVALID_ANGLE_SAMPLE_RESOLUTION:
                            type_error=TOE_CRITICAL;
                        break;
                        case INVALID_SCAN_AREA:
                            type_error=TOE_CRITICAL;
                        break;
                        case UNKNOWN_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case START_MEASURE_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case STOP_MEASURE_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case SELECT_USER_LEVEL_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case SET_LMS_OUTPUT_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case SAVE_DATA_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case START_DEVICE_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case INCORRECT_ANSWER:
                            type_error=TOE_CRITICAL;
                        break;
                        case FRAME_OVERFLOW:
                            type_error=TOE_CRITICAL;
                        break;
                        case LASER_LOG_FILE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_CONFIG_FILE_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case LASER_CONFIG_FILE_STRUCTURE_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_REAR_LASER:
                    switch (msg.id_error){
                        case LASER_SOCKET_FAIL:
                            type_error=TOE_CRITICAL;
                        break;
                        case CONNECTION_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case COMM_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case INVALID_SAMPLE_FREQ:
                            type_error=TOE_CRITICAL;
                        break;
                        case INVALID_ANGLE_RESOLUTION:
                            type_error=TOE_CRITICAL;
                        break;
                        case INVALID_ANGLE_SAMPLE_RESOLUTION:
                            type_error=TOE_CRITICAL;
                        break;
                        case INVALID_SCAN_AREA:
                            type_error=TOE_CRITICAL;
                        break;
                        case UNKNOWN_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case START_MEASURE_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case STOP_MEASURE_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case SELECT_USER_LEVEL_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case SET_LMS_OUTPUT_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case SAVE_DATA_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case START_DEVICE_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case INCORRECT_ANSWER:
                            type_error=TOE_CRITICAL;
                        break;
                        case FRAME_OVERFLOW:
                            type_error=TOE_CRITICAL;
                        break;
                        case LASER_LOG_FILE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_CONFIG_FILE_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case LASER_CONFIG_FILE_STRUCTURE_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_LASER_3D:
                    switch (msg.id_error){
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_BEACON:
                    switch(msg.id_error){
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;
                    }
                    break;
                case SUBS_RANGE_DATA_FUSION:
                    switch(msg.id_error){
                        case RDF_INSUFFICIENT_DATA:
                            type_error=TOE_WARNING;
                            break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;
                    }
                    break;
                case SUBS_HUMAN_LOCALIZATION:
                    switch(msg.id_error){
                        case HL_INSUFFICIENT_DATA:
                            type_error=TOE_WARNING;
                            break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;
                    }
                    break;
                case SUBS_CONVOY:
                    switch (msg.id_error){
                        case UDP_SOCKET_FAIL:
                            type_error=TOE_CRITICAL;
                            break;
                        case BIND_SOCKET_FAIL:
                            type_error=TOE_CRITICAL;
                            break;
                        case SOCKET_RECEIVE_ERROR:
                            type_error=TOE_CRITICAL;
                            break;
                        case LEADER_CRITICAL_ERROR:
                            type_error=TOE_CRITICAL;
                            break;
                        case FOLLOWER_GPS_ERROR:
                            type_error=TOE_CRITICAL;
                            break;
                        case FOLLOWER_LASER_ERROR:
                            type_error=TOE_CRITICAL;
                            break;
                        case FOLLOWER_NAVIGATION_ERROR:
                            type_error=TOE_CRITICAL;
                            break;
                        case FOLLOWER_CAMERA_ERROR:
                            type_error=TOE_WARNING;
                            break;
                        default:
                            type_error=TOE_UNAVAILABLE;         // Ha llegado un id_error no contemplado
                            break;
                        }
                    break;
                default:
                    type_error=TOE_UNAVAILABLE;
                    break;     // id_subsystem incorrecto
            }
     return type_error;
}

// Método que define la gravedad del error para el modo CONVOY+AUTO
short mode_conv_auto_error(Common_files::msg_error msg)
{
     short type_error=0;    
     switch (msg.id_subsystem){
                case SUBS_COMMUNICATION:
                    switch (msg.id_error){
                        case (JAUS_CONFIG_ERROR):
                            type_error=TOE_CRITICAL;
                            break;
                        case (CREATE_COMPONENT_ERROR):
                            type_error=TOE_CRITICAL;
                            break;
                        case (RUN_COMPONENT_ERROR):
                            type_error=TOE_CRITICAL;
                            break;
                        case (COMM_LOG_FILE_ERROR):
                            type_error=TOE_WARNING;
                            break;
                        case (COMM_CONFIG_FILE_ERROR):
                            type_error=TOE_CRITICAL;
                            break;
                        case (COMM_CONFIG_FILE_STRUCTURE_ERROR):
                            type_error=TOE_CRITICAL;
                            break;
                        case COMMUNICATION_UCR_FAIL:
                            type_error=TOE_WARNING;
                            break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_SYSTEM_MGMNT:
                    switch (msg.id_error){
                        case (MODE_NOT_AVAILABLE):
                            type_error=TOE_WARNING;
                        break;
                        case FUNCTION_NOT_AVAILABLE:
                            type_error=TOE_WARNING;
                            break;
                        case (COMM_MODULE_NA):
                            type_error=TOE_CRITICAL;
                        break;
                        case (REMOTE_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (DRIVING_MODULE_NA):
                            type_error=TOE_CRITICAL;
                        break;
                        case (NAVIGATION_MODULE_NA):
                            type_error=TOE_CRITICAL;
                        break;
                        case (CAMERA_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (GPS_MODULE_NA):
                            type_error=TOE_CRITICAL;
                        break;
                        case (FRONT_LASER_1_MODULE_NA):
                            type_error=TOE_CRITICAL;
                        break;
                        case (REAR_LASER_MODULE_NA):
                            type_error=TOE_CRITICAL;
                        break;
                        case (LASER3D_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (BEACON_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (RDF_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (HL_MODULE_NA):
                            type_error=TOE_WARNING;
                        break;
                        case (CONVOY_MODULE_NA):
                            type_error=TOE_CRITICAL;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_REMOTE:
                    switch (msg.id_error){
                        case REMOTE_PARAMETER_OUTRANGE:
                            type_error=TOE_WARNING;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_DRIVING:
                    switch (msg.id_error){
                        case CONNECTION_CAN_FAIL:
                            type_error=TOE_CRITICAL;
                        break;
                        case COMMUNICATION_CAN_FAIL:
                            type_error=TOE_CRITICAL;
                        break;
                        case START_STOP_FAILURE:
                            type_error=TOE_CRITICAL;
                        break;
                        case THROTTLE_FAILURE:
                            type_error=TOE_CRITICAL;
                        break;
                        case HANDBRAKE_FAILURE:
                            type_error=TOE_CRITICAL;
                        break;
                        case BRAKE_FAILURE:
                            type_error=TOE_CRITICAL;
                        break;
                        case GEAR_SHIFT_FAILURE:
                            type_error=TOE_CRITICAL;
                        break;
                        case STEER_FAILURE:
                            type_error=TOE_CRITICAL;
                        break;
                        case DIFFERENTIAL_LOCK_FAILURE:
                            type_error=TOE_CRITICAL;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_NAVIGATION:
                    switch (msg.id_error){
                        case (ERROR_NAVIGATION_OBSTACLE_UNAVOIDABLE):
                            type_error=TOE_WARNING;
                        break;
                        case (ERROR_NAVIGATION_WAYPOINTS_GETTING_TIMEOUT):
                            type_error=TOE_WARNING;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_CAMERA:
                    switch (msg.id_error){
                        break;
                        case (ERROR_CAMERA_COMMUNICATION):
                            type_error=TOE_WARNING;
                        break;
                        case (ERROR_CAMERA_CONFIGURATION):
                            type_error=TOE_WARNING;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_GPS:
                    switch (msg.id_error){
                        case GPS_GLOBAL_ERROR:
                            type_error=TOE_END_ERROR;
                        break;
                        case DATA_RCV_FAILED:
                            type_error=TOE_CRITICAL;
                        break;
                        case CONFIG_OPEN_SERIALPORT_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case CONFIG_CONFIG_SERIALPORT_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case CONFIG_ALIGNMENT_MODE_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case CONFIG_INITIAL_AZIMUT_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case CONFIG_ANTOFFSET_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case INS_INACTIVE:
                            type_error=TOE_CRITICAL;
                        break;
                        case INS_ALIGNING:
                            type_error=TOE_CRITICAL;
                        break;
                        case INS_SOLUTION_NOT_GOOD:
                            type_error=TOE_CRITICAL;
                        break;
                        case INS_BAD_GPS_AGREEMENT:
                            type_error=TOE_CRITICAL;
                        break;
                        case INS_ALIGNMENT_COMPLETE:
                            type_error=TOE_CRITICAL;
                        break;
                        case INSUFFICIENT_OBS:
                            type_error=TOE_CRITICAL;
                        break;
                        case NO_CONVERGENCE:
                            type_error=TOE_CRITICAL;
                        break;
                        case SINGULARITY:
                            type_error=TOE_CRITICAL;
                        break;
                        case COV_TRACE:
                            type_error=TOE_CRITICAL;
                        break;
                        case TEST_DIST:
                            type_error=TOE_CRITICAL;
                        break;
                        case COLD_START:
                            type_error=TOE_CRITICAL;
                        break;
                        case V_H_LIMIT:
                            type_error=TOE_CRITICAL;
                        break;
                        case VARIANCE:
                            type_error=TOE_CRITICAL;
                        break;
                        case RESIDUALS:
                            type_error=TOE_CRITICAL;
                        break;
                        case DELTA_POS:
                            type_error=TOE_CRITICAL;
                        break;
                        case NEGATIVE_VAR:
                            type_error=TOE_CRITICAL;
                        break;
                        case INTEGRITY_WARNING:
                            type_error=TOE_CRITICAL;
                        break;
                        case IMU_UNPLUGGED:
                            type_error=TOE_CRITICAL;
                        break;
                        case PENDING:
                            type_error=TOE_CRITICAL;
                        break;
                        case INVALID_FIX:
                            type_error=TOE_CRITICAL;
                        break;
                        case UNAUTHORIZED_STATE:
                            type_error=TOE_CRITICAL;
                        break;

                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_FRONT_LASER_1:
                    switch (msg.id_error){
                        case LASER_SOCKET_FAIL:
                            type_error=TOE_CRITICAL;
                        break;
                        case CONNECTION_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case COMM_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case INVALID_SAMPLE_FREQ:
                            type_error=TOE_CRITICAL;
                        break;
                        case INVALID_ANGLE_RESOLUTION:
                            type_error=TOE_CRITICAL;
                        break;
                        case INVALID_ANGLE_SAMPLE_RESOLUTION:
                            type_error=TOE_CRITICAL;
                        break;
                        case INVALID_SCAN_AREA:
                            type_error=TOE_CRITICAL;
                        break;
                        case UNKNOWN_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case START_MEASURE_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case STOP_MEASURE_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case SELECT_USER_LEVEL_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case SET_LMS_OUTPUT_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case SAVE_DATA_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case START_DEVICE_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case INCORRECT_ANSWER:
                            type_error=TOE_CRITICAL;
                        break;
                        case FRAME_OVERFLOW:
                            type_error=TOE_CRITICAL;
                        break;
                        case LASER_LOG_FILE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_CONFIG_FILE_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case LASER_CONFIG_FILE_STRUCTURE_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_FRONT_LASER_2:
                    switch (msg.id_error){
                        case LASER_SOCKET_FAIL:
                            type_error=TOE_CRITICAL;
                        break;
                        case CONNECTION_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case COMM_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case INVALID_SAMPLE_FREQ:
                            type_error=TOE_CRITICAL;
                        break;
                        case INVALID_ANGLE_RESOLUTION:
                            type_error=TOE_CRITICAL;
                        break;
                        case INVALID_ANGLE_SAMPLE_RESOLUTION:
                            type_error=TOE_CRITICAL;
                        break;
                        case INVALID_SCAN_AREA:
                            type_error=TOE_CRITICAL;
                        break;
                        case UNKNOWN_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case START_MEASURE_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case STOP_MEASURE_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case SELECT_USER_LEVEL_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case SET_LMS_OUTPUT_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case SAVE_DATA_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case START_DEVICE_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case INCORRECT_ANSWER:
                            type_error=TOE_CRITICAL;
                        break;
                        case FRAME_OVERFLOW:
                            type_error=TOE_CRITICAL;
                        break;
                        case LASER_LOG_FILE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_CONFIG_FILE_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case LASER_CONFIG_FILE_STRUCTURE_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_REAR_LASER:
                    switch (msg.id_error){
                        case LASER_SOCKET_FAIL:
                            type_error=TOE_CRITICAL;
                        break;
                        case CONNECTION_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case COMM_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case INVALID_SAMPLE_FREQ:
                            type_error=TOE_CRITICAL;
                        break;
                        case INVALID_ANGLE_RESOLUTION:
                            type_error=TOE_CRITICAL;
                        break;
                        case INVALID_ANGLE_SAMPLE_RESOLUTION:
                            type_error=TOE_CRITICAL;
                        break;
                        case INVALID_SCAN_AREA:
                            type_error=TOE_CRITICAL;
                        break;
                        case UNKNOWN_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case START_MEASURE_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case STOP_MEASURE_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case SELECT_USER_LEVEL_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case SET_LMS_OUTPUT_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case SAVE_DATA_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case START_DEVICE_NA:
                            type_error=TOE_CRITICAL;
                        break;
                        case INCORRECT_ANSWER:
                            type_error=TOE_CRITICAL;
                        break;
                        case FRAME_OVERFLOW:
                            type_error=TOE_CRITICAL;
                        break;
                        case LASER_LOG_FILE_ERROR:
                            type_error=TOE_WARNING;
                        break;
                        case LASER_CONFIG_FILE_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        case LASER_CONFIG_FILE_STRUCTURE_ERROR:
                            type_error=TOE_CRITICAL;
                        break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_LASER_3D:
                    switch (msg.id_error){
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;     // Ha llegado un id_error no contemplado
                    }
                break;
                case SUBS_BEACON:
                    switch(msg.id_error){
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;
                    }
                    break;
                case SUBS_RANGE_DATA_FUSION:
                    switch(msg.id_error){
                        case RDF_INSUFFICIENT_DATA:
                            type_error=TOE_WARNING;
                            break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;
                    }
                    break;
                case SUBS_HUMAN_LOCALIZATION:
                    switch(msg.id_error){
                        case HL_INSUFFICIENT_DATA:
                            type_error=TOE_WARNING;
                            break;
                        default:
                            type_error=TOE_UNAVAILABLE;
                            break;
                    }
                    break;
                case SUBS_CONVOY:
                    switch (msg.id_error){
                        case UDP_SOCKET_FAIL:
                            type_error=TOE_CRITICAL;
                            break;
                        case BIND_SOCKET_FAIL:
                            type_error=TOE_CRITICAL;
                            break;
                        case SOCKET_RECEIVE_ERROR:
                            type_error=TOE_CRITICAL;
                            break;
                        case LEADER_CRITICAL_ERROR:
                            type_error=TOE_CRITICAL;
                            break;
                        case FOLLOWER_GPS_ERROR:
                            type_error=TOE_CRITICAL;
                            break;
                        case FOLLOWER_LASER_ERROR:
                            type_error=TOE_CRITICAL;
                            break;
                        case FOLLOWER_NAVIGATION_ERROR:
                            type_error=TOE_CRITICAL;
                            break;
                        case FOLLOWER_CAMERA_ERROR:
                            type_error=TOE_WARNING;
                            break;
                        default:
                            type_error=TOE_UNAVAILABLE;         // Ha llegado un id_error no contemplado
                            break;
                        }
                    break;
                default:
                    type_error=TOE_UNAVAILABLE;
                    break;     // id_subsystem incorrecto
            }
     return type_error;
}

// Cambia a modo neutro
void switchNeutral(){
    // Envia msg_mode con estado EXIT cuando llega confirmacion de DRIVING
    Common_files::msg_mode msg_ch_neutral;
    msg_ch_neutral.mode=currentMode;
    msg_ch_neutral.status=MODE_EXIT;
    msg_ch_neutral.type_msg=SET;
    pub_mode.publish(msg_ch_neutral);
}

// Convierte los errores de entrada en sus correspondientes errores de salida
int convertOutputError(Common_files::msg_error msg){
    int out_error = 0;
    switch (msg.id_subsystem){
        case SUBS_COMMUNICATION:
            switch (msg.id_error){
                case JAUS_CONFIG_ERROR:
                case CREATE_COMPONENT_ERROR:
                case RUN_COMPONENT_ERROR:
                    out_error = JAUS_ERROR;
                    break;
                case COMM_LOG_FILE_ERROR:
                case COMM_CONFIG_FILE_ERROR:
                case COMM_CONFIG_FILE_STRUCTURE_ERROR:
                    out_error = COMM_FILE_ERROR;
                    break;
                case COMMUNICATION_UCR_FAIL:
                    out_error = LINK_ERROR_OUTPUT;
                    break;
                default:
                    cout << "Error indefinido" << endl;
                    break;
            }
            break;
        case SUBS_SYSTEM_MGMNT:
            switch (msg.id_error){
                case MODE_NOT_AVAILABLE:
                case FUNCTION_NOT_AVAILABLE:
                    out_error = MODE_OR_FUNCTION_NA;
                    break;
                case COMM_MODULE_NA:
                case REMOTE_MODULE_NA:
                case DRIVING_MODULE_NA:
                case NAVIGATION_MODULE_NA:
                case CAMERA_MODULE_NA:
                case GPS_MODULE_NA:
                case FRONT_LASER_1_MODULE_NA:
                case FRONT_LASER_2_MODULE_NA:
                case REAR_LASER_MODULE_NA:
                case LASER3D_MODULE_NA:
                case BEACON_MODULE_NA:
                case RDF_MODULE_NA:
                case HL_MODULE_NA:
                case CONVOY_MODULE_NA:
                    out_error = MODULE_NOT_AVAILABLE;
                    break;
                default:
                    cout << "Error indefinido" << endl;
                    break;
            }
            break;
        case SUBS_REMOTE:
            switch (msg.id_error){
                case REMOTE_PARAMETER_OUTRANGE:
                    out_error = REMOTE_PARAMETER_ERROR;
                    break;
                default:
                    cout << "Error indefinido" << endl;
                    break;
            }
            break;
        case SUBS_DRIVING:
            switch (msg.id_error){
                case CONNECTION_CAN_FAIL:
                case COMMUNICATION_CAN_FAIL:
                    out_error = CAN_FAILURE;
                    break;
                case START_STOP_FAILURE:
                case THROTTLE_FAILURE:
                case HANDBRAKE_FAILURE:
                case BRAKE_FAILURE:
                case GEAR_SHIFT_FAILURE:
                case STEER_FAILURE:
                case DIFFERENTIAL_LOCK_FAILURE:
                    out_error = MECHANICAL_FAILURE;
                    break;
                default:
                    cout << "Error indefinido" << endl;
                    break;
            }
            break;
        case SUBS_NAVIGATION:
            switch (msg.id_error){
                case ERROR_NAVIGATION_WAYPOINTS_GETTING_TIMEOUT:
                case ERROR_NAVIGATION_OBSTACLE_UNAVOIDABLE:
                    out_error = NAV_ERROR;
                    break;
                default:
                    cout << "Error indefinido" << endl;
                    break;
            }
            break;
        case SUBS_CAMERA:
            switch (msg.id_error){
                case ERROR_CAMERA_COMMUNICATION:
                case ERROR_CAMERA_CONFIGURATION:
                    out_error = CAMERA_ERROR;
                    break;
                default:
                    cout << "Error indefinido" << endl;
                    break;
            }
            break;
        case SUBS_GPS:
            switch (msg.id_error){
                case DATA_RCV_FAILED:
                    out_error = DATA_RCV_FAILED_OUT;
                    break;
                case CONFIG_OPEN_SERIALPORT_ERROR:
                case CONFIG_CONFIG_SERIALPORT_ERROR:
                    out_error = SERIALPORT_ERROR;
                    break;
                case CONFIG_ALIGNMENT_MODE_ERROR:
                case CONFIG_INITIAL_AZIMUT_ERROR:
                case CONFIG_ANTOFFSET_ERROR:
                    out_error = GPS_CONFIG_ERROR;
                    break;
                case INS_INACTIVE:
                case INS_ALIGNING:
                case INS_SOLUTION_NOT_GOOD:
                case INS_BAD_GPS_AGREEMENT:
                case INS_ALIGNMENT_COMPLETE:
                    out_error = IMU_ERROR;
                    break;
                case INSUFFICIENT_OBS:
                case NO_CONVERGENCE:
                case SINGULARITY:
                case COV_TRACE:
                case TEST_DIST:
                case COLD_START:
                case V_H_LIMIT:
                case VARIANCE:
                case RESIDUALS:
                case DELTA_POS:
                case NEGATIVE_VAR:
                case INTEGRITY_WARNING:
                case IMU_UNPLUGGED:
                case PENDING:
                case INVALID_FIX:
                case UNAUTHORIZED_STATE:
                    out_error = GPS_RECEIVER_ERROR;
                    break;
                default:
                    cout << "Error indefinido" << endl;
                    break;
            }
            break;
        case SUBS_FRONT_LASER_1:
            switch (msg.id_error){
                case LASER_SOCKET_FAIL:
                case CONNECTION_ERROR:
                    out_error = LASER_CONNECTION_ERROR;
                    break;
                case COMM_ERROR:
                    out_error = LASER_COMMUNICATION_ERROR;
                    break;
                case INVALID_SAMPLE_FREQ:
                case INVALID_ANGLE_RESOLUTION:
                case INVALID_ANGLE_SAMPLE_RESOLUTION:
                case INVALID_SCAN_AREA:
                case UNKNOWN_ERROR:
                    out_error = LASER_CONFIGURATION_ERROR;
                    break;
                case START_MEASURE_NA:
                case STOP_MEASURE_NA:
                case SELECT_USER_LEVEL_NA:
                case SET_LMS_OUTPUT_NA:
                case SAVE_DATA_NA:
                case START_DEVICE_NA:
                    out_error = LASER_COMMAND_ERROR;
                    break;
                case INCORRECT_ANSWER:
                case FRAME_OVERFLOW:
                    out_error = LASER_FRAME_ERROR;
                    break;
                case LASER_LOG_FILE_ERROR:
                case LASER_CONFIG_FILE_ERROR:
                case LASER_CONFIG_FILE_STRUCTURE_ERROR:
                    out_error = LASER_FILE_ERROR;
                    break;
                default:
                    cout << "Error indefinido" << endl;
                    break;
            }
            break;
        case SUBS_FRONT_LASER_2:
            switch (msg.id_error){
                case LASER_SOCKET_FAIL:
                case CONNECTION_ERROR:
                    out_error = LASER_CONNECTION_ERROR;
                    break;
                case COMM_ERROR:
                    out_error = LASER_COMMUNICATION_ERROR;
                    break;
                case INVALID_SAMPLE_FREQ:
                case INVALID_ANGLE_RESOLUTION:
                case INVALID_ANGLE_SAMPLE_RESOLUTION:
                case INVALID_SCAN_AREA:
                case UNKNOWN_ERROR:
                    out_error = LASER_CONFIGURATION_ERROR;
                    break;
                case START_MEASURE_NA:
                case STOP_MEASURE_NA:
                case SELECT_USER_LEVEL_NA:
                case SET_LMS_OUTPUT_NA:
                case SAVE_DATA_NA:
                case START_DEVICE_NA:
                    out_error = LASER_COMMAND_ERROR;
                    break;
                case INCORRECT_ANSWER:
                case FRAME_OVERFLOW:
                    out_error = LASER_FRAME_ERROR;
                    break;
                case LASER_LOG_FILE_ERROR:
                case LASER_CONFIG_FILE_ERROR:
                case LASER_CONFIG_FILE_STRUCTURE_ERROR:
                    out_error = LASER_FILE_ERROR;
                    break;
                default:
                    cout << "Error indefinido" << endl;
                    break;
            }
            break;
        case SUBS_REAR_LASER:
            switch (msg.id_error){
                case LASER_SOCKET_FAIL:
                case CONNECTION_ERROR:
                    out_error = LASER_CONNECTION_ERROR;
                    break;
                case COMM_ERROR:
                    out_error = LASER_COMMUNICATION_ERROR;
                    break;
                case INVALID_SAMPLE_FREQ:
                case INVALID_ANGLE_RESOLUTION:
                case INVALID_ANGLE_SAMPLE_RESOLUTION:
                case INVALID_SCAN_AREA:
                case UNKNOWN_ERROR:
                    out_error = LASER_CONFIGURATION_ERROR;
                    break;
                case START_MEASURE_NA:
                case STOP_MEASURE_NA:
                case SELECT_USER_LEVEL_NA:
                case SET_LMS_OUTPUT_NA:
                case SAVE_DATA_NA:
                case START_DEVICE_NA:
                    out_error = LASER_COMMAND_ERROR;
                    break;
                case INCORRECT_ANSWER:
                case FRAME_OVERFLOW:
                    out_error = LASER_FRAME_ERROR;
                    break;
                case LASER_LOG_FILE_ERROR:
                case LASER_CONFIG_FILE_ERROR:
                case LASER_CONFIG_FILE_STRUCTURE_ERROR:
                    out_error = LASER_FILE_ERROR;
                    break;
                default:
                    cout << "Error indefinido" << endl;
                    break;
            }
            break;
        case SUBS_LASER_3D:
            switch (msg.id_error){
                
            }
            break;
        case SUBS_BEACON:
            switch (msg.id_error){
                
            }
            break;
        case SUBS_RANGE_DATA_FUSION:
            switch (msg.id_error){
                case RDF_INSUFFICIENT_DATA:
                    out_error = RDF_INSUFFICIENT_DATA_OUT;
                    break;
                default:
                    cout << "Error indefinido" << endl;
                    break;                    
            }
            break;
        case SUBS_HUMAN_LOCALIZATION:
            switch (msg.id_error){
                case HL_INSUFFICIENT_DATA:
                    out_error = HL_INSUFFICIENT_DATA_OUT;
                    break;
                default:
                    cout << "Error indefinido" << endl;
                    break;                    
            }
            break;
        case SUBS_CONVOY:
            switch (msg.id_error){
                case UDP_SOCKET_FAIL:
                case BIND_SOCKET_FAIL:
                    out_error = CONVOY_CONNECTION_ERROR;
                    break;
                case SOCKET_RECEIVE_ERROR:
                    out_error = CONVOY_SOCKET_RECEIVE_ERROR;
                    break;
                case LEADER_CRITICAL_ERROR:
                    out_error = LEADER_CRITICAL_ERROR_OUTPUT;
                    break;
                case FOLLOWER_GPS_ERROR:
                case FOLLOWER_LASER_ERROR:
                case FOLLOWER_NAVIGATION_ERROR:
                case FOLLOWER_CAMERA_ERROR:
                    out_error = FOLLOWER_ERROR;
                    break;
                default:
                    cout << "Error indefinido" << endl;
                    break;
            }
            break;
        default:
            cout << "Modulo indefinido" << endl;
            break;            
    }
    return out_error;
}

// Funcion para escribir en el fichero log
void writeToLog(Common_files::msg_error msg) {
    struct timeval timeLog;
    gettimeofday(&timeLog, NULL);
    flog << timeLog.tv_sec % 18000 << "." << timeLog.tv_usec << "\n";
    flog << msg << endl;
}

// Actualiza la tabla de modos disponibles ante la llegada de un mensaje de error
bool updateModeAvailable (Common_files::msg_error msg,bool originalTable[13]){
    int i=0;
    short type_error;
    for (i=MODE_REMOTE; i<=MODE_TEACH; i++){
        type_error=isWarningOrCritical(msg,i);                  // Comprueba si el error recibido es CRIT o WARN para cada modo
        if (type_error == TOE_CRITICAL){
            numErrorMode[i][msg.id_subsystem][msg.id_error]++;
            avail_mode.available[i] = false;                    // ACTUALIZA CAMPO AVAILABLE_MODE
        }
        /******* Ordenes para debug parte available mode *******/
        int summ=0,j,k;
        for(j=0;j<MAX_MODULES;j++){
            for(k=0;k<MAX_ERRORS;k++)
                summ+=numErrorMode[i][j][k];
        }
        //cout << "Numeros de errores en modo " << i << " es igual a " << summ << endl;
        //cout << "Modo disponible con ID " << i << " es " << (short) avail_mode.available[i] << endl;
        /*******************************************************/
    }
    return compareTable(originalTable,avail_mode);      // La función devuelve false si ha habido cambios en la tabla de modos disponibles
}

// Actualiza la tabla de modos disponibles ante la llegada de un mensaje fin de error
bool updateEndError(Common_files::msg_error msg, bool originalTable[13]){
    short type_error;
    int i,j,summ=0;
        if ((msg.id_subsystem==SUBS_GPS) && (msg.id_error==GPS_GLOBAL_ERROR)){          // Probando, no puedo comparar solo id_error
            for(i=MODE_REMOTE;i<=MODE_TEACH;i++){
                for(j=0;j<MAX_ERRORS;j++){
                    numErrorMode[i][SUBS_GPS][j]=0;             // Este error provoca que la columna del módulo GPS se reseteen a cero
                }
                summ=checkErrorTable(i);
                if (summ == 0)                                  // Si no es cero, el modo sigue sin estar disponible 
                    avail_mode.available[i] = true;
            }
        }
        else{
            for (i=MODE_REMOTE; i<=MODE_TEACH; i++) {
                type_error = isWarningOrCritical(msg, i); // Comprueba para cada modo si el error finalizado es critico o warning
                if ((type_error == TOE_CRITICAL) && (numErrorMode[i][msg.id_subsystem][msg.id_error] > 0)) {
                    numErrorMode[i][msg.id_subsystem][msg.id_error]--;
                    summ=checkErrorTable(i);
                }
                if (summ == 0)                            // Si no es cero, el modo sigue sin estar disponible
                    avail_mode.available[i] = true; 
            }
        }
    summ=0;
    return compareTable(originalTable,avail_mode);      // La función devuelve false si ha habido cambios en la tabla de modos disponibles
}

// Comprueba el número de errores de la tabla de errores global
int checkErrorTable(int mode)
{
    int i,j;
    int summ=0;
    for (i=0;i<MAX_MODULES;i++){
        for(j=0;j<MAX_ERRORS;j++){
            summ+=numErrorMode[mode][i][j];
        }
    }
    return summ;
    /******* Pruebas modo debug **********/
    //cout << "Numeros de errores en modo " << mode << " es igual a " << summ << endl;
    //cout << "Modo disponible con ID " << mode << " es " << (short) avail_mode.available[mode] << endl;
}

// Compara las tablas de modos disponibles para publicar o no
bool compareTable(bool original[13],Common_files::msg_available msg)
{
    for (int i=1;i<13;i++){
        if (original[i]!=msg.available[i]){
            return false;
        }
    }
    return true;
}