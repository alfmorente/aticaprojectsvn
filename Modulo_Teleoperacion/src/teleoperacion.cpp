#include "../include/Modulo_Teleoperacion/teleoperacion.h"

  ros::Publisher pub_error;
  ros::Publisher pub_teleop;

  using namespace std;

int main(int argc, char **argv)
{
  // Obtencion del modo de operacion y comprobacion de que es correcto
  int operationMode;
  if ((operationMode = getOperationMode(argc, argv)) == 0) {
      return 1;
  }

  // Orden para la parada manual con CTtrl+C
  //init_signals();
    
  // Inicio de ROS
  ros::init(argc, argv, "teleoperacion");
  
  // Manejador ROS
  ros::NodeHandle n;
  
  //int estado_actual=STATE_OK;
  // Espera activa de inicio de modulo
  int estado_actual=STATE_OFF;
  while(estado_actual!=STATE_CONF){
          n.getParam("state_module_remote",estado_actual);
  }
  cout << "Atica TELEOPERACION :: Iniciando configuración..." << endl;

  sleep(1);

  // Todo esta correcto, lo especificamos con el correspondiente parametro
  n.setParam("state_module_remote",STATE_OK);
  cout << "Atica TELEOPERACION :: Configurado y funcionando" << endl;
  
  while (ros::ok() && !exitModule){
    switch (operationMode) {
          case OPERATION_MODE_DEBUG:
                n.getParam("state_module_remote", estado_actual);
                if (estado_actual == STATE_OFF || estado_actual == STATE_ERROR) {
                    exitModule = true;
                }
                ros::spinOnce();
                break;
          case OPERATION_MODE_RELEASE:
                n.getParam("state_module_remote", estado_actual);
                if (estado_actual == STATE_OFF || estado_actual == STATE_ERROR) {
                    exitModule = true;
                }
                ros::spinOnce();
                break;
          case OPERATION_MODE_SIMULATION:
                n.getParam("state_module_remote", estado_actual);
                if (estado_actual == STATE_OFF || estado_actual == STATE_ERROR) {
                    exitModule = true;
                }
                ros::spinOnce();    
                break;
          default:
              break;
    }
  }
  cout << "Atica TELEOPERACION :: Módulo finalizado" << endl;
  return 0;
}

/*******************************************************************************
 *******************************************************************************
 *                              SUSCRIPTORES
 * *****************************************************************************
 * ****************************************************************************/

// Suscriptor de habilitacion de modulo
void fcn_sub_enable_module(const Common_files::msg_module_enable msg)
{    
    if((msg.id_module==ID_MOD_REMOTE) && (msg.status==1)){
        cout << "Modulo activado\n";
        enableModule=true;
    }
    else if ((msg.id_module==ID_MOD_REMOTE) && (msg.status==0)){
        enableModule=false;
        cout << "Modulo desactivado\n";
    }
}

// Suscriptor de teleoperado que introduce los parametros dentro de rango
void fcn_sub_com_teleop(const Common_files::msg_com_teleop msg)
{
    //cout << "Comando recibido\n";
    //cout << msg << endl;
    if(enableModule==true){
        // El nuevo mensaje esta depurado y conserva el id_elemento
        Common_files::msg_com_teleop msg_cteleop;
        Common_files::msg_error msg_err;
        msg_cteleop.id_element=msg.id_element;

        // Proceso de depuracion de valores
        msg_cteleop.value = convertToCorrectValues(msg.id_element,msg.value);
        cout << "Comando depurado\n";
        cout << msg_cteleop;
        // Publicacion de mensaje ya depurado
        pub_teleop.publish(msg_cteleop);
        
        //cout << "Numero errores: " << error_count << endl;
        if (error_count>=MAX_OUTRANGE_ERROR){
            cout << "error en modulo\n";
            msg_err.id_subsystem=SUBS_REMOTE;
            msg_err.id_error=REMOTE_PARAMETER_OUTRANGE;
            msg_err.type_error=TOE_UNDEFINED;
            pub_error.publish(msg_err);
            error_count=0;
        }
        /*else if (end_error == true){
            msg_err.id_subsystem=SUBS_REMOTE;
            msg_err.id_error=REMOTE_PARAMETER_OUTRANGE;
            msg_err.type_error=TOE_END_ERROR;
            pub_error.publish(msg_err);
            end_error=false;
        }*/
    }
}

// Servicio de heartbeat con Gestion del sistema
bool fcn_heartbeat(Common_files::srv_data::Request &req, Common_files::srv_data::Response &resp)
{
    if(req.param==PARAM_ALIVE)
    {  
        resp.value=0;
        return true;
    }
    else
        return false;
}

/*******************************************************************************
 *******************************************************************************
 *                              FUNCIONES PROPIAS
 * *****************************************************************************
 * ****************************************************************************/

// Funciones propias
// Funcion inicialización de variables
void initialize(ros::NodeHandle n) {
    // Creacion de suscriptores
    ros::Subscriber sub_hab_modulo = n.subscribe("modEnable", 1000, fcn_sub_enable_module);
    ros::Subscriber sub_com_teleop = n.subscribe("commands_unclean", 1000, fcn_sub_com_teleop);
    ros::ServiceServer server=n.advertiseService("module_alive_2",fcn_heartbeat);
    // Generación de publicadores
    pub_teleop = n.advertise<Common_files::msg_com_teleop>("commands_clean", 1000);
    pub_error = n.advertise<Common_files::msg_error>("error", 1000);

    // Inicializacion de variables
    exitModule = false;
    enableModule = false;
    error_count = 0;
    end_error = false;
}

// Acota los valores para que no se salgan de rango
int convertToCorrectValues(int id_elem, int value){
    switch (id_elem){
        case ID_REMOTE_STEER:
            if(value<MIN_STEER_VALUE){
                error_count++;
                return MIN_STEER_VALUE;
            }
            else if(value>MAX_STEER_VALUE){
                error_count++;
                return MAX_STEER_VALUE;
            }
            else{
                end_error=true;
                error_count=0;
                return value;
            }
            break;
        case ID_REMOTE_THROTTLE:
            if(value<MIN_THROTTLE_VALUE){
                error_count++;
                return MIN_THROTTLE_VALUE;
            }
            else if(value>MAX_THROTTLE_VALUE){
                error_count++;
                return MAX_THROTTLE_VALUE;

            }
            else{
                end_error=true;
                error_count=0;                
                return value;
            }
            break;
        case ID_REMOTE_BRAKE:
            if(value<MIN_BRAKE_VALUE){
                error_count++;
                return MIN_BRAKE_VALUE;
            }
            else if(value>MAX_BRAKE_VALUE){
                error_count++;                
                return MAX_BRAKE_VALUE;
            }
            else{
                end_error=true;                
                error_count=0;
                return value;
            }
            break;
        case ID_REMOTE_HANDBRAKE:
            if(value<MIN_HANDBRAKE_VALUE){
                error_count++;
                return MIN_HANDBRAKE_VALUE;
            }
            else if(value>MAX_HANDBRAKE_VALUE){
                error_count++;
                return MAX_HANDBRAKE_VALUE;
            }
            else{
                end_error=true;
                error_count=0;
                return value;
            }
            break;
        case ID_REMOTE_GEAR:
            if(value<MIN_GEAR_VALUE){
                error_count++;
                return MIN_GEAR_VALUE;
            }
            else if(value>MAX_GEAR_VALUE){
                error_count++;
                return MAX_GEAR_VALUE;
            }
            else{
                end_error=true;
                error_count=0;
                return value;
            }
            break;
        case ID_REMOTE_LIGHT_CONVENTIONAL:
            if(value<MIN_LIGHT_VALUE){
                error_count++;
                return MIN_LIGHT_VALUE;
            }
            else if(value>MAX_LIGHT_VALUE){
                error_count++;
                return MAX_LIGHT_VALUE;
            }
            else{
                end_error=true;
                error_count=0;
                return value;
            }
            break;
        case ID_REMOTE_LIGHT_IR:
            if(value<MIN_LIGHT_IR_VALUE){
                error_count++;                
                return MIN_LIGHT_IR_VALUE;
            }
            else if(value>MAX_LIGHT_IR_VALUE){
                error_count++;                
                return MAX_LIGHT_IR_VALUE;
            }
            else{
                end_error=true;                
                error_count=0;
                return value;
            }
            break;
        case ID_REMOTE_ENGINE:
            if(value<MIN_ENGINE_VALUE){
                error_count++;    
                return MIN_ENGINE_VALUE;
            }
            else if(value>MAX_ENGINE_VALUE){
                error_count++;                
                return MAX_ENGINE_VALUE;
            }
            else{
                end_error=true;                
                error_count=0;
                return value;
            }
            break;
        case ID_REMOTE_DIFF:
            if(value<MIN_DIFF_VALUE){
                error_count++;
                return MIN_DIFF_VALUE;                
            }
            else if(value>MAX_DIFF_VALUE){
                error_count++;
                return MAX_DIFF_VALUE;
            }
            else{
                end_error=true;
                error_count=0;
                return value;                
            }
            break;
        case ID_REMOTE_ACT_LASER2D:
            if(value<MIN_LASER_VALUE){
                error_count++;
                return MIN_LASER_VALUE;
            }
            else if(value>MAX_LASER_VALUE){
                error_count++;                
                return MAX_LASER_VALUE;
            }
            else{
                end_error=true;                
                error_count=0;
                return value;
            }         
            break;
        default:
            return -1;
            break;
    }
}

// Obtener el modo de operacion
int getOperationMode(int argc, char **argv){
    if(argc!=2){
        printCorrectSyntax();
        return 0;
    }
    int a = atoi(argv[1]);
    switch (a) {
        case OPERATION_MODE_DEBUG:
            cout << "ATICA REMOTE :: Mode DEBUG enabled" << endl;
            break;
        case OPERATION_MODE_RELEASE:
            cout << "ATICA REMOTE :: Mode RELEASE enabled" << endl;
            break;
        case OPERATION_MODE_SIMULATION:
            cout << "ATICA REMOTE :: Mode SIMULATION enabled" << endl;
            break;
        default:
            printCorrectSyntax();
            return 0;
    }
    return a;
}

// Sintaxis correcta ante fallo
void printCorrectSyntax() {
    cout << "Invalid option. Syntax: ./teleoperacoion [mode option]" << endl;
    cout << "Options: " << endl;
    cout << "1: Debug" << endl;
    cout << "2: Release" << endl;
    cout << "3: Simulation" << endl;
    cout << "---------------" << endl;
    cout << "Try again" << endl;
}