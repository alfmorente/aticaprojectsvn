/**
  @file gest_errores.cpp
  @brief 

 * Archivo principal del Módulo Teleoperación (Remote). Se encarga de recibir
 * los comandos de teleoperación, filtrarlos por si vienen con valores fuera de
 * rango y publicarlos para el módulo de conducción.

  @author Alfonso Morente
  @date 13/09/2013

*/

#include "../include/Modulo_Teleoperacion/teleoperacion.h"

using namespace std;

/**
 * Método principal del nodo. 
 * @param[in] argc Número de argumentos
 * @param[in] argv Vector de argumentos
 * @return Entero distinto de 0 si ha habido problemas. 0 en caso contrario. 
 */
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
        usleep(50000);
  }
  cout << "Atica TELEOPERACION :: Iniciando configuración..." << endl;
  
  initialize(n);
  ros::Rate loop_rate(40);
  usleep(100000);
  // Todo esta correcto, lo especificamos con el correspondiente parametro
  n.setParam("state_module_remote",STATE_OK);
  cout << "Atica TELEOPERACION :: Configurado y funcionando" << endl;
  
  // Espera la señal de OK proviniento de Gestion del sistema
  int system_status=STATE_SYSTEM_OFF;
  while(system_status!=STATE_SYSTEM_ON && estado_actual != STATE_OFF){
        n.getParam("state_system", system_status);
        n.getParam("state_module_remote", estado_actual);
        usleep(50000);
  }
  
  switch (operationMode) {
      case OPERATION_MODE_DEBUG:
          while (ros::ok() && !exitModule){  
          n.getParam("state_module_remote", estado_actual);
                if (estado_actual == STATE_OFF || estado_actual == STATE_ERROR) {
                    exitModule = true;
                }
                ros::spinOnce();
                loop_rate.sleep();
                //usleep(25000);
            }
          break;
      case OPERATION_MODE_RELEASE:
          while (ros::ok() && !exitModule){
            n.getParam("state_module_remote", estado_actual);
                if (estado_actual == STATE_OFF || estado_actual == STATE_ERROR) {
                    exitModule = true;
                }
                ros::spinOnce();
                usleep(25000);
          }
            break;
      case OPERATION_MODE_SIMULATION:
          while (ros::ok() && !exitModule) {
                n.getParam("state_module_remote", estado_actual);
                if (estado_actual == STATE_OFF || estado_actual == STATE_ERROR) {
                    exitModule = true;
                }
                ros::spinOnce();
                usleep(25000);
          }
            break;
      default:
            break;
  }
  
  cout << "Atica TELEOPERACION :: Módulo finalizado" << endl;
  return 0;
}

/*******************************************************************************
 *******************************************************************************
 *                              SUSCRIPTORES
 * *****************************************************************************
 * ****************************************************************************/

/**
 * Suscriptor de habilitacion de modulo
 * @param msg Mensaje proveniente del publicador
 */
void fcn_sub_enable_module(const Common_files::msg_module_enablePtr& msg)
{    
    if((msg->id_module==ID_MOD_REMOTE) && (msg->status==1)){
        cout << "Modulo activado\n";
        enableModule=true;
    }
    else if ((msg->id_module==ID_MOD_REMOTE) && (msg->status==0)){
        enableModule=false;
        cout << "Modulo desactivado\n";
    }
}

/**
 * Suscriptor de teleoperado que introduce los parametros dentro de rango
 * @param msg Mensaje proveniente del publicador
 */
void fcn_sub_com_teleop(const Common_files::msg_com_teleopPtr& msg)
{
    if(enableModule==true){
        // El nuevo mensaje esta depurado y conserva el id_elemento
        Common_files::msg_com_teleopPtr msg_cteleop(new Common_files::msg_com_teleop);
        Common_files::msg_errorPtr msg_err(new Common_files::msg_error);
        msg_cteleop->id_element=msg->id_element;
        //msg_cteleop.id_element=msg.id_element;

        // Proceso de depuracion de valores
        msg_cteleop->value = convertToCorrectValues(msg->id_element,msg->value);
        // Publicacion de mensaje ya depurado
        pub_teleop.publish(msg_cteleop);
        
        if (error_count>=MAX_OUTRANGE_ERROR){
            cout << "error en modulo\n";
            msg_err->id_subsystem=SUBS_REMOTE;
            msg_err->id_error=REMOTE_PARAMETER_OUTRANGE;
            msg_err->type_error=TOE_UNDEFINED;
            pub_error.publish(msg_err);
            error_count=0;
        }
        /*else if (end_error == true){                  // Fin de error, siempre será Warning por eso está comentado
            msg_err.id_subsystem=SUBS_REMOTE;
            msg_err.id_error=REMOTE_PARAMETER_OUTRANGE;
            msg_err.type_error=TOE_END_ERROR;
            pub_error.publish(msg_err);
            end_error=false;
        }*/
    }
}

/**
 * Servicio de heartbeat para Gestion de sistema
 * @param req 
 * @param resp
 * @return 
 */
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

/**
 * Inicialización de variables del sistema
 * @param n Nodo de trabajo de ROS
 */
void initialize(ros::NodeHandle n) {
    // Creacion de suscriptores
    sub_hab_modulo = n.subscribe("modEnable", 1000, fcn_sub_enable_module);
    sub_com_teleop = n.subscribe("commands_unclean", 1000, fcn_sub_com_teleop);
    server=n.advertiseService("module_alive_2",fcn_heartbeat);
    // Generación de publicadores
    pub_teleop = n.advertise<Common_files::msg_com_teleop>("commands_clean", 1000);
    pub_error = n.advertise<Common_files::msg_error>("error", 1000);

    // Inicializacion de variables
    exitModule = false;
    enableModule = false;
    error_count = 0;
    end_error = false;
    
}

/**
 * Acota los valores de los comandos de entrada para que no se salgan de rango
 * @param id_elem Tipo de comando
 * @param value Valor del comando sin filtrar
 * @return Devuelve el valor del comando dentro de rango 
 */
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

/**
 * Función que elige una versión del software dependiendo del 
 * argumento de entrada
 * @param[in] argc Número de argumentos
 * @param[in] argv Vector de argumentos
 * @return Devuelve 0 si hay errores o número de versión si no hay errores
 */
int getOperationMode(int argc, char **argv){
    if(argc!=4){
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

/**
 * Muestra por pantalla que se ha producido un error al elegir versión.
 * Presenta la manera de ejecutar el programa correctamente
 */
void printCorrectSyntax() {
    cout << "Invalid option. Syntax: ./teleoperacoion [mode option]" << endl;
    cout << "Options: " << endl;
    cout << "1: Debug" << endl;
    cout << "2: Release" << endl;
    cout << "3: Simulation" << endl;
    cout << "---------------" << endl;
    cout << "Try again" << endl;
}