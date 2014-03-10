#include "../include/Modulo_Teleoperacion/teleoperacion.h"

  ros::Publisher pub_modo;
  ros::Publisher pub_error;
  ros::Publisher pub_teleop;

  //Variable de activacion de modulo
  bool exitModule;

  using namespace std;

int main(int argc, char **argv)
{

  // Inicio de ROS
  ros::init(argc, argv, "teleoperacion");

  // Espera activa de inicio de modulo
  int estado_actual=STATE_OFF;
  while(estado_actual!=STATE_CONF){
          n.getParam("estado_modulo_teleoperado",estado_actual);
  }
  cout << "Atica TELEOPERACION :: Iniciando configuración..." << endl;
  
  // Generación de publicadores
  pub_modo = n.advertise<Modulo_Teleoperacion::msg_mode>("mode", 1000);
  pub_teleop = n.advertise<Modulo_Teleoperacion::msg_com_teleop>("clean",1000);
  pub_error = n.advertise<Modulo_Teleoperacion::msg_error>("error",1000);


  // Creacion de suscriptores
  ros::Subscriber sub_hab_modulo = n.subscribe("moduleEnable", 1000, fcn_sub_enable_module);
  ros::Subscriber sub_com_teleop = n.subscribe("unclean",1000,fcn_sub_com_teleop);

  // Inicializacion de variable de habilitacion de modulo
  exitModule=false;
  enableModule=false;
  error_count=0;

  // Todo esta correcto, lo especificamos con el correspondiente parametro
  n.setParam("estado_modulo_teleoperado",STATE_OK);
  cout << "Atica TELEOPERACION :: Configurado y funcionando" << endl;

  while (ros::ok() && !exitModule){
      n.getParam("estado_modulo_teleoperado",estado_actual);
      if(estado_actual==STATE_OFF || estado_actual==STATE_ERROR){
          exitModule = true;
      }
      ros::spinOnce();
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
void fcn_sub_enable_module(const Modulo_Teleoperacion::msg_module_enable msg)
{    
    if((msg.id_module==ID_MOD_REMOTE) && (msg.status==1)){
        enableModule=true;
    }
    else if ((msg.id_module==ID_MOD_REMOTE) && (msg.status==0)){
        enableModule=false;
    }
}

// Suscriptor de teleoperado que introduce los parametros dentro de rango
void fcn_sub_com_teleop(const Modulo_Teleoperacion::msg_com_teleop msg)
{
    if(enableModule==true){
        // El nuevo mensaje esta depurado y conserva el id_elemento
        Modulo_Teleoperacion::msg_com_teleop msg_cteleop;
        msg_cteleop.id_element=msg.id_element;

        // Proceso de depuracion de valores
        msg_cteleop.value = convertToCorrectValues(msg.id_element,msg.value);

        // Publicacion de mensaje ya depurado
        pub_teleop.publish(msg_cteleop);
    }
}

/*******************************************************************************
 *******************************************************************************
 *                              FUNCIONES PROPIAS
 * *****************************************************************************
 * ****************************************************************************/

//Funciones propias

// Acota los valores para que no se salgan de rango
int convertToCorrectValues(int id_elem, int value){
    Modulo_Teleoperacion::msg_error msg_err;
    switch (id_elem){
        case ID_REMOTE_STEER:
            if(value<MIN_STEER_VALUE){
                return MIN_STEER_VALUE;
                error_count++;
            }
            else if(value>MAX_STEER_VALUE){
                return MAX_STEER_VALUE;
                error_count++;
            }
            else{
                return value;
                error_count=0;
            }
            if (error_count>=MAX_OUTRANGE_ERROR){
                msg_err.id_subsystem=SUBS_REMOTE;
                msg_err.id_error=REMOTE_PARAMETER_OUTRANGE;
                msg_err.type_error=TOE_UNDEFINED;
                error_count=0;
            }
            break;
        case ID_REMOTE_THROTTLE:
            if(value<MIN_THROTTLE_VALUE){
                return MIN_THROTTLE_VALUE;
                error_count++;
            }
            else if(value>MAX_THROTTLE_VALUE){
                return MAX_THROTTLE_VALUE;
                error_count++;
            }
            else{
                return value;
                error_count=0;
            }
            if (error_count>=MAX_OUTRANGE_ERROR){
                msg_err.id_subsystem=SUBS_REMOTE;
                msg_err.id_error=REMOTE_PARAMETER_OUTRANGE;
                msg_err.type_error=TOE_UNDEFINED;
                error_count=0;
            }
            break;
        case ID_REMOTE_BRAKE:
            if(value<MIN_BRAKE_VALUE){
                return MIN_BRAKE_VALUE;
                error_count++;
            }
            else if(value>MAX_BRAKE_VALUE){
                return MAX_BRAKE_VALUE;
                error_count++;
            }
            else{
                return value;
                error_count=0;
            }
            if (error_count>=MAX_OUTRANGE_ERROR){
                msg_err.id_subsystem=SUBS_REMOTE;
                msg_err.id_error=REMOTE_PARAMETER_OUTRANGE;
                msg_err.type_error=TOE_UNDEFINED;
                error_count=0;
            }
            break;
        case ID_REMOTE_HANDBRAKE:
            if(value<MIN_HANDBRAKE_VALUE){
                return MIN_HANDBRAKE_VALUE;
                error_count++;
            }
            else if(value>MAX_HANDBRAKE_VALUE){
                return MAX_HANDBRAKE_VALUE;
                error_count++;
            }
            else{
                return value;
                error_count=0;
            }
            if (error_count>=MAX_OUTRANGE_ERROR){
                msg_err.id_subsystem=SUBS_REMOTE;
                msg_err.id_error=REMOTE_PARAMETER_OUTRANGE;
                msg_err.type_error=TOE_UNDEFINED;
                error_count=0;
            }
            break;
        case ID_REMOTE_GEAR:
            if(value<MIN_GEAR_VALUE){
                return MIN_GEAR_VALUE;
                error_count++;
            }
            else if(value>MAX_GEAR_VALUE){
                return MAX_GEAR_VALUE;
                error_count++;
            }
            else{
                return value;
                error_count=0;
            }
            if (error_count>=MAX_OUTRANGE_ERROR){
                msg_err.id_subsystem=SUBS_REMOTE;
                msg_err.id_error=REMOTE_PARAMETER_OUTRANGE;
                msg_err.type_error=TOE_UNDEFINED;
                error_count=0;
            }
            break;
        case ID_REMOTE_LIGHT_STANDARD:
            if(value<MIN_LIGHT_VALUE){
                return MIN_LIGHT_VALUE;
                error_count++;
            }
            else if(value>MAX_LIGHT_VALUE){
                return MAX_LIGHT_VALUE;
                error_count++;
            }
            else{
                return value;
                error_count=0;
            }
            if (error_count>=MAX_OUTRANGE_ERROR){
                msg_err.id_subsystem=SUBS_REMOTE;
                msg_err.id_error=REMOTE_PARAMETER_OUTRANGE;
                msg_err.type_error=TOE_UNDEFINED;
                error_count=0;
            }
            break;
        case ID_REMOTE_LIGHT_IR:
            if(value<MIN_LIGHTS_IR_VALUE){
                return MIN_LIGHTS_VALUE;
                error_count++;
            }
            else if(value>MAX_LIGHTS_IR_VALUE){
                return MAX_LIGHTS_VALUE;
                error_count++;
            }
            else{
                return value;
                error_count=0;
            }
            if (error_count>=MAX_OUTRANGE_ERROR){
                msg_err.id_subsystem=SUBS_REMOTE;
                msg_err.id_error=REMOTE_PARAMETER_OUTRANGE;
                msg_err.type_error=TOE_UNDEFINED;
                error_count=0;
            }
            break;
        case ID_REMOTE_ENGINE:
            if(value<MIN_ENGINE_VALUE){
                return MIN_ENGINE_VALUE;
                error_count++;
            }
            else if(value>MAX_ENGINE_VALUE){
                return MAX_ENGINE_VALUE;
                error_count++;
            }
            else{
                return value;
                error_count=0;
            }
            if (error_count>=MAX_OUTRANGE_ERROR){
                msg_err.id_subsystem=SUBS_REMOTE;
                msg_err.id_error=REMOTE_PARAMATER_OUTRANGE;
                msg_err.type_error=TOE_UNDEFINED;
                error_count=0;
            }
            break;
        case ID_REMOTE_DIFF:
            if(value<MIN_DIFF_VALUE){
                return MIN_DIFF_VALUE;
                error_count++;
            }
            else if(value>MAX_DIFF_VALUE){
                return MAX_DIFF_VALUE;
                error_count++;
            }
            else{
                return value;
                error_count=0;
            }
            if (error_count>=MAX_OUTRANGE_ERROR){
                msg_err.id_subsystem=SUBS_REMOTE;
                msg_err.id_error=REMOTE_PARAMATER_OUTRANGE;
                msg_err.type_error=TOE_UNDEFINED;
                error_count=0;
            }
            break;
        case ID_REMOTE_LASER:
            if(value<MIN_LASER_VALUE){
                return MIN_LASER_VALUE;
                error_count++;
            }
            else if(value>MAX_LASER_VALUE){
                return MAX_LASER_VALUE;
                error_count++;
            }
            else{
                return value;
                error_count=0;
            }
            if (error_count>=MAX_OUTRANGE_ERROR){
                msg_err.id_subsystem=SUBS_REMOTE;
                msg_err.id_error=REMOTE_PARAMATER_OUTRANGE;
                msg_err.type_error=TOE_UNDEFINED;
                error_count=0;
            }
            break;
        default:
            return -1;
            break;
    }
    pub_error.publish(msg_err);
}