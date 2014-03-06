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
  ros::Subscriber sub_laser = n.subscribe("laser", 1000, fcn_sub_laser);
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
    if(msg.id_module==ID_MOD_TELEOP){
        if(msg.status){
            Modulo_Teleoperacion::msg_com_teleop msg_tlp;
            Modulo_Teleoperacion::msg_mode msg_md;
            switch (msg.submodule){
                case SUBMODE_TELEOP_TELEOP:
                    enableModule=true;
                    // Actualizo los valores de las variables configurables
                    n.getParam("distancia_warning",LasFront_warning);
                    n.getParam("distancia_critica",LasFront_alarm);
                case SUBMODE_TELEOP_START_ENGINE:
                    // Publicacion de com_teleoperado
                    msg_tlp.id_element=ID_TELEOP_ENGINE;
                    msg_tlp.value= START_ENGINE;
                    pub_teleop.publish(msg_tlp);
                    // Publicacion del modo
                    msg_md.mode=MODE_START_ENGINE;
                    msg_md.status=MODE_EXIT;
                    pub_modo.publish(msg_md);
                    break;
                case SUBMODE_TELEOP_STOP_ENGINE:
                    // Publicacion de com_teleoperado
                    msg_tlp.id_element=ID_TELEOP_ENGINE;
                    msg_tlp.value= STOP_ENGINE;
                    pub_teleop.publish(msg_tlp);
                    // Publicacion del modo
                    msg_md.mode=MODE_STOP_ENGINE;
                    msg_md.status=MODE_EXIT;
                    pub_modo.publish(msg_md);
                    break;
                case SUBMODE_TELEOP_ENGAGE_BREAK:
                    // Publicacion de com_teleoperado
                    msg_tlp.id_element=ID_TELEOP_BRAKE;
                    msg_tlp.value= HANDBRAKE_ON;
                    pub_teleop.publish(msg_tlp);
                    // Publicacion del modo
                    msg_md.mode=MODE_ENGAGE_BRAKE;
                    msg_md.status=MODE_EXIT;
                    pub_modo.publish(msg_md);
                    break;
                default:
                    break;
            }
        }
        else{
            exitModule=true;
            if (msg.submodule==SUBMODE_TELEOP_TELEOP)
                enableModule=false;
        }
    }
}

// Suscriptor de datos de laser
void fcn_sub_laser(const Modulo_Teleoperacion::msg_laser msg)
{
    if(enableModule==true){
        Modulo_Teleoperacion::msg_error msg_err;
        switch (processDataLaser(msg)){
            case NEAR_OBSTACLE:
                msg_err.id_subsystem=ID_MOD_TELEOP;
                msg_err.id_error=NEAR_OBSTACLE_DETECTION;
                msg_err.type_error=TOE_UNDEFINED;
                pub_error.publish(msg_err);
                break;
            case FAR_OBSTACLE:
                msg_err.id_subsystem=ID_MOD_TELEOP;
                msg_err.id_error=FAR_OBSTACLE_DETECTION;
                msg_err.type_error=TOE_UNDEFINED;
                pub_error.publish(msg_err);
                break;
            case NO_OBSTACLE:
                //msg_err.id_subsystem=ID_MOD_TELEOP;
                //msg_err.id_error=
                break;
            default:
                cout << "Error lectura datos láser" << endl;
                break;
        }
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
        case ID_TELEOP_STEER:
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
                msg_err.id_subsystem=SUBS_TELEOP;
                msg_err.id_error=REMOTE_PARAMATER_OUTRANGE;
                msg_err.type_error=TOE_UNDEFINED;
                error_count=0;
            }
            break;
        case ID_TELEOP_THROTTLE:
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
                msg_err.id_subsystem=SUBS_TELEOP;
                msg_err.id_error=REMOTE_PARAMATER_OUTRANGE;
                msg_err.type_error=TOE_UNDEFINED;
                error_count=0;
            }
            break;
        case ID_TELEOP_BRAKE:
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
                msg_err.id_subsystem=SUBS_TELEOP;
                msg_err.id_error=REMOTE_PARAMATER_OUTRANGE;
                msg_err.type_error=TOE_UNDEFINED;
                error_count=0;
            }
            break;
        case ID_TELEOP_HANDBRAKE:
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
                msg_err.id_subsystem=SUBS_TELEOP;
                msg_err.id_error=REMOTE_PARAMATER_OUTRANGE;
                msg_err.type_error=TOE_UNDEFINED;
                error_count=0;
            }
            break;
        case ID_TELEOP_GEAR:
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
                msg_err.id_subsystem=SUBS_TELEOP;
                msg_err.id_error=REMOTE_PARAMATER_OUTRANGE;
                msg_err.type_error=TOE_UNDEFINED;
                error_count=0;
            }
            break;
        case ID_TELEOP_LIGHTS:
            if(value<MIN_LIGHTS_VALUE){
                return MIN_LIGHTS_VALUE;
                error_count++;
            }
            else if(value>MAX_LIGHTS_VALUE){
                return MAX_LIGHTS_VALUE;
                error_count++;
            }
            else{
                return value;
                error_count=0;
            }
            if (error_count>=MAX_OUTRANGE_ERROR){
                msg_err.id_subsystem=SUBS_TELEOP;
                msg_err.id_error=REMOTE_PARAMATER_OUTRANGE;
                msg_err.type_error=TOE_UNDEFINED;
                error_count=0;
            }
            break;
        case ID_TELEOP_IR_LIGHTS:
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
                msg_err.id_subsystem=SUBS_TELEOP;
                msg_err.id_error=REMOTE_PARAMATER_OUTRANGE;
                msg_err.type_error=TOE_UNDEFINED;
                error_count=0;
            }
            break;
        case ID_TELEOP_ENGINE:
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
                msg_err.id_subsystem=SUBS_TELEOP;
                msg_err.id_error=REMOTE_PARAMATER_OUTRANGE;
                msg_err.type_error=TOE_UNDEFINED;
                error_count=0;
            }
            break;
        case ID_TELEOP_DIFF:
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
                msg_err.id_subsystem=SUBS_TELEOP;
                msg_err.id_error=REMOTE_PARAMATER_OUTRANGE;
                msg_err.type_error=TOE_UNDEFINED;
                error_count=0;
            }
            break;
        case ID_TELEOP_LASER:
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
                msg_err.id_subsystem=SUBS_TELEOP;
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

short processDataLaser(Modulo_Teleoperacion::msg_laser msg){
    short obstacle;
    int i;
    for (i=0;(i=msg.distance.size());i++){
        if ((msg.distance[i]>0) && (msg.distance[i]<=LasFront_alarm)){
            obstacle=NEAR_OBSTACLE;
            return obstacle;
        }
        else if ((msg.distance[i]>LasFront_alarm) && (msg.distance[i]<=LasFront_warning)){
            obstacle=FAR_OBSTACLE;
            return obstacle;
        }
        else{
            obstacle=NO_OBSTACLE;
            return obstacle;
        }
    }
    return -1;
}
