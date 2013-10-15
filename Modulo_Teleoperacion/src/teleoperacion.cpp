#include "../include/Modulo_Teleoperacion/teleoperacion.h"

  ros::Publisher pub_modo;
  ros::Publisher pub_errores;
  ros::Publisher pub_teleop;

  //Variable de activacion de modulo
  bool exitModule;

  using namespace std;

int main(int argc, char **argv)
{

  // Inicio de ROS
  ros::init(argc, argv, "teleoperacion");

  // Manejador ROS
  ros::NodeHandle n;

  // Espera activa de inicio de modulo
  int estado_actual=STATE_OFF;
  while(estado_actual!=STATE_CONF){
          n.getParam("estado_modulo_teleoperado",estado_actual);
  }
  cout << "Atica TELEOPERACION :: Iniciando configuración..." << endl;
  
  // Generación de publicadores
  pub_modo = n.advertise<Modulo_Teleoperacion::msg_modo>("mode", 1000);
  pub_teleop = n.advertise<Modulo_Teleoperacion::msg_com_teleoperado>("teleop",1000);
  pub_errores = n.advertise<Modulo_Teleoperacion::msg_errores>("error",1000);


  // Creacion de suscriptores
  ros::Subscriber sub_laser = n.subscribe("laser", 1000, fcn_sub_laser);
  ros::Subscriber sub_hab_modulo = n.subscribe("module_activation", 1000, fcn_sub_hab_modulos);
  ros::Subscriber sub_com_teleop = n.subscribe("teleop",1000,fcn_sub_com_teleop);

  // Inicializacion de variable de habilitacion de modulo
  exitModule=false;

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
void fcn_sub_hab_modulos(const Modulo_Teleoperacion::msg_habilitacion_modulo msg)
{
    if(msg.id_modulo==ID_MOD_TELEOP){
        if(msg.activo){
            Modulo_Teleoperacion::msg_com_teleoperado msg_tlp;
            Modulo_Teleoperacion::msg_modo msg_md;
            switch (msg.submodo){
                case SUBMODE_TELEOP_START_ENGINE:
                    // Publicacion de com_teleoperado
                    msg_tlp.id_elemento=ID_TELEOP_ENGINE;
                    msg_tlp.valor= 1; // TODO por definir valores de motor/frenos/...
                    msg_tlp.depurado=false;
                    pub_teleop.publish(msg_tlp);
                    // Publicacion del modo
                    msg_md.modo=MODE_START_ENGINE;
                    pub_modo.publish(msg_md);
                    break;
                case SUBMODE_TELEOP_STOP_ENGINE:
                    // Publicacion de com_teleoperado
                    msg_tlp.id_elemento=ID_TELEOP_ENGINE;
                    msg_tlp.valor= 0; // TODO por definir valores de motor/frenos/...
                    msg_tlp.depurado=false;
                    pub_teleop.publish(msg_tlp);
                    // Publicacion del modo
                    msg_md.modo=MODE_STOP_ENGINE;
                    pub_modo.publish(msg_md);
                    break;
                case SUBMODE_TELEOP_ENGAGE_BREAK:
                    // Publicacion de com_teleoperado
                    msg_tlp.id_elemento=ID_TELEOP_BRAKE;
                    msg_tlp.valor= 1; // TODO por definir valores de motor/frenos/...
                    msg_tlp.depurado=false;
                    pub_teleop.publish(msg_tlp);
                    // Publicacion del modo
                    msg_md.modo=MODE_ENGAGE_BRAKE;
                    pub_modo.publish(msg_md);
                    break;
                default:
                    break;
            }
        }else{
            exitModule=true;
        }
    }
}

// Suscriptor de errores
void fcn_sub_laser(const Modulo_Teleoperacion::msg_laser msg)
{
    if(processDataLaser(msg)){
        Modulo_Teleoperacion::msg_errores msg_err;
        msg_err.id_subsistema = SUBS_LASER_DELANTERO;
        msg_err.id_error = 0; // Indeterminado (TODO)
        msg_err.tipo_error = ALARM_CRITICAL; // ???
        pub_errores.publish(msg_err);
    }
}

// Suscriptor de teleoperado que introduce los parametros dentro de rango
void fcn_sub_com_teleop(const Modulo_Teleoperacion::msg_com_teleoperado msg)
{
    if(!msg.depurado){
        // El nuevo mensaje esta depurado y conserva el id_elemento
        Modulo_Teleoperacion::msg_com_teleoperado msg_cteleop;
        msg_cteleop.id_elemento=msg.id_elemento;
        msg_cteleop.depurado=true;

        // Proceso de depuracion de valores
        msg_cteleop.valor = convertToCorrectValues(msg.id_elemento,msg.valor);

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
    switch (id_elem){
        case ID_TELEOP_STEER:
            if(value<MIN_STEER_VALUE)
                return MIN_STEER_VALUE;
            else if(value>MAX_STEER_VALUE)
                return MAX_STEER_VALUE;
            else
                return value;
            break;
        case ID_TELEOP_THROTTLE:
            if(value<MIN_THROTTLE_VALUE)
                return MIN_THROTTLE_VALUE;
            else if(value>MAX_THROTTLE_VALUE)
                return MAX_THROTTLE_VALUE;
            else
                return value;
            break;
        case ID_TELEOP_BRAKE:
            if(value<MIN_BRAKE_VALUE)
                return MIN_BRAKE_VALUE;
            else if(value>MAX_BRAKE_VALUE)
                return MAX_BRAKE_VALUE;
            else
                return value;
            break;
        case ID_TELEOP_HANDBRAKE:
            if(value<MIN_HANDBRAKE_VALUE)
                return MIN_HANDBRAKE_VALUE;
            else if(value>MAX_HANDBRAKE_VALUE)
                return MAX_HANDBRAKE_VALUE;
            else
                return value;
            break;
        case ID_TELEOP_GEAR:
            if(value<MIN_GEAR_VALUE)
                return MIN_GEAR_VALUE;
            else if(value>MAX_GEAR_VALUE)
                return MAX_GEAR_VALUE;
            else
                return value;
            break;
        case ID_TELEOP_LIGHTS:
            if(value<MIN_LIGHTS_VALUE)
                return MIN_LIGHTS_VALUE;
            else if(value>MAX_LIGHTS_VALUE)
                return MAX_LIGHTS_VALUE;
            else
                return value;
            break;
        case ID_TELEOP_ENGINE:
            if(value<MIN_ENGINE_VALUE)
                return MIN_ENGINE_VALUE;
            else if(value>MAX_ENGINE_VALUE)
                return MAX_ENGINE_VALUE;
            else
                return value;
            break;
        default:
            return -1;
            break;
    }
}

bool processDataLaser(Modulo_Teleoperacion::msg_laser msg){
    // TODO
    return true;
}
