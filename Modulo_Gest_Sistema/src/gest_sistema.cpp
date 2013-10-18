#include "../include/Modulo_Gest_Sistema/gest_sistema.h"

using namespace std;

    ros::Publisher pub_hab_modulo;

int main(int argc, char **argv)
{

  // Inicio de ROS   
  ros::init(argc, argv, "gest_sistema");

  // Manejador ROS
  ros::NodeHandle n;

  // Inicio de configuracion de los modulos
  modeManagement(n, STATE_CONF);

  // Espera
  if(waitForAllModulesReady(n)) {
      // Generación de publicadores
        pub_hab_modulo = n.advertise<Modulo_Gest_Sistema::msg_habilitacion_modulo > ("module_activation", 1000);

        // Creacion de suscriptores
        ros::Subscriber sub_errores = n.subscribe("error", 1000, fcn_sub_errores);
        ros::Subscriber sub_modo = n.subscribe("mode", 1000, fcn_sub_modo);

        while (ros::ok()) {
            ros::spin();
        }
  }else{
      modeManagement(n,STATE_OFF);
  }

  return 0;
}

/*******************************************************************************
 *******************************************************************************
 *                              SUSCRIPTORES
 * *****************************************************************************
 * ****************************************************************************/

// Suscriptor de gps
void fcn_sub_modo(const Modulo_Gest_Sistema::msg_modo msg)
{
    Modulo_Gest_Sistema::msg_habilitacion_modulo msg_hab;
    switch (msg.modo){
        case MODE_NEUTRAL:
            break;
        case MODE_TELEOP:
            msg_hab.id_modulo=ID_MOD_TELEOP;
            msg_hab.submodo=SUBMODE_TELEOP_TELEOP;
            msg_hab.activo=true;
            pub_hab_modulo.publish(msg_hab);
            break;
        case MODE_START_ENGINE:
            msg_hab.id_modulo=ID_MOD_TELEOP;
            msg_hab.submodo=SUBMODE_TELEOP_START_ENGINE;
            msg_hab.activo=true;
            pub_hab_modulo.publish(msg_hab);
            break;
        case MODE_STOP_ENGINE:
            msg_hab.id_modulo=ID_MOD_TELEOP;
            msg_hab.submodo=SUBMODE_TELEOP_STOP_ENGINE;
            msg_hab.activo=true;
            pub_hab_modulo.publish(msg_hab);
            break;
        case MODE_ENGAGE_BRAKE:
            msg_hab.id_modulo=ID_MOD_TELEOP;
            msg_hab.submodo=SUBMODE_TELEOP_ENGAGE_BREAK;
            msg_hab.activo=true;
            pub_hab_modulo.publish(msg_hab);
            break;
        case MODE_PLAN:
            msg_hab.id_modulo=ID_MOD_NAVEGACION;
            msg_hab.submodo=SUBMODE_NAV_PLAN;
            msg_hab.activo=true;
            pub_hab_modulo.publish(msg_hab);
            break;
        case MODE_COME_TO_ME:
            msg_hab.id_modulo=ID_MOD_NAVEGACION;
            msg_hab.submodo=SUBMODE_NAV_COME_TO_ME;
            msg_hab.activo=true;
            pub_hab_modulo.publish(msg_hab);
            break;
        case MODE_FOLLOW_ME:
            msg_hab.id_modulo=ID_MOD_NAVEGACION;
            msg_hab.submodo=SUBMODE_NAV_FOLLOW_ME;
            msg_hab.activo=true;
            pub_hab_modulo.publish(msg_hab);
            break;
        case MODE_MAPPING:
            msg_hab.id_modulo=ID_MOD_MAPPING;
            msg_hab.submodo=SUBMODE_MAPPING_UNDFD;
            msg_hab.activo=true;
            pub_hab_modulo.publish(msg_hab);
            break;
        case MODE_TEACH:
            msg_hab.id_modulo=ID_MOD_TEACH;
            msg_hab.submodo=SUBMODE_TEACH_UNDFD;
            msg_hab.activo=true;
            pub_hab_modulo.publish(msg_hab);
            break;;
        default:
            break;
    }
}

// Suscriptor de errores
void fcn_sub_errores(const Modulo_Gest_Sistema::msg_errores msg)
{
  ROS_INFO("I heard a ERROR message");
}

/*******************************************************************************
 *******************************************************************************
 *                              FUNCIONES PROPIAS
 * *****************************************************************************
 * ****************************************************************************/

void modeManagement(ros::NodeHandle n,int mode) {
    n.setParam("estado_modulo_conduccion", mode);
    n.setParam("estado_modulo_navegacion", mode);
    n.setParam("estado_modulo_teleoperado", mode);
    n.setParam("estado_modulo_gestErrores", mode);
    n.setParam("estado_modulo_comunicaciones", mode);
    n.setParam("estado_modulo_laser2D", mode);
    n.setParam("estado_modulo_GPS", mode);
    n.setParam("estado_modulo_camaras", mode);
    n.setParam("estado_modulo_teachMapping", mode);
}

bool waitForAllModulesReady(ros::NodeHandle n){
    int state;
    time_t tstart, tend; // gestiona los timeout's

    cout << "ATICA :: Gestion de Sistema :: Esperando la activacion de modulos" << endl;

    cout << "ATICA :: Gestion de Sistema :: Esperando la activacion de modulo CONDUCCION" << endl;
    
    tstart = time(0);
    while(state!=STATE_OK){
        n.getParam("estado_modulo_conduccion",state);
        tend = time(0);
        if(difftime(tend, tstart)>TIMEOUT_ACTIVATION_MODULE){
            cout << "ATICA :: Gestion de Sistema :: Timeout en la activacion del modulo de CONDUCCION"<< endl;
            return false;
        }
    }

    cout << "ATICA :: Gestion de Sistema :: Esperando la activacion de modulo NAVEGACION" << endl;

    tstart = time(0);
    while(state!=STATE_OK){
        n.getParam("estado_modulo_navegacion",state);
        tend = time(0);
        if(difftime(tend, tstart)>TIMEOUT_ACTIVATION_MODULE){
            cout << "ATICA :: Gestion de Sistema :: Timeout en la activacion del modulo de NAVEGACION"<< endl;
            return false;
        }
    }

    cout << "ATICA :: Gestion de Sistema :: Esperando la activacion de modulo TELEOPERACION" << endl;

    tstart = time(0);
    while(state!=STATE_OK){
        n.getParam("estado_modulo_teleoperado",state);
        tend = time(0);
        if(difftime(tend, tstart)>TIMEOUT_ACTIVATION_MODULE){
            cout << "ATICA :: Gestion de Sistema :: Timeout en la activacion del modulo de TELEOPERACION"<< endl;
            return false;
        }
    }

    cout << "ATICA :: Gestion de Sistema :: Esperando la activacion de modulo GESTION DE ERRORES" << endl;

    tstart = time(0);
    while(state!=STATE_OK){
        n.getParam("estado_modulo_gestErrores",state);
        tend = time(0);
        if(difftime(tend, tstart)>TIMEOUT_ACTIVATION_MODULE){
            cout << "ATICA :: Gestion de Sistema :: Timeout en la activacion del modulo de GESTION DE ERRORES"<< endl;
            return false;
        }
    }

    cout << "ATICA :: Gestion de Sistema :: Esperando la activacion de modulo COMUNICACIONES" << endl;

    tstart = time(0);
    while(state!=STATE_OK){
        n.getParam("estado_modulo_comunicaciones",state);
        tend = time(0);
        if(difftime(tend, tstart)>TIMEOUT_ACTIVATION_MODULE){
            cout << "ATICA :: Gestion de Sistema :: Timeout en la activacion del modulo de COMUNICACIONES"<< endl;
            return false;
        }
    }

    cout << "ATICA :: Gestion de Sistema :: Esperando la activacion de modulo LASER 2D" << endl;

    tstart = time(0);
    while(state!=STATE_OK){
        n.getParam("estado_modulo_laser2D",state);
        tend = time(0);
        if(difftime(tend, tstart)>TIMEOUT_ACTIVATION_MODULE){
            cout << "ATICA :: Gestion de Sistema :: Timeout en la activacion del modulo de LASER 2D"<< endl;
            return false;
        }
    }

    cout << "ATICA :: Gestion de Sistema :: Esperando la activacion de modulo GPS" << endl;

    tstart = time(0);
    while(state!=STATE_OK){
        n.getParam("estado_modulo_GPS",state);
        tend = time(0);
        if(difftime(tend, tstart)>TIMEOUT_ACTIVATION_MODULE){
            cout << "ATICA :: Gestion de Sistema :: Timeout en la activacion del modulo de GPS"<< endl;
            return false;
        }
    }

    cout << "ATICA :: Gestion de Sistema :: Esperando la activacion de modulo CAMARAS" << endl;

    tstart = time(0);
    while(state!=STATE_OK){
        n.getParam("estado_modulo_camaras",state);
        tend = time(0);
        if(difftime(tend, tstart)>TIMEOUT_ACTIVATION_MODULE){
            cout << "ATICA :: Gestion de Sistema :: Timeout en la activacion del modulo de CAMARAS"<< endl;
            return false;
        }
    }

    cout << "ATICA :: Gestion de Sistema :: Esperando la activacion de modulo TEACH&MAPPING" << endl;

    tstart = time(0);
    while(state!=STATE_OK){
        n.getParam("estado_modulo_teachMapping",state);
        tend = time(0);
        if(difftime(tend, tstart)>TIMEOUT_ACTIVATION_MODULE){
            cout << "ATICA :: Gestion de Sistema :: Timeout en la activacion del modulo de TEACH&MAPPING"<< endl;
            return false;
        }
    }
    cout << "ATICA :: Gestion de Sistema :: Todos los modulos activados" << endl;
    return true;
}