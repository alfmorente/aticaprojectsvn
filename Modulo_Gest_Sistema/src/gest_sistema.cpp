#include "../include/Modulo_Gest_Sistema/gest_sistema.h"

using namespace std;

int main(int argc, char **argv)
{

  // Inicio de ROS   
  ros::init(argc, argv, "gest_sistema");

  // Manejador ROS
  ros::NodeHandle n;

  // Inicio de configuracion de los modulos
  modeManagement(n,STATE_CONF);


  // Espera
  if(waitForAllModulesReady(n)) {
      // Generación de publicadores
        pub_module_enable = n.advertise<Modulo_Gest_Sistema::msg_module_enable > ("module_enable", 1000);
        pub_error = n.advertise<Modulo_Gest_Sistema::msg_error > ("error", 1000);
        pub_mode_error = n.advertise<Modulo_Gest_Sistema::msg_mode > ("mode", 1000);
        pub_mode_communication = n.advertise<Modulo_Gest_Sistema::msg_mode > ("mode", 1000);
        pub_ack = n.advertise<std_msgs::Bool> ("mode", 1000);


        // Creacion de suscriptores
        //ros::Subscriber sub_errores = n.subscribe("error", 1000, fcn_sub_errores);
        ros::Subscriber sub_mode_error = n.subscribe("/modeError", 1000, fcn_sub_mode_error);
        ros::Subscriber sub_mode_communication = n.subscribe("/modeCommunication", 1000, fcn_sub_mode_communication);
        ros::Subscriber sub_mode_remote = n.subscribe("/modeRemote", 1000, fcn_sub_mode_remote);
        ros::Subscriber sub_mode_nav = n.subscribe("/modeNav", 1000, fcn_sub_mode_nav);
        ros::Subscriber sub_mode_convoy = n.subscribe("/modeConvoy", 1000, fcn_sub_mode_convoy);

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

//Subscriptor del modo al topic del módulo Communicator
void fcn_sub_mode_communication(const Modulo_Gest_Sistema::msg_mode msg)
{


        std_msgs::Bool ackMode;
        ackMode.data=0;

        if(actualMode==MODE_MANUAL)
            return;

        //Si llega cambio de modo desde la UCR
        if(msg.status==MODE_START)
        {
            if(actualMode==MODE_NEUTRAL)
            {
                if(modeRUN(msg.mode))
                {
                    pub_mode_error.publish(msg);
                    pub_ack.publish(ackMode);
                }
                else
                {
                    Modulo_Gest_Sistema::msg_error error;
                    error.ID_subsystem=SUBS_MANAGE_SYSTEM;
                    error.type_error=ALARM_UNDEFINED;
                    error.ID_error=ERROR_MODE_NOT_UNAVAILABLE;
                    pub_error.publish(error);
                }
            }
        }

        //Modo activo
        else if(actualMode==msg.mode)
        {
            if(msg.status==MODE_EXIT)
            {
                modeEXIT(msg.mode);
                pub_mode_error.publish(msg);
                pub_ack.publish(ackMode);
                actualMode=MODE_NEUTRAL;
            }
            else if(msg.status==MODE_RUN)
                modeRESUME(msg.mode);
            else if(msg.status==MODE_STOP)
                modeSTOP(msg.mode);
        }
}

//Subscriptor del modo al topic del módulo Error
void fcn_sub_mode_error(const Modulo_Gest_Sistema::msg_mode msg)
{
        if(actualMode==MODE_MANUAL)
        return;

        //Modo activo
        if(actualMode==msg.mode)
        {
            if(msg.status==MODE_EXIT)
            {
                modeEXIT(msg.mode);
                actualMode=MODE_NEUTRAL;
            }
        }
}

//Subscriptor del modo al topic del módulo Remote
void fcn_sub_mode_remote(const Modulo_Gest_Sistema::msg_mode msg)
{
        if(actualMode==MODE_MANUAL)
            return;

        //Modo activo
        if(actualMode==msg.mode)
        {
            if(msg.status==MODE_EXIT)
            {
                modeEXIT(msg.mode);
                actualMode=MODE_NEUTRAL;
                pub_mode_communication.publish(msg);
                pub_mode_error.publish(msg);
            }
        }
}
//Subscriptor del modo al topic del módulo de navegación
void fcn_sub_mode_nav(const Modulo_Gest_Sistema::msg_mode msg)
{
        if(actualMode==MODE_MANUAL)
            return;

        //Modo activo
        if(actualMode==msg.mode)
        {
            //Finalización de la ruta
            if(msg.status==MODE_FINISH)
                pub_mode_communication.publish(msg);
        }
}
//Subscriptor del modo al topic del módulo de driving
void fcn_sub_mode_driving(const Modulo_Gest_Sistema::msg_mode msg)
{
        //Cambio del conmutador
        if(msg.mode==MODE_MANUAL )
        {
            if(msg.status==MODE_START && actualMode!=MODE_MANUAL)
            {
                 modeEXIT(actualMode);
                 actualMode=MODE_MANUAL;

                 //¿SE PUBLICA ALARMA A ERROR??
            }
            else if(msg.status==MODE_EXIT && actualMode==MODE_MANUAL)
            {
                 actualMode=MODE_NEUTRAL;
                 pub_mode_communication.publish(msg);
            }
        }
}

//Subscriptor del modo al topic del módulo Convoy
void fcn_sub_mode_convoy(const Modulo_Gest_Sistema::msg_mode msg)
{
    //Por Rellenar
}

/*******************************************************************************
 *******************************************************************************
 *                              FUNCIONES PROPIAS
 * *****************************************************************************
 * ****************************************************************************/

void modeManagement(ros::NodeHandle n,int mode) {
    n.setParam("state_module_driving", mode);
    n.setParam("state_module_navigate", mode);
    n.setParam("state_module_teleoperate", mode);
    n.setParam("state_module_errors", mode);
    n.setParam("state_module_communication", mode);
    n.setParam("state_module_laser3D", mode);
    n.setParam("state_module_laserFrontal", mode);
    n.setParam("state_module_laserBackRight", mode);
    n.setParam("state_module_laserBackLeft", mode);
    n.setParam("state_module_gps", mode);
    n.setParam("state_module_camera", mode);
    n.setParam("state_module_humanLocalization",mode);
    n.setParam("state_module_beacon",mode);
    //n.setParam("state_module_waypointsMap",mode);
    n.setParam("state_module_rangeDataFusion",mode);    //n.setParam("estado_modulo_teachMapping", mode);
}

bool waitForAllModulesReady(ros::NodeHandle n){
    int state;
    time_t tstart, tend; // gestiona los timeout's

    cout << "ATICA :: Gestion de Sistema :: Esperando la activacion de modulos" << endl;

    cout << "ATICA :: Gestion de Sistema :: Esperando la activacion de modulo CONDUCCION" << endl;
    
    tstart = time(0);
    while(state!=STATE_OK){
        n.getParam("state_module_driving",state);
        tend = time(0);
        if(difftime(tend, tstart)>TIMEOUT_ACTIVATION_MODULE){
            cout << "ATICA :: Gestion de Sistema :: Timeout en la activacion del modulo de CONDUCCION"<< endl;
            return false;
        }
    }

    cout << "ATICA :: Gestion de Sistema :: Esperando la activacion de modulo NAVEGACION" << endl;

    tstart = time(0);
    while(state!=STATE_OK){
        n.getParam("state_module_navigate",state);
        tend = time(0);
        if(difftime(tend, tstart)>TIMEOUT_ACTIVATION_MODULE){
            cout << "ATICA :: Gestion de Sistema :: Timeout en la activacion del modulo de NAVEGACION"<< endl;
            return false;
        }
    }

    cout << "ATICA :: Gestion de Sistema :: Esperando la activacion de modulo TELEOPERACION" << endl;

    tstart = time(0);
    while(state!=STATE_OK){
        n.getParam("state_module_teleoperate",state);
        tend = time(0);
        if(difftime(tend, tstart)>TIMEOUT_ACTIVATION_MODULE){
            cout << "ATICA :: Gestion de Sistema :: Timeout en la activacion del modulo de TELEOPERACION"<< endl;
            return false;
        }
    }

    cout << "ATICA :: Gestion de Sistema :: Esperando la activacion de modulo GESTION DE ERRORES" << endl;

    tstart = time(0);
    while(state!=STATE_OK){
        n.getParam("state_module_errors",state);
        tend = time(0);
        if(difftime(tend, tstart)>TIMEOUT_ACTIVATION_MODULE){
            cout << "ATICA :: Gestion de Sistema :: Timeout en la activacion del modulo de GESTION DE ERRORES"<< endl;
            return false;
        }
    }

    cout << "ATICA :: Gestion de Sistema :: Esperando la activacion de modulo COMUNICACIONES" << endl;

    tstart = time(0);
    while(state!=STATE_OK){
        n.getParam("state_module_managed_system",state);
        tend = time(0);
        if(difftime(tend, tstart)>TIMEOUT_ACTIVATION_MODULE){
            cout << "ATICA :: Gestion de Sistema :: Timeout en la activacion del modulo de COMUNICACIONES"<< endl;
            return false;
        }
    }

    cout << "ATICA :: Gestion de Sistema :: Esperando la activacion de modulo LASER 3D" << endl;

    tstart = time(0);
    while(state!=STATE_OK){
        n.getParam("state_module_laser3D",state);
        tend = time(0);
        if(difftime(tend, tstart)>TIMEOUT_ACTIVATION_MODULE){
            cout << "ATICA :: Gestion de Sistema :: Timeout en la activacion del modulo de LASER 3D"<< endl;
            return false;
        }
    }

    cout << "ATICA :: Gestion de Sistema :: Esperando la activacion de modulo GPS" << endl;

    tstart = time(0);
    while(state!=STATE_OK){
        n.getParam("state_module_gps",state);
        tend = time(0);
        if(difftime(tend, tstart)>TIMEOUT_ACTIVATION_MODULE){
            cout << "ATICA :: Gestion de Sistema :: Timeout en la activacion del modulo de GPS"<< endl;
            return false;
        }
    }

    cout << "ATICA :: Gestion de Sistema :: Esperando la activacion de modulo CAMARAS" << endl;

    tstart = time(0);
    while(state!=STATE_OK){
        n.getParam("state_module_camera",state);
        tend = time(0);
        if(difftime(tend, tstart)>TIMEOUT_ACTIVATION_MODULE){
            cout << "ATICA :: Gestion de Sistema :: Timeout en la activacion del modulo de CAMARAS"<< endl;
            return false;
        }
    }

    cout << "ATICA :: Gestion de Sistema :: Esperando la activacion de modulo TEACH&MAPPING" << endl;

 /**   tstart = time(0);
    while(state!=STATE_OK){
        n.getParam("estado_modulo_teachMapping",state);
        tend = time(0);
        if(difftime(tend, tstart)>TIMEOUT_ACTIVATION_MODULE){
            cout << "ATICA :: Gestion de Sistema :: Timeout en la activacion del modulo de TEACH&MAPPING"<< endl;
            return false;
        }
    }**/

    state_teleop=STATE_OK;
    state_nav=STATE_OK;
    state_drive=STATE_OK;
    state_gps=STATE_OK;
    state_laser3D=STATE_OK;
    state_laserBackRight=STATE_OK;
    state_laserBackLeft=STATE_OK;
    state_laserFront=STATE_OK;
    state_humanLocalization=STATE_OK;
    state_beacon=STATE_OK;
    //state_waypointsMap=STATE_OK;
    state_rangeDataFusion=STATE_OK;
    state_errors=STATE_OK;
    cout << "ATICA :: Gestion de Sistema :: Todos los modulos activados" << endl;
    return true;
}

void updateStatusModules(ros::NodeHandle n)
{
    n.getParam("state_module_teleop",state_teleop);
    n.getParam("state_module_nav",state_nav);
    n.getParam("state_module_drive",state_drive);
    n.getParam("state_module_gps",state_gps);
    n.getParam("state_module_laser3D",state_laser3D);
    n.getParam("state_module_laserBackRight",state_laserBackRight);
    n.getParam("state_module_laserBackLeft",state_laserBackLeft);
    n.getParam("state_module_laserFront",state_laserFront);
    n.getParam("state_module_humanLocalization",state_humanLocalization);
    n.getParam("state_module_beacon",state_beacon);
    //n.getParam("state_module_waypointsMap",state_waypointsMap);
    n.getParam("state_module_rangeDataFusion",state_rangeDataFusion);
    n.getParam("state_module_errors",state_errors);

}
bool modeRUN(int mode)
{
    bool runOK=false;
    Modulo_Gest_Sistema::msg_module_enable module_enable;
    switch (mode)
    {
        case MODE_TELEOP:
            if(modesAvailables[MODE_TELEOP])
            {
                actualMode=MODE_TELEOP;
                module_enable.id_modulo=ID_MOD_TELEOP;
                module_enable.submodo=SUBMODE_TELEOP_TELEOP;
                module_enable.activo=MODULE_ON;
                pub_module_enable.publish(module_enable);
                runOK=true;
            }
            else
                runOK=false;
            break;
        case MODE_START_ENGINE:
            if(modesAvailables[MODE_START_ENGINE])
            {
                actualMode=MODE_START_ENGINE;
                module_enable.id_modulo=ID_MOD_TELEOP;
                module_enable.submodo=SUBMODE_TELEOP_START_ENGINE;
                module_enable.activo=MODULE_ON;
                pub_module_enable.publish(module_enable);
                runOK=true;
            }
            else
                runOK=false;
            break;
        case MODE_STOP_ENGINE:
            if(modesAvailables[MODE_START_ENGINE])
            {
                actualMode=MODE_STOP_ENGINE;
                module_enable.id_modulo=ID_MOD_TELEOP;
                module_enable.submodo=SUBMODE_TELEOP_STOP_ENGINE;
                module_enable.activo=MODULE_ON;
                pub_module_enable.publish(module_enable);
                runOK=true;
            }
            else
                runOK=false;
            break;
        case MODE_ENGAGE_BRAKE:
            if(modesAvailables[MODE_START_ENGINE])
            {
                actualMode=MODE_ENGAGE_BRAKE;
                module_enable.id_modulo=ID_MOD_TELEOP;
                module_enable.submodo=SUBMODE_TELEOP_ENGAGE_BRAKE;
                module_enable.activo=MODULE_ON;
                pub_module_enable.publish(module_enable);
                runOK=true;
            }
            else
                runOK=false;
            break;
        case MODE_PLAN:
            if(modesAvailables[MODE_START_ENGINE])
            {
                actualMode=MODE_PLAN;
                module_enable.id_modulo=ID_MOD_NAVIGATE;
                module_enable.submodo=SUBMODE_NAV_PLAN;
                module_enable.activo=MODULE_ON;
                pub_module_enable.publish(module_enable);
                runOK=true;
            }
            else
                runOK=false;
            break;
        case MODE_COME_TO_ME:
            if(modesAvailables[MODE_START_ENGINE])
            {
                actualMode=MODE_COME_TO_ME;
                module_enable.id_modulo=ID_MOD_NAVIGATE;
                module_enable.submodo=SUBMODE_NAV_PLAN;
                module_enable.activo=MODULE_ON;
                pub_module_enable.publish(module_enable);
                runOK=true;
            }
            else
                runOK=false;
            break;
        case MODE_FOLLOW_ME:
            if(modesAvailables[MODE_START_ENGINE])
            {
                actualMode=MODE_FOLLOW_ME;
                module_enable.id_modulo=ID_MOD_NAVIGATE;
                module_enable.submodo=SUBMODE_NAV_FOLLOW_ME;
                module_enable.activo=MODULE_ON;
                pub_module_enable.publish(module_enable);
                runOK=true;
            }
            else
                runOK=false;
            break;
        case MODE_MAPPING:
            if(modesAvailables[MODE_START_ENGINE])
            {
                module_enable.id_modulo=ID_MOD_MAPPING;
                module_enable.submodo=SUBMODE_MAPPING_UNDFD;
                module_enable.activo=MODULE_ON;
                pub_module_enable.publish(module_enable);
                runOK=true;
            }
            else
                runOK=false;
            break;
        case MODE_TEACH:
            if(modesAvailables[MODE_START_ENGINE])
            {
                module_enable.id_modulo=ID_MOD_TEACH;
                module_enable.submodo=SUBMODE_TEACH_UNDFD;
                module_enable.activo=MODULE_ON;
                pub_module_enable.publish(module_enable);
                runOK=true;

            }
            else
                runOK=false;
            break;
        default:
            break;
       }
       return runOK;
}
void modeSTOP(int mode)
{
    Modulo_Gest_Sistema::msg_module_enable module_enable;
    switch (mode)
    {
        case MODE_PLAN:
            module_enable.id_modulo=ID_MOD_NAVIGATE;
            module_enable.submodo=SUBMODE_NAV_PLAN;
            module_enable.activo=MODULE_PAUSE;
            pub_module_enable.publish(module_enable);
            break;
        case MODE_COME_TO_ME:
            module_enable.id_modulo=ID_MOD_NAVIGATE;
            module_enable.submodo=SUBMODE_NAV_COME_TO_ME;
            module_enable.activo=MODULE_PAUSE;
            pub_module_enable.publish(module_enable);
            break;
        case MODE_FOLLOW_ME:
            module_enable.id_modulo=ID_MOD_NAVIGATE;
            module_enable.submodo=SUBMODE_NAV_FOLLOW_ME;
            module_enable.activo=MODULE_PAUSE;
            pub_module_enable.publish(module_enable);
            break;
        default:
            break;
    }
}
void modeEXIT(int mode)
{
   Modulo_Gest_Sistema::msg_module_enable module_enable;
   switch (mode)
   {
        case MODE_PLAN:
            module_enable.id_modulo=ID_MOD_NAVIGATE;
            module_enable.submodo=SUBMODE_NAV_PLAN;
            module_enable.activo=MODULE_OFF;
            pub_module_enable.publish(module_enable);
            break;
        case MODE_COME_TO_ME:
            module_enable.id_modulo=ID_MOD_NAVIGATE;
            module_enable.submodo=SUBMODE_NAV_COME_TO_ME;
            module_enable.activo=MODULE_OFF;
            pub_module_enable.publish(module_enable);
            break;
        case MODE_TELEOP:
            module_enable.id_modulo=ID_MOD_TELEOP;
            module_enable.submodo=SUBMODE_TELEOP_TELEOP;
            module_enable.activo=MODULE_OFF;
            pub_module_enable.publish(module_enable);
        case MODE_START_ENGINE:
            module_enable.id_modulo=ID_MOD_TELEOP;
            module_enable.submodo=SUBMODE_TELEOP_START_ENGINE;
            module_enable.activo=MODULE_OFF;
            pub_module_enable.publish(module_enable);
        case MODE_STOP_ENGINE:
            module_enable.id_modulo=ID_MOD_TELEOP;
            module_enable.submodo=SUBMODE_TELEOP_STOP_ENGINE;
            module_enable.activo=MODULE_OFF;
            pub_module_enable.publish(module_enable);
            break;
         case MODE_ENGAGE_BRAKE:
            module_enable.id_modulo=ID_MOD_TELEOP;
            module_enable.submodo=SUBMODE_TELEOP_ENGAGE_BRAKE;
            module_enable.activo=MODULE_OFF;
            pub_module_enable.publish(module_enable);
         default:
            break;
    }
}
void modeRESUME(int mode)
{
   Modulo_Gest_Sistema::msg_module_enable module_enable;
   switch (mode)
   {
        case MODE_PLAN:
            module_enable.id_modulo=ID_MOD_NAVIGATE;
            module_enable.submodo=SUBMODE_NAV_PLAN;
            module_enable.activo=MODULE_RESUME;
            pub_module_enable.publish(module_enable);
            break;
        case MODE_COME_TO_ME:
            module_enable.id_modulo=ID_MOD_NAVIGATE;
            module_enable.submodo=SUBMODE_NAV_COME_TO_ME;
            module_enable.activo=MODULE_RESUME;
            pub_module_enable.publish(module_enable);
            break;
        case MODE_FOLLOW_ME:
            module_enable.id_modulo=ID_MOD_NAVIGATE;
            module_enable.submodo=SUBMODE_NAV_FOLLOW_ME;
            module_enable.activo=MODULE_RESUME;
            pub_module_enable.publish(module_enable);
            break;
         default:
            break;
    }
}



void fcn_sub_available_mode(const Modulo_Gest_Sistema::msg_available_mode msg)
{
    for(int i=0;i<msg.available_mode.size();i++)
        modesAvailables[i]=msg.available_mode[i];
}