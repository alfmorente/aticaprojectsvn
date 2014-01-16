#include "../include/Modulo_Gest_Sistema/gest_sistema.h"

using namespace std;

ros::Publisher pub_module_enable;
ros::Publisher pub_error;
ros::Publisher pub_mode;


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
        pub_module_enable = n.advertise<Modulo_Gest_Sistema::msg_module_enable > ("module_enable", 1000);
        pub_error = n.advertise<Modulo_Gest_Sistema::msg_error > ("error", 1000);
        pub_mode = n.advertise<Modulo_Gest_Sistema::msg_mode > ("mode", 1000);

        // Creacion de suscriptores
        //ros::Subscriber sub_errores = n.subscribe("error", 1000, fcn_sub_errores);
        ros::Subscriber sub_mode = n.subscribe("mode", 1000, fcn_sub_modo);

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
void fcn_sub_modo(const Modulo_Gest_Sistema::msg_mode msg)
{

    Modulo_Gest_Sistema::msg_module_enable module_enable;
    Modulo_Gest_Sistema::msg_error error;
    Modulo_Gest_Sistema::msg_mode new_mode;
    Modulo_Gest_Sistema::msg_mode last_mode;

    //Si llega peticion del modo desde la UCR
    if(msg.status==MODE_REQUEST)
    {
            switch (msg.mode)
            {
                //El caso de modo Neutral solo le puede llegar a traves del modulo de errrores por falli critico
                case MODE_NEUTRAL:
                    if(actualMode==MODE_COME_TO_ME)
                    {
                           module_enable.id_modulo=ID_MOD_NAVIGATE;
                           module_enable.submodo=SUBMODE_NAV_COME_TO_ME;
                           module_enable.activo=MODULE_OFF;
                           pub_module_enable.publish(module_enable);
                    }
                    else if(actualMode==MODE_PLAN)
                    {
                           module_enable.id_modulo=ID_MOD_NAVIGATE;
                           module_enable.submodo=SUBMODE_NAV_PLAN;
                           module_enable.activo=MODULE_OFF;
                           pub_module_enable.publish(module_enable);

                    }
                    else if(actualMode==MODE_TELEOP)
                    {
                           module_enable.id_modulo=ID_MOD_TELEOP;
                           module_enable.submodo=SUBMODE_TELEOP_TELEOP;
                           module_enable.activo=MODULE_OFF;
                           pub_module_enable.publish(module_enable);

                    }
                    else if(actualMode==MODE_START_ENGINE)
                    {
                           module_enable.id_modulo=ID_MOD_TELEOP;
                           module_enable.submodo=SUBMODE_TELEOP_START_ENGINE;
                           module_enable.activo=MODULE_OFF;
                           pub_module_enable.publish(module_enable);

                    }
                    else if(actualMode==MODE_STOP_ENGINE)
                    {
                           module_enable.id_modulo=ID_MOD_TELEOP;
                           module_enable.submodo=SUBMODE_TELEOP_STOP_ENGINE;
                           module_enable.activo=MODULE_OFF;
                           pub_module_enable.publish(module_enable);

                    }
                    else if(actualMode==MODE_ENGAGE_BRAKE)
                    {
                           module_enable.id_modulo=ID_MOD_TELEOP;
                           module_enable.submodo=SUBMODE_TELEOP_ENGAGE_BRAKE;
                           module_enable.activo=MODULE_OFF;
                           pub_module_enable.publish(module_enable);

                    }
                    else if(actualMode==MODE_FOLLOW_ME)
                    {
                           module_enable.id_modulo=ID_MOD_NAVIGATE;
                           module_enable.submodo=SUBMODE_NAV_FOLLOW_ME;
                           module_enable.activo=MODULE_OFF;
                           pub_module_enable.publish(module_enable);
                    }
                    new_mode.mode=MODE_NEUTRAL;
                    new_mode.status=MODE_START;
                    pub_mode.publish(msg);
                    break;
                case MODE_TELEOP:
                    if(state_drive!=STATE_ERROR && state_teleop!=STATE_ERROR && state_laserBackRight!=STATE_ERROR && state_laserBackLeft!=STATE_ERROR && state_laserFront!=STATE_ERROR)
                    {
                        actualMode=MODE_TELEOP;
                        module_enable.id_modulo=ID_MOD_TELEOP;
                        module_enable.submodo=SUBMODE_TELEOP_TELEOP;
                        module_enable.activo=MODULE_ON;
                        pub_module_enable.publish(module_enable);

                        new_mode.mode=msg.mode;
                        new_mode.status=MODE_START;
                        pub_mode.publish(msg);
                    }
                    else
                    {
                        error.ID_subsystem=SUBS_MANAGE_SYSTEM;
                        error.type_error=ALARM_UNDEFINED;
                        if(state_teleop==STATE_ERROR)
                             error.ID_error=ERROR_MODE_DRIVING;
                        else if(state_drive==STATE_ERROR)
                             error.ID_error=ERROR_MODE_DRIVING;
                        else if(state_laserFront==STATE_ERROR)
                             error.ID_error=ERROR_MODE_LASER;
                        else if(state_laserBackRight==STATE_ERROR)
                             error.ID_error=ERROR_MODE_LASER;
                        else if(state_laserBackLeft==STATE_ERROR)
                             error.ID_error=ERROR_MODE_LASER;
                        pub_error.publish(error);
                    }
                    break;
                case MODE_START_ENGINE:
                    if(state_drive!=STATE_ERROR && state_teleop!=STATE_ERROR)
                    {
                        actualMode=MODE_START_ENGINE;
                        module_enable.id_modulo=ID_MOD_TELEOP;
                        module_enable.submodo=SUBMODE_TELEOP_START_ENGINE;
                        module_enable.activo=MODULE_ON;
                        pub_module_enable.publish(module_enable);
                    }
                    else
                    {
                        error.ID_subsystem=SUBS_MANAGE_SYSTEM;
                        error.type_error=ALARM_UNDEFINED;
                        if(state_teleop==STATE_ERROR)
                             error.ID_error=ERROR_MODE_DRIVING;
                        else if(state_drive==STATE_ERROR)
                             error.ID_error=ERROR_MODE_DRIVING;
                        pub_error.publish(error);

                        new_mode.mode=msg.mode;
                        new_mode.status=MODE_START;
                        pub_mode.publish(msg);
                    }
                    break;
                case MODE_STOP_ENGINE:
                    if(state_drive!=STATE_ERROR && state_teleop!=STATE_ERROR)
                    {
                        actualMode=MODE_STOP_ENGINE;
                        module_enable.id_modulo=ID_MOD_TELEOP;
                        module_enable.submodo=SUBMODE_TELEOP_STOP_ENGINE;
                        module_enable.activo=MODULE_ON;
                        pub_module_enable.publish(module_enable);
                    }
                    else
                    {
                        error.ID_subsystem=SUBS_MANAGE_SYSTEM;
                        error.type_error=ALARM_UNDEFINED;
                        if(state_teleop==STATE_ERROR)
                             error.ID_error=ERROR_MODE_DRIVING;
                        else if(state_drive==STATE_ERROR)
                             error.ID_error=ERROR_MODE_DRIVING;
                        pub_error.publish(error);
                    }
                    break;
                case MODE_ENGAGE_BRAKE:
                    if(state_drive!=STATE_ERROR && state_teleop!=STATE_ERROR)
                    {
                        actualMode=MODE_ENGAGE_BRAKE;
                        module_enable.id_modulo=ID_MOD_TELEOP;
                        module_enable.submodo=SUBMODE_TELEOP_ENGAGE_BRAKE;
                        module_enable.activo=MODULE_ON;
                        pub_module_enable.publish(module_enable);

                        new_mode.mode=msg.mode;
                        new_mode.status=MODE_START;
                        pub_mode.publish(msg);
                    }
                    else
                    {
                        error.ID_subsystem=SUBS_MANAGE_SYSTEM;
                        error.type_error=ALARM_UNDEFINED;
                        if(state_teleop==STATE_ERROR)
                             error.ID_error=ERROR_MODE_DRIVING;
                        else if(state_drive==STATE_ERROR)
                             error.ID_error=ERROR_MODE_DRIVING;
                        pub_error.publish(error);
                    }
                    break;
                case MODE_PLAN:
                    if(state_drive!=STATE_ERROR && state_nav!=STATE_ERROR && state_laserBackRight!=STATE_ERROR && state_laserBackLeft!=STATE_ERROR && state_laserFront!=STATE_ERROR)
                    {
                        actualMode=MODE_PLAN;
                        module_enable.id_modulo=ID_MOD_NAVIGATE;
                        module_enable.submodo=SUBMODE_NAV_PLAN;
                        module_enable.activo=MODULE_ON;
                        pub_module_enable.publish(module_enable);
                      
                        new_mode.mode=msg.mode;
                        new_mode.status=MODE_START;
                        pub_mode.publish(msg);
                    }
                    else
                    {
                        error.ID_subsystem=SUBS_MANAGE_SYSTEM;
                        error.type_error=ALARM_UNDEFINED;
                        if(state_drive==STATE_ERROR)
                             error.ID_error=ERROR_MODE_DRIVING;
                        else if(state_nav==STATE_ERROR)
                             error.ID_error=ERROR_MODE_NAVIGATION;
                        else if(state_laserFront==STATE_ERROR)
                             error.ID_error=ERROR_MODE_LASER;
                        else if(state_laserBackRight==STATE_ERROR)
                             error.ID_error=ERROR_MODE_LASER;
                        else if(state_laserBackLeft==STATE_ERROR)
                             error.ID_error=ERROR_MODE_LASER;
                        else if(state_gps==STATE_ERROR)
                             error.ID_error=ERROR_MODE_GPS;
                        pub_error.publish(error);
                    }
                    break;
                case MODE_COME_TO_ME:
                    if(state_drive!=STATE_ERROR  && state_nav!=STATE_ERROR && state_humanLocalization!=STATE_ERROR && state_laserBackRight!=STATE_ERROR && state_laserBackLeft!=STATE_ERROR && state_laserFront!=STATE_ERROR)
                    {
                        actualMode=MODE_COME_TO_ME;
                        module_enable.id_modulo=ID_MOD_NAVIGATE;
                        module_enable.submodo=SUBMODE_NAV_PLAN;
                        module_enable.activo=MODULE_ON;
                        pub_module_enable.publish(module_enable);

                        new_mode.mode=msg.mode;
                        new_mode.status=MODE_START;
                        pub_mode.publish(msg);
                    }
                    else
                    {
                        error.ID_subsystem=SUBS_MANAGE_SYSTEM;
                        error.type_error=ALARM_UNDEFINED;
                        if(state_drive==STATE_ERROR)
                             error.ID_error=ERROR_MODE_DRIVING;
                        else if(state_nav==STATE_ERROR)
                             error.ID_error=ERROR_MODE_NAVIGATION;
                        else if(state_humanLocalization==STATE_ERROR)
                             error.ID_error=ERROR_MODE_NAVIGATION;
                        else if(state_laserFront==STATE_ERROR)
                             error.ID_error=ERROR_MODE_LASER;
                        else if(state_laserBackRight==STATE_ERROR)
                             error.ID_error=ERROR_MODE_LASER;
                        else if(state_laserBackLeft==STATE_ERROR)
                             error.ID_error=ERROR_MODE_LASER;
                        else if(state_gps==STATE_ERROR)
                             error.ID_error=ERROR_MODE_GPS;
                        pub_error.publish(error);
                    }
                    break;
                case MODE_FOLLOW_ME:
                    if(state_drive!=STATE_ERROR  && state_nav!=STATE_ERROR && state_humanLocalization!=STATE_ERROR && state_laserBackRight!=STATE_ERROR && state_laserBackLeft!=STATE_ERROR && state_laserFront!=STATE_ERROR && state_gps!=STATE_ERROR && state_beacon!=STATE_ERROR && state_rangeDataFusion)
                    {
                        actualMode=MODE_FOLLOW_ME;
                        module_enable.id_modulo=ID_MOD_NAVIGATE;
                        module_enable.submodo=SUBMODE_NAV_FOLLOW_ME;
                        module_enable.activo=MODULE_ON;
                        pub_module_enable.publish(module_enable);

                        new_mode.mode=msg.mode;
                        new_mode.status=MODE_START;
                        pub_mode.publish(msg);
                    }
                    else
                    {
                        error.ID_subsystem=SUBS_MANAGE_SYSTEM;
                        error.type_error=ALARM_UNDEFINED;
                        if(state_drive==STATE_ERROR)
                             error.ID_error=ERROR_MODE_DRIVING;
                        else if(state_nav==STATE_ERROR)
                             error.ID_error=ERROR_MODE_NAVIGATION;
                        else if(state_humanLocalization==STATE_ERROR)
                             error.ID_error=ERROR_MODE_NAVIGATION;
                        else if(state_rangeDataFusion==STATE_ERROR)
                             error.ID_error=ERROR_MODE_NAVIGATION;
                        else if(state_laserFront==STATE_ERROR)
                             error.ID_error=ERROR_MODE_LASER;
                        else if(state_laserBackRight==STATE_ERROR)
                             error.ID_error=ERROR_MODE_LASER;
                        else if(state_laserBackLeft==STATE_ERROR)
                             error.ID_error=ERROR_MODE_LASER;
                        else if(state_gps==STATE_ERROR)
                             error.ID_error=ERROR_MODE_GPS;
                        pub_error.publish(error);
                    }
                    break;
                case MODE_MAPPING:
                    if(state_laser3D!=STATE_ERROR)
                    {
                        module_enable.id_modulo=ID_MOD_MAPPING;
                        module_enable.submodo=SUBMODE_MAPPING_UNDFD;
                        module_enable.activo=MODULE_ON;
                        pub_module_enable.publish(module_enable);

                        new_mode.mode=msg.mode;
                        new_mode.status=MODE_START;
                        pub_mode.publish(msg);

                    }
                    else
                    {
                        error.ID_subsystem=SUBS_MANAGE_SYSTEM;
                        error.type_error=ALARM_UNDEFINED;
                        error.ID_error=ERROR_MODE_LASER;
                        pub_error.publish(error);
                    }
                    break;
                case MODE_TEACH:
                    if(state_gps!=STATE_ERROR)
                    {
                        module_enable.id_modulo=ID_MOD_TEACH;
                        module_enable.submodo=SUBMODE_TEACH_UNDFD;
                        module_enable.activo=MODULE_ON;
                        pub_module_enable.publish(module_enable);

                        new_mode.mode=msg.mode;
                        new_mode.status=MODE_START;
                        pub_mode.publish(msg);
                    }
                    else
                    {
                        error.ID_subsystem=SUBS_MANAGE_SYSTEM;
                        error.type_error=ALARM_UNDEFINED;
                        error.ID_error=ERROR_MODE_GPS;
                        pub_error.publish(error);
                    }
                    break;
                default:
                    break;
               }
    }
    //Si llega un EXIT del modo actual desde la UCR
    else if(msg.status==MODE_EXIT)
    {
            if(msg.mode==actualMode)
            {
                actualMode=MODE_NEUTRAL;
                new_mode.mode=MODE_NEUTRAL;
                new_mode.status=MODE_START;
                pub_mode.publish(msg);
            }

            switch (msg.mode)
            {
                case MODE_TELEOP:
                    module_enable.id_modulo=ID_MOD_TELEOP;
                    module_enable.submodo=SUBMODE_TELEOP_TELEOP;
                    module_enable.activo=MODULE_OFF;
                    pub_module_enable.publish(module_enable);
                    break;
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
                case MODE_FOLLOW_ME:
                    module_enable.id_modulo=ID_MOD_NAVIGATE;
                    module_enable.submodo=SUBMODE_NAV_FOLLOW_ME;
                    module_enable.activo=MODULE_OFF;
                    pub_module_enable.publish(module_enable);
                    break;
                case MODE_MAPPING:
                    module_enable.id_modulo=ID_MOD_MAPPING;
                    module_enable.submodo=SUBMODE_MAPPING_UNDFD;
                    module_enable.activo=MODULE_OFF;
                    pub_module_enable.publish(module_enable);
                    break;
                case MODE_TEACH:
                    module_enable.id_modulo=ID_MOD_TEACH;
                    module_enable.submodo=SUBMODE_TEACH_UNDFD;
                    module_enable.activo=MODULE_OFF;
                    pub_module_enable.publish(module_enable);
                    break;
                default:
                    break;
           }
    }
    //Si llega un STOP del modo desde la UCR
    else if(msg.status==MODE_STOP)
    {
            switch (msg.mode)
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
    else if(msg.status==MODE_RUN)
    {
           switch (msg.mode)
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
    else if(msg.status==MODE_FINISH)
    {
           switch (msg.mode)
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
            if(msg.mode==actualMode)
            {
                last_mode.mode=actualMode;
                last_mode.status=MODE_FINISH;
                pub_mode.publish(msg);

                actualMode=MODE_NEUTRAL;
                new_mode.mode=MODE_NEUTRAL;
                new_mode.status=MODE_START;
                pub_mode.publish(msg);
            }
    }

}

// Suscriptor de errores
void fcn_sub_errores(const Modulo_Gest_Sistema::msg_error msg)
{
  ROS_INFO("I heard a ERROR message");
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
