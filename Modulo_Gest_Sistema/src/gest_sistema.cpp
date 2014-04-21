#include "Modulo_Gest_Sistema/gest_sistema.h"
#include "Modulo_Gest_Sistema/interaction.h"
#include "../../Common_files/include/Common_files/constant.h"

using namespace std;


int main(int argc, char **argv)
{
   // Obtencion del modo de operacion y comprobacion de que es correcto
   int operationMode=getOperationMode(argc, argv);
   if (operationMode == 0) {
        return 1;
   }
  
  // Inicio de ROS   
  ros::init(argc, argv, "gest_sistema");

  // Manejador ROS
  ros::NodeHandle n;
  ROS_INFO("Operation mode: %d",operationMode);
  switch (operationMode) 
  {
        case OPERATION_MODE_DEBUG:
            // Funcionamiento del modo debug

            // Inicio de configuracion de los modulos
            ROS_INFO("Start activing modules");
            modeManagement(n,STATE_CONF);

            //Inicio variables del módulo
            emergencyACK=false;
            convoyACK=false;
            actualMode=MODE_MANUAL;
            for(unsigned int i=0;i<NUM_MODULES;i++)
                 modesAvailables[i]=ON;
            

            // Espera
            if(waitForAllModulesReady(n))
            {
                // Generación de publicadores
                  pub_module_enable = n.advertise<Common_files::msg_module_enable > ("modEnable", 1000);
                  pub_error = n.advertise<Common_files::msg_error > ("error", 1000);
                  pub_mode_error = n.advertise<Common_files::msg_mode > ("modeSE", 1000);
                  pub_mode_communication = n.advertise<Common_files::msg_mode > ("modeSC", 1000);
                  pub_mode_convoy = n.advertise<Common_files::msg_mode > ("modeSCN", 1000);                  
                  pub_fcn_aux = n.advertise<Common_files::msg_fcn_aux > ("engBrake", 1000);  
                  pub_emergency_stop = n.advertise<Common_files::msg_emergency_stop > ("emergSet", 1000);
                  pub_fcn_aux_ack = n.advertise<Common_files::msg_fcn_aux > ("fcnAuxACK", 1000);


                  // Creacion de suscriptores
                  //ros::Subscriber sub_errores = n.subscribe("error", 1000, fcn_sub_errores);
                  ros::Subscriber sub_mode_error = n.subscribe("modeES", 1000, fcn_sub_mode_error);
                  ros::Subscriber sub_mode_communication = n.subscribe("modeCS", 1000, fcn_sub_mode_communication);
                  ros::Subscriber sub_mode_convoy = n.subscribe("modeCNS", 1000, fcn_sub_mode_convoy);
                  ros::Subscriber sub_avilable= n.subscribe("avail", 1000, fcn_sub_available);
                  ros::Subscriber sub_fcn_aux = n.subscribe("fcnAux", 1000, fcn_sub_fcn_aux);
                  ros::Subscriber sub_switch = n.subscribe("switch", 1000, fcn_sub_switch); 
                  ros::Subscriber sub_emergency_stop = n.subscribe("emergInfo", 1000, fcn_sub_emergency_stop);         
                  ros::ServiceServer server=n.advertiseService("serviceParam",fcn_server_data);  
                  ros::spin();
            }
            else
            {
                modeManagement(n,STATE_OFF);
            }
            break;
        case OPERATION_MODE_RELEASE:
            // Funcionamiento del modo release

            break;
        case OPERATION_MODE_SIMULATION:
            // Funcionamiento del modo simulacion
            break;
        default:
            break;
    }
    return 0;
}

//Funcion servidor de datos (Devuelve el dato que se le solicita)
bool fcn_server_data(Common_files::srv_data::Request &req, Common_files::srv_data::Response &resp)
{

    if(req.param==PARAM_MODE)
    {
        ROS_INFO("Peticion del Modo Actual");        
        resp.value=actualMode;
        return true;
    }
    else
        return false;
}

/*******************************************************************************
 *******************************************************************************
 *                              SUSCRIPTORES
 * *****************************************************************************
 * ****************************************************************************/


//Subscriptor del modo al topic del módulo Communicator
void fcn_sub_mode_communication(const Common_files::msg_mode msg)
{
        Common_files::msg_mode mode;
        if(actualMode==MODE_MANUAL || stoppingCar)
            return;

        //Si llega cambio de modo desde la UCR
        if(msg.type_msg==SET)
        {
            carType=LEADER;
            if(msg.status==MODE_START)
            {
                if((actualMode==MODE_NEUTRAL) || (actualMode==MODE_CONVOY 
                    && (msg.mode==MODE_CONVOY_TELEOP || msg.mode==MODE_CONVOY_AUTO)))
                {
                    if(modeRUN(msg.mode))
                    {
                        mode.type_msg=INFO;
                        mode.mode=msg.mode;
                        mode.status=msg.status;
                        pub_mode_error.publish(mode);
                        pub_mode_communication.publish(mode);
                    }
                    else
                    {
                        ROS_INFO("Error to change mode");
                        Common_files::msg_error error;
                        error.id_subsystem=SUBS_SYSTEM_MGMNT;
                        error.id_error=MODE_NOT_AVAILABLE;
                        error.type_error=TOE_UNDEFINED;
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
                    Common_files::msg_mode mode;
                    mode.type_msg=INFO;
                    mode.mode=actualMode;
                    mode.status=MODE_EXIT;
                    pub_mode_communication.publish(mode);
                    pub_mode_error.publish(mode);
                    if(msg.mode!=MODE_CONVOY_TELEOP && msg.mode!=MODE_CONVOY_AUTO )
                        actualMode=MODE_NEUTRAL;
                    else
                        actualMode=MODE_CONVOY;                        
                        
                }
                else if(msg.status==MODE_RUN)
                    modeRESUME(msg.mode);
                else if(msg.status==MODE_STOP)
                    modeSTOP(msg.mode);
            }
        }
}

//Subscriptor del modo al topic del módulo Error
void fcn_sub_mode_error(const Common_files::msg_mode msg)
{
        if(actualMode==MODE_MANUAL || stoppingCar)
                 return;

        //Modo activo
        if(actualMode==msg.mode)
        {
            if(msg.type_msg==SET && msg.status==MODE_EXIT)
            {
                ROS_INFO("ERROR: MODE ACTUAL EXIT ");
                modeEXIT(msg.mode);
                Common_files::msg_mode mode;
                mode.type_msg=INFO;
                mode.mode=actualMode;
                mode.status=MODE_EXIT;
                pub_mode_error.publish(mode);
                actualMode=MODE_NEUTRAL;
                ROS_INFO("MODE NEUTRO");          
            }
        }
}


void fcn_sub_available(const Common_files::msg_available msg)
{
    for(unsigned int i=AVAILABLE_POS_REMOTE;i<=AVAILABLE_POS_TEACH;i++)
        modesAvailables[i]=msg.available[i];
}

void fcn_sub_fcn_aux(const Common_files::msg_fcn_aux msg)
{    
    Common_files::msg_fcn_aux msg_ack; 
    msg_ack=msg;
    msg_ack.type_msg=INFO;
    if(msg.type_msg==SET)
    {
        switch(msg.function)
        {
            case ENGINE:
                if(actualMode==MODE_MANUAL || stoppingCar)
                    return;
                if(msg.value==ON) 
                {
                     if(modesAvailables[AVAILABLE_POS_START_ENGINE])
                     {
                            pub_fcn_aux.publish(msg);
                            pub_fcn_aux_ack.publish(msg_ack);
                            ROS_INFO("START ENGINE");
                     }
                     else
                     {
                        ROS_INFO("FUNCTION ENGINE START AVAILABLE"); 
                        Common_files::msg_error error;
                        error.id_subsystem=SUBS_SYSTEM_MGMNT;
                        error.id_error=FUNCTION_NOT_AVAILABLE;
                        error.type_error=TOE_UNDEFINED;
                        pub_error.publish(error);
                     }                     
                }
                else 
                {
                    if(modesAvailables[AVAILABLE_POS_STOP_ENGINE])
                    {
                            pub_fcn_aux.publish(msg);
                            pub_fcn_aux_ack.publish(msg_ack);
                            ROS_INFO("STOP ENGINE");
                    }
                    else
                    {
                        ROS_INFO("FUNCTION ENGINE STOP AVAILABLE"); 
                        Common_files::msg_error error;
                        error.id_subsystem=SUBS_SYSTEM_MGMNT;
                        error.id_error=FUNCTION_NOT_AVAILABLE;
                        error.type_error=TOE_UNDEFINED;
                        pub_error.publish(error);
                    }                    
                }
                break;
            case BRAKE:
                if(actualMode==MODE_MANUAL || stoppingCar)
                    return;
                if(modesAvailables[AVAILABLE_POS_ENGAGE_BRAKE])
                {
                            pub_fcn_aux.publish(msg);
                            pub_fcn_aux_ack.publish(msg_ack);
                            ROS_INFO("ENGAGE BRAKE");
                }
                else
                {
                    ROS_INFO("FUNCTION ENGAGE BRAKE NOT AVAILABLE");                     
                    Common_files::msg_error error;
                    error.id_subsystem=SUBS_SYSTEM_MGMNT;
                    error.id_error=FUNCTION_NOT_AVAILABLE;
                    error.type_error=TOE_UNDEFINED;
                    pub_error.publish(error);
                }
                break;
            case TEACH:
                if(msg.value==ON)
                {
                    if(modesAvailables[AVAILABLE_POS_TEACH])
                    {
                            Common_files::msg_module_enable enable;
                            enable.id_module=ID_MOD_TEACH;
                            enable.status=MOD_ON;
                            enable.submode=SUBMODE_TEACH;
                            pub_module_enable.publish(enable);
                            pub_fcn_aux_ack.publish(msg_ack);
                            ROS_INFO("TEACH ON");
                    }
                    else
                    {
                        ROS_INFO("FUNCTION TEACH NOT AVAILABLE"); 
                        Common_files::msg_error error;
                        error.id_subsystem=SUBS_SYSTEM_MGMNT;
                        error.id_error=FUNCTION_NOT_AVAILABLE;
                        error.type_error=TOE_UNDEFINED;
                        pub_error.publish(error);
                    }
                    
                }
                else
                {
                        Common_files::msg_module_enable enable;
                        enable.id_module=ID_MOD_TEACH;
                        enable.status=MOD_OFF;
                        enable.submode=SUBMODE_TEACH;
                        pub_module_enable.publish(enable);
                        pub_fcn_aux_ack.publish(msg_ack);
                        ROS_INFO("TEACH OFF");
                }   
                break;
            case MAPPING:
                if(msg.value==ON)
                {
                    if(modesAvailables[AVAILABLE_POS_MAPPING])
                    {
                            Common_files::msg_module_enable enable;
                            enable.id_module=ID_MOD_MAPPING;
                            enable.status=MOD_ON;
                            enable.submode=SUBMODE_MAPPING;
                            pub_module_enable.publish(enable);
                            pub_fcn_aux_ack.publish(msg_ack);
                            ROS_INFO("MAPPING ON");
                    }
                    else
                    {
                        ROS_INFO("FUNCTION MAPPING NOT AVAILABLE"); 
                        Common_files::msg_error error;
                        error.id_subsystem=SUBS_SYSTEM_MGMNT;
                        error.id_error=FUNCTION_NOT_AVAILABLE;
                        error.type_error=TOE_UNDEFINED;
                        pub_error.publish(error);
                    }                    
                }
                else
                {
                            Common_files::msg_module_enable enable;
                            enable.id_module=ID_MOD_MAPPING;
                            enable.status=MOD_OFF;
                            enable.submode=SUBMODE_MAPPING;
                            pub_module_enable.publish(enable);
                            pub_fcn_aux_ack.publish(msg_ack);
                            ROS_INFO("MAPPING OFF");
                } 
                break;
          default:
                break;
        }
    }
}

void fcn_sub_emergency_stop(const Common_files::msg_emergency_stop msg)
{
    if(msg.value==INFO){
        emergencyACK=true;
        ROS_INFO("Recibido ACK Parada emergencia");
    }
}

void fcn_sub_switch(const Common_files::msg_switch msg)
{
    Common_files::msg_mode mode_aux;
    if(msg.value==MANUAL && actualMode!=MODE_MANUAL)
    {
        ROS_INFO("SWITCH MANUAL");
        ROS_INFO("EMERGENCY STOP");
        
        if(actualMode!=MODE_NEUTRAL)
        {
            modeEXIT(actualMode);
            /*mode_aux.type_msg=INFO;  Modificación ordenador Alfonso 9/4/14 
            mode_aux.mode=actualMode;
            mode_aux.status=MODE_EXIT;
            pub_mode_error.publish(mode_aux);
            pub_mode_communication.publish(mode_aux);
            actualMode=MODE_NEUTRAL;*/
        }
        else
        {
            emergencySTOP();
        }
        
        actualMode=MODE_MANUAL;
        mode_aux.type_msg=INFO;
        mode_aux.mode=MODE_MANUAL;
        mode_aux.status=MODE_RUN;
        pub_mode_error.publish(mode_aux);
        pub_mode_communication.publish(mode_aux);
        ROS_INFO("CAR MANUAL");           
 
    }
    else if(msg.value==TELEOP && actualMode==MODE_MANUAL)
    {
        ROS_INFO("SWITCH TELEOP");
        ROS_INFO("EMERGENCY STOP");

        emergencySTOP();
        ROS_INFO("EMERGENCY STOP DONE");
        mode_aux.type_msg=INFO;
        mode_aux.mode=MODE_MANUAL;
        mode_aux.status=MODE_EXIT;
        pub_mode_error.publish(mode_aux);
        pub_mode_communication.publish(mode_aux);
        ROS_INFO("CAR TELEOP");
        actualMode=MODE_NEUTRAL;
    
    }
    
}

//Subscriptor del modo al topic del módulo Convoy
void fcn_sub_mode_convoy(const Common_files::msg_mode msg)
{
     if(actualMode==MODE_MANUAL || stoppingCar)
         return;

    if(msg.type_msg==SET && msg.mode==MODE_CONVOY)
    {
        carType=FOLLOWER;        
        if(msg.status==MODE_START)
        {
 
            if(modeRUN(MODE_CONVOY))
            {
                    ROS_INFO("MODE CONVOY FOLLOWER ON ");
                    Common_files::msg_mode modeAct;
                    modeAct.type_msg=INFO;
                    modeAct.mode=MODE_CONVOY;
                    modeAct.status=MODE_START;
                    pub_mode_convoy.publish(modeAct);  
                    pub_mode_error.publish(modeAct);
            }
            else
            {
                   ROS_INFO("MODE CONVOY FOLLOWER ERROR ");
                   Common_files::msg_error error;
                   error.id_subsystem=SUBS_SYSTEM_MGMNT;
                   error.id_error=MODE_NOT_AVAILABLE;
                   error.type_error=TOE_UNDEFINED;
                   pub_error.publish(error);              
            }
        }
        else if(msg.status==MODE_EXIT)
        {
            modeEXIT(MODE_CONVOY);
            ROS_INFO("MODE CONVOY FOLLOWER ON ");
            Common_files::msg_mode modeAct;
            modeAct.type_msg=INFO;
            modeAct.mode=MODE_CONVOY;
            modeAct.status=MODE_EXIT;
            pub_mode_convoy.publish(modeAct);  
            pub_mode_error.publish(modeAct);            
        }
    }
    else if(msg.type_msg==INFO && msg.mode==MODE_CONVOY)
    {
        //el if es opcional
        //if((actualMode==MODE_NEUTRAL && msg.status==MODE_START)||(actualMode==MODE_CONVOY && && msg.status==MODE_EXIT))
                convoyACK=true;
    }

}
/*******************************************************************************
 *******************************************************************************
 *                              FUNCIONES PROPIAS
 * *****************************************************************************
 * ****************************************************************************/

void modeManagement(ros::NodeHandle n,int mode) {
    n.setParam("state_module_driving", mode);
    n.setParam("state_module_navigation", mode);
    n.setParam("state_module_remote", mode);
    n.setParam("state_module_error_management", mode);
    n.setParam("state_module_communication", mode);
    n.setParam("state_module_laser3D", mode);
    n.setParam("state_module_front_laser_1", mode);
    n.setParam("state_module_front_laser_1", mode);
    n.setParam("state_module_gps", mode);
    n.setParam("state_module_camera", mode);
    n.setParam("state_module_human_localization",mode);
    n.setParam("state_module_beacon",mode);
    n.setParam("state_module_range_data_fusion",mode);   
    n.setParam("state_module_convoy", mode);
}

bool waitForAllModulesReady(ros::NodeHandle n){
    
    int state;
    time_t tstart, tend; // gestiona los timeout's

    cout << "ATICA :: Gestion de Sistema :: Esperando la activacion de modulos" << endl;

    cout << "ATICA :: Gestion de Sistema :: Esperando la activacion de modulo CONDUCCION" << endl;
    
    tstart = time(0);
    do{
        n.getParam("state_module_driving",state);
        tend = time(0);
        if(difftime(tend, tstart)>TIMEOUT_ACTIVATION_MODULE){
            cout << "ATICA :: Gestion de Sistema :: Timeout en la activacion del modulo de CONDUCCION"<< endl;
            return false;
        }
    }while(state!=STATE_OK);

    cout << "ATICA :: Gestion de Sistema :: Esperando la activacion de modulo NAVEGACION" << endl;

    tstart = time(0);
    do{
        n.getParam("state_module_navigation",state);
        tend = time(0);
        if(difftime(tend, tstart)>TIMEOUT_ACTIVATION_MODULE){
            cout << "ATICA :: Gestion de Sistema :: Timeout en la activacion del modulo de NAVEGACION"<< endl;
            return false;
        }
    }while(state!=STATE_OK);

    cout << "ATICA :: Gestion de Sistema :: Esperando la activacion de modulo TELEOPERACION" << endl;

    tstart = time(0);
    do{
        n.getParam("state_module_remote",state);
        tend = time(0);
        if(difftime(tend, tstart)>TIMEOUT_ACTIVATION_MODULE){
            cout << "ATICA :: Gestion de Sistema :: Timeout en la activacion del modulo de TELEOPERACION"<< endl;
            return false;
        }
    }while(state!=STATE_OK);

    cout << "ATICA :: Gestion de Sistema :: Esperando la activacion de modulo GESTION DE ERRORES" << endl;

    tstart = time(0);
    do{
        n.getParam("state_module_error_management",state);
        tend = time(0);
        if(difftime(tend, tstart)>TIMEOUT_ACTIVATION_MODULE){
            cout << "ATICA :: Gestion de Sistema :: Timeout en la activacion del modulo de GESTION DE ERRORES"<< endl;
            return false;
        }
    }while(state!=STATE_OK);

    cout << "ATICA :: Gestion de Sistema :: Esperando la activacion de modulo COMUNICACIONES" << endl;

    tstart = time(0);
    do{
        n.getParam("state_module_communication",state);
        tend = time(0);
        if(difftime(tend, tstart)>TIMEOUT_ACTIVATION_MODULE){
            cout << "ATICA :: Gestion de Sistema :: Timeout en la activacion del modulo de COMUNICACIONES"<< endl;
            return false;
        }
    }while(state!=STATE_OK);


    cout << "ATICA :: Gestion de Sistema :: Esperando la activacion de modulo GPS" << endl;

    tstart = time(0);
    do{
        n.getParam("state_module_gps",state);
        tend = time(0);
        if(difftime(tend, tstart)>TIMEOUT_ACTIVATION_MODULE){
            cout << "ATICA :: Gestion de Sistema :: Timeout en la activacion del modulo de GPS"<< endl;
            return false;
        }
    }while(state!=STATE_OK);

    cout << "ATICA :: Gestion de Sistema :: Esperando la activacion de modulo CAMARAS" << endl;

    tstart = time(0);
    do{
        n.getParam("state_module_camera",state);
        tend = time(0);
        if(difftime(tend, tstart)>TIMEOUT_ACTIVATION_MODULE){
            cout << "ATICA :: Gestion de Sistema :: Timeout en la activacion del modulo de CAMARAS"<< endl;
            return false;
        }
    }while(state!=STATE_OK);

    cout << "ATICA :: Gestion de Sistema :: Esperando la activacion de modulo LASER FRONTAL 1" << endl;

    tstart = time(0);
    do{
        n.getParam("state_module_frontalLaser1",state);
        tend = time(0);
        if(difftime(tend, tstart)>TIMEOUT_ACTIVATION_MODULE){
            cout << "ATICA :: Gestion de Sistema :: Timeout en la activacion del modulo de LASER FRONTAL 1"<< endl;
            return false;
        }
    }while(state!=STATE_OK);

    tstart = time(0);
    do{
        n.getParam("state_module_frontalLaser2",state);
        tend = time(0);
        if(difftime(tend, tstart)>TIMEOUT_ACTIVATION_MODULE){
            cout << "ATICA :: Gestion de Sistema :: Timeout en la activacion del modulo de LASER FRONTAL 2"<< endl;
            return false;
        }
    }while(state!=STATE_OK);    

    cout << "ATICA :: Gestion de Sistema :: Esperando la activacion de modulo LASER 3D" << endl;

    tstart = time(0);
    do{
        n.getParam("state_module_laser3D",state);
        tend = time(0);
        if(difftime(tend, tstart)>TIMEOUT_ACTIVATION_MODULE){
            cout << "ATICA :: Gestion de Sistema :: Timeout en la activacion del modulo de LASER 3D"<< endl;
            return false;
        }
    }while(state!=STATE_OK); 
    
    cout << "ATICA :: Gestion de Sistema :: Esperando la activacion de modulo RANGE DATA FUSION" << endl;

    tstart = time(0);
    do{
        n.getParam("state_module_range_data_fusion",state);
        tend = time(0);
        if(difftime(tend, tstart)>TIMEOUT_ACTIVATION_MODULE){
            cout << "ATICA :: Gestion de Sistema :: Timeout en la activacion del modulo de RANGE DATA FUSION"<< endl;
            return false;
        }
    }while(state!=STATE_OK);
    
    cout << "ATICA :: Gestion de Sistema :: Esperando la activacion de modulo HUMAN LOCALIZATION" << endl;

    tstart = time(0);
    do{
        n.getParam("state_module_human_localization",state);
        tend = time(0);
        if(difftime(tend, tstart)>TIMEOUT_ACTIVATION_MODULE){
            cout << "ATICA :: Gestion de Sistema :: Timeout en la activacion del modulo de HUMAN LOCALIZATION"<< endl;
            return false;
        }
    }while(state!=STATE_OK);    

    cout << "ATICA :: Gestion de Sistema :: Esperando la activacion de modulo BEACON" << endl;

    tstart = time(0);
    do{
        n.getParam("state_module_beacon",state);
        tend = time(0);
        if(difftime(tend, tstart)>TIMEOUT_ACTIVATION_MODULE){
            cout << "ATICA :: Gestion de Sistema :: Timeout en la activacion del modulo de BEACON"<< endl;
            return false;
        }
    }while(state!=STATE_OK);
    
    state_remote=STATE_OK;
    state_navigation=STATE_OK;
    state_driving=STATE_OK;
    state_gps=STATE_OK;
    state_laser3D=STATE_OK;
    state_frontLaser1=STATE_OK;
    state_frontLaser2=STATE_OK;
    state_camera=STATE_OK;
    state_humanLocalization=STATE_OK;
    state_beacon=STATE_OK;
    state_rangeDataFusion=STATE_OK;
    state_errorMgmnt=STATE_OK;
    state_communication=STATE_OK;
    cout << "ATICA :: Gestion de Sistema :: Todos los modulos activados" << endl;
    return true;
}

void updateStatusModules(ros::NodeHandle n)
{
    n.getParam("state_module_remote",state_remote);
    n.getParam("state_module_navigation",state_navigation);
    n.getParam("state_module_driving",state_driving);
    n.getParam("state_module_gps",state_gps);
    n.getParam("state_module_laser3D",state_laser3D);
    n.getParam("state_module_front_laser_1",state_frontLaser1);
    n.getParam("state_module_front_laser_2",state_frontLaser2);
    n.getParam("state_module_camera",state_camera);
    n.getParam("state_module_human_localization",state_humanLocalization);
    n.getParam("state_module_beacon",state_beacon);
    n.getParam("state_module_convoy",state_convoy);
    n.getParam("state_module_range_data_fusion",state_rangeDataFusion);
    n.getParam("state_module_error_management",state_errorMgmnt);
    n.getParam("state_module_communication",state_communication);

}
bool modeRUN(int mode)
{
    bool runOK;
    Common_files::msg_module_enable module_enable;
    switch (mode)
    {
        case MODE_REMOTE:
            if(modesAvailables[AVAILABLE_POS_REMOTE])
            {
                ROS_INFO("MODE REMOTE ON");
                actualMode=MODE_REMOTE;
                module_enable.id_module=ID_MOD_REMOTE;
                module_enable.submode=SUBMODE_REMOTE;
                module_enable.status=MOD_ON;
                pub_module_enable.publish(module_enable);
                runOK=true;
            }
            else
                runOK=false;
            break;
        case MODE_PLAN:
            if(modesAvailables[AVAILABLE_POS_PLAN])
            {
                ROS_INFO("MODE PLAN ON");
                actualMode=MODE_PLAN;
                module_enable.id_module=ID_MOD_NAVIGATION;
                module_enable.submode=SUBMODE_NAV_PLAN;
                module_enable.status=MOD_ON;
                pub_module_enable.publish(module_enable);
                runOK=true;
            }
            else
                runOK=false;
            break;
        case MODE_COME_TO_ME:           
            if(modesAvailables[AVAILABLE_POS_COME_TO_ME])
            {
                ROS_INFO("MODE COME_TO_ME ON"); 
                actualMode=MODE_COME_TO_ME;
                module_enable.id_module=ID_MOD_NAVIGATION;
                module_enable.submode=SUBMODE_NAV_PLAN;
                module_enable.status=MOD_ON;
                pub_module_enable.publish(module_enable);
                runOK=true;
            }
            else
                runOK=false;
            break;
        case MODE_FOLLOW_ME: 
            if(modesAvailables[AVAILABLE_POS_FOLLOW_ME])
            {
                ROS_INFO("MODE FOLLOW_ME ON");
                actualMode=MODE_FOLLOW_ME;
                module_enable.id_module=ID_MOD_NAVIGATION;
                module_enable.submode=SUBMODE_NAV_FOLLOW_ME;
                module_enable.status=MOD_ON;
                pub_module_enable.publish(module_enable);
                runOK=true;
            }
            else
                runOK=false;
            break;
         case MODE_CONVOY:
            if(carType==LEADER)
            {
                
                if(modesAvailables[AVAILABLE_POS_CONVOY])
                {
                    module_enable.id_module=ID_MOD_CONVOY;
                    module_enable.submode=SUBMODE_CONVOY;
                    module_enable.status=MOD_ON;
                    pub_module_enable.publish(module_enable);
                    convoyACK=false;
                    if(timerACK(5,CONVOY_ACK))
                    {
                            ROS_INFO("MODE CONVOY ON"); 
                            actualMode=MODE_CONVOY;
                            runOK=true;
                    }
                    else
                        runOK=false;
                }
                else
                    runOK=false;
                break; 
            }
            else
            {
                if(modesAvailables[AVAILABLE_POS_CONVOY])
                {
                    ROS_INFO("MODE CONVOY FOLLOWER ON");                 
                    actualMode=MODE_CONVOY_AUTO;
                    module_enable.id_module=ID_MOD_NAVIGATION;
                    module_enable.submode=SUBMODE_CONVOY; // Dfinir el submode nav_convoy;
                    module_enable.status=MOD_ON;
                    pub_module_enable.publish(module_enable);
                    runOK=true;
                }
                else
                    runOK=false;
            }
            break;       
        case MODE_CONVOY_TELEOP:
            if(modesAvailables[AVAILABLE_POS_CONVOY_TELEOP] && actualMode==MODE_CONVOY)
            {
                ROS_INFO("MODE CONVOY TELEOP ON");
                actualMode=MODE_CONVOY_TELEOP;
                module_enable.id_module=ID_MOD_REMOTE;
                module_enable.submode=SUBMODE_REMOTE;
                module_enable.status=MOD_ON;
                pub_module_enable.publish(module_enable);
                runOK=true;
            }
            else
                runOK=false;
            break;
        case MODE_CONVOY_AUTO:
            if(modesAvailables[AVAILABLE_POS_CONVOY_AUTO] && actualMode==MODE_CONVOY)
            {
                ROS_INFO("MODE CONVOY AUTO ON");                
                actualMode=MODE_CONVOY_AUTO;
                module_enable.id_module=ID_MOD_NAVIGATION;
                module_enable.submode=SUBMODE_NAV_CONVOY;
                module_enable.status=MOD_ON;
                pub_module_enable.publish(module_enable);
                runOK=true;
            }
            else
                runOK=false;
            break;            
        default:
            runOK=false;
            break;
       }
       return runOK;
}
void modeSTOP(int mode)
{
    Common_files::msg_module_enable module_enable;
    switch (mode)
    {
        case MODE_PLAN:
            ROS_INFO("MODE PLAN STOP");             
            module_enable.id_module=ID_MOD_NAVIGATION;
            module_enable.submode=SUBMODE_NAV_PLAN;
            module_enable.status=MOD_PAUSE;
            pub_module_enable.publish(module_enable);
            break;
        case MODE_COME_TO_ME:
            ROS_INFO("MODE COME_TO_ME STOP");             
            module_enable.id_module=ID_MOD_NAVIGATION;
            module_enable.submode=SUBMODE_NAV_COME_TO_ME;
            module_enable.status=MOD_PAUSE;
            pub_module_enable.publish(module_enable);
            break;
        case MODE_FOLLOW_ME:
            ROS_INFO("MODE FOLLOW_ME STOP");             
            module_enable.id_module=ID_MOD_NAVIGATION;
            module_enable.submode=SUBMODE_NAV_FOLLOW_ME;
            module_enable.status=MOD_PAUSE;
            pub_module_enable.publish(module_enable);
            break;
        case MODE_CONVOY_AUTO:
            ROS_INFO("MODE CONVOY AUTO STOP");             
            module_enable.id_module=ID_MOD_NAVIGATION;
            module_enable.submode=SUBMODE_NAV_CONVOY;
            module_enable.status=MOD_PAUSE;
            pub_module_enable.publish(module_enable); 
            break;
        default:
            break;
    }
}
bool modeEXIT(int mode)
{
   bool exitOK;
   ROS_INFO("EMERGENCY STOP");
   emergencySTOP();
   {
        Common_files::msg_module_enable module_enable;
        switch (mode)
        {
             case MODE_PLAN:
                 ROS_INFO("MODE PLAN EXIT");             
                 module_enable.id_module=ID_MOD_NAVIGATION;
                 module_enable.submode=SUBMODE_NAV_PLAN;
                 module_enable.status=MOD_OFF;
                 pub_module_enable.publish(module_enable);
                 exitOK=true;
                 break;
             case MODE_COME_TO_ME:
                 ROS_INFO("MODE COME_TO_ME EXIT");             
                 module_enable.id_module=ID_MOD_NAVIGATION;
                 module_enable.submode=SUBMODE_NAV_COME_TO_ME;
                 module_enable.status=MOD_OFF;
                 pub_module_enable.publish(module_enable);   
                 exitOK=true;                 
                 break;
             case MODE_FOLLOW_ME:
                 ROS_INFO("MODE FOLLOW_ME EXIT"); 
                 module_enable.id_module=ID_MOD_NAVIGATION;
                 module_enable.submode=SUBMODE_NAV_FOLLOW_ME;
                 module_enable.status=MOD_OFF;
                 pub_module_enable.publish(module_enable);   
                 exitOK=true;                 
                 break;            
             case MODE_REMOTE:
                 ROS_INFO("MODE REMOTE EXIT");             
                 module_enable.id_module=ID_MOD_REMOTE;
                 module_enable.submode=SUBMODE_REMOTE;
                 module_enable.status=MOD_OFF;
                 pub_module_enable.publish(module_enable);   
                 exitOK=true;                 
                 break;
             case MODE_CONVOY:
                 if(carType==LEADER)
                 {
                    ROS_INFO("MODE CONVOY EXIT");             
                    module_enable.id_module=ID_MOD_CONVOY;
                    module_enable.submode=SUBMODE_CONVOY;
                    module_enable.status=MOD_OFF;
                    pub_module_enable.publish(module_enable);  
                    convoyACK=false;
                    if(timerACK(5,CONVOY_ACK))
                          exitOK=true;       
                    else
                          exitOK=false;
                    break;
                 }
                 else 
                 {
                    ROS_INFO("MODE CONVOY FOLLOWER ON"); 
                    module_enable.id_module=ID_MOD_NAVIGATION;
                    module_enable.submode=SUBMODE_CONVOY; // Dfinir el submode nav_convoy;
                    module_enable.status=MOD_OFF;
                    pub_module_enable.publish(module_enable);
                    exitOK=true;
                 }
                 break;   
             case MODE_CONVOY_AUTO:
                ROS_INFO("MODE CONVOY AUTO EXIT");             
                module_enable.id_module=ID_MOD_NAVIGATION;
                module_enable.submode=SUBMODE_NAV_CONVOY;
                module_enable.status=MOD_OFF;
                pub_module_enable.publish(module_enable);
                exitOK=true;
                break; 
             case MODE_CONVOY_TELEOP:
                ROS_INFO("MODE CONVOY TELEOP EXIT");             
                module_enable.id_module=ID_MOD_REMOTE;
                module_enable.submode=SUBMODE_REMOTE;
                module_enable.status=MOD_OFF;
                pub_module_enable.publish(module_enable);
                exitOK=true;
                break;                   
              default:
                 exitOK=false;
                 break;
        }

    } 
    /**else
       exitOK=false;**/
    return exitOK;

}
void modeRESUME(int mode)
{
   Common_files::msg_module_enable module_enable;
   switch (mode)
   {
       case MODE_PLAN:
            ROS_INFO("MODE PLAN RESUME");            
            module_enable.id_module=ID_MOD_NAVIGATION;
            module_enable.submode=SUBMODE_NAV_PLAN;
            module_enable.status=MOD_ON;
            pub_module_enable.publish(module_enable);
            break;
        case MODE_COME_TO_ME:
            ROS_INFO("MODE COME_TO_ME RESUME");             
            module_enable.id_module=ID_MOD_NAVIGATION;
            module_enable.submode=SUBMODE_NAV_COME_TO_ME;
            module_enable.status=MOD_ON;
            pub_module_enable.publish(module_enable);
            break;
        case MODE_FOLLOW_ME:
            ROS_INFO("MODE FOLLO_ME RESUME");             
            module_enable.id_module=ID_MOD_NAVIGATION;
            module_enable.submode=SUBMODE_NAV_FOLLOW_ME;
            module_enable.status=MOD_ON;
            pub_module_enable.publish(module_enable);
            break;
        case MODE_CONVOY_AUTO:
            ROS_INFO("MODE CONVOY AUTO RESUME");             
            module_enable.id_module=ID_MOD_NAVIGATION;
            module_enable.submode=SUBMODE_NAV_CONVOY;
            module_enable.status=MOD_ON;
            pub_module_enable.publish(module_enable);         
            break;          
         default:
            break;
    }
}
bool emergencySTOP()
{
    stoppingCar=true;
    Common_files::msg_emergency_stop emergency;
    emergency.value=SET;
    
    emergencyACK=false;
    pub_emergency_stop.publish(emergency);
    
    if(timerACK(TIMEOUT_ACK,EMERGENCY_ACK))
    {
        stoppingCar=false;
        return true;
    }
    else
    {
        stoppingCar=false;
        return false;
    }   
}

bool timerACK(double sec, int typeACK)
{
    int ack=false;
    clock_t tstart; // gestiona los timeout's
    tstart=clock();
    
    double diffTime;
    //ROS_INFO("TIMER START");
    do
    {
        diffTime=(clock()-tstart)/(double)CLOCKS_PER_SEC;
        if(typeACK== CONVOY_ACK)
            ack=convoyACK;
        else if(typeACK== EMERGENCY_ACK)
            ack=emergencyACK;
    }while(!ack && diffTime< sec);
    //ROS_INFO("TIMER END");
    if(!ack)
        return false;
    else
        return true;
}

