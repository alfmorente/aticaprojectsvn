/* 
 * File:   NodeROSCommunication.cpp
 * Author: atica
 * 
 * Created on 28 de abril de 2014, 12:33
 */

#include <Modulo_Comunicaciones/NodeROSCommunication.h>
#define NO_ERROR -1

JausSubsystemVehicle* subsystemJAUS=NULL;
NodeROSCommunication* NodeROSCommunication::nodeCom=NULL;
bool NodeROSCommunication::instanceROSCreate=false;

NodeROSCommunication* NodeROSCommunication::getInstance()
{
    if(!instanceROSCreate)
    {

        nodeCom=new NodeROSCommunication();
        instanceROSCreate=true;
        subsystemJAUS=JausSubsystemVehicle::getInstance();

    }
    return  nodeCom;
}

NodeROSCommunication::NodeROSCommunication() 
{
    n=new ros::NodeHandle();
}

NodeROSCommunication::~NodeROSCommunication()
{
    delete nodeCom;
}

void NodeROSCommunication::init(int argc,char** argv,char* name)
{
     ros::init(argc,argv,"COMMUNICATION_NODE");
}

void NodeROSCommunication::createSubscribers()
{
  sub_gps = n->subscribe("gps", 1000, fcn_sub_gps);
  sub_error = n->subscribe("errorToUCR", 1000, fcn_sub_error);
  sub_camera = n->subscribe("camera", 1000, fcn_sub_camera);
  sub_modeSC = n->subscribe("modeSC", 1000, fcn_sub_mode);
  sub_modeNC = n->subscribe("modeNC", 1000, fcn_sub_mode);
  sub_backup = n->subscribe("backup", 1000, fcn_sub_backup);
  sub_file_teach = n->subscribe("teachFile", 1000, fcn_sub_teach_file);  
  sub_available= n->subscribe("avail", 1000, fcn_sub_available); 
  sub_info_stop = n->subscribe("infoStop", 1000, fcn_sub_info_stop); 
  sub_fcn_aux_ack = n->subscribe("fcnAuxACK", 1000, fcn_sub_fcn_aux); 
  ros::ServiceServer server_alive=n->advertiseService("module_alive_0",fcn_server_alive);
    
    
}
void NodeROSCommunication::createPublishers()
{
  //Publicadores
  pub_mode = n->advertise<Common_files::msg_mode>("modeCS", 1000);
  pub_comteleop_unclean = n->advertise<Common_files::msg_com_teleop>("commands_unclean", 1000);
  pub_comteleop_clean = n->advertise<Common_files::msg_com_teleop>("commands_clean", 1000);
  pub_waypoint = n->advertise<Common_files::msg_waypoint>("waypoints", 1000);
  pub_error = n->advertise<Common_files::msg_error>("error", 1000);
  pub_fcn_aux = n->advertise<Common_files::msg_fcn_aux>("fcnAux", 1000);
  pub_plan = n->advertise<Common_files::msg_stream>("file", 1000);  
  pub_ctrl_camera=n->advertise<Common_files::msg_ctrl_camera>("ctrlCamera", 1000);    
}

void NodeROSCommunication::createServers()
{
    server_alive=n->advertiseService("module_alive_0",fcn_server_alive);
}

void NodeROSCommunication::createClients()
{
    clientMode=n->serviceClient<Common_files::srv_data>("serviceParam");    
}


//Funcion servidor de datos (Devuelve el dato que se le solicita)
bool NodeROSCommunication::fcn_server_alive(Common_files::srv_data::Request &req, Common_files::srv_data::Response &resp)
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
 *                              SUSCRIPTORES
 * *****************************************************************************
 * ****************************************************************************/

//Subscriptor de modos disponibles
void NodeROSCommunication::fcn_sub_available(const Common_files::msg_available msg)
{
    // Se genera el mensaje a enviar
    ROSmessage msg_ROS;
    msg_ROS.tipo_mensaje=TOM_AVAILABLE;
    msg_ROS.mens_available=msg;

    // Espera que la comunicacion este activa
    if(subsystemJAUS->communicationState==COM_ON)
    {
        //ackAvailable=false;
    	subsystemJAUS->sendJAUSMessage(convertROStoJAUS(msg_ROS),ACK_AVAILABLE);
    }
}
//Subscriptor de ack de funcioens axuailiares
void NodeROSCommunication::fcn_sub_fcn_aux(const Common_files::msg_fcn_aux msg)
{
    // Se genera el mensaje a enviar
    ROSmessage msg_ROS;
    msg_ROS.tipo_mensaje=TOM_FUNC_AUX;
    msg_ROS.mens_fcn_aux=msg;

    // Espera que la comunicacion este activa
    if(subsystemJAUS->communicationState==COM_ON)
    {
        //ackFunctionAuxiliar=false;
    	subsystemJAUS->sendJAUSMessage(convertROStoJAUS(msg_ROS),ACK_FUNC_AUX);
    }
    
}
// Suscriptor de gps
void NodeROSCommunication::fcn_sub_gps(const Common_files::msg_gps msg)
{

    // Se genera el mensaje a enviar
    ROSmessage msg_ROS;
    msg_ROS.tipo_mensaje=TOM_GPS;
    msg_ROS.mens_gps=msg;

    // Espera que la comunicacion este activa
    if(subsystemJAUS->communicationState==COM_ON)
    {
	//JAUS_message_type=JAUS_REPORT_GLOBAL_POSE;
	// Se envia el mensaje por JAUS
    	subsystemJAUS->sendJAUSMessage(convertROStoJAUS(msg_ROS),NO_ACK);

	//JAUS_message_type=JAUS_REPORT_VELOCITY_STATE;
	// Se envia el mensaje por JAUS
    	//sendJAUSMessage(convertROStoJAUS(msg_ROS));
    }
}

// Suscriptor de errores
void NodeROSCommunication::fcn_sub_error(const Common_files::msg_error msg)
{
    // Se genera el mensaje a enviar
    ROSmessage msg_ROS;
    msg_ROS.tipo_mensaje=TOM_ERROR;
    msg_ROS.mens_error=msg;

    // Espera que la comunicacion este activa
    if(subsystemJAUS->communicationState==COM_ON)
    {
        //ackError=false;
        // Se envia el mensaje por JAUS
        subsystemJAUS->sendJAUSMessage(convertROStoJAUS(msg_ROS),ACK_ERROR);
    }
}

// Suscriptor de camaras
void NodeROSCommunication::fcn_sub_camera(const Common_files::msg_camera msg)
{

    // Se genera el mensaje a enviar
    ROSmessage msg_ROS;
    msg_ROS.tipo_mensaje=TOM_CAMERA;
    msg_ROS.mens_cam=msg;

    // Espera que la comunicacion este activa
    if(subsystemJAUS->communicationState==COM_ON)
  // Se envia el mensaje por JAUS
    subsystemJAUS->sendJAUSMessage(convertROStoJAUS(msg_ROS),NO_ACK);
	
}

//Suscriptor de modo
void NodeROSCommunication::fcn_sub_mode(const Common_files::msg_mode msg)
{
    // Se genera el mensaje a enviar
    ROSmessage msg_ROS;
    msg_ROS.tipo_mensaje=TOM_MODE;
    msg_ROS.mens_mode=msg;

    // Espera que la comunicacion este activa
    if(subsystemJAUS->communicationState==COM_ON)
    //Se envia el mensaje por JAUS
    {
        //ackMode=false;
        subsystemJAUS->sendJAUSMessage(convertROStoJAUS(msg_ROS),ACK_MODE);
    }
        
}

// Suscriptor de backup
void NodeROSCommunication::fcn_sub_backup(const Common_files::msg_backup msg)
{
    // Se genera el mensaje a enviar
    ROSmessage msg_ROS;
    msg_ROS.mens_backup=msg;

    // Espera que la comunicacion este activa
    if(subsystemJAUS->communicationState==COM_ON)
    {
        // Se envia el mensaje por JAUS
        msg_ROS.tipo_mensaje=TOM_BACKUP_WRENCH;
    	subsystemJAUS->sendJAUSMessage(convertROStoJAUS(msg_ROS),NO_ACK);
      
        // Se envia el mensaje por JAUS
        msg_ROS.tipo_mensaje=TOM_BACKUP_DISCRETE;
    	subsystemJAUS->sendJAUSMessage(convertROStoJAUS(msg_ROS),NO_ACK);
        
        // Se envia el mensaje por JAUS
        msg_ROS.tipo_mensaje=TOM_BACKUP_SPEED;
    	subsystemJAUS->sendJAUSMessage(convertROStoJAUS(msg_ROS),NO_ACK);       
    }
}

void NodeROSCommunication::fcn_sub_teach_file(const Common_files::msg_stream msg)
{
    // Se genera el mensaje a enviar
    ROSmessage msg_ROS;
    msg_ROS.tipo_mensaje=TOM_FILE;
    msg_ROS.mens_file=msg;

    // Espera que la comunicacion este activa
    if(subsystemJAUS->communicationState==COM_ON)
    // Se envia el mensaje por JAUS
    	subsystemJAUS->sendJAUSMessage(convertROStoJAUS(msg_ROS),NO_ACK);
    
}

void NodeROSCommunication::fcn_sub_info_stop(const Common_files::msg_info_stop msg)
{
    // Se genera el mensaje a enviar
    ROSmessage msg_ROS;
    msg_ROS.tipo_mensaje=TOM_INFO_STOP;
    msg_ROS.mens_info_stop=msg;

    // Espera que la comunicacion este activa
    if(subsystemJAUS->communicationState==COM_ON)
    // Se envia el mensaje por JAUS
    	subsystemJAUS->sendJAUSMessage(convertROStoJAUS(msg_ROS),NO_ACK);
    
}

int NodeROSCommunication::getStateModule()
{
    int state;
    n->getParam("state_module_communication",state);
    return state; 
}
void NodeROSCommunication::setStateModule(int state)
{
    n->setParam("state_module_communication",state);
}

string NodeROSCommunication::getDebugConfiguration()
{
    stringstream fileConf;
    double param;
    ros::NodeHandle nparam;
    nparam.getParam("PLAN_vel_max",param);
    fileConf << "PLAN_vel_max " << param << endl;
    nparam.getParam("Follow_vel_max",param);
    fileConf << "Follow_vel_max " << param << endl;
    nparam.getParam("Follow_dist_max",param);
    fileConf << "Follow_dist_max " << param << endl;    
    nparam.getParam("Follow_dist_min",param);
    fileConf << "Follow_dist_min " << param << endl;    
    nparam.getParam("Come_vel_max",param);
    fileConf << "Come_vel_max " << param << endl;    
    //nparam->getParam("Follow_dist_min",param);   
    
    return fileConf.str();
    
}
bool NodeROSCommunication::setDebugConfiguration(string file)
{  
    int error=false;
    string nameParam;
    double param;
    stringstream fileConf;
    ros::NodeHandle nparam;

    fileConf << file;
    fileConf >> nameParam >> param;
    if(nameParam=="PLAN_vel_max")
        nparam.setParam("PLAN_vel_max",param);
    else
        error=true;
    fileConf >> nameParam >> param;
    if(nameParam=="Follow_vel_max")
        nparam.setParam("Follow_vel_max",param);  
    else
        error=true;   
    fileConf >> nameParam >> param;
    if(nameParam=="Follow_dist_max")    
        nparam.setParam("Follow_dist_max",param); 
    else
        error=true;   
    fileConf >> nameParam >> param;
    if(nameParam=="Follow_dist_min")    
        nparam.setParam("Follow_dist_min",param);  
    else
        error=true;  
    fileConf >> nameParam >> param;
    if(nameParam=="Come_vel_max")    
         nparam.setParam("Come_vel_max",param);    
    else
        error=true;

    return error;
    
}

bool NodeROSCommunication::requestMode(int*mode)
{
    Common_files::srv_data servMode;
    servMode.request.param=PARAM_MODE;
    if(clientMode.call(servMode))   
    {
        *mode=servMode.response.value;
        return true;
    }
    else
    {
        //Envia error no se pudo acceder al servicio
        return false;
    }
    
}

/**
bool NodeROSCommunication::requestAvailable()
{
    Common_files::srv_data servMode;
    servMode.request.param=PARAM_MODE;
    if(clientMode.call(servMode))   
    {
        return true;
    }
    else
    {
        //Envia error no se pudo acceder al servicio
        return false;
    }  
}

bool NodeROSCommunication::requestErrors()
{
    Common_files::srv_data servMode;
    servMode.request.param=PARAM_MODE;
    if(clientMode.call(servMode))   
        return true;
    else
    {
        //Envia error no se pudo acceder al servicio        
        return false;
    }

}**/