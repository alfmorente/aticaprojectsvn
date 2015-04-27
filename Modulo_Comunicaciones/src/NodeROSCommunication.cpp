/**
 * @file   NodeROSCommunication.cpp
 * @brief  Fichero fuente para la gestión del Nodo ROS de comunicaciones
 * @author David Jiménez 
 * @date   2013, 2014, 2015
 */


#include <Modulo_Comunicaciones/JausSubsystemVehicle.h>
#include <Modulo_Comunicaciones/NodeROSCommunication.h>
#include <Modulo_Comunicaciones/JAUSmessages.h>
#define NO_ERROR -1

JausSubsystemVehicle* subsystemJAUS=NULL;
NodeROSCommunication* NodeROSCommunication::nodeCom=NULL;
bool NodeROSCommunication::instanceROSCreate=false;

/**
 * Método que obtiene una instancia de la clase
 * @return Puntero a un objeto NodeROSCommunication
 */
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

/**
 * Constructor de la clase
 */
NodeROSCommunication::NodeROSCommunication() 
{
    n=new ros::NodeHandle();
}

/**
 * Destructor de la clase
 */
NodeROSCommunication::~NodeROSCommunication()
{
    delete nodeCom;
}

/**
 * Método para inicializar el nodo ROS
 * @param argc Número de argumentosde entrada del nodo
 * @param argv Valores de los argumentos de entrada del nodo
 * @param name Nombre del nodo
 */
void NodeROSCommunication::init(int argc,char** argv,char* name)
{
     ros::init(argc,argv,"COMMUNICATION_NODE");
}

/**
 * Método para crear los subscriptores
 */
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

/**
 * Método para crear los publicadores
 */
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

/**
 * Método para crear los servidores
 */
void NodeROSCommunication::createServers()
{
    server_alive=n->advertiseService("module_alive_0",fcn_server_alive);
}

/**
 * Método para crear los clientes
 */
void NodeROSCommunication::createClients()
{
    clientMode=n->serviceClient<Common_files::srv_data>("serviceParam");    
}

/**
 * Callback servidor ROS para avisa de que el nodo sigue vivo
 * @param req  Petición de estado actual del nodo  
 * @param resp Respuesta con el estado actual
 * @return Booleano indicando que ha llegado petición del estado y no de otra cosa
 */
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

/**
 * Callback subscriptor ROS de la disponibilidad de modos
 * @param[in] msg Mensaje ROS con disponibilidad de modos
 */
//Subscriptor de modos disponibles
void NodeROSCommunication::fcn_sub_available(const Common_files::msg_availablePtr& msg)
{
    // Espera que la comunicacion este activa
    if(subsystemJAUS->communicationState==COM_ON)
    {
        //ackAvailable=false;
        JausMessage msg_JAUS=NULL;
        JausAddress destino;
        destino = jausAddressCreate(); // Destino.
        destino->subsystem = 1; // TODO a definir
        destino->node = 1; // TODO a definir
        destino->instance = 1; // TODO a definir
        destino->component = JAUS_SUBSYSTEM_COMMANDER;
        
        mensajeJAUS tipoMensajeJAUS;
        
        ROS_INFO("ENVIO AVAILABLE");         
        //Files::writeDataInLOG("ENVIO AVAILABLE"); 
        tipoMensajeJAUS.avail=reportAvailableMessageCreate();
        jausAddressCopy(tipoMensajeJAUS.avail->destination,destino);
        tipoMensajeJAUS.avail->remote=(JausBoolean)msg->available[AVAILABLE_POS_REMOTE];
        tipoMensajeJAUS.avail->start_engine=(JausBoolean)msg->available[AVAILABLE_POS_START_ENGINE]; 
        tipoMensajeJAUS.avail->stop_engine=(JausBoolean)msg->available[AVAILABLE_POS_STOP_ENGINE]; 
        tipoMensajeJAUS.avail->engage_brake=(JausBoolean)msg->available[AVAILABLE_POS_ENGAGE_BRAKE]; 
        tipoMensajeJAUS.avail->plan=(JausBoolean)msg->available[AVAILABLE_POS_PLAN];
        tipoMensajeJAUS.avail->come_to_me=(JausBoolean)msg->available[AVAILABLE_POS_COME_TO_ME]; 
        tipoMensajeJAUS.avail->follow_me=(JausBoolean)msg->available[AVAILABLE_POS_FOLLOW_ME]; 
        tipoMensajeJAUS.avail->mapping=(JausBoolean)msg->available[AVAILABLE_POS_MAPPING];  
        tipoMensajeJAUS.avail->teach=(JausBoolean)msg->available[AVAILABLE_POS_TEACH];
        tipoMensajeJAUS.avail->convoy=(JausBoolean)msg->available[AVAILABLE_POS_CONVOY]; 
        tipoMensajeJAUS.avail->convoy_auto=(JausBoolean)msg->available[AVAILABLE_POS_CONVOY_AUTO]; 
        tipoMensajeJAUS.avail->convoy_teleop=(JausBoolean)msg->available[AVAILABLE_POS_CONVOY_TELEOP];             
        tipoMensajeJAUS.avail->properties.ackNak=JAUS_ACK_NAK_REQUIRED;
        msg_JAUS = reportAvailableMessageToJausMessage(tipoMensajeJAUS.avail);
        reportAvailableMessageDestroy(tipoMensajeJAUS.avail);
        
    	subsystemJAUS->sendJAUSMessage(msg_JAUS,ACK_AVAILABLE);
    }
}

/**
 * Callback subscriptor ROS delack para funciones auxiliares
 * @param[in] msg Mensaje ROS con el ack para la función auxiliar
 */
//Subscriptor de ack de funcioens axuailiares
void NodeROSCommunication::fcn_sub_fcn_aux(const Common_files::msg_fcn_auxPtr& msg)
{


    // Espera que la comunicacion este activa
    if(subsystemJAUS->communicationState==COM_ON)
    {
        //ackFunctionAuxiliar=false;
        JausMessage msg_JAUS=NULL;
        JausAddress destino;
        destino = jausAddressCreate(); // Destino.
        destino->subsystem = 1; // TODO a definir
        destino->node = 1; // TODO a definir
        destino->instance = 1; // TODO a definir
        destino->component = JAUS_SUBSYSTEM_COMMANDER;
        mensajeJAUS tipoMensajeJAUS;
        
        ROS_INFO("ENVIO ESTADO FUNCION AUXILIAR");
            //Files::writeDataInLOG("ENVIO ESTADO FUNCION AUXILIAR"); 
        tipoMensajeJAUS.fauxACK=reportFunctionAuxiliarMessageCreate();
        jausAddressCopy(tipoMensajeJAUS.fauxACK->destination,destino);
        tipoMensajeJAUS.fauxACK->function=msg->function;
        tipoMensajeJAUS.fauxACK->activated=(JausBoolean)msg->value;
        tipoMensajeJAUS.fauxACK->properties.ackNak=JAUS_ACK_NAK_REQUIRED;
        msg_JAUS = reportFunctionAuxiliarMessageToJausMessage(tipoMensajeJAUS.fauxACK);
        reportFunctionAuxiliarMessageDestroy(tipoMensajeJAUS.fauxACK);            
    	
        subsystemJAUS->sendJAUSMessage(msg_JAUS,ACK_FUNC_AUX);
    }
    
}

/**
 * Callback subscriptor ROS de los datos del GPS
 * @param[in] msg Mensaje con los datos del GPS
 */
// Suscriptor de gps
void NodeROSCommunication::fcn_sub_gps(const Common_files::msg_gpsPtr& msg)
{


    // Espera que la comunicacion este activa
    if(subsystemJAUS->communicationState==COM_ON)
    {
	//JAUS_message_type=JAUS_REPORT_GLOBAL_POSE;
	// Se envia el mensaje por JAUS
        JausMessage msg_JAUS=NULL;
        JausAddress destino;
        destino = jausAddressCreate(); // Destino.
        destino->subsystem = 1; // TODO a definir
        destino->node = 1; // TODO a definir
        destino->instance = 1; // TODO a definir
        destino->component = JAUS_SUBSYSTEM_COMMANDER;
        mensajeJAUS tipoMensajeJAUS;
        
        //ROS_INFO("ENVIO GPS");
        //Files::writeDataInLOG("ENVIO GPS");
        tipoMensajeJAUS.posGPS=reportGlobalPoseMessageCreate();
        jausAddressCopy(tipoMensajeJAUS.posGPS->destination, destino);
	tipoMensajeJAUS.posGPS->presenceVector=0x0077;
        tipoMensajeJAUS.posGPS->latitudeDegrees=msg->latitude;
        tipoMensajeJAUS.posGPS->longitudeDegrees=msg->longitude;
        tipoMensajeJAUS.posGPS->elevationMeters=msg->altitude;
        tipoMensajeJAUS.posGPS->rollRadians=msg->roll;
        tipoMensajeJAUS.posGPS->pitchRadians=msg->pitch;
        tipoMensajeJAUS.posGPS->yawRadians=msg->yaw;
        msg_JAUS = reportGlobalPoseMessageToJausMessage(tipoMensajeJAUS.posGPS);
        reportGlobalPoseMessageDestroy(tipoMensajeJAUS.posGPS);
        
    	subsystemJAUS->sendJAUSMessage(msg_JAUS,NO_ACK);

	//JAUS_message_type=JAUS_REPORT_VELOCITY_STATE;
	// Se envia el mensaje por JAUS
    	//sendJAUSMessage(convertROStoJAUS(msg_ROS));
    }
}

/**
 * Callback subscriptor ROS de errores
 * @param[in] msg Mensaje con el error producido
 */
// Suscriptor de errores
void NodeROSCommunication::fcn_sub_error(const Common_files::msg_errorPtr& msg)
{
 
    // Espera que la comunicacion este activa
    if(subsystemJAUS->communicationState==COM_ON)
    {
        //ackError=false;
        // Se envia el mensaje por JAUS
        
        JausMessage msg_JAUS=NULL;
        JausAddress destino;
        destino = jausAddressCreate(); // Destino.
        destino->subsystem = 1; // TODO a definir
        destino->node = 1; // TODO a definir
        destino->instance = 1; // TODO a definir
        destino->component = JAUS_SUBSYSTEM_COMMANDER;
        mensajeJAUS tipoMensajeJAUS;
        
        ROS_INFO("ENVIO ERROR");
            //Files::writeDataInLOG("ENVIO ERROR");
        tipoMensajeJAUS.error=reportErrorMessageCreate();
        jausAddressCopy(tipoMensajeJAUS.error->destination, destino);
        tipoMensajeJAUS.error->subsystem=msg->id_subsystem;
        tipoMensajeJAUS.error->idError=msg->id_error;
        tipoMensajeJAUS.error->typeError=msg->type_error;
        tipoMensajeJAUS.error->properties.ackNak=JAUS_ACK_NAK_REQUIRED;
        msg_JAUS = reportErrorMessageToJausMessage(tipoMensajeJAUS.error);
        reportErrorMessageDestroy(tipoMensajeJAUS.error);
        
        subsystemJAUS->sendJAUSMessage(msg_JAUS,ACK_ERROR);
    }
}

/**
 * Callback subscriptor ROS de la imagen de la cámara (no se utiliza)
 * @param[in] msg Mensaje ROS con la imagen de la cámara
 */
// Suscriptor de camaras
void NodeROSCommunication::fcn_sub_camera(const Common_files::msg_cameraPtr& msg)
{  
    // Espera que la comunicacion este activa
    if(subsystemJAUS->communicationState==COM_ON) {
  // Se envia el mensaje por JAUS
        
        JausMessage msg_JAUS=NULL;
        JausAddress destino;
        destino = jausAddressCreate(); // Destino.
        destino->subsystem = 1; // TODO a definir
        destino->node = 1; // TODO a definir
        destino->instance = 1; // TODO a definir
        destino->component = JAUS_SUBSYSTEM_COMMANDER;
        mensajeJAUS tipoMensajeJAUS;
        
        ROS_INFO("ENVIO IMAGEN");
            //Files::writeDataInLOG("ENVIO IMAGEN");
        tipoMensajeJAUS.image = reportImageMessageCreate();
	jausAddressCopy(tipoMensajeJAUS.image->destination, destino);
	tipoMensajeJAUS.image->cameraID=0;
	tipoMensajeJAUS.image->bufferSizeBytes=msg->image.size();
	tipoMensajeJAUS.image->data=new JausByte[tipoMensajeJAUS.image->bufferSizeBytes];

        for(unsigned int i=0;i<msg->image.size();i++)
           tipoMensajeJAUS.image->data[i]=msg->image[i];
        msg_JAUS = reportImageMessageToJausMessage(tipoMensajeJAUS.image);
        reportImageMessageDestroy(tipoMensajeJAUS.image);
        
        subsystemJAUS->sendJAUSMessage(msg_JAUS,NO_ACK);
    }
}

/**
 * Callback subscriptor ROS del estado del modo actual
 * @param[in] msg Mensaje ROS con el estado del modo
 */
//Suscriptor de modo
void NodeROSCommunication::fcn_sub_mode(const Common_files::msg_modePtr& msg)
{
    // Espera que la comunicacion este activa
    if(subsystemJAUS->communicationState==COM_ON)
    //Se envia el mensaje por JAUS
    {
        //ackMode=false;
        JausMessage msg_JAUS=NULL;
        JausAddress destino;
        destino = jausAddressCreate(); // Destino.
        destino->subsystem = 1; // TODO a definir
        destino->node = 1; // TODO a definir
        destino->instance = 1; // TODO a definir
        destino->component = JAUS_SUBSYSTEM_COMMANDER;
        mensajeJAUS tipoMensajeJAUS;
             

        //Files::writeDataInLOG("ENVIO ESTADO MODO");
        tipoMensajeJAUS.missionStatus=reportMissionStatusMessageCreate();
        jausAddressCopy(tipoMensajeJAUS.missionStatus->destination, destino);
        tipoMensajeJAUS.missionStatus->type=JAUS_MISSION;
        tipoMensajeJAUS.missionStatus->missionId=msg->mode;
        tipoMensajeJAUS.missionStatus->properties.ackNak=JAUS_ACK_NAK_REQUIRED;
	    
        switch (msg->status){
            case MODE_FINISH:
                ROS_INFO("ENVIO ESTADO MODO FINISH %d",tipoMensajeJAUS.missionStatus->missionId);
                tipoMensajeJAUS.missionStatus->status=JAUS_FINISHED;
                break;
            case MODE_START:
                ROS_INFO("ENVIO ESTADO MODO START %d",tipoMensajeJAUS.missionStatus->missionId);
                tipoMensajeJAUS.missionStatus->status=JAUS_SPOOLING;
                break;
            case MODE_EXIT:
                ROS_INFO("ENVIO ESTADO MODO EXIT %d",tipoMensajeJAUS.missionStatus->missionId);
                tipoMensajeJAUS.missionStatus->status=JAUS_ABORTED; 
                break;
            case MODE_STOP:
                ROS_INFO("ENVIO ESTADO MODO STOP %d",tipoMensajeJAUS.missionStatus->missionId);
                tipoMensajeJAUS.missionStatus->status=JAUS_PAUSED;
                break;
            case MODE_RUN:
                ROS_INFO("ENVIO ESTADO MODO RUN %d",tipoMensajeJAUS.missionStatus->missionId);
                tipoMensajeJAUS.missionStatus->status=JAUS_SPOOLING;
                break;
            default:
                break;                              
        }

        jausAddressCopy(tipoMensajeJAUS.missionStatus->destination, destino);
        msg_JAUS = reportMissionStatusMessageToJausMessage(tipoMensajeJAUS.missionStatus);
        reportMissionStatusMessageDestroy(tipoMensajeJAUS.missionStatus);           
        
        subsystemJAUS->sendJAUSMessage(msg_JAUS,ACK_MODE);
    }
        
}

/**
 * Callback Subscriptor ROS de la información de backup del vehículo
 * @param[in] msg Mensaje ROS con el backup del vehículo
 */
// Suscriptor de backup
void NodeROSCommunication::fcn_sub_backup(const Common_files::msg_backupPtr& msg)
{
    // Espera que la comunicacion este activa
    if(subsystemJAUS->communicationState==COM_ON)
    {        
        JausMessage msg_JAUS=NULL;
        JausAddress destino;
        destino = jausAddressCreate(); // Destino.
        destino->subsystem = 1; // TODO a definir
        destino->node = 1; // TODO a definir
        destino->instance = 1; // TODO a definir
        destino->component = JAUS_SUBSYSTEM_COMMANDER;
        mensajeJAUS tipoMensajeJAUS;
     
        // Se envia el mensaje por JAUS
        //ROS_INFO("ENVIO BACKUP WRENCH");
        //Files::writeDataInLOG("ENVIO BACKUP WRENCH");            
        tipoMensajeJAUS.backupWrench=reportWrenchEffortMessageCreate();
        jausAddressCopy(tipoMensajeJAUS.backupWrench->destination, destino);
	tipoMensajeJAUS.backupWrench->presenceVector=0x0061;
        tipoMensajeJAUS.backupWrench->propulsiveLinearEffortXPercent=msg->throttle;
        tipoMensajeJAUS.backupWrench->resistiveLinearEffortXPercent=msg->brake;
        tipoMensajeJAUS.backupWrench->propulsiveRotationalEffortZPercent=msg->steer;        
        msg_JAUS = reportWrenchEffortMessageToJausMessage(tipoMensajeJAUS.backupWrench);
        reportWrenchEffortMessageDestroy(tipoMensajeJAUS.backupWrench);
            
    	subsystemJAUS->sendJAUSMessage(msg_JAUS,NO_ACK);
      
        // Se envia el mensaje por JAUS
        // ROS_INFO("ENVIO BACKUP DISCRETE");
        //Files::writeDataInLOG("ENVIO BACKUP DISCRETE"); 
        tipoMensajeJAUS.backupDiscrete=reportDiscreteDevicesMessageCreate();
        jausAddressCopy(tipoMensajeJAUS.backupDiscrete->destination, destino);
	tipoMensajeJAUS.backupDiscrete->presenceVector=0x07;
        tipoMensajeJAUS.backupDiscrete->mainPropulsion=(JausBoolean)msg->engine;
        tipoMensajeJAUS.backupDiscrete->parkingBrake=(JausBoolean)msg->handbrake;
        
        switch(msg->gear){
            case GEAR_HIGH:
                tipoMensajeJAUS.backupDiscrete->gear=JAUS_HIGH;
                break;
            case GEAR_NEUTRAL_HIGH:
                tipoMensajeJAUS.backupDiscrete->gear=JAUS_NEUTRAL_HIGH;
                break;
            case GEAR_REVERSE:
                tipoMensajeJAUS.backupDiscrete->gear=JAUS_REVERSE;
                break;
            case GEAR_NEUTRAL_LOW:
                tipoMensajeJAUS.backupDiscrete->gear=JAUS_NEUTRAL_LOW;
                break;
            case GEAR_LOW:
                tipoMensajeJAUS.backupDiscrete->gear=JAUS_LOW;
                break;
            default:
                break;
                
        }
                        
        msg_JAUS = reportDiscreteDevicesMessageToJausMessage(tipoMensajeJAUS.backupDiscrete);
        reportDiscreteDevicesMessageDestroy(tipoMensajeJAUS.backupDiscrete);            
    	subsystemJAUS->sendJAUSMessage(msg_JAUS,NO_ACK);
        
        // Se envia el mensaje por JAUS
        //ROS_INFO("ENVIO BACKUP SPEED");   
        //Files::writeDataInLOG("ENVIO BACKUP SPEED"); 
        tipoMensajeJAUS.backupSpeed=reportVelocityStateMessageCreate();
        jausAddressCopy(tipoMensajeJAUS.backupSpeed->destination, destino);
	tipoMensajeJAUS.backupSpeed->presenceVector=0x0008;
        tipoMensajeJAUS.backupSpeed->velocityRmsMps=msg->speed;       
        msg_JAUS = reportVelocityStateMessageToJausMessage(tipoMensajeJAUS.backupSpeed);
        reportVelocityStateMessageDestroy(tipoMensajeJAUS.backupSpeed);
        
    	subsystemJAUS->sendJAUSMessage(msg_JAUS,NO_ACK);       
    }
}

/**
 * Callback subscriptor ROS del fichero teach
 * @param[in] msg Mensaje ROS con el fichero teach
 */
void NodeROSCommunication::fcn_sub_teach_file(const Common_files::msg_streamPtr& msg)
{
    // Espera que la comunicacion este activa
    ROS_INFO("Modulo de Comunicaciones: Fichero TEACH");
    if(subsystemJAUS->communicationState == COM_ON) {
        // Se envia el mensaje por JAUS
        JausMessage msg_JAUS=NULL;
        JausAddress destino;
        destino = jausAddressCreate(); // Destino.
        destino->subsystem = 1; // TODO a definir
        destino->node = 1; // TODO a definir
        destino->instance = 1; // TODO a definir
        destino->component = JAUS_SUBSYSTEM_COMMANDER;
        mensajeJAUS tipoMensajeJAUS;
        
        //Files::writeDataInLOG("ENVIO FICHERO"); 
        tipoMensajeJAUS.file=reportFileDataMessageCreate();
        jausAddressCopy(tipoMensajeJAUS.file->destination,destino);
        tipoMensajeJAUS.file->typeFile=msg->id_file;
        tipoMensajeJAUS.file->bufferSizeBytes=msg->stream.size();
	tipoMensajeJAUS.file->data=new JausByte[tipoMensajeJAUS.file->bufferSizeBytes];
        ROS_INFO("Tamano: %d",tipoMensajeJAUS.file->bufferSizeBytes);
        for(unsigned int i=0;i< tipoMensajeJAUS.file->bufferSizeBytes;i++)
            tipoMensajeJAUS.file->data[i]=msg->stream[i];
        msg_JAUS = reportFileDataMessageToJausMessage(tipoMensajeJAUS.file);
        reportFileDataMessageDestroy(tipoMensajeJAUS.file);
        
    	subsystemJAUS->sendJAUSMessage(msg_JAUS,NO_ACK);
    }
    
    
}

/**
 * Callback subscriptor ROS con la información de la parada de emergencia
 * @param[in] msg Mensaje con el estado de la parada de emergencia
 */
void NodeROSCommunication::fcn_sub_info_stop(const Common_files::msg_info_stopPtr& msg)
{
    // Espera que la comunicacion este activa
    if(subsystemJAUS->communicationState==COM_ON) {
        // Se envia el mensaje por JAUS
        JausMessage msg_JAUS=NULL;
        JausAddress destino;
        destino = jausAddressCreate(); // Destino.
        destino->subsystem = 1; // TODO a definir
        destino->node = 1; // TODO a definir
        destino->instance = 1; // TODO a definir
        destino->component = JAUS_SUBSYSTEM_COMMANDER;
        mensajeJAUS tipoMensajeJAUS;
        
        ROS_INFO("ENVIO INFO STOP");            
        //Files::writeDataInLOG("ENVIO INFO STOP"); 
        tipoMensajeJAUS.infoStop=reportInfoStopMessageCreate();
        jausAddressCopy(tipoMensajeJAUS.infoStop->destination,destino);
        tipoMensajeJAUS.infoStop->stopType=msg->id_event;
        tipoMensajeJAUS.infoStop->activated=(JausBoolean)msg->value;
        msg_JAUS = reportInfoStopMessageToJausMessage(tipoMensajeJAUS.infoStop);
        reportInfoStopMessageDestroy(tipoMensajeJAUS.infoStop);     
        
    	subsystemJAUS->sendJAUSMessage(msg_JAUS,NO_ACK);
    }
    
}

/**
 * Método para obtener el estado del módulo
 * @return Entero con el estado del módulo
 */
int NodeROSCommunication::getStateModule()
{
    int state;
    n->getParam("state_module_communication",state);
    return state; 
}

/**
 * Método para poner módulo en un estado determinado
 * @param[in] state Estado en el que se pone el módulo
 */
void NodeROSCommunication::setStateModule(int state)
{
    n->setParam("state_module_communication",state);
}

/**
 * Método para obtener los parámetros de configuración de navegación autónoma
 * @return String con el fichero con los parámetros de configuración
 */
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

/**
 * Método para guardar los valores de configuración para la navegación autónoma
 * @param[in] file Fichero de parámetros de configuración de la navegación autónoma
 * @return Booleano indicando que el formato del fichero es correcto
 */
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

/**
 * Método que llama al servicio que devuelve el modo actual 
 * @param[io] mode Puntero donde se guarda el modo actual recibido en el servicio
 * @return Booleano indicando que el servidor respondio correctamente
 */
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
