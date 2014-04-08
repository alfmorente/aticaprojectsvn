#include "Modulo_Comunicaciones/comunicaciones.h"

using namespace std;
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

  // Inicio de ROS

  ros::init(argc, argv, "COMMUNICATION_NODE");

  // Manejador ROS
  ros::NodeHandle n;

  // Espera activa de inicio de modulo
  int state_aux;
  /**do
  {
    state_aux=getStateModule(n);
    if(state_aux==STATE_ERROR)
    {
          ROS_INFO("ATICA COMMUNICATION VEHICLE:: Module finish for error in other module");
          exit(1);
    }   
    usleep(500000);
  }while(state_aux!=STATE_CONF);**/


          
  ROS_INFO("ATICA COMMUNICATION VEHICLE :: Init configuration...");
 
  // Creación de suscriptores
  ros::Subscriber sub_gps = n.subscribe("gps", 1000, fcn_sub_gps);
  ros::Subscriber sub_error = n.subscribe("error", 1000, fcn_sub_error);
  ros::Subscriber sub_camera = n.subscribe("camera", 1000, fcn_sub_camera);
  ros::Subscriber sub_mode = n.subscribe("modeSC", 1000, fcn_sub_mode);
  ros::Subscriber sub_backup = n.subscribe("backup", 1000, fcn_sub_backup);
  ros::Subscriber sub_file_teach = n.subscribe("teachFile", 1000, fcn_sub_teach_file);  

  // Inicializacion de publicadores globales
  pub_mode = n.advertise<Common_files::msg_mode>("modeCS", 1000);
  pub_comteleop_unclean = n.advertise<Common_files::msg_com_teleop>("commands_unclean", 1000);
  pub_comteleop_clean = n.advertise<Common_files::msg_com_teleop>("commands_clean", 1000);
  pub_waypoint = n.advertise<Common_files::msg_waypoint>("waypoints", 1000);
  pub_error = n.advertise<Common_files::msg_error>("error", 1000);
  pub_fcn_aux = n.advertise<Common_files::msg_fcn_aux>("fcnAux", 1000);
  pub_plan = n.advertise<Common_files::msg_stream>("file", 1000);  
  pub_ctrl_camera=n.advertise<Common_files::msg_ctrl_camera>("ctrlCamera", 1000); 
  
  // Cliente ROS
  clientMode=n.serviceClient<Common_files::srv_data>("serviceParam");  

  pthread_t threadSpin;
  pthread_create (&threadSpin, NULL,&spinThread,NULL);

  //Configuracion de JAUS
  /**if(!configureJAUS())
  {
      setStateModule(n,STATE_ERROR); //completar
      exit(1);
  }**/

  // Realización de la conexión JAUS
  communicationState=COM_OFF;

  // Todo esta correcto, lo especificamos con el correspondiente parametro
  setStateModule(n,STATE_OK); //completar
  ROS_INFO("ATICA COMMUNICATION VEHICLE:: Configurate and Run");

  while (ros::ok() && getStateModule(n)!=STATE_OFF){
      /**if(communicationState==COM_ON)
      {
        if(!checkConnection())
          communicationState=COM_LOSED;
      }
      else if(communicationState==COM_LOSED)
      {
          losedCommunication();   
	  disconnect();
          communicationState=COM_OFF;          
      }
      else**/ if(communicationState==COM_OFF)
      {
	  if(connect())
           communicationState=COM_ON;   
      }
      //ros::spinOnce();
  }
  if(communicationState==COM_ON)
      disconnect();
  ROS_INFO("ATICA COMMUNICATION VEHICLE:: Module finish");
  return 0;
}

/*******************************************************************************
 *******************************************************************************
 *                              ROS SPIN
 * *****************************************************************************
 * ****************************************************************************/
void* spinThread(void* obj)
{
	ros::spin();
        return NULL;
}
/*******************************************************************************
 *******************************************************************************
 *                              SUSCRIPTORES
 * *****************************************************************************
 * ****************************************************************************/

//Subscriptor de modos disponibles
void fcn_sub_available_mode(const Common_files::msg_available msg)
{
    // Se genera el mensaje a enviar
    ROSmessage msg_ROS;
    msg_ROS.tipo_mensaje=TOM_AVAILABLE;
    msg_ROS.mens_available=msg;

    // Espera que la comunicacion este activa
    if(communicationState==COM_ON)
    	sendJAUSMessage(convertROStoJAUS(msg_ROS));
}
// Suscriptor de gps
void fcn_sub_gps(const Common_files::msg_gps msg)
{

    // Se genera el mensaje a enviar
    ROSmessage msg_ROS;
    msg_ROS.tipo_mensaje=TOM_GPS;
    msg_ROS.mens_gps=msg;

    // Espera que la comunicacion este activa
    if(communicationState==COM_ON)
    {
	//JAUS_message_type=JAUS_REPORT_GLOBAL_POSE;
	// Se envia el mensaje por JAUS
    	sendJAUSMessage(convertROStoJAUS(msg_ROS));

	/**JAUS_message_type=JAUS_REPORT_VELOCITY_STATE;
	// Se envia el mensaje por JAUS
    	sendJAUSMessage(convertROStoJAUS(msg_ROS));**/
    }
}

// Suscriptor de errores
void fcn_sub_error(const Common_files::msg_error msg)
{
    // Se genera el mensaje a enviar
    ROSmessage msg_ROS;
    msg_ROS.tipo_mensaje=TOM_ERROR;
    msg_ROS.mens_error=msg;

    // Espera que la comunicacion este activa
    if(communicationState==COM_ON)
    // Se envia el mensaje por JAUS
    sendJAUSMessage(convertROStoJAUS(msg_ROS));
}

// Suscriptor de camaras
void fcn_sub_camera(const Common_files::msg_camera msg)
{

    // Se genera el mensaje a enviar
    ROSmessage msg_ROS;
    msg_ROS.tipo_mensaje=TOM_CAMERA;
    msg_ROS.mens_cam=msg;

    // Espera que la comunicacion este activa
    if(communicationState==COM_ON)
  // Se envia el mensaje por JAUS
    sendJAUSMessage(convertROStoJAUS(msg_ROS));
	
}

//Suscriptor de modo
void fcn_sub_mode(const Common_files::msg_mode msg)
{
    ROS_INFO("Estado del modo");
    // Se genera el mensaje a enviar
    ROSmessage msg_ROS;
    msg_ROS.tipo_mensaje=TOM_MODE;
    msg_ROS.mens_mode=msg;

    // Espera que la comunicacion este activa
    if(communicationState==COM_ON)
    // Se envia el mensaje por JAUS
    {
        int numTrying=0;
        bool msgOK=false;
        do
        {
                ackMode=false;
                sendJAUSMessage(convertROStoJAUS(msg_ROS));
                if(waitForACK(TOM_MODE,10))
                    msgOK=true;
                else
                {
                    ROS_INFO("Reintentando enviar el mensaje.....");
                    numTrying++;
                }

        }while(!msgOK && numTrying<3);
        
        if(msgOK)
            ackMode=false;
        else
            ROS_INFO("Numero maximo de intentos realizado");
    }
        
}

// Suscriptor de backup
void fcn_sub_backup(const Common_files::msg_backup msg)
{
    // Se genera el mensaje a enviar
    ROSmessage msg_ROS;
    msg_ROS.mens_backup=msg;

    // Espera que la comunicacion este activa
    if(communicationState==COM_ON)
    {
        // Se envia el mensaje por JAUS
        msg_ROS.tipo_mensaje=TOM_BACKUP_WRENCH;
    	sendJAUSMessage(convertROStoJAUS(msg_ROS));
      
        // Se envia el mensaje por JAUS
        msg_ROS.tipo_mensaje=TOM_BACKUP_DISCRETE;
    	sendJAUSMessage(convertROStoJAUS(msg_ROS));
        
        // Se envia el mensaje por JAUS
        msg_ROS.tipo_mensaje=TOM_BACKUP_SPEED;
    	sendJAUSMessage(convertROStoJAUS(msg_ROS));       
    }
}

void fcn_sub_teach_file(const Common_files::msg_stream msg)
{
    // Se genera el mensaje a enviar
    ROSmessage msg_ROS;
    msg_ROS.tipo_mensaje=TOM_FILE;
    msg_ROS.mens_file=msg;

    // Espera que la comunicacion este activa
    if(communicationState==COM_ON)
    // Se envia el mensaje por JAUS
    	sendJAUSMessage(convertROStoJAUS(msg_ROS));
    
}

void fcn_sub_info_stop(const Common_files::msg_info_stop msg)
{
    // Se genera el mensaje a enviar
    ROSmessage msg_ROS;
    msg_ROS.tipo_mensaje=TOM_INFO_STOP;
    msg_ROS.mens_info_stop=msg;

    // Espera que la comunicacion este activa
    if(communicationState==COM_ON)
    // Se envia el mensaje por JAUS
    	sendJAUSMessage(convertROStoJAUS(msg_ROS));
    
}

/*******************************************************************************
 *******************************************************************************
 *                       FUNCIONES PROPIAS DEL MODULO
 * *****************************************************************************
 * ****************************************************************************/

// Conexion JAUS
bool configureJAUS(){

    string nombre= "NodeManager.conf";
    ROS_INFO("OpenJAUS Node Manager %s", OJ_NODE_MANAGER_VERSION);
    try
    {
            configData = new FileLoader(nombre.c_str());
            handler = new MyHandler();
            nm = new NodeManager(configData, handler);
            return true;
    }
    catch(...)
    {
            ROS_INFO("Node Manager Construction Failed");
            ROS_INFO("Note: Check the NodeManager.conf...");
            return false;
    }
}

bool connect(){

    string nameComponent="VEHICLE";
    /**if(handler->controlJaus()!=JAUS_EVENT_CONNECT)
        return false;
    
    else**/
    {

        //Creo componente
        compVehicle=ojCmptCreate((char*)nameComponent.c_str(),JAUS_SUBSYSTEM_COMMANDER ,1);
        /**compCamera=ojCmptCreate("CAMERA_COMPONENT",JAUS_VISUAL_SENSOR ,1);
        compVehicle=ojCmptCreate("VEHICLE_COMPONENT",JAUS_PRIMITIVE_DRIVER ,1);
        compMission=ojCmptCreate("MISSION_COMPONENT",JAUS_MISSION_SPOOLER,1);
        compGPS=ojCmptCreate("GPS_COMPONENT",JAUS_GLOBAL_POSE_SENSOR,1);
        compNavigation=ojCmptCreate("NAVIGATION_COMPONENT",JAUS_GLOBAL_WAYPOINT_DRIVER,1);
        compVelSensor=ojCmptCreate("VELOCITY_COMPONENT",JAUS_VELOCITY_STATE_SENSOR,1);**/

        //Configuro Componente
        ojCmptAddService(compVehicle, JAUS_SUBSYSTEM_COMMANDER);
        /**ojCmptAddService(compCamera, JAUS_VISUAL_SENSOR);
        ojCmptAddService(compVehicle, JAUS_PRIMITIVE_DRIVER);
        ojCmptAddService(compMission, JAUS_MISSION_SPOOLER);
        ojCmptAddService(compGPS, JAUS_GLOBAL_POSE_SENSOR);
        ojCmptAddService(compNavigation, JAUS_GLOBAL_WAYPOINT_DRIVER);
        ojCmptAddService(compVelSensor, JAUS_VELOCITY_STATE_SENSOR);
        **/

        ojCmptAddServiceOutputMessage(compVehicle, JAUS_SUBSYSTEM_COMMANDER, JAUS_REPORT_WRENCH_EFFORT, 0xFF);
        ojCmptAddServiceInputMessage(compVehicle, JAUS_SUBSYSTEM_COMMANDER, JAUS_SET_WRENCH_EFFORT, 0xFF);
        ojCmptAddServiceOutputMessage(compVehicle, JAUS_SUBSYSTEM_COMMANDER, JAUS_REPORT_DISCRETE_DEVICES, 0xFF);
        ojCmptAddServiceInputMessage(compVehicle, JAUS_SUBSYSTEM_COMMANDER, JAUS_SET_DISCRETE_DEVICES, 0xFF);
        ojCmptAddServiceInputMessage(compVehicle, JAUS_SUBSYSTEM_COMMANDER , JAUS_REPORT_GLOBAL_WAYPOINT, 0xFF);
        ojCmptAddServiceInputMessage(compVehicle,JAUS_SUBSYSTEM_COMMANDER , JAUS_REPORT_WAYPOINT_COUNT, 0xFF);
        ojCmptAddServiceInputMessage(compVehicle, JAUS_SUBSYSTEM_COMMANDER , JAUS_RUN_MISSION, 0xFF);
        ojCmptAddServiceInputMessage(compVehicle, JAUS_SUBSYSTEM_COMMANDER , JAUS_PAUSE_MISSION, 0xFF);
        ojCmptAddServiceInputMessage(compVehicle, JAUS_SUBSYSTEM_COMMANDER , JAUS_RESUME_MISSION, 0xFF);
        ojCmptAddServiceInputMessage(compVehicle, JAUS_SUBSYSTEM_COMMANDER , JAUS_ABORT_MISSION, 0xFF);
        ojCmptAddServiceOutputMessage(compVehicle, JAUS_SUBSYSTEM_COMMANDER , JAUS_REPORT_VELOCITY_STATE, 0xFF);
        ojCmptAddServiceOutputMessage(compVehicle, JAUS_SUBSYSTEM_COMMANDER, JAUS_REPORT_IMAGE, 0xFF);
        ojCmptAddServiceOutputMessage(compVehicle, JAUS_SUBSYSTEM_COMMANDER, JAUS_REPORT_PLATFORM_OPERATIONAL_DATA, 0xFF);
        ojCmptAddServiceOutputMessage(compVehicle, JAUS_SUBSYSTEM_COMMANDER , JAUS_REPORT_GLOBAL_POSE, 0xFF);

        /**
        ojCmptAddServiceOutputMessage(compVehicle, JAUS_PRIMITIVE_DRIVER, JAUS_REPORT_WRENCH_EFFORT, 0xFF);
        ojCmptAddServiceInputMessage(compVehicle, JAUS_PRIMITIVE_DRIVER, JAUS_SET_WRENCH_EFFORT, 0xFF);
        ojCmptAddServiceOutputMessage(compVehicle, JAUS_PRIMITIVE_DRIVER, JAUS_REPORT_DISCRETE_DEVICES, 0xFF);
        ojCmptAddServiceInputMessage(compVehicle, JAUS_PRIMITIVE_DRIVER, JAUS_SET_DISCRETE_DEVICES, 0xFF);
        ojCmptAddServiceInputMessage(compNavigation, JAUS_GLOBAL_WAYPOINT_DRIVER , JAUS_REPORT_GLOBAL_WAYPOINT, 0xFF);
        ojCmptAddServiceInputMessage(compNavigation,JAUS_GLOBAL_WAYPOINT_DRIVER , JAUS_REPORT_WAYPOINT_COUNT, 0xFF);
        ojCmptAddServiceInputMessage(compMission, JAUS_MISSION_SPOOLER , JAUS_RUN_MISSION, 0xFF);
        ojCmptAddServiceInputMessage(compMission, JAUS_MISSION_SPOOLER , JAUS_PAUSE_MISSION, 0xFF);
        ojCmptAddServiceInputMessage(compMission, JAUS_MISSION_SPOOLER , JAUS_RESUME_MISSION, 0xFF);
        ojCmptAddServiceInputMessage(compMission, JAUS_MISSION_SPOOLER , JAUS_ABORT_MISSION, 0xFF);
        ojCmptAddServiceOutputMessage(compVelSensor, JAUS_VELOCITY_STATE_SENSOR , JAUS_REPORT_VELOCITY_STATE, 0xFF);
        ojCmptAddServiceOutputMessage(compCamera, JAUS_VISUAL_SENSOR, JAUS_REPORT_IMAGE, 0xFF);
        ojCmptAddServiceOutputMessage(compVehicle, JAUS_PRIMITIVE_DRIVER, JAUS_REPORT_PLATFORM_OPERATIONAL_DATA, 0xFF);
        ojCmptAddServiceOutputMessage(compGPS, JAUS_GLOBAL_POSE_SENSOR , JAUS_REPORT_GLOBAL_POSE, 0xFF);
        **/

        ojCmptSetMessageProcessorCallback(compVehicle,rcvJAUSMessage);
        /**ojCmptSetMessageProcessorCallback(compVehicle,rcvJAUSMessage);
        ojCmptSetMessageProcessorCallback(compNavigation,rcvJAUSMessage);
        ojCmptSetMessageProcessorCallback(compMission,rcvJAUSMessage);
        ojCmptSetMessageProcessorCallback(compVelSensor,rcvJAUSMessage);
        ojCmptSetMessageProcessorCallback(compCamera,rcvJAUSMessage);
        ojCmptSetMessageProcessorCallback(compGPS,rcvJAUSMessage);
        //ojCmptSetStateCallback(compVeh, JAUS_READY_STATE,jausComunicator::process_data);
        **/

        //ojCmptSetStateCallback(compVehicle,JAUS_READY_STATE,fcn_ready);

        //run
        ojCmptRun(compVehicle);
        /**ojCmptRun(compVehicle);
        ojCmptRun(compNavigation);
        ojCmptRun(compMission);
        ojCmptRun(compVelSensor);
        ojCmptRun(compCamera);
        ojCmptRun(compGPS);**/
        return true;
    }
}

// Desconexion JAUS

void disconnect(){
   // ojCmptDestroy(compSubsystem);
}



// Comprobacion de la conexion
bool checkConnection()
{

    if(handler->controlJaus()!=JAUS_EVENT_DISCONNECT)
        return true;
    else
        return false;
}

// Conversion de mensaje ROS a JAUS (Vehiculo - UCR))

JausMessage convertROStoJAUS(ROSmessage msg_ROS){
   
    JausMessage msg_JAUS=NULL;
    JausAddress destino;
    mensajeJAUS tipoMensajeJAUS;

    // Destino al que se envia el mensaje
    destino = jausAddressCreate(); // Destino.
    destino->subsystem = 1; // TODO a definir
    destino->node = 1; // TODO a definir
    destino->instance = 1; // TODO a definir
    destino->component = JAUS_SUBSYSTEM_COMMANDER;
    
    switch (msg_ROS.tipo_mensaje){
        case TOM_CAMERA:
            tipoMensajeJAUS.image = reportImageMessageCreate();
	    jausAddressCopy(tipoMensajeJAUS.image->destination, destino);
	    tipoMensajeJAUS.image->cameraID=0;
	    tipoMensajeJAUS.image->bufferSizeBytes=msg_ROS.mens_cam.image.size();
	    tipoMensajeJAUS.image->data=new JausByte[tipoMensajeJAUS.image->bufferSizeBytes];

            for(unsigned int i=0;i<msg_ROS.mens_cam.image.size();i++)
                tipoMensajeJAUS.image->data[i]=msg_ROS.mens_cam.image[i];
            msg_JAUS = reportImageMessageToJausMessage(tipoMensajeJAUS.image);
            reportImageMessageDestroy(tipoMensajeJAUS.image);
            break;

        case TOM_GPS:
            tipoMensajeJAUS.posGPS=reportGlobalPoseMessageCreate();
            jausAddressCopy(tipoMensajeJAUS.posGPS->destination, destino);
            tipoMensajeJAUS.posGPS->latitudeDegrees=msg_ROS.mens_gps.latitude;
            tipoMensajeJAUS.posGPS->longitudeDegrees=msg_ROS.mens_gps.longitude;
            tipoMensajeJAUS.posGPS->elevationMeters=msg_ROS.mens_gps.altitude;
            tipoMensajeJAUS.posGPS->rollRadians=msg_ROS.mens_gps.roll;
            tipoMensajeJAUS.posGPS->pitchRadians=msg_ROS.mens_gps.pitch;
            tipoMensajeJAUS.posGPS->yawRadians=msg_ROS.mens_gps.yaw;
            msg_JAUS = reportGlobalPoseMessageToJausMessage(tipoMensajeJAUS.posGPS);
            reportGlobalPoseMessageDestroy(tipoMensajeJAUS.posGPS);
            break;

        case TOM_ERROR:
            tipoMensajeJAUS.error=reportErrorMessageCreate();
            jausAddressCopy(tipoMensajeJAUS.error->destination, destino);
            tipoMensajeJAUS.error->subsystem=msg_ROS.mens_error.id_subsystem;
            tipoMensajeJAUS.error->idError=msg_ROS.mens_error.id_error;
            tipoMensajeJAUS.error->typeError=msg_ROS.mens_error.type_error;
            tipoMensajeJAUS.error->properties.ackNak=JAUS_ACK_NAK_REQUIRED;
            msg_JAUS = reportErrorMessageToJausMessage(tipoMensajeJAUS.error);
            reportErrorMessageDestroy(tipoMensajeJAUS.error);
            break;
        case TOM_MODE:
            tipoMensajeJAUS.missionStatus=reportMissionStatusMessageCreate();
            jausAddressCopy(tipoMensajeJAUS.missionStatus->destination, destino);
            tipoMensajeJAUS.missionStatus->type=JAUS_MISSION;
            tipoMensajeJAUS.missionStatus->missionId=msg_ROS.mens_mode.mode;
            tipoMensajeJAUS.missionStatus->properties.ackNak=JAUS_ACK_NAK_REQUIRED;
	    
            //Camino finalizado o modo terminado
	    if(msg_ROS.mens_mode.status==MODE_FINISH)
		tipoMensajeJAUS.missionStatus->status=JAUS_FINISHED;
            else if(msg_ROS.mens_mode.status==MODE_START)
                tipoMensajeJAUS.missionStatus->status=JAUS_SPOOLING;
            else if(msg_ROS.mens_mode.status==MODE_EXIT)
                tipoMensajeJAUS.missionStatus->status=JAUS_ABORTED; 
            else if(msg_ROS.mens_mode.status==MODE_STOP)
                tipoMensajeJAUS.missionStatus->status=JAUS_PAUSED;
            else if(msg_ROS.mens_mode.status==MODE_RUN)
                tipoMensajeJAUS.missionStatus->status=JAUS_SPOOLING;
            jausAddressCopy(tipoMensajeJAUS.missionStatus->destination, destino);
            msg_JAUS = reportMissionStatusMessageToJausMessage(tipoMensajeJAUS.missionStatus);
            reportMissionStatusMessageDestroy(tipoMensajeJAUS.missionStatus);           
            break;
        case TOM_BACKUP_WRENCH:
            tipoMensajeJAUS.backupWrench=reportWrenchEffortMessageCreate();
            jausAddressCopy(tipoMensajeJAUS.backupWrench->destination, destino);
            tipoMensajeJAUS.backupWrench->propulsiveLinearEffortXPercent=msg_ROS.mens_backup.throttle;
            tipoMensajeJAUS.backupWrench->resistiveLinearEffortXPercent=msg_ROS.mens_backup.brake;
            tipoMensajeJAUS.backupWrench->propulsiveRotationalEffortYPercent=msg_ROS.mens_backup.steer;        
            msg_JAUS = reportWrenchEffortMessageToJausMessage(tipoMensajeJAUS.backupWrench);
            reportWrenchEffortMessageDestroy(tipoMensajeJAUS.backupWrench);
            break;
        case TOM_BACKUP_DISCRETE:
            tipoMensajeJAUS.backupDiscrete=reportDiscreteDevicesMessageCreate();
            jausAddressCopy(tipoMensajeJAUS.backupDiscrete->destination, destino);
            tipoMensajeJAUS.backupDiscrete->mainPropulsion=(JausBoolean)msg_ROS.mens_backup.engine;
            tipoMensajeJAUS.backupDiscrete->parkingBrake=(JausBoolean)msg_ROS.mens_backup.handbrake;
            if(msg_ROS.mens_backup.gear==GEAR_HIGH)
                    tipoMensajeJAUS.backupDiscrete->gear=JAUS_HIGH;
            else if(msg_ROS.mens_backup.gear==GEAR_NEUTRAL_HIGH)
                    tipoMensajeJAUS.backupDiscrete->gear=JAUS_NEUTRAL_HIGH;
            else if(msg_ROS.mens_backup.gear==GEAR_REVERSE)
                    tipoMensajeJAUS.backupDiscrete->gear=JAUS_REVERSE;
            else if(msg_ROS.mens_backup.gear==GEAR_NEUTRAL_LOW)
                    tipoMensajeJAUS.backupDiscrete->gear=JAUS_NEUTRAL_LOW;
            else
                   tipoMensajeJAUS.backupDiscrete->gear=JAUS_LOW;      
            msg_JAUS = reportDiscreteDevicesMessageToJausMessage(tipoMensajeJAUS.backupDiscrete);
            reportDiscreteDevicesMessageDestroy(tipoMensajeJAUS.backupDiscrete);            
            break; 
        case TOM_BACKUP_SPEED:
            tipoMensajeJAUS.backupSpeed=reportVelocityStateMessageCreate();
            jausAddressCopy(tipoMensajeJAUS.backupSpeed->destination, destino);
            tipoMensajeJAUS.backupSpeed->velocityRmsMps=msg_ROS.mens_backup.speed;       
            msg_JAUS = reportVelocityStateMessageToJausMessage(tipoMensajeJAUS.backupSpeed);
            reportVelocityStateMessageDestroy(tipoMensajeJAUS.backupSpeed);
            break;            
          
        case TOM_AVAILABLE:
            tipoMensajeJAUS.avail=reportAvailableMessageCreate();
            jausAddressCopy(tipoMensajeJAUS.avail->destination,destino);
            tipoMensajeJAUS.avail->remote=(JausBoolean)msg_ROS.mens_available.available[AVAILABLE_POS_REMOTE];
            tipoMensajeJAUS.avail->start_engine=(JausBoolean)msg_ROS.mens_available.available[AVAILABLE_POS_START_ENGINE]; 
            tipoMensajeJAUS.avail->stop_engine=(JausBoolean)msg_ROS.mens_available.available[AVAILABLE_POS_STOP_ENGINE]; 
            tipoMensajeJAUS.avail->engage_brake=(JausBoolean)msg_ROS.mens_available.available[AVAILABLE_POS_ENGAGE_BRAKE]; 
            tipoMensajeJAUS.avail->plan=(JausBoolean)msg_ROS.mens_available.available[AVAILABLE_POS_PLAN];
            tipoMensajeJAUS.avail->come_to_me=(JausBoolean)msg_ROS.mens_available.available[AVAILABLE_POS_COME_TO_ME]; 
            tipoMensajeJAUS.avail->follow_me=(JausBoolean)msg_ROS.mens_available.available[AVAILABLE_POS_FOLLOW_ME]; 
            tipoMensajeJAUS.avail->mapping=(JausBoolean)msg_ROS.mens_available.available[AVAILABLE_POS_MAPPING];  
            tipoMensajeJAUS.avail->teach=(JausBoolean)msg_ROS.mens_available.available[AVAILABLE_POS_TEACH];
            tipoMensajeJAUS.avail->convoy=(JausBoolean)msg_ROS.mens_available.available[AVAILABLE_POS_CONVOY]; 
            tipoMensajeJAUS.avail->convoy_auto=(JausBoolean)msg_ROS.mens_available.available[AVAILABLE_POS_CONVOY_AUTO]; 
            tipoMensajeJAUS.avail->convoy_teleop=(JausBoolean)msg_ROS.mens_available.available[AVAILABLE_POS_CONVOY_TELEOP];             
            tipoMensajeJAUS.avail->properties.ackNak=JAUS_ACK_NAK_REQUIRED;
            msg_JAUS = reportAvailableMessageToJausMessage(tipoMensajeJAUS.avail);
            reportAvailableMessageDestroy(tipoMensajeJAUS.avail);
            break;
        case TOM_FILE:
            tipoMensajeJAUS.file=reportFileDataMessageCreate();
            jausAddressCopy(tipoMensajeJAUS.file->destination,destino);
            tipoMensajeJAUS.file->typeFile=msg_ROS.mens_file.id_file;
            tipoMensajeJAUS.file->bufferSizeBytes=msg_ROS.mens_file.stream.size();
            for(unsigned int i=0;i< tipoMensajeJAUS.file->bufferSizeBytes;i++)
                tipoMensajeJAUS.file->data[i]=msg_ROS.mens_file.stream[i];
            msg_JAUS = reportFileDataMessageToJausMessage(tipoMensajeJAUS.file);
            reportFileDataMessageDestroy(tipoMensajeJAUS.file);
        case TOM_INFO_STOP:
            tipoMensajeJAUS.infoStop=reportInfoStopMessageCreate();
            jausAddressCopy(tipoMensajeJAUS.infoStop->destination,destino);
            tipoMensajeJAUS.infoStop->stopType=msg_ROS.mens_info_stop.id_event;
            tipoMensajeJAUS.infoStop->activated=(JausBoolean)msg_ROS.mens_info_stop.value;
            msg_JAUS = reportInfoStopMessageToJausMessage(tipoMensajeJAUS.infoStop);
            reportInfoStopMessageDestroy(tipoMensajeJAUS.infoStop);            
            
            break;
          	
        default:
            break;
    }
    return msg_JAUS;
}

// Conversion de mensaje JAUS a ROS
ROSmessage convertJAUStoROS(JausMessage msg_JAUS){

    // TODO Conversion JAUS a ROS
    ROSmessage msg_ROS;
    mensajeJAUS tipoMensajeJAUS;
    msg_ROS.tipo_mensaje=TOM_UNKNOW;
    switch(msg_JAUS->commandCode)
    {
        //Comandos de misión (modos de la plataforma)
        //RUN: Comenzar el Modo x
        //PAUSE: STOP de la navegación
        //RESUME: RUN de la navegación
        //ABORT: EXIT del modo

        case JAUS_RUN_MISSION:
            ROS_INFO("Mensaje de Modo RUN recibido");
            tipoMensajeJAUS.startMode=runMissionMessageFromJausMessage(msg_JAUS);
            if(tipoMensajeJAUS.startMode)
            {
                msg_ROS.tipo_mensaje=TOM_MODE;
                msg_ROS.mens_mode.type_msg=SET;
                msg_ROS.mens_mode.mode=tipoMensajeJAUS.startMode->missionId;
                msg_ROS.mens_mode.status=MODE_START;
                runMissionMessageDestroy(tipoMensajeJAUS.startMode);

            }
            break;
        case JAUS_PAUSE_MISSION:
            ROS_INFO("Mensaje de Modo PAUSE recibido");
            tipoMensajeJAUS.pauseMode=pauseMissionMessageFromJausMessage(msg_JAUS);
            if(tipoMensajeJAUS.pauseMode)
            {
                msg_ROS.tipo_mensaje=TOM_MODE;
                msg_ROS.mens_mode.type_msg=SET;
                msg_ROS.mens_mode.mode=tipoMensajeJAUS.pauseMode->missionId;
                msg_ROS.mens_mode.status=MODE_STOP;
                pauseMissionMessageDestroy(tipoMensajeJAUS.pauseMode);

            }
            break;
        case JAUS_RESUME_MISSION:
            ROS_INFO("Mensaje de Modo RESUME recibido");
            tipoMensajeJAUS.resumeMode=resumeMissionMessageFromJausMessage(msg_JAUS);
            if(tipoMensajeJAUS.resumeMode)
            {
                msg_ROS.tipo_mensaje=TOM_MODE;
                msg_ROS.mens_mode.type_msg=SET;                
                msg_ROS.mens_mode.mode=tipoMensajeJAUS.resumeMode->missionId;
                msg_ROS.mens_mode.status=MODE_RUN;
                resumeMissionMessageDestroy(tipoMensajeJAUS.resumeMode);

            }
            break;
        case JAUS_ABORT_MISSION:
            ROS_INFO("Mensaje de Modo ABORT recibido");
            tipoMensajeJAUS.exitMode=abortMissionMessageFromJausMessage(msg_JAUS);
            if(tipoMensajeJAUS.exitMode)
            {
                msg_ROS.tipo_mensaje=TOM_MODE;
                msg_ROS.mens_mode.type_msg=SET;                
                msg_ROS.mens_mode.mode=tipoMensajeJAUS.exitMode->missionId;
                msg_ROS.mens_mode.status=MODE_EXIT;
                abortMissionMessageDestroy(tipoMensajeJAUS.exitMode);

            }
            break;

        //Navegacion
        //Waypoints en modo de navegación
        case JAUS_REPORT_GLOBAL_WAYPOINT:
            ROS_INFO("Nuevo Waypoint Follow ME");
            tipoMensajeJAUS.waypoint=reportGlobalWaypointMessageFromJausMessage(msg_JAUS);
            if(tipoMensajeJAUS.waypoint)
            {
                msg_ROS.mens_waypoint.wp_latitude=tipoMensajeJAUS.waypoint->latitudeDegrees;
                msg_ROS.mens_waypoint.wp_longitude=tipoMensajeJAUS.waypoint->longitudeDegrees;
                reportGlobalWaypointMessageDestroy(tipoMensajeJAUS.waypoint);
            }
            break;
            
        //Conducción
        case JAUS_SET_WRENCH_EFFORT:
            tipoMensajeJAUS.mainCommand=setWrenchEffortMessageFromJausMessage(msg_JAUS);
            if(tipoMensajeJAUS.mainCommand)
            {
                msg_ROS.tipo_mensaje=TOM_REMOTE;
                switch(tipoMensajeJAUS.mainCommand->presenceVector)
                {
                    case PRESENCE_VECTOR_THROTTLE:
                        msg_ROS.mens_teleop.id_element=ID_REMOTE_THROTTLE;
                        msg_ROS.mens_teleop.value=redondea(tipoMensajeJAUS.mainCommand->propulsiveLinearEffortXPercent);
	   		ROS_INFO("Acelerador: %d",msg_ROS.mens_teleop.value);
                        break;
                    case PRESENCE_VECTOR_STEER:
                        msg_ROS.mens_teleop.id_element=ID_REMOTE_STEER;
                        msg_ROS.mens_teleop.value=redondea(tipoMensajeJAUS.mainCommand->propulsiveRotationalEffortYPercent);
	   		ROS_INFO("Direccion: %d",msg_ROS.mens_teleop.value );
                        break;
                    case PRESENCE_VECTOR_BRAKE:
                        msg_ROS.mens_teleop.id_element=ID_REMOTE_BRAKE;
                        msg_ROS.mens_teleop.value=redondea(tipoMensajeJAUS.mainCommand->resistiveLinearEffortXPercent);
	   		ROS_INFO("Freno: %d",msg_ROS.mens_teleop.value);
                        break;
                    default:
                        break;
                }
                setWrenchEffortMessageDestroy(tipoMensajeJAUS.mainCommand);

            }
            break;

        case JAUS_SET_DISCRETE_DEVICES:
            //ROS_INFO("Mensaje de Teleoperacion recibido");
            tipoMensajeJAUS.auxCommand=setDiscreteDevicesMessageFromJausMessage(msg_JAUS);
            if(tipoMensajeJAUS.auxCommand)
            {
                msg_ROS.tipo_mensaje=TOM_REMOTE;
                switch(tipoMensajeJAUS.auxCommand->presenceVector)
                {
                    case PRESENCE_VECTOR_ENGINE:
			//Start
                        msg_ROS.mens_teleop.id_element=ID_REMOTE_ENGINE;
                        msg_ROS.mens_teleop.value=tipoMensajeJAUS.auxCommand->mainPropulsion;
                       	ROS_INFO("Power: %d",msg_ROS.mens_teleop.value);
                        break;
                    case PRESENCE_VECTOR_PARKING_BRAKE:
			//Freno de mano
                        msg_ROS.mens_teleop.id_element=ID_REMOTE_HANDBRAKE;
                        msg_ROS.mens_teleop.value=tipoMensajeJAUS.auxCommand->parkingBrake;
                        ROS_INFO("Freno de mano: %d",msg_ROS.mens_teleop.value);
                        break;
                    case PRESENCE_VECTOR_LIGHT_IR:
                        //Luz IR
                        msg_ROS.mens_teleop.id_element=ID_REMOTE_LIGHT_IR;
                        msg_ROS.mens_teleop.value=tipoMensajeJAUS.auxCommand->lightIR;
                        ROS_INFO("Luz IR: %d",msg_ROS.mens_teleop.value);
                        break;
                    case PRESENCE_VECTOR_LIGHT_CONVENTIONAL:
                        //Luz convencional
                        msg_ROS.mens_teleop.id_element=ID_REMOTE_LIGHT_CONVENTIONAL;
                        msg_ROS.mens_teleop.value=tipoMensajeJAUS.auxCommand->lightConventional;
                        ROS_INFO("Luz Convencional: %d",msg_ROS.mens_teleop.value);
                        break;                
                    
                    case PRESENCE_VECTOR_DIFERENTIAL_LOCK:
                        //Diferencial
                        msg_ROS.mens_teleop.id_element=ID_REMOTE_DIFF;
                        msg_ROS.mens_teleop.value=tipoMensajeJAUS.auxCommand->diferentialLock;
                        ROS_INFO("Diferencial: %d",msg_ROS.mens_teleop.value);
                        break;
                        
                    case PRESENCE_VECTOR_ENABLE_LASER2D:
                        //activacion laser 2D
                        msg_ROS.mens_teleop.id_element=ID_REMOTE_ACT_LASER2D;
                        msg_ROS.mens_teleop.value=tipoMensajeJAUS.auxCommand->enableLaser2D;
                        ROS_INFO("Activacion laser 2D",msg_ROS.mens_teleop.value);
                        break;
 
                    case PRESENCE_VECTOR_GEAR:
			//Marcha
                        msg_ROS.mens_teleop.id_element=ID_REMOTE_GEAR;
			if(tipoMensajeJAUS.auxCommand->gear<JAUS_NEUTRAL_HIGH)
                        	msg_ROS.mens_teleop.value=GEAR_HIGH;
			else if(tipoMensajeJAUS.auxCommand->gear<JAUS_REVERSE)
                        	msg_ROS.mens_teleop.value=GEAR_NEUTRAL_HIGH;
			else if(tipoMensajeJAUS.auxCommand->gear<JAUS_NEUTRAL_LOW)
                        	msg_ROS.mens_teleop.value=GEAR_REVERSE;
                	else if(tipoMensajeJAUS.auxCommand->gear<JAUS_LOW)
                        	msg_ROS.mens_teleop.value=GEAR_NEUTRAL_LOW;
                	else
                        	msg_ROS.mens_teleop.value=GEAR_LOW;
	   		ROS_INFO("Marcha Recibida %d",msg_ROS.mens_teleop.value);

			
                        break;
                    default:
                        break;
                }
                setDiscreteDevicesMessageDestroy(tipoMensajeJAUS.auxCommand);

            }
            break;
        
        //Funciones auxiliares
        case JAUS_SET_FUNCTION_AUXILIAR:
            ROS_INFO("Funcion auxiliar: ");
            tipoMensajeJAUS.faux=setFunctionAuxiliarMessageFromJausMessage(msg_JAUS);
            if(tipoMensajeJAUS.faux)
            {
                msg_ROS.tipo_mensaje=TOM_FUNC_AUX;
                msg_ROS.mens_fcn_aux.function=tipoMensajeJAUS.faux->function;
                msg_ROS.mens_fcn_aux.value=tipoMensajeJAUS.faux->activated;
                setFunctionAuxiliarMessageDestroy(tipoMensajeJAUS.faux);
            }
            break;                       
        //Peticion de fichero
        case JAUS_QUERY_FILE_DATA:
            ROS_INFO("Peticion de fichero de configuración");
            tipoMensajeJAUS.petFile=queryFileDataMessageFromJausMessage(msg_JAUS);
            if(tipoMensajeJAUS.petFile)
            {
                msg_ROS.tipo_mensaje=TOM_PET_FILE;
                queryFileDataMessageDestroy(tipoMensajeJAUS.petFile);
            }            
            break;
        //Report de fichero actualizado
        case JAUS_REPORT_FILE_DATA:
            tipoMensajeJAUS.file=reportFileDataMessageFromJausMessage(msg_JAUS);
            if(tipoMensajeJAUS.file)
            {
                msg_ROS.mens_file.id_file=tipoMensajeJAUS.file->typeFile;
                msg_ROS.tipo_mensaje=TOM_FILE;
                for(unsigned int i=0;i< tipoMensajeJAUS.file->bufferSizeBytes;i++)
                        msg_ROS.mens_file.stream.push_back( tipoMensajeJAUS.file->data[i]);               
                reportFileDataMessageDestroy(tipoMensajeJAUS.file);
                ROS_INFO("Fichero de datos con %d bytes: ",tipoMensajeJAUS.file->bufferSizeBytes);
                cout << msg_ROS.mens_file;
            }            
            break; 
          //Camaras
          //PAN&TILT&HOME
        case JAUS_SET_CAMERA_POSE:
            tipoMensajeJAUS.camPose=setCameraPoseMessageFromJausMessage(msg_JAUS);
            if(tipoMensajeJAUS.camPose)
            {
                //PAN
                if(tipoMensajeJAUS.camPose->presenceVector== PRESENCE_VECTOR_PAN)
                {
                        if(tipoMensajeJAUS.camPose->zAngularMode==RATE_MODE)
                        {
                            msg_ROS.tipo_mensaje=TOM_CTRL_CAMERA;
                            msg_ROS.mens_ctrl_cam.id_control=CAMERA_PAN;
                            if(tipoMensajeJAUS.camPose->zAngularPositionOrRatePercent>0)
                                msg_ROS.mens_ctrl_cam.value=CAMERA_PAN_RIGHT;
                            else if(tipoMensajeJAUS.camPose->zAngularPositionOrRatePercent<0)
                                msg_ROS.mens_ctrl_cam.value=CAMERA_PAN_LEFT;
                            else
                                msg_ROS.mens_ctrl_cam.value=CAMERA_PAN_STOP;             
                        }
                }
                //TILT
                else if(tipoMensajeJAUS.camPose->presenceVector==PRESENCE_VECTOR_TILT)
                {
                        if(tipoMensajeJAUS.camPose->yAngularMode==RATE_MODE)
                        {
                            msg_ROS.tipo_mensaje=TOM_CTRL_CAMERA;                            
                            msg_ROS.mens_ctrl_cam.id_control=CAMERA_TILT;
                            if(tipoMensajeJAUS.camPose->xAngularPositionOrRatePercent>0)
                                msg_ROS.mens_ctrl_cam.value=CAMERA_TILT_UP;
                            else if(tipoMensajeJAUS.camPose->xAngularPositionOrRatePercent<0)
                                msg_ROS.mens_ctrl_cam.value=CAMERA_TILT_DOWN;
                            else
                                msg_ROS.mens_ctrl_cam.value=CAMERA_TILT_STOP;                            
                        }
                }
                //HOME
                else if(tipoMensajeJAUS.camPose->presenceVector==PRESENCE_VECTOR_HOME)
                {
                        if(tipoMensajeJAUS.camPose->yAngularMode==RATE_MODE && tipoMensajeJAUS.camPose->zAngularMode==RATE_MODE)
                        {
                           if(tipoMensajeJAUS.camPose->zAngularPositionOrRatePercent==0 && tipoMensajeJAUS.camPose->xAngularPositionOrRatePercent==0 )
                           {
                             msg_ROS.tipo_mensaje=TOM_CTRL_CAMERA;                               
                             msg_ROS.mens_ctrl_cam.id_control=CAMERA_HOME;
                             msg_ROS.mens_ctrl_cam.value=CAMERA_HOME_VALUE;
                           }
                        }                    
                }
                
            }
            break;
        case JAUS_SET_CAMERA_CAPABILITIES:
            tipoMensajeJAUS.camZoom=setCameraCapabilitiesMessageFromJausMessage(msg_JAUS);  
            if(tipoMensajeJAUS.camZoom)
            {
                if(tipoMensajeJAUS.camZoom->presenceVector==PRESENCE_VECTOR_ZOOM)
                {
                    msg_ROS.tipo_mensaje=TOM_CTRL_CAMERA;
                    msg_ROS.mens_ctrl_cam.id_control=CAMERA_ZOOM;
                    if(tipoMensajeJAUS.camZoom->horizontalFovRadians==0)
                        msg_ROS.mens_ctrl_cam.value=CAMERA_ZOOM_IN;
                    else
                        msg_ROS.mens_ctrl_cam.value=CAMERA_ZOOM_OUT;                                       
                }                    
            }
            break;
            
          //Ack de mensajes enviados 
          case JAUS_REPORT_MISSION_STATUS:
              break;
          case JAUS_REPORT_ERROR:
              break;    
          case JAUS_REPORT_AVAILABLE:
              break;                  
              
          
            
        default:
            break;

    }
    return msg_ROS;
}

// Envio de mensaje JAUS
void sendJAUSMessage(JausMessage msg_JAUS ){

    ojCmptSendMessage(compVehicle,msg_JAUS);
}

// Recepcion de mensaje JAUS
void rcvJAUSMessage(OjCmpt comp,JausMessage rxMessage){

   ROSmessage msg_ROS=convertJAUStoROS(rxMessage);
   if(msg_ROS.tipo_mensaje == TOM_REMOTE)
	 pub_comteleop_unclean.publish(msg_ROS.mens_teleop);
   else if(msg_ROS.tipo_mensaje == TOM_MODE)
	 pub_mode.publish(msg_ROS.mens_mode);
   else if(msg_ROS.tipo_mensaje == TOM_WAYPOINT)
	 pub_waypoint.publish(msg_ROS.mens_waypoint);
   else if(msg_ROS.tipo_mensaje == TOM_FUNC_AUX)
	 pub_fcn_aux.publish(msg_ROS.mens_fcn_aux);
   else if(msg_ROS.tipo_mensaje == TOM_FILE)
   {
       if(msg_ROS.mens_file.id_file==TOF_PLAN)
           	 pub_plan.publish(msg_ROS.mens_file); 
       else if(msg_ROS.mens_file.id_file==TOF_CONFIGURATION){
           /** Escribo datos de configuracion**/
           if(!setDebugConfiguration(msg_ROS.mens_file.stream))
           {
               //Publico error en fichero Debug Configuration
               
           }  
       }        
   }
   else if(msg_ROS.tipo_mensaje == TOM_PET_FILE)
   {
       /** Leo datos de configuracion y envio a UCR **/
       msg_ROS.tipo_mensaje=TOM_FILE;
       msg_ROS.mens_file.id_file =TOF_CONFIGURATION;
       msg_ROS.mens_file.stream=getDebugConfiguration();

       // Espera que la comunicacion este activa
       if(communicationState==COM_ON)
        // Se envia el mensaje por JAUS
    	sendJAUSMessage(convertROStoJAUS(msg_ROS));            
   }
   else if(msg_ROS.tipo_mensaje == TOM_CTRL_CAMERA)
       pub_ctrl_camera.publish(msg_ROS.mens_ctrl_cam);
   else
	ojCmptDefaultMessageProcessor(comp,rxMessage);
}
string getDebugConfiguration()
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
bool setDebugConfiguration(string file)
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

void losedCommunication()
{
    Common_files::srv_data servMode;
    servMode.request.param=PARAM_MODE;
    if(clientMode.call(servMode))   
    {
        if(servMode.response.value==MODE_REMOTE)
        {
            Common_files::msg_com_teleop stopVehicle;
            stopVehicle.id_element=ID_REMOTE_THROTTLE;
            stopVehicle.value=0;
            pub_comteleop_clean.publish(stopVehicle);

            stopVehicle.id_element=ID_REMOTE_BRAKE;
            stopVehicle.value=100;
            pub_comteleop_clean.publish(stopVehicle);                  
        }
    }    
    else
    {
        //Envia error no se pudo acceder al servicio
    }
    
    //Envia error de comunicación 
    Common_files::msg_error error;
    error.id_subsystem=0;
    error.id_error=0;
    error.type_error=0;
    pub_error.publish(error);
    
}
int getStateModule(ros::NodeHandle n)
{
    int state;
    n.getParam("state_module_communication",state);
    return state; 
}
void setStateModule(ros::NodeHandle n,int state)
{
    n.setParam("state_module_communication",state);
}

int redondea(float value)
{
	int res;
	if(value>=0)
		res=(int)(value+ 0.5);
	else
		res=(int)(value-0.5);
	return res;
}

bool waitForACK(int type,int timeout)
{
   time_t tstart, tend; // gestiona los timeout's
   tstart=time(0);
   bool ack;
   
   do
   {
       tend=time(0);
       if(type==TOM_MODE)
           ack=ackMode;
       //ROS_INFO("dIFERENCIA DE TIEMPO: %f",difftime(tend,tstart));
   }
   while((difftime(tend,tstart)< timeout) && !ack);
   
   if(ack)
   {
       ROS_INFO("ACK recibido");
       return true;
   }
   else
   {
       ROS_INFO("Timeout expirado");
       return false;
   }
       
}