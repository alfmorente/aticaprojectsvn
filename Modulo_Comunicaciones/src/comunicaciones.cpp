#include "../include/Modulo_Comunicaciones/comunicaciones.h"

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
  int estado_actual=STATE_OFF;
  /*while(estado_actual!=STATE_CONF){
          n.getParam("estado_modulo_comunicaciones",estado_actual);
  }*/
  ROS_INFO("ATICA COMMUNICATION VEHICLE :: Init configuration...");

  // Creación de suscriptores
  ros::Subscriber sub_gps = n.subscribe("gps", 1000, fcn_sub_gps);
  ros::Subscriber sub_error = n.subscribe("error", 1000, fcn_sub_error);
  ros::Subscriber sub_camera = n.subscribe("camera", 1000, fcn_sub_camera);
  ros::Subscriber sub_mode = n.subscribe("mode", 1000, fcn_sub_mode);
  ros::Subscriber sub_backup = n.subscribe("backup", 1000, fcn_sub_backup);

  // Inicializacion de publicadores globales
  pub_mode = n.advertise<Modulo_Comunicaciones::msg_mode>("modeRequest", 1000);
  pub_comteleop = n.advertise<Modulo_Comunicaciones::msg_com_teleoperate>("com_teleoperate", 1000);
  pub_waypoints = n.advertise<Modulo_Comunicaciones::msg_waypoints>("waypoints", 1000);
  pub_error = n.advertise<Modulo_Comunicaciones::msg_error>("errorCom", 1000);

  pthread_t threadSpin;
  pthread_create (&threadSpin, NULL,&spinThread,NULL);

  // Variable comActiva para el chequeo de la conexion JAUS
  comActiva=false;

  //Variable para el comprobar la llegada de todos los waypoints
  numberWaypoints=0;

  // Variable para finalizacion de modulo
  bool exitCom = false;

  //Configuracion de JAUS
  if(!configureJAUS())
  {
    n.setParam("state_module_communication",STATE_ERROR);
    exit(1);
  }

  ROS_INFO( "Request petition to connect");
  // Realización de la conexión JAUS
  connect();
  comActiva=true;

  // Todo esta correcto, lo especificamos con el correspondiente parametro
  n.setParam("state_module_communication",STATE_OK);
  ROS_INFO("ATICA COMMUNICATION VEHICLE:: Configurate and Run");

  while (ros::ok() && !exitCom){
      if(checkConnection()){
          comActiva=true;
          /**n.getParam("estado_modulo_comunicaciones",estado_actual);
          if(estado_actual==STATE_OFF || estado_actual==STATE_ERROR){
              disconnect();
              exitCom = true;
          }**/
      }else{
          comActiva=false;
	  disconnect();
	  Modulo_Comunicaciones::msg_error error;
	  error.ID_subsystem=0;
	  error.ID_error=0;
	  error.type_error=0;
	  fcn_pub_error(error);
	  sleep(1);
	  connect();
	  comActiva=true;
          /**n.getParam("estado_modulo_comunicaciones",estado_actual);
          if(estado_actual==STATE_OFF || estado_actual==STATE_ERROR){
              exitCom = true;
          }else{
              connect();
          }**/
      }

     // ros::spinOnce();
  }
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
}
/*******************************************************************************
 *******************************************************************************
 *                              SUSCRIPTORES
 * *****************************************************************************
 * ****************************************************************************/

// Suscriptor de gps
void fcn_sub_gps(const Modulo_Comunicaciones::msg_gps msg)
{

    // Se genera el mensaje a enviar
    ROSmessage msg_ROS;
    msg_ROS.tipo_mensaje=TOM_GPS;
    msg_ROS.mens_gps=msg;

    // Espera que la comunicacion este activa
    if(comActiva)
    {
	JAUS_message_type=JAUS_REPORT_GLOBAL_POSE;
	// Se envia el mensaje por JAUS
    	sendJAUSMessage(convertROStoJAUS(msg_ROS));

	JAUS_message_type=JAUS_REPORT_VELOCITY_STATE;
	// Se envia el mensaje por JAUS
    	sendJAUSMessage(convertROStoJAUS(msg_ROS));
    }
}

// Suscriptor de errores
void fcn_sub_error(const Modulo_Comunicaciones::msg_error msg)
{
    // Se genera el mensaje a enviar
    ROSmessage msg_ROS;
    msg_ROS.tipo_mensaje=TOM_ERROR;
    msg_ROS.mens_error=msg;

    // Espera que la comunicacion este activa
    if(comActiva)
    // Se envia el mensaje por JAUS
    sendJAUSMessage(convertROStoJAUS(msg_ROS));
}

// Suscriptor de camaras
void fcn_sub_camera(const Modulo_Comunicaciones::msg_camera msg)
{

    // Se genera el mensaje a enviar
    ROSmessage msg_ROS;
    msg_ROS.tipo_mensaje=TOM_CAMERA;
    msg_ROS.mens_cam=msg;

    // Espera que la comunicacion este activa
    if(comActiva)
  // Se envia el mensaje por JAUS
    sendJAUSMessage(convertROStoJAUS(msg_ROS));
	
}

//Suscriptor de modo
void fcn_sub_mode(const Modulo_Comunicaciones::msg_mode msg)
{


    // Se genera el mensaje a enviar
    ROSmessage msg_ROS;
    msg_ROS.tipo_mensaje=TOM_MODE;
    msg_ROS.mens_mode=msg;

    // Espera que la comunicacion este activa
    if(comActiva)
    // Se envia el mensaje por JAUS
    	sendJAUSMessage(convertROStoJAUS(msg_ROS));
}

// Suscriptor de backup
void fcn_sub_backup(const Modulo_Comunicaciones::msg_backup msg)
{


    // Se genera el mensaje a enviar
    ROSmessage msg_ROS;
    msg_ROS.tipo_mensaje=TOM_BACKUP;
    msg_ROS.mens_backup=msg;

    // Espera que la comunicacion este activa
    if(comActiva)
    // Se envia el mensaje por JAUS
    	sendJAUSMessage(convertROStoJAUS(msg_ROS));
}

/*******************************************************************************
 *******************************************************************************
 *                              PUBLICADORES
 * *****************************************************************************
 * ****************************************************************************/


//Publicador del modo de operacion
void fcn_pub_mode(Modulo_Comunicaciones::msg_mode mode)
{
	//ROS_INFO("Peticion de cambio de modo %d",mode.mode);
	pub_mode.publish(mode);
	//ROS_INFO("He pulicado");
}

//Publicador de comandos de teleoperacion
void fcn_pub_comteleop(Modulo_Comunicaciones::msg_com_teleoperate comteleop)
{
	//ROS_INFO("Comando de teleoperacion: %d  %d",comteleop.id_elemento,comteleop.valor);
	pub_comteleop.publish(comteleop);
}

//Publicador de los waypoints de la ruta
void fcn_pub_waypoints(Modulo_Comunicaciones::msg_waypoints waypoints)
{
	pub_waypoints.publish(waypoints);
}

//Publicador del error del propio módulo
void fcn_pub_error(Modulo_Comunicaciones::msg_error error)
{
	pub_error.publish(error);
}


/*******************************************************************************
 *******************************************************************************
 *                       FUNCIONES PROPIAS DEL MODULO
 * *****************************************************************************
 * ****************************************************************************/

// Conexion JAUS
bool configureJAUS(){

    char* nombre="NodeManager.conf";
    ROS_INFO("OpenJAUS Node Manager", OJ_NODE_MANAGER_VERSION);
    try
    {
            configData = new FileLoader(nombre);
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

void connect(){

    while(handler->controlJaus()!=JAUS_EVENT_CONNECT && ros::ok())
    {
	if(!ros::ok())  //Para acabar con Ctrl-C
   		exit(0);
        sleep(1);
    }

    //Creo componente
    compCamera=ojCmptCreate("CAMERA_COMPONENT",JAUS_VISUAL_SENSOR ,1);
    compVehicle=ojCmptCreate("VEHICLE_COMPONENT",JAUS_PRIMITIVE_DRIVER ,1);
    compMission=ojCmptCreate("MISSION_COMPONENT",JAUS_MISSION_SPOOLER,1);
    compGPS=ojCmptCreate("GPS_COMPONENT",JAUS_GLOBAL_POSE_SENSOR,1);
    
    //Configuro Componente
/*    ojCmptAddService(compSubsystem, JAUS_SUBSYSTEM_COMMANDER);
    ojCmptAddServiceOutputMessage(compSubsystem, JAUS_PRIMITIVE_DRIVER, JAUS_REPORT_WRENCH_EFFORT, 0xFF);
    ojCmptAddServiceInputMessage(compSubsystem, JAUS_PRIMITIVE_DRIVER, JAUS_SET_WRENCH_EFFORT, 0xFF);
    ojCmptAddServiceOutputMessage(compSubsystem, JAUS_PRIMITIVE_DRIVER, JAUS_REPORT_DISCRETE_DEVICES, 0xFF);
    ojCmptAddServiceInputMessage(compSubsystem, JAUS_PRIMITIVE_DRIVER, JAUS_SET_DISCRETE_DEVICES, 0xFF);
    ojCmptAddServiceInputMessage(compSubsystem, JAUS_SUBSYSTEM_COMMANDER , JAUS_REPORT_GLOBAL_WAYPOINT, 0xFF);
    ojCmptAddServiceInputMessage(compSubsystem, JAUS_SUBSYSTEM_COMMANDER , JAUS_REPORT_WAYPOINT_COUNT, 0xFF);
    ojCmptAddServiceInputMessage(compSubsystem, JAUS_MISSION_SPOOLER , JAUS_RUN_MISSION, 0xFF);
    ojCmptAddServiceInputMessage(compSubsystem, JAUS_MISSION_SPOOLER , JAUS_PAUSE_MISSION, 0xFF);
    ojCmptAddServiceInputMessage(compSubsystem, JAUS_MISSION_SPOOLER , JAUS_RESUME_MISSION, 0xFF);
    ojCmptAddServiceInputMessage(compSubsystem, JAUS_MISSION_SPOOLER , JAUS_ABORT_MISSION, 0xFF);
    ojCmptAddServiceOutputMessage(compSubsystem, JAUS_GLOBAL_POSE_SENSOR , JAUS_REPORT_VELOCITY_STATE, 0xFF);
    ojCmptAddServiceOutputMessage(compSubsystem, JAUS_VISUAL_SENSOR, JAUS_REPORT_IMAGE, 0xFF);
    ojCmptAddServiceOutputMessage(compSubsystem, JAUS_PRIMITIVE_DRIVER, JAUS_REPORT_PLATFORM_OPERATIONAL_DATA, 0xFF);
    ojCmptAddServiceOutputMessage(compSubsystem, JAUS_GLOBAL_POSE_SENSOR , JAUS_REPORT_GLOBAL_POSE, 0xFF);
    ojCmptSetMessageProcessorCallback(compSubsystem,rcvJAUSMessage);
    //ojCmptSetStateCallback(compVeh, JAUS_READY_STATE,jausComunicator::process_data);
*/
    //run
//    ojCmptRun(compSubsystem);
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

// Conversion de mensaje ROS a JAUS

JausMessage convertROStoJAUS(ROSmessage msg_ROS){
    JausMessage msg_JAUS;
    JausAddress destino;
    mensajeJAUS tipoMensajeJAUS;

    // Destino al que se envia el mensaje
    destino = jausAddressCreate(); // Destino.
    destino->subsystem = 1; // TODO a definir
    destino->node = 2; // TODO a definir
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
	    if(JAUS_message_type==JAUS_REPORT_GLOBAL_POSE)
	    {
	            tipoMensajeJAUS.posGPS=reportGlobalPoseMessageCreate();
        	    jausAddressCopy(tipoMensajeJAUS.posGPS->destination, destino);
        	    tipoMensajeJAUS.posGPS->latitudeDegrees=msg_ROS.mens_gps.latitude;
        	    tipoMensajeJAUS.posGPS->longitudeDegrees=msg_ROS.mens_gps.longitude;
        	    tipoMensajeJAUS.posGPS->elevationMeters=msg_ROS.mens_gps.height;
        	    tipoMensajeJAUS.posGPS->rollRadians=msg_ROS.mens_gps.roll;
        	    tipoMensajeJAUS.posGPS->pitchRadians=msg_ROS.mens_gps.pitch;
        	    tipoMensajeJAUS.posGPS->yawRadians=msg_ROS.mens_gps.yaw;
        	    msg_JAUS = reportGlobalPoseMessageToJausMessage(tipoMensajeJAUS.posGPS);
        	    reportGlobalPoseMessageDestroy(tipoMensajeJAUS.posGPS);
	    }
	    else if(JAUS_message_type==JAUS_REPORT_VELOCITY_STATE)
	    {
	            tipoMensajeJAUS.velGPS=reportVelocityStateMessageCreate();
        	    jausAddressCopy(tipoMensajeJAUS.velGPS->destination, destino);
        	    tipoMensajeJAUS.velGPS->velocityXMps=msg_ROS.mens_gps.velocity_x;
        	    tipoMensajeJAUS.velGPS->velocityYMps=msg_ROS.mens_gps.velocity_y;
        	    tipoMensajeJAUS.velGPS->velocityZMps=msg_ROS.mens_gps.velocity_z;
        	    msg_JAUS = reportVelocityStateMessageToJausMessage(tipoMensajeJAUS.velGPS);
        	    reportVelocityStateMessageDestroy(tipoMensajeJAUS.velGPS);
	    }
            break;

        case TOM_ERROR:
            tipoMensajeJAUS.error=reportErrorMessageCreate();
            jausAddressCopy(tipoMensajeJAUS.error->destination, destino);
            tipoMensajeJAUS.error->subsystem=msg_ROS.mens_error.ID_subsystem;
            tipoMensajeJAUS.error->idError=msg_ROS.mens_error.ID_error;
            tipoMensajeJAUS.error->typeError=msg_ROS.mens_error.type_error;
            msg_JAUS = reportErrorMessageToJausMessage(tipoMensajeJAUS.error);
            reportErrorMessageDestroy(tipoMensajeJAUS.error);
            break;
        case TOM_MODE:
	    //Camino finalizado o modo terminado
	    if(msg_ROS.mens_mode.status==MODE_EXIT)
	    {
            	tipoMensajeJAUS.missionStatus=reportMissionStatusMessageCreate();
            	jausAddressCopy(tipoMensajeJAUS.missionStatus->destination, destino);
           	tipoMensajeJAUS.missionStatus->type=MISSION;
		tipoMensajeJAUS.missionStatus->status=FINISHED;
            	msg_JAUS = reportMissionStatusMessageToJausMessage(tipoMensajeJAUS.missionStatus);
            	reportMissionStatusMessageDestroy(tipoMensajeJAUS.missionStatus);
	    }
	    //ACK del modo
	    else if(msg_ROS.mens_mode.status==MODE_START)
	    {
            	tipoMensajeJAUS.missionStatus=reportMissionStatusMessageCreate();
            	jausAddressCopy(tipoMensajeJAUS.missionStatus->destination, destino);
           	tipoMensajeJAUS.missionStatus->type=MISSION;
		tipoMensajeJAUS.missionStatus->status=SPOOLING;
            	msg_JAUS = reportMissionStatusMessageToJausMessage(tipoMensajeJAUS.missionStatus);
            	reportMissionStatusMessageDestroy(tipoMensajeJAUS.missionStatus);
	    }
            break;
        case TOM_BACKUP:
            tipoMensajeJAUS.missionStatus=reportMissionStatusMessageCreate();
            jausAddressCopy(tipoMensajeJAUS.missionStatus->destination, destino);
            tipoMensajeJAUS.missionStatus->type=TASK;
	    tipoMensajeJAUS.missionStatus->status=FINISHED;
            msg_JAUS = reportMissionStatusMessageToJausMessage(tipoMensajeJAUS.missionStatus);
            reportMissionStatusMessageDestroy(tipoMensajeJAUS.missionStatus);
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
        //Comandos de tele-operacion
        case JAUS_SET_WRENCH_EFFORT:
            tipoMensajeJAUS.mainCommand=setWrenchEffortMessageFromJausMessage(msg_JAUS);
            if(tipoMensajeJAUS.mainCommand)
            {
                msg_ROS.tipo_mensaje=TOM_TELEOP;
                switch(tipoMensajeJAUS.mainCommand->presenceVector)
                {
                    case 0x0004:
                        msg_ROS.mens_teleop.ID_element=ID_TELEOP_THROTTLE;
                        msg_ROS.mens_teleop.value=redondea(tipoMensajeJAUS.mainCommand->propulsiveLinearEffortZPercent);
	   		//ROS_INFO("Acelerador Recibido %d",msg_ROS.mens_teleop.value);
                        break;
                    case 0x0008:
                        msg_ROS.mens_teleop.ID_element=ID_TELEOP_STEER;
                        msg_ROS.mens_teleop.value=redondea(tipoMensajeJAUS.mainCommand->propulsiveRotationalEffortXPercent);
	   		//ROS_INFO("Direccion recibida %d",msg_ROS.mens_teleop.value );
                        break;
                    case 0x0100:
                        msg_ROS.mens_teleop.ID_element=ID_TELEOP_BRAKE;
                        msg_ROS.mens_teleop.value=redondea(tipoMensajeJAUS.mainCommand->resistiveLinearEffortZPercent);
	   		//ROS_INFO("Freno recibido %d",msg_ROS.mens_teleop.value);
                        break;
                    default:
                        break;
                }
                msg_ROS.mens_teleop.debug=false;
                setWrenchEffortMessageDestroy(tipoMensajeJAUS.mainCommand);

            }
            break;

        case JAUS_SET_DISCRETE_DEVICES:
            //ROS_INFO("Mensaje de Teleoperacion recibido");
            tipoMensajeJAUS.auxCommand=setDiscreteDevicesMessageFromJausMessage(msg_JAUS);
            if(tipoMensajeJAUS.auxCommand)
            {
                msg_ROS.tipo_mensaje=TOM_TELEOP;
                switch(tipoMensajeJAUS.auxCommand->presenceVector)
                {
                    case 0x0001:
			//Start
	   		//ROS_INFO("Start recibido");
                        msg_ROS.mens_teleop.ID_element=ID_TELEOP_ENGINE;
                        msg_ROS.mens_teleop.value=tipoMensajeJAUS.auxCommand->mainPropulsion;
                        break;
                    case 0x0002:
			//Freno de mano
                        if(tipoMensajeJAUS.auxCommand->auxiliarComandos==0)
                        {
	   		    //ROS_INFO("Freno de mano Recibida");
                            msg_ROS.mens_teleop.ID_element=ID_TELEOP_HANDBRAKE;
                            msg_ROS.mens_teleop.value=tipoMensajeJAUS.auxCommand->parkingBrake;

                        }
			//Luces
                        else if(tipoMensajeJAUS.auxCommand->auxiliarComandos==1)
                        {
			    if(tipoMensajeJAUS.auxCommand->beam)
			    {
	   		    	//ROS_INFO("Luz larga Recibida");
                            	msg_ROS.mens_teleop.ID_element=ID_TELEOP_LIGHTS;
                            	msg_ROS.mens_teleop.value=BEAM;
			    }
			    else if(tipoMensajeJAUS.auxCommand->crossing_light)
			    {
	   		    	//ROS_INFO("Luz corta Recibida");
                            	msg_ROS.mens_teleop.ID_element=ID_TELEOP_LIGHTS;
                            	msg_ROS.mens_teleop.value=CROSSING;
			    }
			    else if(tipoMensajeJAUS.auxCommand->position_light)
			    {
	   		    	//ROS_INFO("Luz de posicion Recibida");
                            	msg_ROS.mens_teleop.ID_element=ID_TELEOP_LIGHTS;
                            	msg_ROS.mens_teleop.value=POSITION;
			    }
			    else 
			    {
	   		    	//ROS_INFO("Luces apagadas");
                            	msg_ROS.mens_teleop.ID_element=ID_TELEOP_LIGHTS;
                            	msg_ROS.mens_teleop.value=OFF;
			    }

                        }

                        break;
                    case 0x0004:
			//Marcha
                        msg_ROS.mens_teleop.ID_element=ID_TELEOP_GEAR;
			if(tipoMensajeJAUS.auxCommand->gear<JAUS_NEUTRAL)
                        	msg_ROS.mens_teleop.value=BACK;
			else if(tipoMensajeJAUS.auxCommand->gear==JAUS_NEUTRAL)
                        	msg_ROS.mens_teleop.value=NEUTRAL;
			else if(tipoMensajeJAUS.auxCommand->gear>JAUS_NEUTRAL)
                        	msg_ROS.mens_teleop.value=DRIVE;
	   		ROS_INFO("Marcha Recibida %d",msg_ROS.mens_teleop.value);

			
                        break;
                    default:
                        break;
                }
                msg_ROS.mens_teleop.debug=false;
                setDiscreteDevicesMessageDestroy(tipoMensajeJAUS.auxCommand);

            }
            break;

        //Comandos de misión (modos de la plataforma)
        //RUN: Comenzar el Modo x
        //PAUSE: STOP de la navegación
        //RESUME: RUN de la navegación
        //ABORT: EXIT del modo

        case JAUS_RUN_MISSION:
            //ROS_INFO("Mensaje de Modo RUN recibido");
            tipoMensajeJAUS.startMode=runMissionMessageFromJausMessage(msg_JAUS);
            if(tipoMensajeJAUS.startMode)
            {
                msg_ROS.tipo_mensaje=TOM_MODE;
                msg_ROS.mens_mode.mode=tipoMensajeJAUS.startMode->missionId;
                msg_ROS.mens_mode.status=MODE_REQUEST;
                runMissionMessageDestroy(tipoMensajeJAUS.startMode);

            }
            break;
        case JAUS_PAUSE_MISSION:
            //ROS_INFO("Mensaje de Modo PAUSE recibido");
            tipoMensajeJAUS.pauseMode=pauseMissionMessageFromJausMessage(msg_JAUS);
            if(tipoMensajeJAUS.pauseMode)
            {
                msg_ROS.tipo_mensaje=TOM_MODE;
                msg_ROS.mens_mode.mode=tipoMensajeJAUS.pauseMode->missionId;
                msg_ROS.mens_mode.status=MODE_STOP;
                pauseMissionMessageDestroy(tipoMensajeJAUS.pauseMode);

            }
            break;
        case JAUS_RESUME_MISSION:
            //ROS_INFO("Mensaje de Modo RESUME recibido");
            tipoMensajeJAUS.resumeMode=resumeMissionMessageFromJausMessage(msg_JAUS);
            if(tipoMensajeJAUS.resumeMode)
            {
                msg_ROS.tipo_mensaje=TOM_MODE;
                msg_ROS.mens_mode.mode=tipoMensajeJAUS.resumeMode->missionId;
                msg_ROS.mens_mode.status=MODE_RUN;
                resumeMissionMessageDestroy(tipoMensajeJAUS.resumeMode);

            }
            break;
        case JAUS_ABORT_MISSION:
            //ROS_INFO("Mensaje de Modo ABORT recibido");
            tipoMensajeJAUS.exitMode=abortMissionMessageFromJausMessage(msg_JAUS);
            if(tipoMensajeJAUS.exitMode)
            {
                msg_ROS.tipo_mensaje=TOM_MODE;
                msg_ROS.mens_mode.mode=tipoMensajeJAUS.exitMode->missionId;
                msg_ROS.mens_mode.status=MODE_EXIT;
                abortMissionMessageDestroy(tipoMensajeJAUS.exitMode);

            }
            break;

        //Waypoints en modo de navegación
        case JAUS_REPORT_GLOBAL_WAYPOINT:
            //ROS_INFO("Mensaje de Waypoints recibido");
            tipoMensajeJAUS.waypoint=reportGlobalWaypointMessageFromJausMessage(msg_JAUS);
            if(tipoMensajeJAUS.waypoint)
            {
                int indexWaypoint=tipoMensajeJAUS.waypoint->waypointNumber-1;
		auxWaypointLat[indexWaypoint]=tipoMensajeJAUS.waypoint->latitudeDegrees;
		auxWaypointLon[indexWaypoint]=tipoMensajeJAUS.waypoint->longitudeDegrees;
                if(tipoMensajeJAUS.waypoint->waypointNumber==numberWaypoints)
                {
                    msg_ROS.tipo_mensaje=TOM_WAYPOINT;
                    for(int i=0;i<numberWaypoints-1;i++)
                    {
                        msg_ROS.mens_waypoints.waypoint_lat.push_back(auxWaypointLat[i]);
                        msg_ROS.mens_waypoints.waypoint_lon.push_back(auxWaypointLon[i]);
                    }
                    msg_ROS.mens_waypoints.num_waypoints=numberWaypoints;
                }
                reportGlobalWaypointMessageDestroy(tipoMensajeJAUS.waypoint);

            }
            break;
        case JAUS_REPORT_WAYPOINT_COUNT:
           // ROS_INFO("Mensaje de Numero de Waypoints recibido");
            tipoMensajeJAUS.numberWaypoints=reportWaypointCountMessageFromJausMessage(msg_JAUS);
            if(tipoMensajeJAUS.numberWaypoints)
            {
                    tipoMensajeJAUS.numberWaypoints=reportWaypointCountMessageFromJausMessage(msg_JAUS);
                    numberWaypoints=tipoMensajeJAUS.numberWaypoints->waypointCount;
                    auxWaypointLat=new double[numberWaypoints];
                    auxWaypointLon=new double[numberWaypoints];
            }
	    reportWaypointCountMessageDestroy(tipoMensajeJAUS.numberWaypoints);
            break;
        default:
            break;

    }
    return msg_ROS;
}

// Envio de mensaje JAUS
void sendJAUSMessage(JausMessage msg_JAUS ){

    //ojCmptSendMessage(compSubsystem,msg_JAUS);
}

// Recepcion de mensaje JAUS
void rcvJAUSMessage(OjCmpt comp,JausMessage rxMessage){

   ROSmessage msg_ROS=convertJAUStoROS(rxMessage);
   if(msg_ROS.tipo_mensaje == TOM_TELEOP)
	fcn_pub_comteleop(msg_ROS.mens_teleop);
   else if(msg_ROS.tipo_mensaje == TOM_MODE)
	fcn_pub_mode(msg_ROS.mens_mode);
   else if(msg_ROS.tipo_mensaje == TOM_WAYPOINT)
	fcn_pub_waypoints(msg_ROS.mens_waypoints);
   else
	ojCmptDefaultMessageProcessor(comp,rxMessage);
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

