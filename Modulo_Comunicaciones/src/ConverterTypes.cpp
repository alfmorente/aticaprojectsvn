#include <Modulo_Comunicaciones/ConverterTypes.h>

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
            ROS_INFO("ENVIO IMAGEN");
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
            ROS_INFO("ENVIO GPS");
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
            ROS_INFO("ENVIO ERROR");
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
            ROS_INFO("ENVIO ESTADO MODO");
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
            ROS_INFO("ENVIO BACKUP WRENCH");
            tipoMensajeJAUS.backupWrench=reportWrenchEffortMessageCreate();
            jausAddressCopy(tipoMensajeJAUS.backupWrench->destination, destino);
            tipoMensajeJAUS.backupWrench->propulsiveLinearEffortXPercent=msg_ROS.mens_backup.throttle;
            tipoMensajeJAUS.backupWrench->resistiveLinearEffortXPercent=msg_ROS.mens_backup.brake;
            tipoMensajeJAUS.backupWrench->propulsiveRotationalEffortYPercent=msg_ROS.mens_backup.steer;        
            msg_JAUS = reportWrenchEffortMessageToJausMessage(tipoMensajeJAUS.backupWrench);
            reportWrenchEffortMessageDestroy(tipoMensajeJAUS.backupWrench);
            break;
        case TOM_BACKUP_DISCRETE:
            ROS_INFO("ENVIO BACKUP DISCRETE");
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
            ROS_INFO("ENVIO BACKUP SPEED");   
            tipoMensajeJAUS.backupSpeed=reportVelocityStateMessageCreate();
            jausAddressCopy(tipoMensajeJAUS.backupSpeed->destination, destino);
            tipoMensajeJAUS.backupSpeed->velocityRmsMps=msg_ROS.mens_backup.speed;       
            msg_JAUS = reportVelocityStateMessageToJausMessage(tipoMensajeJAUS.backupSpeed);
            reportVelocityStateMessageDestroy(tipoMensajeJAUS.backupSpeed);
            break;            
          
        case TOM_AVAILABLE:
            ROS_INFO("ENVIO AVAILABLE");         
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
            ROS_INFO("ENVIO FICHERO");
            tipoMensajeJAUS.file=reportFileDataMessageCreate();
            jausAddressCopy(tipoMensajeJAUS.file->destination,destino);
            tipoMensajeJAUS.file->typeFile=msg_ROS.mens_file.id_file;
            tipoMensajeJAUS.file->bufferSizeBytes=msg_ROS.mens_file.stream.size();
            for(unsigned int i=0;i< tipoMensajeJAUS.file->bufferSizeBytes;i++)
                tipoMensajeJAUS.file->data[i]=msg_ROS.mens_file.stream[i];
            msg_JAUS = reportFileDataMessageToJausMessage(tipoMensajeJAUS.file);
            reportFileDataMessageDestroy(tipoMensajeJAUS.file);
        case TOM_INFO_STOP:
            ROS_INFO("ENVIO INFO STOP");
            tipoMensajeJAUS.infoStop=reportInfoStopMessageCreate();
            jausAddressCopy(tipoMensajeJAUS.infoStop->destination,destino);
            tipoMensajeJAUS.infoStop->stopType=msg_ROS.mens_info_stop.id_event;
            tipoMensajeJAUS.infoStop->activated=(JausBoolean)msg_ROS.mens_info_stop.value;
            msg_JAUS = reportInfoStopMessageToJausMessage(tipoMensajeJAUS.infoStop);
            reportInfoStopMessageDestroy(tipoMensajeJAUS.infoStop);            
            break;
        case TOM_FUNC_AUX:
            ROS_INFO("ENVIO ESTADO FUNCION AUXILIAR");
            tipoMensajeJAUS.fauxACK=reportFunctionAuxiliarMessageCreate();
            jausAddressCopy(tipoMensajeJAUS.fauxACK->destination,destino);
            tipoMensajeJAUS.fauxACK->function=msg_ROS.mens_fcn_aux.function;
            tipoMensajeJAUS.fauxACK->activated=(JausBoolean)msg_ROS.mens_fcn_aux.value;
            msg_JAUS = reportFunctionAuxiliarMessageToJausMessage(tipoMensajeJAUS.fauxACK);
            reportFunctionAuxiliarMessageDestroy(tipoMensajeJAUS.fauxACK);            
            break;            
          	
        default:
            break;
    }
    return msg_JAUS;
}

// Conversion de mensaje JAUS a ROS
ROSmessage convertJAUStoROS(JausMessage msg_JAUS)
{
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
        //Waypoints en modo de navegación (Follow me)
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
                        ROS_INFO("Activacion laser 2D: %d",msg_ROS.mens_teleop.value);
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
            
            tipoMensajeJAUS.faux=setFunctionAuxiliarMessageFromJausMessage(msg_JAUS);
            if(tipoMensajeJAUS.faux)
            {
                ROS_INFO("Funcion auxiliar: %d %d",msg_ROS.mens_fcn_aux.function,msg_ROS.mens_fcn_aux.value);
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
                //cout << msg_ROS.mens_file;
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
        default:
            break;

    }
    return msg_ROS;
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