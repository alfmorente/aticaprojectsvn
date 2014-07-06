/* 
 * File:   JausSubsystemVehicle.cpp
 * Author: atica
 * 
 * Created on 28 de abril de 2014, 12:09
 */

#include <Modulo_Comunicaciones/JausSubsystemVehicle.h>
#include <Modulo_Comunicaciones/NodeROSCommunication.h>
#include <Modulo_Comunicaciones/ROSmessages.h>
#include <Modulo_Comunicaciones/JAUSmessages.h>
#define NO_ERROR -1

NodeROSCommunication* nodeROS=NULL;
JausSubsystemVehicle* JausSubsystemVehicle::subsystemJAUS=NULL;
bool JausSubsystemVehicle::instanceJAUSCreate=false;

JausSubsystemVehicle* JausSubsystemVehicle::getInstance()
{
    if(!instanceJAUSCreate)
    {

        subsystemJAUS= new JausSubsystemVehicle();
        instanceJAUSCreate=true;
        nodeROS=NodeROSCommunication::getInstance();
    }
    return subsystemJAUS;
}

JausSubsystemVehicle::JausSubsystemVehicle() 
{   

}

JausSubsystemVehicle::~JausSubsystemVehicle()
{
    if(instanceJAUSCreate)
        delete subsystemJAUS;   
}

bool JausSubsystemVehicle::connect()
{
    if(handler->controlJaus()!=JAUS_EVENT_CONNECT)
        return false;
    else
        return true;
}

// Desconexion JAUS
void JausSubsystemVehicle::disconnect()
{
    ojCmptDestroy(compVehicle);
    delete nm;
    delete handler;
    delete configData;
}


int JausSubsystemVehicle::configureJAUS(){

    string nombre= "/home/atica/catkin_ws/src/Modulo_Comunicaciones/bin/NodeManager.conf";
    string nameComponent="VEHICLE";
    ROS_INFO("OpenJAUS Node Manager %s", OJ_NODE_MANAGER_VERSION);
    try
    {
            configData = new FileLoader(nombre.c_str());
            handler = new HandlerJAUS();
            nm = new NodeManager(configData, handler);
           
            //Creo componente
            compVehicle=ojCmptCreate((char*)nameComponent.c_str(),JAUS_SUBSYSTEM_COMMANDER ,1);   
            if(compVehicle==NULL)
            {
                  ROS_INFO("Error al crear el componente JAUS");
                  return CREATE_COMPONENT_ERROR;
            }

            //Configuro Componente
            ojCmptAddService(compVehicle, JAUS_SUBSYSTEM_COMMANDER);

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

            ojCmptSetMessageProcessorCallback(compVehicle,rcvJAUSMessage);

            //run
            if(ojCmptRun(compVehicle)<0)
            {
                  ROS_INFO("Error al ejecutar el componente JAUS");
                  return RUN_COMPONENT_ERROR;
            }            
            return NO_ERROR;
    }
    catch(...)
    {   
            ROS_INFO("Node Manager Construction Failed");
            ROS_INFO("Note: Check the NodeManager.conf...");
            return JAUS_CONFIG_ERROR;
    }
}

// Comprobacion de la conexion
bool JausSubsystemVehicle::checkConnection()
{
    if(handler->controlJaus()!=JAUS_EVENT_DISCONNECT)
        return true;
    else
        return false;
}

// Envio de mensaje JAUS
void JausSubsystemVehicle::sendJAUSMessage(JausMessage msg_JAUS, int typeACK)
{
    int numTrying=0;
    bool msgOK=false;
    if(typeACK!=NO_ACK)
    {
        do
        {
            subsystemJAUS->ackReceived=NO_ACK;
            ojCmptSendMessage(compVehicle,msg_JAUS);
            if(waitForACK(typeACK,TIMEOUT_ACK))
                msgOK=true;
            else
            {
                ROS_INFO("Reintentando enviar el mensaje.....");
                
                //Generar error a gestión de errores
                numTrying++;
            }
        }while(!msgOK && numTrying<3);
        if(!msgOK)
                ROS_INFO("Numero maximo de intentos realizado");
    }
    else
        ojCmptSendMessage(compVehicle,msg_JAUS);
}

// Recepcion de mensaje JAUS
void JausSubsystemVehicle::rcvJAUSMessage(OjCmpt comp,JausMessage rxMessage){
   
   if(subsystemJAUS==NULL)
        return;
   
   //Recepción de ack
   if(rxMessage->properties.ackNak==JAUS_ACKNOWLEDGE)
   {
       if(rxMessage->commandCode==JAUS_REPORT_FUNCTION_AUXILIAR)
           subsystemJAUS->ackReceived=ACK_FUNC_AUX;       
       else if(rxMessage->commandCode==JAUS_REPORT_MISSION_STATUS)
           subsystemJAUS->ackReceived=ACK_MODE;
       else if(rxMessage->commandCode==JAUS_REPORT_ERROR)
           subsystemJAUS->ackReceived=ACK_ERROR;          
       else if(rxMessage->commandCode==JAUS_REPORT_AVAILABLE)
           subsystemJAUS->ackReceived=ACK_AVAILABLE;
   }
    else {
        //Si el mensaje Requiere ACK, lo envio
        if (rxMessage->properties.ackNak == JAUS_ACK_NAK_REQUIRED) {
            JausMessage ack = jausMessageClone(rxMessage);
            ack->dataSize = 0;
            ack->destination = rxMessage->source;
            ack->source = rxMessage->destination;
            ack->properties.ackNak = JAUS_ACKNOWLEDGE;
            subsystemJAUS->sendJAUSMessage(ack, NO_ACK);
        }
        //Convierto a mensaje ROS
        short tipo_mensaje = TOM_UNKNOW;
        mensajeJAUS tipoMensajeJAUS;

        switch (rxMessage->commandCode) {

                //Comandos de misión (modos de la plataforma)
                //RUN: Comenzar el Modo x
                //PAUSE: STOP de la navegación
                //RESUME: RUN de la navegación
                //ABORT: EXIT del modo

            case JAUS_RUN_MISSION:
                ROS_INFO("Mensaje de Modo RUN recibido");
                //Files::writeDataInLOG("ENVIO ESTADO FUNCION AUXILIAR"); 
                tipoMensajeJAUS.startMode = runMissionMessageFromJausMessage(rxMessage);
                if (tipoMensajeJAUS.startMode) {
                    tipo_mensaje = TOM_MODE;
                    mens_mode->type_msg = SET;
                    mens_mode->mode = tipoMensajeJAUS.startMode->missionId;
                    mens_mode->status = MODE_START;
                    runMissionMessageDestroy(tipoMensajeJAUS.startMode);

                }
                break;
            case JAUS_PAUSE_MISSION:
                ROS_INFO("Mensaje de Modo PAUSE recibido");
                //Files::writeDataInLOG("ENVIO ESTADO FUNCION AUXILIAR"); 
                tipoMensajeJAUS.pauseMode = pauseMissionMessageFromJausMessage(rxMessage);
                if (tipoMensajeJAUS.pauseMode) {
                    tipo_mensaje = TOM_MODE;
                    mens_mode->type_msg = SET;
                    mens_mode->mode = tipoMensajeJAUS.pauseMode->missionId;
                    mens_mode->status = MODE_STOP;
                    pauseMissionMessageDestroy(tipoMensajeJAUS.pauseMode);

                }
                break;
            case JAUS_RESUME_MISSION:
                ROS_INFO("Mensaje de Modo RESUME recibido");
                //Files::writeDataInLOG("ENVIO ESTADO FUNCION AUXILIAR"); 
                tipoMensajeJAUS.resumeMode = resumeMissionMessageFromJausMessage(rxMessage);
                if (tipoMensajeJAUS.resumeMode) {
                    tipo_mensaje = TOM_MODE;
                    mens_mode->type_msg = SET;
                    mens_mode->mode = tipoMensajeJAUS.resumeMode->missionId;
                    mens_mode->status = MODE_RUN;
                    resumeMissionMessageDestroy(tipoMensajeJAUS.resumeMode);

                }
                break;
            case JAUS_ABORT_MISSION:
                ROS_INFO("Mensaje de Modo ABORT recibido");
                //Files::writeDataInLOG("ENVIO ESTADO FUNCION AUXILIAR"); 
                tipoMensajeJAUS.exitMode = abortMissionMessageFromJausMessage(rxMessage);
                if (tipoMensajeJAUS.exitMode) {
                    tipo_mensaje = TOM_MODE;
                    mens_mode->type_msg = SET;
                    mens_mode->mode = tipoMensajeJAUS.exitMode->missionId;
                    mens_mode->status = MODE_EXIT;
                    abortMissionMessageDestroy(tipoMensajeJAUS.exitMode);

                }
                break;

                //Navegacion
                //Waypoints en modo de navegación (Follow me)
            case JAUS_REPORT_GLOBAL_WAYPOINT:
                ROS_INFO("Nuevo Waypoint Follow ME");
                //Files::writeDataInLOG("ENVIO ESTADO FUNCION AUXILIAR");  
                tipoMensajeJAUS.waypoint = reportGlobalWaypointMessageFromJausMessage(rxMessage);
                if (tipoMensajeJAUS.waypoint) {
                    mens_waypoint->wp_latitude = tipoMensajeJAUS.waypoint->latitudeDegrees;
                    mens_waypoint->wp_longitude = tipoMensajeJAUS.waypoint->longitudeDegrees;
                    reportGlobalWaypointMessageDestroy(tipoMensajeJAUS.waypoint);
                }
                break;

                //Conducción
            case JAUS_SET_WRENCH_EFFORT:
                //ROS_INFO("Nuevo comando de teleoperacion");
                //Files::writeDataInLOG("Nuevo comando de teleoperacion");  
                tipoMensajeJAUS.mainCommand = setWrenchEffortMessageFromJausMessage(rxMessage);
                if (tipoMensajeJAUS.mainCommand) {
                    tipo_mensaje = TOM_REMOTE;
                    switch (tipoMensajeJAUS.mainCommand->presenceVector) {
                        case PRESENCE_VECTOR_THROTTLE:
                            mens_teleop->id_element = ID_REMOTE_THROTTLE;
                            mens_teleop->value = redondea(tipoMensajeJAUS.mainCommand->propulsiveLinearEffortXPercent);
                            ROS_INFO("Acelerador: %d", mens_teleop->value);
                            break;
                        case PRESENCE_VECTOR_STEER:
                            mens_teleop->id_element = ID_REMOTE_STEER;
                            mens_teleop->value = redondea(tipoMensajeJAUS.mainCommand->propulsiveRotationalEffortYPercent);
                            ROS_INFO("Direccion: %d", mens_teleop->value);
                            break;
                        case PRESENCE_VECTOR_BRAKE:
                            mens_teleop->id_element = ID_REMOTE_BRAKE;
                            mens_teleop->value = redondea(tipoMensajeJAUS.mainCommand->resistiveLinearEffortXPercent);
                            ROS_INFO("Freno: %d", mens_teleop->value);
                            break;
                        default:
                            break;
                    }
                    setWrenchEffortMessageDestroy(tipoMensajeJAUS.mainCommand);
                }
                break;

            case JAUS_SET_DISCRETE_DEVICES:
                //ROS_INFO("Mensaje de Teleoperacion recibido");
                tipoMensajeJAUS.auxCommand = setDiscreteDevicesMessageFromJausMessage(rxMessage);
                if (tipoMensajeJAUS.auxCommand) {
                    tipo_mensaje = TOM_REMOTE;
                    switch (tipoMensajeJAUS.auxCommand->presenceVector) {
                        case PRESENCE_VECTOR_ENGINE:
                            //Start
                            mens_teleop->id_element = ID_REMOTE_ENGINE;
                            mens_teleop->value = tipoMensajeJAUS.auxCommand->mainPropulsion;
                            ROS_INFO("Power: %d", mens_teleop->value);
                            break;
                        case PRESENCE_VECTOR_PARKING_BRAKE:
                            //Freno de mano
                            mens_teleop->id_element = ID_REMOTE_HANDBRAKE;
                            mens_teleop->value = tipoMensajeJAUS.auxCommand->parkingBrake;
                            ROS_INFO("Freno de mano: %d", mens_teleop->value);
                            break;
                        case PRESENCE_VECTOR_LIGHT_IR:
                            //Luz IR
                            mens_teleop->id_element = ID_REMOTE_LIGHT_IR;
                            mens_teleop->value = tipoMensajeJAUS.auxCommand->lightIR;
                            ROS_INFO("Luz IR: %d", mens_teleop->value);
                            break;
                        case PRESENCE_VECTOR_LIGHT_CONVENTIONAL:
                            //Luz convencional
                            mens_teleop->id_element = ID_REMOTE_LIGHT_CONVENTIONAL;
                            mens_teleop->value = tipoMensajeJAUS.auxCommand->lightConventional;
                            ROS_INFO("Luz Convencional: %d", mens_teleop->value);
                            break;

                        case PRESENCE_VECTOR_DIFERENTIAL_LOCK:
                            //Diferencial
                            mens_teleop->id_element = ID_REMOTE_DIFF;
                            mens_teleop->value = tipoMensajeJAUS.auxCommand->diferentialLock;
                            ROS_INFO("Diferencial: %d", mens_teleop->value);
                            break;

                        case PRESENCE_VECTOR_ENABLE_LASER2D:
                            //activacion laser 2D
                            mens_teleop->id_element = ID_REMOTE_ACT_LASER2D;
                            mens_teleop->value = tipoMensajeJAUS.auxCommand->enableLaser2D;
                            ROS_INFO("Activacion laser 2D: %d", mens_teleop->value);
                            break;

                        case PRESENCE_VECTOR_GEAR:
                            //Marcha
                            mens_teleop->id_element = ID_REMOTE_GEAR;
                            if (tipoMensajeJAUS.auxCommand->gear < JAUS_NEUTRAL_HIGH)
                                mens_teleop->value = GEAR_HIGH;
                            else if (tipoMensajeJAUS.auxCommand->gear < JAUS_REVERSE)
                                mens_teleop->value = GEAR_NEUTRAL_HIGH;
                            else if (tipoMensajeJAUS.auxCommand->gear < JAUS_NEUTRAL_LOW)
                                mens_teleop->value = GEAR_REVERSE;
                            else if (tipoMensajeJAUS.auxCommand->gear < JAUS_LOW)
                                mens_teleop->value = GEAR_NEUTRAL_LOW;
                            else
                                mens_teleop->value = GEAR_LOW;
                            ROS_INFO("Marcha Recibida %d", mens_teleop->value);


                            break;
                        default:
                            break;
                    }
                    setDiscreteDevicesMessageDestroy(tipoMensajeJAUS.auxCommand);

                }
                break;

                //Funciones auxiliares
            case JAUS_SET_FUNCTION_AUXILIAR:

                tipoMensajeJAUS.faux = setFunctionAuxiliarMessageFromJausMessage(rxMessage);
                if (tipoMensajeJAUS.faux) {
                    ROS_INFO("Funcion auxiliar: %d %d", tipoMensajeJAUS.faux->function, tipoMensajeJAUS.faux->activated);
                    tipo_mensaje = TOM_FUNC_AUX;
                    mens_fcn_aux->function = tipoMensajeJAUS.faux->function;
                    mens_fcn_aux->value = tipoMensajeJAUS.faux->activated;
                    setFunctionAuxiliarMessageDestroy(tipoMensajeJAUS.faux);
                }
                break;
                //Peticion de fichero
            case JAUS_QUERY_FILE_DATA:
                ROS_INFO("Peticion de fichero de configuración");
                tipoMensajeJAUS.petFile = queryFileDataMessageFromJausMessage(rxMessage);
                if (tipoMensajeJAUS.petFile) {
                    tipo_mensaje = TOM_PET_FILE;
                    queryFileDataMessageDestroy(tipoMensajeJAUS.petFile);
                }
                break;
                //Report de fichero actualizado
            case JAUS_REPORT_FILE_DATA:
                tipoMensajeJAUS.file = reportFileDataMessageFromJausMessage(rxMessage);
                if (tipoMensajeJAUS.file) {
                    mens_file->id_file = tipoMensajeJAUS.file->typeFile;
                    tipo_mensaje = TOM_FILE;
                    for (unsigned int i = 0; i < tipoMensajeJAUS.file->bufferSizeBytes; i++)
                        mens_file->stream.push_back(tipoMensajeJAUS.file->data[i]);
                    reportFileDataMessageDestroy(tipoMensajeJAUS.file);
                    ROS_INFO("Fichero de datos con %d bytes: ", tipoMensajeJAUS.file->bufferSizeBytes);
                    //cout << mens_file;
                }
                break;
                //Camaras
                //PAN&TILT&HOME
            case JAUS_SET_CAMERA_POSE:
                tipoMensajeJAUS.camPose = setCameraPoseMessageFromJausMessage(rxMessage);
                if (tipoMensajeJAUS.camPose) {
 		    //ROS_INFO("Presence vector %d\n",tipoMensajeJAUS.camPose->presenceVector);
                    //PAN
                    if (tipoMensajeJAUS.camPose->presenceVector == PRESENCE_VECTOR_PAN) {
                        if (tipoMensajeJAUS.camPose->zAngularMode == RATE_MODE) {
                            tipo_mensaje = TOM_CTRL_CAMERA;
                            mens_ctrl_cam->id_control = CAMERA_PAN;
 		            ROS_INFO("Camara PAN 2 %f\n",tipoMensajeJAUS.camPose->zAngularPositionOrRatePercent);
                            if (tipoMensajeJAUS.camPose->zAngularPositionOrRatePercent > 0)
                                mens_ctrl_cam->value = CAMERA_PAN_RIGHT;
                            else if (tipoMensajeJAUS.camPose->zAngularPositionOrRatePercent < 0)
                                mens_ctrl_cam->value = CAMERA_PAN_LEFT;
                            else
                                mens_ctrl_cam->value = CAMERA_PAN_STOP;
                        }
                    }//TILT
                    else if (tipoMensajeJAUS.camPose->presenceVector == PRESENCE_VECTOR_TILT) {
                        if (tipoMensajeJAUS.camPose->yAngularMode == RATE_MODE) {
                            tipo_mensaje = TOM_CTRL_CAMERA;
                            mens_ctrl_cam->id_control = CAMERA_TILT;
 		            ROS_INFO("Camara TILT 2 %f\n",tipoMensajeJAUS.camPose->yAngularPositionOrRatePercent);
                            if (tipoMensajeJAUS.camPose->yAngularPositionOrRatePercent > 0)
                                mens_ctrl_cam->value = CAMERA_TILT_UP;
                            else if (tipoMensajeJAUS.camPose->yAngularPositionOrRatePercent < 0)
                                mens_ctrl_cam->value = CAMERA_TILT_DOWN;
                            else
                                mens_ctrl_cam->value = CAMERA_TILT_STOP;
                        }
                    }//HOME
                    else if (tipoMensajeJAUS.camPose->presenceVector == PRESENCE_VECTOR_HOME) {
                        if (tipoMensajeJAUS.camPose->yAngularMode == RATE_MODE && tipoMensajeJAUS.camPose->zAngularMode == RATE_MODE) {
                            if (tipoMensajeJAUS.camPose->zAngularPositionOrRatePercent == 0 && tipoMensajeJAUS.camPose->xAngularPositionOrRatePercent == 0) {
                                tipo_mensaje = TOM_CTRL_CAMERA;
                                mens_ctrl_cam->id_control = CAMERA_HOME;
                                mens_ctrl_cam->value = CAMERA_HOME_VALUE;
                            }
                        }
                    }

                }
                break;
            case JAUS_SET_CAMERA_CAPABILITIES:
                tipoMensajeJAUS.camZoom = setCameraCapabilitiesMessageFromJausMessage(rxMessage);
                if (tipoMensajeJAUS.camZoom) {
                    if (tipoMensajeJAUS.camZoom->presenceVector == PRESENCE_VECTOR_ZOOM) {
                        tipo_mensaje = TOM_CTRL_CAMERA;
                        mens_ctrl_cam->id_control = CAMERA_ZOOM;
 		        ROS_INFO("ZOOM 2 %f\n",tipoMensajeJAUS.camZoom->horizontalFovRadians);
                        if (tipoMensajeJAUS.camZoom->horizontalFovRadians == 0)
                            mens_ctrl_cam->value = CAMERA_ZOOM_IN;
                        else
                            mens_ctrl_cam->value = CAMERA_ZOOM_OUT;
                    }
                }
                break;
            default:
                break;

        }

        switch (tipo_mensaje) {
            case TOM_REMOTE:
                nodeROS->pub_comteleop_unclean.publish(mens_teleop);
                break;
            case TOM_MODE:
                nodeROS->pub_mode.publish(mens_mode);
                break;
            case TOM_WAYPOINT:
                nodeROS->pub_waypoint.publish(mens_waypoint);
                break;
            case TOM_FUNC_AUX:
                nodeROS->pub_fcn_aux.publish(mens_fcn_aux);
                break;
            case TOM_FILE:
                switch (mens_file->id_file) {
                    case TOF_PLAN:
                        nodeROS->pub_plan.publish(mens_file);
                        break;
                    case TOF_CONFIGURATION:
                        // Escribo datos de configuracion
                        if (!nodeROS->setDebugConfiguration(mens_file->stream)) {
                            //Publico error en fichero Debug Configuration
                        }
                        break;
                    default:
                        break;
                }
                break;
            case TOM_PET_FILE:
                // Leo datos de configuracion y envio a UCR
                tipo_mensaje = TOM_FILE;
                mens_file->id_file = TOF_CONFIGURATION;
                mens_file->stream = nodeROS->getDebugConfiguration();

                JausMessage msg_JAUS;
                JausAddress destino;

                destino = jausAddressCreate(); // Destino.
                destino->subsystem = 1; // TODO a definir
                destino->node = 1; // TODO a definir
                destino->instance = 1; // TODO a definir
                destino->component = JAUS_SUBSYSTEM_COMMANDER;
                mensajeJAUS tipoMensajeJAUS;

                ROS_INFO("ENVIO FICHERO");
                //Files::writeDataInLOG("ENVIO FICHERO");

                tipoMensajeJAUS.file = reportFileDataMessageCreate();
                jausAddressCopy(tipoMensajeJAUS.file->destination, destino);
                tipoMensajeJAUS.file->typeFile = mens_file->id_file;
                tipoMensajeJAUS.file->bufferSizeBytes = mens_file->stream.size();

                for (unsigned int i = 0; i < tipoMensajeJAUS.file->bufferSizeBytes; i++)

                    tipoMensajeJAUS.file->data[i] = mens_file->stream[i];
                msg_JAUS = reportFileDataMessageToJausMessage(tipoMensajeJAUS.file);
                reportFileDataMessageDestroy(tipoMensajeJAUS.file);

                subsystemJAUS->sendJAUSMessage(msg_JAUS, NO_ACK);
                break;
            case TOM_CTRL_CAMERA:
                nodeROS->pub_ctrl_camera.publish(mens_ctrl_cam);
                break;
            default:
                ojCmptDefaultMessageProcessor(comp, rxMessage);
                break;
        }
    }
}

bool JausSubsystemVehicle::waitForACK(int typeACK,int timeout)
{
   clock_t tstart; // gestiona los timeout's
   tstart=clock();
   double diffTime;
   bool ack=false;
   do
   {
       diffTime=(clock()-tstart)/(double)CLOCKS_PER_SEC;
       if(typeACK==ackReceived)
           ack=true;

   }
   while((diffTime< timeout) && !ack);
   
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

void JausSubsystemVehicle::losedCommunication()
{
    int mode;
    if(nodeROS->requestMode(&mode))   
    {
        if(mode==MODE_REMOTE || mode==MODE_CONVOY_TELEOP)
        {
            ROS_INFO("PARADA DEL VEHICULO POR SEGURIDAD");
            Common_files::msg_com_teleopPtr stopVehicle(new Common_files::msg_com_teleop);
            stopVehicle->id_element=ID_REMOTE_THROTTLE;
            stopVehicle->value=0;
            nodeROS->pub_comteleop_clean.publish(stopVehicle);

            stopVehicle->id_element=ID_REMOTE_BRAKE;
            stopVehicle->value=100;
            nodeROS->pub_comteleop_clean.publish(stopVehicle);                  
        }
    }    
    else
    {
        //Envia error no se pudo acceder al servicio
    }
    
    //Envia error de comunicación 
    Common_files::msg_errorPtr error(new Common_files::msg_error);
    error->id_subsystem=SUBS_COMMUNICATION;
    error->id_error=COMMUNICATION_UCR_FAIL;           /************ PRUEBA **********************/
    error->type_error=TOE_UNDEFINED;
    nodeROS->pub_error.publish(error);
    
}

void JausSubsystemVehicle::establishedCommunication()
{
    int modeActual;
    JausMessage msg_JAUS=NULL;
    JausAddress destino;
    mensajeJAUS tipoMensajeJAUS;

    // Destino al que se envia el mensaje
    destino = jausAddressCreate(); // Destino.
    destino->subsystem = 1; // TODO a definir
    destino->node = 1; // TODO a definir
    destino->instance = 1; // TODO a definir
    destino->component = JAUS_SUBSYSTEM_COMMANDER;
    
    //Envio a la UCR el modo actual tras iniciarse la comunicacion
    if(nodeROS->requestMode(&modeActual))
    {
            ROS_INFO("ENVIO ESTADO MODO");
            //Files::writeDataInLOG("ENVIO ESTADO MODO");
            tipoMensajeJAUS.missionStatus=reportMissionStatusMessageCreate();
            jausAddressCopy(tipoMensajeJAUS.missionStatus->destination, destino);
            tipoMensajeJAUS.missionStatus->type=JAUS_MISSION;
            tipoMensajeJAUS.missionStatus->missionId=modeActual;
	    tipoMensajeJAUS.missionStatus->status=MODE_START;	
            tipoMensajeJAUS.missionStatus->properties.ackNak=JAUS_ACK_NAK_REQUIRED;
            jausAddressCopy(tipoMensajeJAUS.missionStatus->destination, destino);
            msg_JAUS = reportMissionStatusMessageToJausMessage(tipoMensajeJAUS.missionStatus);
            reportMissionStatusMessageDestroy(tipoMensajeJAUS.missionStatus); 
            subsystemJAUS->sendJAUSMessage(msg_JAUS,NO_ACK);
            jausMessageDestroy(msg_JAUS);
    }

    //Envia fin de error de comunicación 
    Common_files::msg_errorPtr error(new Common_files::msg_error);
    error->id_subsystem=SUBS_COMMUNICATION;
    error->id_error=COMMUNICATION_UCR_FAIL;       
    error->type_error=TOE_END_ERROR;
    nodeROS->pub_error.publish(error);
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
