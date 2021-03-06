/* 
 * File:   JausSubsystemVehicle.cpp
 * Author: atica
 * 
 * Created on 28 de abril de 2014, 12:09
 */

#include <Modulo_Comunicaciones/JausSubsystemVehicle.h>
#include <Modulo_Comunicaciones/NodeROSCommunication.h>
#include <Modulo_Comunicaciones/ConverterTypes.h>
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
   
   //Recepción de mensajes de UCR
   else
   {
       //Si el mensaje Requiere ACK, lo envio
       if(rxMessage->properties.ackNak==JAUS_ACK_NAK_REQUIRED)
       {
             JausMessage ack=jausMessageClone(rxMessage);
             ack->dataSize=0;
             ack->destination=rxMessage->source;
             ack->source=rxMessage->destination;
             ack->properties.ackNak=JAUS_ACKNOWLEDGE;
             subsystemJAUS->sendJAUSMessage(ack,NO_ACK);        
       }
       //Convierto a mensaje ROS
       ROSmessage msg_ROS=convertJAUStoROS(rxMessage);
       if(msg_ROS.tipo_mensaje == TOM_REMOTE)
             nodeROS->pub_comteleop_unclean.publish(msg_ROS.mens_teleop);
       else if(msg_ROS.tipo_mensaje == TOM_MODE)
             nodeROS->pub_mode.publish(msg_ROS.mens_mode);
       else if(msg_ROS.tipo_mensaje == TOM_WAYPOINT)
             nodeROS->pub_waypoint.publish(msg_ROS.mens_waypoint);
       else if(msg_ROS.tipo_mensaje == TOM_FUNC_AUX)
             nodeROS->pub_fcn_aux.publish(msg_ROS.mens_fcn_aux);
       else if(msg_ROS.tipo_mensaje == TOM_FILE)
       {
           if(msg_ROS.mens_file.id_file==TOF_PLAN)
                     nodeROS->pub_plan.publish(msg_ROS.mens_file); 
           else if(msg_ROS.mens_file.id_file==TOF_CONFIGURATION){
               // Escribo datos de configuracion
               if(!nodeROS->setDebugConfiguration(msg_ROS.mens_file.stream))
               {
                   //Publico error en fichero Debug Configuration
               }  
           }        
       }
       else if(msg_ROS.tipo_mensaje == TOM_PET_FILE)
       {
           // Leo datos de configuracion y envio a UCR 
           msg_ROS.tipo_mensaje=TOM_FILE;
           msg_ROS.mens_file.id_file =TOF_CONFIGURATION;
           msg_ROS.mens_file.stream=nodeROS->getDebugConfiguration();

           subsystemJAUS->sendJAUSMessage(convertROStoJAUS(msg_ROS),NO_ACK);            
       }
       else if(msg_ROS.tipo_mensaje == TOM_CTRL_CAMERA)
           nodeROS->pub_ctrl_camera.publish(msg_ROS.mens_ctrl_cam);
       else
            ojCmptDefaultMessageProcessor(comp,rxMessage);
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
            Common_files::msg_com_teleop stopVehicle;
            stopVehicle.id_element=ID_REMOTE_THROTTLE;
            stopVehicle.value=0;
            nodeROS->pub_comteleop_clean.publish(stopVehicle);

            stopVehicle.id_element=ID_REMOTE_BRAKE;
            stopVehicle.value=100;
            nodeROS->pub_comteleop_clean.publish(stopVehicle);                  
        }
    }    
    else
    {
        //Envia error no se pudo acceder al servicio
    }
    
    //Envia error de comunicación 
    Common_files::msg_error error;
    error.id_subsystem=SUBS_COMMUNICATION;
    error.id_error=COMMUNICATION_UCR_FAIL;           /************ PRUEBA **********************/
    error.type_error=TOE_UNDEFINED;
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
            tipoMensajeJAUS.missionStatus->properties.ackNak=JAUS_ACK_NAK_REQUIRED;
            jausAddressCopy(tipoMensajeJAUS.missionStatus->destination, destino);
            msg_JAUS = reportMissionStatusMessageToJausMessage(tipoMensajeJAUS.missionStatus);
            reportMissionStatusMessageDestroy(tipoMensajeJAUS.missionStatus); 
            subsystemJAUS->sendJAUSMessage(msg_JAUS,NO_ACK);
            jausMessageDestroy(msg_JAUS);
    }
}