#include <Modulo_Comunicaciones/NodeROSCommunication.h>
#include <Modulo_Comunicaciones/JausSubsystemVehicle.h>
#include <Modulo_Comunicaciones/interaction.h>
#include <Modulo_Comunicaciones/Files.h>

int main(int argc, char **argv)
{
    // Indica el modo de Operacion del modulo
    int operationMode=getOperationMode(argc, argv);
    if (operationMode == 0) {
         return 1;
    }
    NodeROSCommunication::init(argc,argv,"COMMUNICATION"); 
    NodeROSCommunication* rosnode=NodeROSCommunication::getInstance();
    
    // Espera activa de inicio de modulo
    int state_module;
    do
    {
      state_module=rosnode->getStateModule(); 
      usleep(500000);
    }while(state_module!=STATE_CONF);

    rosnode->createSubscribers();
    rosnode->createServers();
    rosnode->createPublishers();
    rosnode->createClients();    
    
    Files files;
    ComConfig configModule;

    int error=files.openFiles();

    if(error!=NO_ERROR)
    {
         //Publico el error
        Common_files::msg_errorPtr errorCOM(new Common_files::msg_error);
        errorCOM->id_subsystem = SUBS_COMMUNICATION;
        errorCOM->type_error = TOE_UNDEFINED;
        errorCOM->id_error=error; //por definir
        rosnode->pub_error.publish(errorCOM); 
        if(error==COMM_LOG_FILE_ERROR)
                Files::writeErrorInLOG(error,"Fichero LOG data: ");
        else if(error==COMM_CONFIG_FILE_ERROR)
                Files::writeErrorInLOG(error,"Fichero de configuracion: ");
        exit(1);
    }
    error =files.readConfig(&configModule);
    if(error!=NO_ERROR)
    {
        //Publico el error
        Common_files::msg_errorPtr errorCOM(new Common_files::msg_error);
        errorCOM->id_subsystem = SUBS_COMMUNICATION;
        errorCOM->type_error = TOE_UNDEFINED;
        errorCOM->id_error=error; //por definir
        rosnode->pub_error.publish(errorCOM);    
        Files::writeErrorInLOG(error,"Fichero de configuracion: ");
        exit(1);
    }  
    
    ROS_INFO("ATICA COMMUNICATION VEHICLE:: Init configuration...");
    Files::writeDataInLOG("ATICA COMMUNICATION VEHICLE:: Init configuration...");


    JausSubsystemVehicle* subsystemVehicle=JausSubsystemVehicle::getInstance();
    
    //Configuración de la comunicacion
    subsystemVehicle->communicationState=COM_OFF;

    error=subsystemVehicle->configureJAUS();
    if(error!=NO_ERROR)
{
        Common_files::msg_errorPtr errorCOM(new Common_files::msg_error);
        errorCOM->id_subsystem = SUBS_COMMUNICATION;
        errorCOM->type_error = TOE_UNDEFINED;
        errorCOM->id_error = error;
        rosnode->pub_error.publish(errorCOM);
        rosnode->setStateModule(STATE_ERROR); //completar
        Files::writeErrorInLOG(error, "Configuracion: ");
        exit(1);
    }

    //Configuracion realizada. Modulo preparado y activo
    rosnode->setStateModule(STATE_OK); //completar
    ROS_INFO("ATICA COMMUNICATION VEHICLE:: Configurate and Run");
    Files::writeDataInLOG("ATICA COMMUNICATION VEHICLE:: Configurate and Run");

    // Espera activa de inicio general de los módulos
    int state_system;
    do
    {
      rosnode->n->getParam("state_system",state_system);
      state_module=rosnode->getStateModule();
      if(state_module==STATE_OFF)
      {
            ROS_INFO("ATICA COMMUNICATION VEHICLE:: Module finish for error in other module");
            exit(1);
      }   
      usleep(500000);
    }while(state_system!=STATE_SYSTEM_ON);
    

    while (ros::ok() && rosnode->getStateModule()!=STATE_OFF){
        if(subsystemVehicle->communicationState==COM_ON)
        {
          if(!subsystemVehicle->checkConnection())
            subsystemVehicle->communicationState=COM_LOSED;
        }
        else if(subsystemVehicle->communicationState==COM_LOSED)
        {
            subsystemVehicle->losedCommunication();   
            subsystemVehicle->communicationState=COM_OFF;    
            Files::writeErrorInLOG(COMMUNICATION_UCR_FAIL,"Communication");      
            Files::writeDataInLOG("ATICA COMMUNICATION VEHICLE:: Communication losed");            
        }
        else if(subsystemVehicle->communicationState==COM_OFF)
        {
            if(subsystemVehicle->connect())
            {
                Files::writeDataInLOG("ATICA COMMUNICATION VEHICLE:: Communication stablished");
                subsystemVehicle->communicationState=COM_ON; 
                subsystemVehicle->establishedCommunication();
                
            }
        }
        ros::spinOnce();
        usleep(25000);
    }
    subsystemVehicle->disconnect();
    ROS_INFO("ATICA COMMUNICATION VEHICLE:: Module finish");
    return 0;
}
