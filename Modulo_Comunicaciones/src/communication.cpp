#include <Modulo_Comunicaciones/NodeROSCommunication.h>
#include <Modulo_Comunicaciones/JausSubsystemVehicle.h>
#include <Modulo_Comunicaciones/interaction.h>

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
    if(state_module==STATE_ERROR)
    {
          ROS_INFO("ATICA COMMUNICATION VEHICLE:: Module finish for error in other module");
          exit(1);
    }   
    usleep(500000);
  }while(state_module!=STATE_CONF);

  ROS_INFO("ATICA COMMUNICATION VEHICLE :: Init configuration...");
 
  rosnode->createSubscribers();
  rosnode->createServers();
  rosnode->createPublishers();
  rosnode->createClients();


  JausSubsystemVehicle* subsystemVehicle=JausSubsystemVehicle::getInstance();
  //Configuración de la comunicacion
  subsystemVehicle->communicationState=COM_OFF;
  if(!subsystemVehicle->configureJAUS())
  {
      Common_files::msg_error errorCOM;
      errorCOM.id_subsystem=SUBS_COMMUNICATION;
      errorCOM.type_error=TOE_UNDEFINED;
      errorCOM.id_error=JAUS_CONFIG_ERROR;
      rosnode->pub_error.publish(errorCOM);
      rosnode->setStateModule(STATE_ERROR); //completar
      exit(1);
  }


  //Configuracion realizada. Modulo preparado y activo
  rosnode->setStateModule(STATE_OK); //completar
  ROS_INFO("ATICA COMMUNICATION VEHICLE:: Configurate and Run");

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
      }
      else if(subsystemVehicle->communicationState==COM_OFF)
      {
	  if(subsystemVehicle->connect())
           subsystemVehicle->communicationState=COM_ON;   
      }
      ros::spinOnce();
  }
  subsystemVehicle->disconnect();
  ROS_INFO("ATICA COMMUNICATION VEHICLE:: Module finish");
  return 0;
}
