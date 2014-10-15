
/** 
 * @file  main.cpp
 * @brief Funcion principal del nodo Driving del subsistema de control
 * @author Carlos Amores
 * @date 2013, 2014
 */


#include "RosNode_Electric.h"

/**
 * Metodo principal del nodo. Inicializa modulos ROS y lanza el
 * intercambio y la recepcion de mensajes con el vehiculo y su logica de
 * negocio
 * @param[in] argc Numero de argumentos
 * @param[in] argv Vector de argumentos
 * @return Entero distinto de 0 si ha habido problemas. 0 en caso contrario.
 */
int main(int argc, char** argv) {

  // Iniciacion del middleware (ROS) para el nodo Driving
  ros::init(argc, argv, "Control_ROS_Node_Electric");

  RosNode_Electric *nodeElectric = new RosNode_Electric();

  // Intentos de reconexion
  short numOfAttemps = 0;
  ROS_INFO("[Control] Electric - Conectando con subsistema de gestion de energia");
  while (!nodeElectric->getDriverMng()->connectVehicle() && numOfAttemps < MAX_ATTEMPS) {
    numOfAttemps++;
  }

  if (numOfAttemps < MAX_ATTEMPS) {
    // Inicio de artefactos ROS
    nodeElectric->initROS();

    // Espera a permiso para comenzar a operar ROSNODE_OK
    //if (nodeElectric->getEMNodeStatus() != NODESTATUS_OFF) {
    if(nodeElectric->getNodeStatus() != NODESTATUS_OFF){
      ROS_INFO("[Control] Electric - Nodo listo para operar");
    }

    // Temporizador de requerimiento de informacion
    clock_t initTime, finalTime;
    initTime = clock();

    //Bucle principal
    //while (ros::ok() && nodeElectric->getEMNodeStatus() != NODESTATUS_OFF) {
    while(ros::ok() && nodeElectric->getNodeStatus()!=NODESTATUS_OFF){

      // Recepcion de mensaje ROS
      ros::spinOnce();

      // Comprobacion de recpcion de mensajes de vehiculo
      nodeElectric->getDriverMng()->checkForVehicleMessages();

      // Comprobacion de señalizacion de apagado
      if (nodeElectric->getDriverMng()->getTurnOffFlag()) {

        // Solicitud de cambio a vehicleStatus -> APAGANDO
        CITIUS_Control_Electric::srv_vehicleStatus srvTurnOff;
        srvTurnOff.request.status = OPERATION_MODE_APAGANDO;
        while (!nodeElectric->getClientVehicleStatus().call(srvTurnOff)) {
          ros::spinOnce();
        }

        if (srvTurnOff.response.confirmation) {

          // Cambio delmodo de operacion
          ros::NodeHandle nh;
          nh.setParam("vehicleStatus", OPERATION_MODE_APAGANDO);

          // Enviar confirmacion de apagado                    
          nodeElectric->getDriverMng()->setTurnOff();

          // Desconexion del socket con vehiculo
          nodeElectric->getDriverMng()->disconnectVehicle();

          // Cambiar el estado del nodo para finalizar
          //nodeElectric->setEMNodeStatus(NODESTATUS_OFF);
          nodeElectric->setNodeStatus(NODESTATUS_OFF);

        } else {
          ROS_INFO("[Control] Electric - Sin confirmacion para apagar el vehiculo");
        }

      }

      // Comprobacion de cambio en posicion conmutador local/teleop
      if (nodeElectric->getDriverMng()->getSwitcherStruct().flag) {

        // Envio de mensaje msg_switcher
        nodeElectric->publishSwitcherInfo(nodeElectric->getDriverMng()->getSwitcherStruct().position);

        // Activacion / Desactivacion de actuadores
        if (nodeElectric->getDriverMng()->getSwitcherStruct().position == 1) {
          nodeElectric->publishSetupCommands(true);
        } else {
          nodeElectric->publishSetupCommands(false);
        }

        // Clear del indicador de cambio
        nodeElectric->getDriverMng()->setSwitcherStruct(false);

      }

      // Tratamiento de alarmas 
      // TODO

      // Comprobación del temporizador y requerimiento de info
      finalTime = clock() - initTime;
      if (((double) finalTime / ((double) CLOCKS_PER_SEC)) >= FREC_2HZ) {

        // Clear del timer
        initTime = clock();

        // Requerimiento de informacion de sistema energetico
        nodeElectric->getDriverMng()->reqElectricInfo();

        // Publicacion de la informacion existente
        nodeElectric->publishElectricInfo(nodeElectric->getDriverMng()->getVehicleInfo());

      }

    }

  } else {
    ROS_INFO("[Control] Electric - No se puede conectar al vehiculo. Maximo numero de reintentos realizados.");
  }
  ROS_INFO("[Control] Electric - Nodo finalizado");
  return 0;
}