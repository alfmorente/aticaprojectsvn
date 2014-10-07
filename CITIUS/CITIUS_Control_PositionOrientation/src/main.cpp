
/** 
 * @file  main.cpp
 * @brief Funcion principal del nodo PositionOrientation del subsistema de 
 * control
 * @author: Carlos Amores
 * @date: 2013, 2014
 */

#include "RosNose_PositionOrientation.h"

/**
 * Metodo principal del nodo. Inicializa modulos ROS y lanza el
 * intercambio y la recepcion de mensajes con los dispositivos
 * @param[in] argc Numero de argumentos
 * @param[in] argv Vector de argumentos
 * @return Entero distinto de 0 si ha habido problemas. 0 en caso contrario.
 */
int main(int argc, char** argv) {

  // Iniciacion del middleware (ROS) para el nodo Driving
  ros::init(argc, argv, "RosNode_Control_Position_Orientation");

  RosNode_PositionOrientation *nodePosOri = new RosNode_PositionOrientation();

  // Conexion con GPS/INS
  if (nodePosOri->getXSensManager()->connectToDevice()) {

    ROS_INFO("[Control] Position / Orientation - Conectado a dispositivo GPS/INS");
    nodePosOri->setGpsStatus(true);

    if (nodePosOri->getMagnetometerManager()->connectToDevice()) {

      ROS_INFO("[Control] Position / Orientation - Conectado a dispositivo Magnetometro");

      nodePosOri->setMagnStatus(true);

      ROS_INFO("[Control] Position / Orientation - Funcionamiento completo (GPS/INS + Magnetometro)");

    } else {

      ROS_INFO("[Control] Position / Orientation - No se puede conectar al dispositivo Magnetometro");
      nodePosOri->setMagnStatus(false);
      ROS_INFO("[Control] Position / Orientation - Funcionamiento con GPS/INS (sin Magnetometro)");

    }

  } else {

    ROS_INFO("[Control] Position / Orientation - No se puede conectar al dispositivo GPS/INS");
    nodePosOri->setGpsStatus(false);

    if (nodePosOri->getMagnetometerManager()->connectToDevice()) {

      ROS_INFO("[Control] Position / Orientation - Conectado a dispositivo Magnetometro");
      nodePosOri->setMagnStatus(true);
      ROS_INFO("[Control] Position / Orientation - Funcionamiento con Magnetometro (sin GPS/INS)");

    } else {

      ROS_INFO("[Control] Position / Orientation - No se puede conectar al dispositivo Magnetometro");
      nodePosOri->setMagnStatus(false);
      ROS_INFO("[Control] Position / Orientation - No hay dispositivos conectados");
    }

  }

  // Check de conexion de dispositivos
  if (nodePosOri->getGpsStatus() || nodePosOri->getMagnStatus()) {
    // Inicio de artefactos ROS
    nodePosOri->initROS();

    // Espera a permiso para comenzar a operar ROSNODE_OK
    ROS_INFO("[Control] Position / Orientation - Esperando activacion de nodo");

    while (nodePosOri->getPONodeStatus() == NODESTATUS_INIT) {
      ros::spinOnce();
    }

    if (nodePosOri->getPONodeStatus() == NODESTATUS_OK) {

      nodePosOri->configureDevices();
      ROS_INFO("[Control] Position / Orientation - Nodo activado y listo para operar");

    }

    // Bucle principal Funcionamiento completo (GPS/INS + Mag)
    if (nodePosOri->getGpsStatus() && nodePosOri->getMagnStatus()) {

      while (ros::ok() && nodePosOri->getPONodeStatus() != NODESTATUS_OFF) {

        ros::spinOnce();

        // Requerimiento y publicacion de informacion de dispositivo GPS/INS
        nodePosOri->publishInformation();

      }


    }

      // Bucle principal Funcionamiento parcial solo GPS/INS
    else if (nodePosOri->getGpsStatus() && !nodePosOri->getMagnStatus()) {

      while (ros::ok() && nodePosOri->getPONodeStatus() != NODESTATUS_OFF) {

        ros::spinOnce();

        // Requerimiento y publicacion de informacion de dispositivo GPS/INS
        nodePosOri->publishInformation();

      }
    }

      // Bucle principal Funcionamiento parcial solo Mag
    else if (!nodePosOri->getGpsStatus() && nodePosOri->getMagnStatus()) {
      clock_t t0 = clock();
      clock_t t1;
      while (ros::ok() && nodePosOri->getPONodeStatus() != NODESTATUS_OFF) {

        ros::spinOnce();

        t1 = clock();
        if ((float(t1 - t0) / CLOCKS_PER_SEC) >= 0.1) {
          // Requerimiento y publicacion de informacion de dispositivo GPS/INS
          nodePosOri->publishInformation();
          t0 = clock();

        }
      }
    }


    // Desconectar dispositivo GPS/INS
    nodePosOri->getXSensManager()->disconnectDevice();


    ROS_INFO("[Control] Position / Orientation - Desconectado dispositivo GPS/INS");

  }

  ROS_INFO("[Control] Position / Orientation - Nodo finalizado");

  nodePosOri->~RosNode_PositionOrientation();

  return 0;
}


