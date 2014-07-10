/* 
 * File:   main.cpp
 * Author: Carlos Amores
 *
 * Created on 4 de julio de 2014, 9:04
 */

#include <cstdlib>
#include "ros/ros.h"
#include "RosNose_PositionOrientation.h"

using namespace std;

/*
 * 
 */
int main(int argc, char** argv) {

    // Iniciacion del middleware (ROS) para el nodo Driving
    ros::init(argc, argv, "RosNode_Control_Position_Orientation");

    RosNode_PositionOrientation *nodePosOri = new RosNode_PositionOrientation();

    // Conexion con GPS/INS
    if (nodePosOri->getXSensManager()->connectToDevice()) {

        ROS_INFO("[Control] Position / Orientation - Conectado a dispositivo GPS/INS");

        // Inicio de artefactos ROS
        nodePosOri->initROS();

        // Espera a permiso para comenzar a operar ROSNODE_OK
        ROS_INFO("[Control] Position / Orientation - Esperando activacion de nodo");
        while (nodePosOri->getPONodeStatus() == NODESTATUS_INIT) {
            ros::spinOnce();
        }

        if (nodePosOri->getPONodeStatus() == NODESTATUS_OK) {
            printf("cnsocvs\n");
            nodePosOri->getXSensManager()->configureDevice();
            ROS_INFO("[Control] Position / Orientation - Configuracion de GPS/INS establecida");
            ROS_INFO("[Control] Position / Orientation - Nodo listo para operar");
        }

        //Bucle principal
        while (ros::ok() && nodePosOri->getPONodeStatus() != NODESTATUS_OFF) {

            ros::spinOnce();

            // Requerimiento y publicacion de informacion de dispositivo GPS/INS
            nodePosOri->publishInformation();

        }

        // Desconectar dispositivo GPS/INS
        nodePosOri->getXSensManager()->disconnectDevice();
        
        
        ROS_INFO("[Control] Position / Orientation - Desconectado dispositivo GPS/INS");

    } else {

        ROS_INFO("[Control] Position / Orientation - No se puede conectar al dispositivo GPS/INS");

    }

    ROS_INFO("[Control] Position / Orientation - Nodo finalizado");
    return 0;
}

