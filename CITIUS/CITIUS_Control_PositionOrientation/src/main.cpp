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
        
        if(nodePosOri->getTraxManager()->connectToDevice()) {
            
            ROS_INFO("[Control] Position / Orientation - Conectado a dispositivo Magnetometro");
            
            // Inicio de artefactos ROS
            nodePosOri->initROS();

            // Espera a permiso para comenzar a operar ROSNODE_OK
            ROS_INFO("[Control] Position / Orientation - Esperando activacion de nodo");
            while (nodePosOri->getPONodeStatus() != NODESTATUS_OK) {
                ros::spinOnce();
            }
            ROS_INFO("[Control] Position / Orientation - Nodo listo para operar");

            // Temporizador de requerimiento de informacion
            clock_t initTime, finalTime;
            initTime = clock();
            
            //Bucle principal
            while (ros::ok() && nodePosOri->getPONodeStatus() != NODESTATUS_OFF) {
                
                ros::spinOnce();
                
                // Comprobación del temporizador y requerimiento de info
                finalTime = clock() - initTime;

                if (((double) finalTime / ((double) CLOCKS_PER_SEC)) >= FREC_30HZ) {
                    
                    // Requerimiento y publicacion de informacion de dispositivo GPS/INS
                    nodePosOri->publishInformation();
                    
                    // Clear del timer
                    initTime = clock();
                }
            }
            
        }else{
            
            ROS_INFO("[Control] Position / Orientation - No se puede conectar al dispositivo Magnetometro");
        
        }
        
    } else {
        
        ROS_INFO("[Control] Position / Orientation - No se puede conectar al dispositivo GPS/INS");
    
    }
    
    ROS_INFO("[Control] Position / Orientation - Nodo finalizado");
    
    return 0;
}

