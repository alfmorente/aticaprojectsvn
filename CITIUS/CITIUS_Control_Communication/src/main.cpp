/* 
 * File:   main.cpp
 * Author: Carlos Amores
 *
 * Created on 12 de junio de 2014, 18:15
 */

#include <cstdlib>
#include "RosNode_Communications.h"

using namespace std;

/*
 * 
 */
int main(int argc, char** argv) {

    // Iniciacion del middleware (ROS) para el nodo Communications
    ros::init(argc, argv, "Control_ROS_Node_Communications");

    RosNode_Communications *nodeComm = RosNode_Communications::getInstance();

    nodeComm->initROS();
    nodeComm->initJAUS();

    ROS_INFO("[Control] Communications - Nodo listo para operar");

    // Temporizador de envio de estado
    clock_t initTime, finalTime;
    initTime = clock();

    while (ros::ok()) {
        
        // Recepcion de mensajeria
        ros::spinOnce();
        
        // Temporizador para envio de estado
        finalTime = clock() - initTime;
        
        if (((double) finalTime / ((double) CLOCKS_PER_SEC)) >= FREC_10HZ) {
            
            // Clear del timer
            initTime = clock();
            
            // Requerimiento de informacion de dispositivo
            nodeComm->informStatus();
            
        }
    }
    
    nodeComm->endJAUS();

    ROS_INFO("[Control] Communications - Nodo finalizado");
    
    return 0;
}

