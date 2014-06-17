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

    RosNode_Communications *nodeComm = new RosNode_Communications();

    nodeComm->initROS();
    nodeComm->initJAUS();

    ROS_INFO("[Control] Communications - Nodo listo para operar");

    ros::spin();

    ROS_INFO("[Control] Communications - Nodo finalizado");
    
    return 0;
}

