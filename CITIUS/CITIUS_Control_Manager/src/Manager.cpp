/* 
 * File:   main.cpp
 * Author: Carlos Amores (AICIA)
 *
 * Created on 13 de mayo de 2014, 12:34
 */

#include <cstdlib>
#include "ros/ros.h"
#include "CITIUS_Control_Manager/ROSNode_Manager.h"

using namespace std;



int main(int argc, char** argv) {
    
    ros::init(argc,argv,"ROSNose_Manager");
    ROSNode_Manager *manager = new ROSNode_Manager();
    manager->initialize();
    manager->run();
    return 0;
}
