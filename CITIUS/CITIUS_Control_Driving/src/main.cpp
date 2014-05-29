/* 
 * File:   main.cpp
 * Author: atica
 *
 * Created on 27 de mayo de 2014, 13:38
 */

#include <cstdlib>
#include "CITIUS_Control_Driving/DrivingConnectionManager.h"
#include "CITIUS_Control_Driving/MenuHandler.h"


DrivingConnectionManager *drivingManager;

using namespace std;

int main(int argc, char** argv) {
        
    ros::init(argc,argv,"CITIUS_ROSNODE_Driving"); 
    drivingManager = new DrivingConnectionManager((char *)"/dev/ttyUSB0");
    ros::spin();
    
    /*MenuHandler *menuManager = new MenuHandler();
    if(drivingManager->getPortOpened()){
        short selection = menuManager->printMainMenu();
        short value = menuManager->enterValueForDevice();
        cout << drivingManager->createCommand((char *)"SET",selection,value) << endl;
    }*/
    return 0;
}


