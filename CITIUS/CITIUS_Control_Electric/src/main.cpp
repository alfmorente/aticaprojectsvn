/* 
 * File:   main.cpp
 * Author: Carlos Amores
 *
 * Created on 12 de junio de 2014, 19:03
 */

#include <cstdlib>

using namespace std;

#include "RosNode_Electric.h"

/*******************************************************************************
 *                     METODO MAIN DEL NODO ELECTRIC                            *
 ******************************************************************************/

// Main del nodo
int main(int argc, char** argv) {
    
    // Iniciacion del middleware (ROS) para el nodo Driving
    ros::init(argc, argv, "Control_ROS_Node_Electric");
    
    RosNode_Electric *nodeElectric = new RosNode_Electric();
    
    // Intentos de reconexion
    short numOfAttemps = 0;
    ROS_INFO("[Control] Electric - Conectando con subsistema de gestion de energia");
    while(!nodeElectric->getDriverMng()->connectVehicle() && numOfAttemps < MAX_ATTEMPS){
        numOfAttemps++;
    }
    
    if(numOfAttemps < MAX_ATTEMPS){
     // Inicio de artefactos ROS
        nodeElectric->initROS();

        // Espera a permiso para comenzar a operar ROSNODE_OK
        if(nodeElectric->getEMNodeStatus() != NODESTATUS_OFF){
                ROS_INFO("[Control] Electric - Nodo listo para operar");
        }
        
        // Temporizador de requerimiento de informacion
        clock_t initTime, finalTime;
        initTime = clock();
        
        // Estructura receptora
        FrameDriving frameRcv;

        //Bucle principal
        while (ros::ok() && nodeElectric->getEMNodeStatus() != NODESTATUS_OFF) {
            // Recepcion de mensaje
            ros::spinOnce();

            // Lectura via socket (NO BLOQUEANTE)
            if (read(nodeElectric->getDriverMng()->getSocketDescriptor(), &frameRcv, sizeof(frameRcv)) >= 0) {
                ROS_INFO("[Control] Electric - Recibida trama via socket");
                nodeElectric->manageMessage(frameRcv);
            }
            
            // Comprobación del temporizador y requerimiento de info
            finalTime = clock() - initTime;
            if (((double) finalTime / ((double) CLOCKS_PER_SEC)) >= FREC_2HZ) {
                
                // Clear del timer
                initTime = clock();
                
                // Requerimiento de informacion de dispositivo
                nodeElectric->getDriverMng()->reqElectricInfo();
                
            }
        }
    } else {
        ROS_INFO("[Control] Electric - No se puede conectar al vehiculo. Maximo numero de reintentos realizados.");
    }
    ROS_INFO("[Control] Electric - Nodo finalizado");
    return 0;
}