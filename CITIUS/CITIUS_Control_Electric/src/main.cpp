/* 
 * File:   main.cpp
 * Author: Carlos Amores
 *
 * Created on 12 de junio de 2014, 19:03
 */

#include <cstdlib>

using namespace std;

#include "CITIUS_Control_Electric/ElectricConnectionManager.h"

/*******************************************************************************
 *                     METODO MAIN DEL NODO DRIVING                            *
 ******************************************************************************/

int main(int argc, char** argv) {

        // Iniciacion del middleware (ROS) para el nodo Driving
    ros::init(argc,argv,"ROSNODE_Control_Electric");
    
    // Con la creacion de dcm, se crean e inician suscriptores, publicadores,
    // y servicios. Tambien se abre el socket y se pone a disposicion del nodo.
    ElectricConnectionManager *ecm = new ElectricConnectionManager();
    
    // Comprobacion de una correcta inicializacion
    if(ecm->getNodeStatus()==NODESTATUS_OK){
        
        // Almacenaje de informacion procedente de Payload de conduccion
        FrameDriving recvFrame;
        
        // Temporizador para requerimiento de informacion a Payload de conduccion
        clock_t initTime, finalTime;
        initTime = clock();
        
        // Bucle principal. Control+C y Estado del nodo
        while(ros::ok() && (ecm->getNodeStatus()!=NODESTATUS_OFF)){    

            // Comprobacion mensajeria (topics/servicios) - No bloqueante
            ros::spinOnce();
            
            // Comprobacion mensajeria (socket) - Establecido lectura no bloqueante
            if (recv(ecm->getSocketDescriptor(), &recvFrame, sizeof (recvFrame), 0) >= 0) {
                ROS_INFO("[Control] Electric :: Mensaje recibido del Payload de conduccion");
                ecm->messageManager(recvFrame);
            }
            
            // Cada segundo se solicita informacion del vehiculo
            finalTime = clock() - initTime;
            if(((double)finalTime / ((double)CLOCKS_PER_SEC))>=1){
                ecm->reqVehicleInformation();
            }
            
        }
        
    }else{
        
        ROS_INFO("[Control] Driving :: Fallo en la inicializacion del nodo");
        
    }
    
    ROS_INFO("[Control] Driving :: Nodo finalizado");
    
    return 0;
}

