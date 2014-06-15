#include "RosNode_Driving.h"

/*******************************************************************************
 *                     METODO MAIN DEL NODO DRIVING                            *
 ******************************************************************************/

// Main del nodo
int main(int argc, char** argv) {
    
    // Iniciacion del middleware (ROS) para el nodo Driving
    ros::init(argc,argv,"[Control] ROS Node - Driving");
    
    RosNode_Driving *nodeDriving = new RosNode_Driving();
    
    if(nodeDriving->getDriverMng()->connectVehicle()){
     // Inicio de artefactos ROS
        nodeDriving->initROS();

        // Espera a permiso para comenzar a operar ROSNODE_OK
        ROS_INFO("[Control] Driving - Esperando activacion de nodo");
        while (nodeDriving->getVMNodeStatus() != NODESTATUS_OK) {
            ros::spinOnce();
        }
        ROS_INFO("[Control] Driving - Nodo listo para operar");
        
        // Temporizador de requerimiento de informacion
        clock_t initTime, finalTime;
        initTime = clock();
        
        // Estructura receptora
        FrameDriving frameRcv;

        //Bucle principal
        while (ros::ok() && nodeDriving->getVMNodeStatus() != NODESTATUS_OFF) {
            // Recepcion de mensaje
            ros::spinOnce();

            // Lectura via socket (NO BLOQUEANTE)
            if (read(nodeDriving->getDriverMng()->getSocketDescriptor(), &frameRcv, sizeof(frameRcv)) >= 0) {
                ROS_INFO("[Control] Driving - Recibida trama via socket");
                nodeDriving->manageMessage(frameRcv);
            }
            
            // Comprobación del temporizador y requerimiento de info
            finalTime = clock() - initTime;
            if (((double) finalTime / ((double) CLOCKS_PER_SEC)) >= FREC_2HZ) {
                // Requerimiento de informacion de dispositivo
                nodeDriving->getDriverMng()->reqVehicleInfo();
                // Clear del timer
                initTime = clock();
            }
        }
    } else {
        ROS_INFO("[Control] Driving - No se puede conectar al vehículo");
    }
    ROS_INFO("[Control] Driving - Nodo finalizado");
    return 0;
}
