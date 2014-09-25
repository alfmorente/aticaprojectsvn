#include "RosNode_Driving.h"

/*******************************************************************************
 *                     METODO MAIN DEL NODO DRIVING                            *
 ******************************************************************************/

// Main del nodo

int main(int argc, char** argv) {

    // Iniciacion del middleware (ROS) para el nodo Driving
    ros::init(argc, argv, "RosNode_Control_Driving");

    RosNode_Driving *nodeDriving = new RosNode_Driving();

    if (nodeDriving->getDriverMng()->connectVehicle()) {
        // Inicio de artefactos ROS
        nodeDriving->initROS();

        // Espera a permiso para comenzar a operar ROSNODE_OK
        ROS_INFO("[Control] Driving - Esperando activacion de nodo");
        while (nodeDriving->getVMNodeStatus() == NODESTATUS_INIT) {
            ros::spinOnce();
        }
        if(nodeDriving->getVMNodeStatus() == NODESTATUS_OK)
                ROS_INFO("[Control] Driving - Nodo listo para operar");

        // Temporizador de requerimiento de informacion
        clock_t initTime, finalTime;
        initTime = clock();

        // Conteo de 5 hz -> 5 * 1hz (Vehiculo 5Hz, Senalizacion 1Hz))
        short hzCount = 0;

        //Bucle principal
        while (ros::ok() && nodeDriving->getVMNodeStatus() != NODESTATUS_OFF) {
            if (nodeDriving->getDriverMng()->getSocketDescriptor() == -1) {
                ROS_INFO("Se ha perdido comunicacion con servidor");
            } else {
                
                // Comprobacion de recepcion de mensaje ROS
                ros::spinOnce();
                
                // Comprobacion de recpcion de mensajes de vehiculo
                nodeDriving->getDriverMng()->checkForVehicleMessages();

                // Comprobación del temporizador y requerimiento de info
                finalTime = clock() - initTime;
                if (((double) finalTime / ((double) CLOCKS_PER_SEC)) >= FREC_5HZ) {
                    
                    // Clear del timer
                    initTime = clock();
                    
                    hzCount++;
                    DrivingInfo info;
                    if(hzCount==5){
                        
                        hzCount = 0;
                        // Informacion completa: dispositivos y senalizacion
                        // Requiere al vehiculo (GET)
                        nodeDriving->getDriverMng()->reqVehicleInfo(true);
                        // Obtiene informacion existente para publicar
                        info = nodeDriving->getDriverMng()->getVehicleInfo(true);
                   
                    }else{
                        
                        // Informacion basica: dispositivos
                        // Requiere al vehiculo (GET)
                        nodeDriving->getDriverMng()->reqVehicleInfo(false);
                        // Obtiene informacion existente para publicar
                        info = nodeDriving->getDriverMng()->getVehicleInfo(false);
                    
                    }
                    nodeDriving->publishDrivingInfo(info);
                    
                }
            }

        }
    } else {
        ROS_INFO("[Control] Driving - No se puede conectar al vehiculo");
    }
    ROS_INFO("[Control] Driving - Nodo finalizado");
    return 0;
}
