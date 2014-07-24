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
        while (nodeDriving->getVMNodeStatus() != NODESTATUS_OK) {
            ros::spinOnce();
        }
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
                // Recepcion de mensaje
                ros::spinOnce();
                // ROS_INFO("Operando...");
                // Lectura via socket (NO BLOQUEANTE)
                // Buffer de envio
                char bufData[6];
                if (recv(nodeDriving->getDriverMng()->getSocketDescriptor(), bufData, sizeof (bufData), 0) > 0) {
                    // Estructura de recepcion
                    FrameDriving fdr;
                    // Rellenado del buffer
                    memcpy(&fdr.instruction, &bufData[0], sizeof (fdr.instruction));
                    memcpy(&fdr.element, &bufData[2], sizeof (fdr.element));
                    memcpy(&fdr.value, &bufData[4], sizeof (fdr.value));
                    nodeDriving->manageAlarmsMessage(fdr);
                }

                // Comprobación del temporizador y requerimiento de info
                finalTime = clock() - initTime;
                if (((double) finalTime / ((double) CLOCKS_PER_SEC)) >= FREC_5HZ) {
                    
                    hzCount++;
                    DrivingInfo info;
                    if(hzCount==5){
                        
                        hzCount = 0;
                        // Informacion completa: dispositivos y senalizacion
                        info = nodeDriving->getDriverMng()->reqFullVehicleInfo();
                   
                    }else{
                        
                        // Informacion basica: dispositivos
                        info = nodeDriving->getDriverMng()->reqBasicVehicleInfo();
                    
                    }
                    nodeDriving->publishDrivingInfo(info);
                    
                    // Clear del timer
                    initTime = clock();
                }
            }

        }
    } else {
        ROS_INFO("[Control] Driving - No se puede conectar al vehiculo");
    }
    ROS_INFO("[Control] Driving - Nodo finalizado");
    return 0;
}
