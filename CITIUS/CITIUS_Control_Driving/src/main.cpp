#include "CITIUS_Control_Driving/DrivingConnectionManager.h"

/*******************************************************************************
 *                     METODO MAIN DEL NODO DRIVING                            *
 ******************************************************************************/

// Main del nodo
int main(int argc, char** argv) {
    
    // Iniciacion del middleware (ROS) para el nodo Driving
    ros::init(argc,argv,"ROSNODE_Control_Driving");
    
    // Con la creacion de dcm, se crean e inician suscriptores, publicadores,
    // y servicios. Tambien se abre el socket y se pone a disposicion del nodo.
    DrivingConnectionManager *dcm = new DrivingConnectionManager();
    
    // Comprobacion de una correcta inicializacion
    if(dcm->getNodeStatus()==NODESTATUS_OK){
        
        // Almacenaje de informacion procedente de Payload de conduccion
        FrameDriving recvFrame;
        
        // Temporizador para requerimiento de informacion a Payload de conduccion
        clock_t initTime, finalTime;
        initTime = clock();
        
        // Bucle principal. Control+C y Estado del nodo
        while(ros::ok() && (dcm->getNodeStatus()!=NODESTATUS_OFF)){    

            // Comprobacion mensajeria (topics/servicios) - No bloqueante
            ros::spinOnce();
            
            // Comprobacion mensajeria (socket) - Establecido lectura no bloqueante
            if (recv(dcm->getSocketDescriptor(), &recvFrame, sizeof (recvFrame), 0) >= 0) {
                ROS_INFO("[Control] Driving :: Mensaje recibido del Payload de conduccion");
                dcm->messageManager(recvFrame);
            }
            
            // Cada segundo se solicita informacion del vehiculo
            finalTime = clock() - initTime;
            if(((double)finalTime / ((double)CLOCKS_PER_SEC))>=1){
                dcm->reqVehicleInformation();
            }
            
        }
        
    }else{
        
        ROS_INFO("[Control] Driving :: Fallo en la inicializacion del nodo");
        
    }
    
    ROS_INFO("[Control] Driving :: Nodo finalizado");
    
    return 0;
}
