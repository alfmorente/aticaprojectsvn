/* 
 * File:   main.cpp
 * Author: Carlos Amores
 *
 * Created on 15 de junio de 2014, 16:54
 */

#include <cstdlib>
#include "RosNode_FrontCamera.h"

using namespace std;

/*
 * 
 */
int main(int argc, char** argv) {
    
    ros::init(argc,argv, "RosNode_Control_FrontCam");
    
    RosNode_FrontCamera *fc = new RosNode_FrontCamera();

    // Conexión
    if (fc->getDriverMng()->checkConnection()) {
        // Inicio de artefactos ROS
        fc->initROS();

        // Espera a permiso para comenzar a operar ROSNODE_OK
        ROS_INFO("[Control] FrontCamera - Esperando activacion de nodo");
        while (fc->getFcNodeStatus() != NODESTATUS_OK) {
            ros::spinOnce();
        }
        ROS_INFO("[Control] FrontCamera - Nodo listo para operar");

        // Temporizador de requerimiento de informacion
        clock_t initTime, finalTime;
        initTime = clock();

        //Bucle principal
        while (ros::ok() && fc->getFcNodeStatus() != NODESTATUS_OFF) {
            
            // Recepcion de mensaje
            ros::spinOnce();
            // Comprobación del temporizador y requerimiento de info
            finalTime = clock() - initTime;
            
            if (((double) finalTime / ((double) CLOCKS_PER_SEC)) >= FREC_2HZ) {
                
                // Clear del timer
                initTime = clock();
                                
                // Envio de parametros (activo para no perder informacion)
                fc->getDriverMng()->sentSetToDevice(ORDER_PAN,fc->getDriverMng()->getPan());
                fc->getDriverMng()->sentSetToDevice(ORDER_TILT,fc->getDriverMng()->getTilt());
                fc->getDriverMng()->sentSetToDevice(ORDER_ZOOM,fc->getDriverMng()->getZoom());
                
                // Requerimiento de informacion de dispositivo
                LensPosition lensPos = fc->getDriverMng()->getPosition();
                if(lensPos.state){
                    CITIUS_Control_FrontCamera::msg_frontCameraInfo fcMsg;
                    fcMsg.pan = lensPos.pan / 50; // * (100/5000) Conversion de formato camara
                    fcMsg.tilt = lensPos.tilt / 50; // * (100/5000) Conversion de formato camara
                    fcMsg.zoom = lensPos.zoom / 50; // * (100/5000) Conversion de formato camara
                    fc->getPubFrontCameraInfo().publish(fcMsg);
                }
            }
        }
    } else {
        ROS_INFO("[Control] FrontCamera - No se puede conectar con la cámara");
    }
    ROS_INFO("[Control] FrontCamera - Nodo finalizado");
    return 0;
}

