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
    
    ros::init(argc,argv, "[Control] ROS Node - Front Camera");
    
    RosNode_FrontCamera *fc = new RosNode_FrontCamera();

    // Conexión
    if (fc->getDriverMng()->connect()) {
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
                // Requerimiento de informacion de dispositivo
                float pan = fc->getDriverMng()->getParam(IDPARAM_PAN);
                float tilt = fc->getDriverMng()->getParam(IDPARAM_TILT);
                // Montaje del mensaje y publicacion
                CITIUS_Control_FrontCamera::msg_frontCameraInfo fcMsg;
                fcMsg.pan = pan;
                fcMsg.tilt = tilt;
                fc->getPubFrontCameraInfo().publish(fcMsg);
                // Clear del timer
                initTime = clock();
            }
        }
    } else {
        ROS_INFO("[Control] FrontCamera - No se puede conectar con la cámara");
    }
    ROS_INFO("[Control] FrontCamera - Nodo finalizado");
    return 0;
}

