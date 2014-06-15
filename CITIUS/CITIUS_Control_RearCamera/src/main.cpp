/* 
 * File:   main.cpp
 * Author: Carlos Amores
 *
 * Created on 15 de junio de 2014, 16:54
 */

#include <cstdlib>
#include "RosNode_RearCamera.h"

using namespace std;

/*
 * 
 */
int main(int argc, char** argv) {
    
    ros::init(argc,argv, "[Control] ROS Node - Rear Camera");
    
    RosNode_RearCamera *fc = new RosNode_RearCamera();

    // Conexión
    if (fc->getDriverMng()->connect()) {
        // Inicio de artefactos ROS
        fc->initROS();

        // Espera a permiso para comenzar a operar ROSNODE_OK
        ROS_INFO("[Control] RearCamera - Esperando activacion de nodo");
        while (fc->getFcNodeStatus() != NODESTATUS_OK) {
            ros::spinOnce();
        }
        ROS_INFO("[Control] RearCamera - Nodo listo para operar");

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
                CITIUS_Control_RearCamera::msg_rearCameraInfo rcMsg;
                rcMsg.pan = pan;
                rcMsg.tilt = tilt;
                fc->getPubRearCameraInfo().publish(rcMsg);
                // Clear del timer
                initTime = clock();
            }
        }
    } else {
        ROS_INFO("[Control] RearCamera - No se puede conectar con la cámara");
    }
    ROS_INFO("[Control] RearCamera - Nodo finalizado");
    return 0;
}

