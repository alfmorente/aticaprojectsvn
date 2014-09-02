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
    
    ros::init(argc,argv, "RosNode_Control_RearCam");
    
    RosNode_RearCamera *rc = new RosNode_RearCamera();

    // Conexión
    if (rc->getDriverMng()->checkConnection()) {
        // Inicio de artefactos ROS
        rc->initROS();

        // Espera a permiso para comenzar a operar ROSNODE_OK
        ROS_INFO("[Control] RearCamera - Esperando activacion de nodo");
        while (rc->getRcNodeStatus() != NODESTATUS_OK) {
            ros::spinOnce();
        }
        ROS_INFO("[Control] RearCamera - Nodo listo para operar");

        // Temporizador de requerimiento de informacion
        clock_t initTime, finalTime;
        initTime = clock();

        //Bucle principal
        while (ros::ok() && rc->getRcNodeStatus() != NODESTATUS_OFF) {
            
            // Recepcion de mensaje
            ros::spinOnce();
            // Comprobación del temporizador y requerimiento de info
            finalTime = clock() - initTime;
            
            if (((double) finalTime / ((double) CLOCKS_PER_SEC)) >= FREC_2HZ) {
                
                // Clear del timer
                initTime = clock();
                                
                // Envio de parametros (activo para no perder informacion)
                rc->getDriverMng()->sentSetToDevice(ORDER_PAN,rc->getDriverMng()->getPan());
                rc->getDriverMng()->sentSetToDevice(ORDER_TILT,rc->getDriverMng()->getTilt());
                rc->getDriverMng()->sentSetToDevice(ORDER_ZOOM,rc->getDriverMng()->getZoom());
                
                // Requerimiento de informacion de dispositivo
                LensPosition lensPos = rc->getDriverMng()->getPosition();
                if(lensPos.state){
                    CITIUS_Control_RearCamera::msg_rearCameraInfo rcMsg;
                    rcMsg.pan = lensPos.pan / 50; // * (100/5000) Conversion de formato camara
                    rcMsg.tilt = lensPos.tilt / 50; // * (100/5000) Conversion de formato camara
                    rcMsg.zoom = lensPos.zoom / 50; // * (100/5000) Conversion de formato camara
                    rc->getPubRearCameraInfo().publish(rcMsg);
                }
            }
        }
    } else {
        ROS_INFO("[Control] RearCamera - No se puede conectar con la cámara");
    }
    ROS_INFO("[Control] RearCamera - Nodo finalizado");
    return 0;
}

