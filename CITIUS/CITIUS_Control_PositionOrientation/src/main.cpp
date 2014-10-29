/** 
 * @file  main.cpp
 * @brief Funcion principal del nodo PositionOrientation del subsistema de 
 * control
 * @author Carlos Amores
 * @date 2013, 2014
 */

#include "RosNose_PositionOrientation.h"

/**
 * Método principal del nodo. Inicializa modulos ROS y lanza el intercambio de 
 * mensajes con los dispositivos
 * @param[in] argc Número de argumentos
 * @param[in] argv Vector de argumentos
 * @return Entero distinto de 0 si ha habido problemas. 0 en caso contrario.
 */
int main(int argc, char** argv) {

    // Iniciacion del middleware (ROS) para el nodo Driving
    ros::init(argc, argv, "RosNode_Control_Position_Orientation");

    RosNode_PositionOrientation *nodePosOri = new RosNode_PositionOrientation();

    // Conexion con GPS/INS
    if (nodePosOri->getXSensManager()->connectToDevice()) {
        ROS_INFO("[Control] Position / Orientation - Conectado a dispositivo GPS/INS");
        nodePosOri->setGpsStatus(true);
    } else {
        ROS_INFO("[Control] Position / Orientation - No se puede conectar al dispositivo GPS/INS");
        nodePosOri->setGpsStatus(false);
    }

    // Conexion con magnetometro
    if (nodePosOri->getMagnetometerManager()->connectToDevice()) {
        ROS_INFO("[Control] Position / Orientation - Conectado a dispositivo Magnetometro");
        nodePosOri->setMagnStatus(true);
    } else {
        ROS_INFO("[Control] Position / Orientation - No se puede conectar al dispositivo Magnetometro");
        nodePosOri->setMagnStatus(false);
    }

    // Check de conexion de dispositivos
    if (nodePosOri->getGpsStatus() || nodePosOri->getMagnStatus()) {
        // Inicio de artefactos ROS
        nodePosOri->initROS();

        // Espera a permiso para comenzar a operar ROSNODE_OK
        ROS_INFO("[Control] Position / Orientation - Esperando activacion de nodo");

        while (nodePosOri->getNodeStatus() == NODESTATUS_INIT) {
            ros::spinOnce();
        }

        if (nodePosOri->getNodeStatus() == NODESTATUS_OK) {

            nodePosOri->configureDevices();
            ROS_INFO("[Control] Position / Orientation - Nodo activado y listo para operar");

        }

        // Bucle principal Funcionamiento completo (GPS/INS + Mag)
        if (nodePosOri->getGpsStatus() && nodePosOri->getMagnStatus()) {
            ROS_INFO("[Control] Position / Orientation - Funcionando con GPS/INS y Magnetometro");
            while (ros::ok() && nodePosOri->getNodeStatus() != NODESTATUS_OFF) {
                ros::spinOnce();
                // Requerimiento y publicacion de informacion de dispositivo GPS/INS
                nodePosOri->publishInformation();
            }
        }
            // Bucle principal Funcionamiento parcial solo GPS/INS
        else if (nodePosOri->getGpsStatus() && !nodePosOri->getMagnStatus()) {
            ROS_INFO("[Control] Position / Orientation - Funcionando solo con GPS/INS");
            while (ros::ok() && nodePosOri->getNodeStatus() != NODESTATUS_OFF) {
                ros::spinOnce();
                // Requerimiento y publicacion de informacion de dispositivo GPS/INS
                nodePosOri->publishInformation();
            }
        }
            // Bucle principal Funcionamiento parcial solo Mag
        else if (!nodePosOri->getGpsStatus() && nodePosOri->getMagnStatus()) {
            clock_t t0 = clock();
            clock_t t1;
            ROS_INFO("[Control] Position / Orientation - Funcionando solo con Magnetometro");
            while (ros::ok() && nodePosOri->getNodeStatus() != NODESTATUS_OFF) {
                ros::spinOnce();
                t1 = clock();
                if ((float(t1 - t0) / CLOCKS_PER_SEC) >= 0.1) {
                    // Requerimiento y publicacion de informacion de dispositivo GPS/INS
                    nodePosOri->publishInformation();
                    t0 = clock();
                }
            }
        }

        // Desconectar dispositivo GPS/INS
        nodePosOri->getXSensManager()->disconnectDevice();

        ROS_INFO("[Control] Position / Orientation - Desconectado dispositivo GPS/INS");

    }
    delete(nodePosOri);
    ROS_INFO("[Control] Position / Orientation - Nodo finalizado");

    return 0;
}