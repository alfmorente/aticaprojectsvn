/* 
 * File:   main.cpp
 * Author: Carlos Amores
 *
 * Created on 12 de junio de 2014, 19:03
 */

#include <cstdlib>

using namespace std;

#include "RosNode_Electric.h"

/*******************************************************************************
 *                     METODO MAIN DEL NODO ELECTRIC                            *
 ******************************************************************************/

// Main del nodo
int main(int argc, char** argv) {
    
    // Iniciacion del middleware (ROS) para el nodo Driving
    ros::init(argc, argv, "Control_ROS_Node_Electric");
    
    RosNode_Electric *nodeElectric = new RosNode_Electric();
    
    // Intentos de reconexion
    short numOfAttemps = 0;
    ROS_INFO("[Control] Electric - Conectando con subsistema de gestion de energia");
    while(!nodeElectric->getDriverMng()->connectVehicle() && numOfAttemps < MAX_ATTEMPS){
        numOfAttemps++;
    }
    
    if(numOfAttemps < MAX_ATTEMPS){
     // Inicio de artefactos ROS
        nodeElectric->initROS();

        // Espera a permiso para comenzar a operar ROSNODE_OK
        if(nodeElectric->getEMNodeStatus() != NODESTATUS_OFF){
                ROS_INFO("[Control] Electric - Nodo listo para operar");
        }
        
        // Temporizador de requerimiento de informacion
        clock_t initTime, finalTime;
        initTime = clock();

        //Bucle principal
        while (ros::ok() && nodeElectric->getEMNodeStatus() != NODESTATUS_OFF) {
            
            // Recepcion de mensaje ROS
            ros::spinOnce();
            
            // Comprobacion de recpcion de mensajes de vehiculo
            nodeElectric->getDriverMng()->checkForVehicleMessages();

            // Comprobacion de señalizacion de apagado
            if (nodeElectric->getDriverMng()->getTurnOffFlag()) {
                
                // Solicitud de cambio a vehicleStatus -> APAGANDO
                CITIUS_Control_Electric::srv_vehicleStatus srvTurnOff;
                srvTurnOff.request.status = OPERATION_MODE_APAGANDO;
                while(!nodeElectric->getClientVehicleStatus().call(srvTurnOff)){
                    ros::spinOnce();
                }
                
                if(srvTurnOff.response.confirmation){
                    
                    // Cambio delmodo de operacion
                    ros::NodeHandle nh;
                    nh.setParam("vehicleStatus", OPERATION_MODE_APAGANDO);

                    // Enviar confirmacion de apagado
                    FrameDriving frame;
                    frame.instruction = SET;
                    frame.id_instruction = -1;
                    frame.element = TURN_OFF;
                    frame.value = 1;
                    nodeElectric->getDriverMng()->sendToVehicle(frame);

                    // Desconexion del socket con vehiculo
                    nodeElectric->getDriverMng()->disconnectVehicle();

                    // Cambiar el estado del nodo para finalizar
                    nodeElectric->setEMNodeStatus(NODESTATUS_OFF);
                
                }else{
                    ROS_INFO("[Control] Electric - Sin confirmacion para apagar el vehiculo");
                }

            } else {

                // Comprobacion de cambio en posicion conmutador local/teleop
                if (nodeElectric->getDriverMng()->getSwitcherStruct().flag) {
                    
                    // Envio de mensaje msg_switcher
                    CITIUS_Control_Electric::msg_switcher msg;
                    msg.switcher = nodeElectric->getDriverMng()->getSwitcherStruct().position;
                    nodeElectric->getPubElectricInfo().publish(msg);
                    
                    // Clear del indicador de cambio
                    nodeElectric->getDriverMng()->setSwitcherStruct(false);
                    
                }

                // Comprobación del temporizador y requerimiento de info
                finalTime = clock() - initTime;
                if (((double) finalTime / ((double) CLOCKS_PER_SEC)) >= FREC_2HZ) {

                    // Clear del timer
                    initTime = clock();

                    // Requerimiento de informacion de dispositivo
                    ElectricInfo info = nodeElectric->getDriverMng()->getVehicleInfo();

                    // Publicacion de la informacion
                    nodeElectric->publishElectricInfo(info);

                }

            }
            
        }
    } else {
        ROS_INFO("[Control] Electric - No se puede conectar al vehiculo. Maximo numero de reintentos realizados.");
    }
    ROS_INFO("[Control] Electric - Nodo finalizado");
    return 0;
}