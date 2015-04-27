/**
 * @file   gest_navegacion.cpp
 * @brief  Fichero fuente que gestióna la navegación
 * @author David Jimenez 
 * @date   2013, 2014, 2015
 */
#include <cstdlib>
#include "Modulo_Navegacion/NodeROSNavigation.h"
//#include "Modulo_Navegacion_CL/navegacion.h"

using namespace std;

/**
 * Función principal de gestión de la navegación 
 * @param argc Numero de argumentos de entrada
 * @param argv Valores de los argumentos de entrada
 * @return Entero indicando si el módulo finalizó correctamente
 */
int main(int argc, char** argv) 
{
    
    // Inicio de ROS
    NodeROSNavigation rosNode(argc,argv,"navegacion");

    // Espera activa de inicio de modulo
    int current_state=STATE_OFF;
    while(current_state!=STATE_CONF){
            current_state=rosNode.getStateModule();
    }
    cout << "Atica Navegacion :: Iniciando configuración..." << endl;

    rosNode.createPublishers();
    rosNode.createSubscribers();
    rosNode.setStateModule(STATE_OK);

    // Todo esta correcto, lo especificamos con el correspondiente parametro
    cout << "Atica Navegacion :: Configurado y funcionando" << endl; 
    bool exitModule=false;
    int navSubmodule;   
    int navStatus;    
    while (ros::ok() && !exitModule)
    {
        current_state=rosNode.getStateModule();
        navSubmodule=rosNode.nav.getCurrentType();
        navStatus=rosNode.nav.getCurrentStatus();       
        if(current_state== STATE_ERROR || current_state==STATE_OFF){
             exitModule=true;
        }
        else if(navStatus==MOD_ON)
        {
            switch(navSubmodule)
            {
                case SUBMODE_NAV_COME_TO_ME:
                    ROS_INFO("INICIO COME TO ME");
                    rosNode.nav.Come_To_Me();
                    ROS_INFO("FINALIZO COME TO ME");                  
                    break;
                case SUBMODE_NAV_PLAN:
                    ROS_INFO("INICIO PLAN");                  
                    rosNode.nav.Plan();
                    ROS_INFO("FINALIZO PLAN");
                    break;
                case SUBMODE_NAV_FOLLOW_ME:
                    ROS_INFO("INICIO FOLLOW ME");                  
                    rosNode.nav.Follow_Me();
                    ROS_INFO("FINALIZO FOLLOW ME");                  
                    break;
                default:
                    break;
            }
        }
        ros::spinOnce();
    }
    return 0;
}

