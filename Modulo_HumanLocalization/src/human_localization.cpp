/* 
 * File:   human_localization.cpp
 * Author: atica
 *
 * Created on 10 de diciembre de 2013, 12:40
 */

#include "../include/Modulo_HumanLocalization/human_localization.h"

 // Publicadores

 ros::Publisher pub_waypoints;
 ros::Publisher pub_errores;

using namespace std;

// Variable de continuacion de modulo
  bool exitModule;

// Variable control de modo-habilitación módulo
  bool enableModule;
/*
 * 
 */
int main(int argc, char** argv) {

  // Inicio de ROS
  ros::init(argc, argv, "human_localization");

  // Manejador ROS
  ros::NodeHandle n;

  // Espera activa de inicio de modulo
  int estado_actual=STATE_OFF;
  while(estado_actual!=STATE_CONF){
          n.getParam("estado_modulo_humanLocalization",estado_actual);
  }
  cout << "Atica HUMAN LOCALIZATION :: Iniciando configuración..." << endl;

  // Inicializacion de publicadores
  pub_errores = n.advertise<Modulo_HumanLocalization::msg_errores>("error", 1000);
  pub_waypoints = n.advertise<Modulo_HumanLocalization::msg_waypoint>("waypoints_outcome", 1000);

  // Creacion de suscriptores
  ros::Subscriber sub_waypoints = n.subscribe("waypoints_income", 1000, fcn_sub_waypoint);
  ros::Subscriber sub_moduleEnable = n.subscribe("moduleEnable", 1000, fcn_sub_enableModule);

  // Variable de continuacion de modulo
  exitModule=false;

  // Todo esta correcto, lo especificamos con el correspondiente parametro
  n.setParam("estado_modulo_humanLocaliz",STATE_OK);
  cout << "Atica HUMAN LOCALIZATION :: Configurado y funcionando" << endl;

  while (ros::ok() && !exitModule)
  {
      n.getParam("estado_modulo_humanLocaliz",estado_actual);
      if(estado_actual==STATE_ERROR || estado_actual== STATE_OFF){
          exitModule=true;
      }
      ros::spinOnce();
  }
  cout << "Atica HUMAN LOCALIZATION :: Módulo finalizado" << endl;

  return 0;
}

/*******************************************************************************
 *******************************************************************************
 *                              SUSCRIPTORES
 * *****************************************************************************
 * ****************************************************************************/

// Suscriptor de waypoints
void fcn_sub_waypoint(const Modulo_HumanLocalization::msg_waypoint waypoints_income)
{
    if (enableModule == true){
        pub_waypoints.publish (waypoints_income);
    }
}

// Suscriptor habilitación módulos
void fcn_sub_enableModule(const Modulo_HumanLocalization::msg_module_enable module)
{
    if ((module.id_module == ID_MOD_NAVEGACION) && (module.submodule == SUBMODE_NAV_FOLLOW_ME) && (module.status == MODULE_ON)){
        enableModule = true;
    }
    else
        enableModule = false;
}