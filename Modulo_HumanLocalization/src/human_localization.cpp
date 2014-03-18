/* 
 * File:   human_localization.cpp
 * Author: atica
 *
 * Created on 10 de diciembre de 2013, 12:40
 */

#include "../include/Modulo_HumanLocalization/human_localization.h"

 // Publicadores

 ros::Publisher pub_waypoints;
 ros::Publisher pub_error;

using namespace std;

// Variable de continuacion de modulo
  bool exitModule;

// Variable control de modo-habilitación módulo
  bool enableModule;
/*
 * 
 */
int main(int argc, char** argv) {

  // Obtencion del modo de operacion y comprobacion de que es correcto
  int operationMode;
  if ((operationMode = getOperationMode(argc, argv)) == 0) {
      return 1;
  }
    
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
  pub_error = n.advertise<Common_files::msg_error>("error", 1000);
  pub_waypoints = n.advertise<Common_files::msg_waypoint>("wpHN", 1000);

  // Creacion de suscriptores
  ros::Subscriber sub_waypoints = n.subscribe("wpCH", 1000, fcn_sub_waypoint);
  ros::Subscriber sub_moduleEnable = n.subscribe("modEnable", 1000, fcn_sub_enableModule);
  ros::Subscriber sub_rangedatafusion = n.subscribe("rangedata", 1000, fcn_sub_rangedatafusion);

  // Variable de continuacion de modulo
  exitModule=false;

  // Todo esta correcto, lo especificamos con el correspondiente parametro
  n.setParam("estado_modulo_humanLocaliz",STATE_OK);
  cout << "Atica HUMAN LOCALIZATION :: Configurado y funcionando" << endl;

  while (ros::ok() && !exitModule)
  {
      switch (operationMode) {
        case OPERATION_MODE_DEBUG:
            // Funcionamiento del modo debug
            n.getParam("estado_modulo_humanLocalization",estado_actual);
            if(estado_actual==STATE_ERROR || estado_actual== STATE_OFF){
                exitModule=true;
            } 
            ros::spinOnce();
            break;
        case OPERATION_MODE_RELEASE:
            // Funcionamiento del modo release
            n.getParam("estado_modulo_humanLocalization",estado_actual);
            if(estado_actual==STATE_ERROR || estado_actual== STATE_OFF){
                exitModule=true;
            } 
            ros::spinOnce();
            break;
        case OPERATION_MODE_SIMULATION:
            // Funcionamiento del modo simulacion
            n.getParam("estado_modulo_humanLocalization",estado_actual);
            if(estado_actual==STATE_ERROR || estado_actual== STATE_OFF){
                exitModule=true;
            } 
            ros::spinOnce();
            break;
        default:
            break;
    }
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
void fcn_sub_waypoint(const Common_files::msg_waypoint waypoints_income)
{
    if (enableModule == true){
        pub_waypoints.publish (waypoints_income);
    }
}

// Suscriptor habilitación módulos
void fcn_sub_enableModule(const Common_files::msg_module_enable module)
{
    if ((module.id_module == ID_MOD_NAVIGATION) && (module.submode == SUBMODE_NAV_FOLLOW_ME) && (module.status == MOD_ON)){
        enableModule = true;
    }
    else
        enableModule = false;
}

// Suscriptor de RangeDataFusion
void fcn_sub_rangedatafusion(const Common_files::msg_rangedatafusion msg)
{
    
}