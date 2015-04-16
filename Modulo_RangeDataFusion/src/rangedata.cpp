/**
  @file rangedata.cpp
  @brief 

 * Archivo principal del Módulo Range Data Fusion. 

  @author Alfonso Morente
  @date 18/03/2014

*/

#include "../include/Modulo_RangeDataFusion/rangedata.h"

// Publicadores
ros::Publisher pub_rangedata;

using namespace std;

// Variable de continuacion de modulo
bool exitModule;

/**
 * Método principal del nodo. 
 * @param[in] argc Número de argumentos
 * @param[in] argv Vector de argumentos
 * @return Entero distinto de 0 si ha habido problemas. 0 en caso contrario. 
 */
int main(int argc, char** argv) {
  // Obtencion del modo de operacion y comprobacion de que es correcto
  int operationMode;
  if ((operationMode = getOperationMode(argc, argv)) == 0) {
      return 1;
  }
    
  // Inicio de ROS
  ros::init(argc, argv, "range_data_fusion");

  // Manejador ROS
  ros::NodeHandle n;

  // Espera activa de inicio de modulo
  int estado_actual=STATE_OFF;
  while(estado_actual!=STATE_CONF){
          n.getParam("estado_modulo_rangeDataFusion",estado_actual);
  }
  cout << "Atica RANGE DATA FUSION :: Iniciando configuración..." << endl;

  // Inicializacion de publicadores
  pub_rangedata = n.advertise<Common_files::msg_rangedatafusion>("rangedata", 1000);
  
  // Creacion de suscriptores
  ros::Subscriber sub_gps = n.subscribe("gps", 1000, fcn_sub_gps);
  ros::Subscriber sub_laser = n.subscribe("laser", 1000, fcn_sub_laser);
  ros::Subscriber sub_moduleEnable = n.subscribe("modEnable", 1000, fcn_sub_module_enable);
  //ros::Subscriber sub_beacon = n.subscribe("beacon", 1000, fcn_sub_beacon);

  // Variable de continuacion de modulo
  exitModule=false;

  // Todo esta correcto, lo especificamos con el correspondiente parametro
  n.setParam("estado_modulo_humanLocaliz",STATE_OK);
  cout << "Atica RANGE DATA FUSION :: Configurado y funcionando" << endl;

  while (ros::ok() && !exitModule)
  {
      switch (operationMode) {
        case OPERATION_MODE_DEBUG:
            // Funcionamiento del modo debug
            n.getParam("estado_modulo_rangeDataFusion",estado_actual);
            if(estado_actual==STATE_ERROR || estado_actual== STATE_OFF){
                exitModule=true;
            } 
            ros::spinOnce();
            break;
        case OPERATION_MODE_RELEASE:
            // Funcionamiento del modo release
            n.getParam("estado_modulo_rangeDataFusion",estado_actual);
            if(estado_actual==STATE_ERROR || estado_actual== STATE_OFF){
                exitModule=true;
            } 
            ros::spinOnce();
            break;
        case OPERATION_MODE_SIMULATION:
            // Funcionamiento del modo simulacion
            n.getParam("estado_modulo_rangeDataFusion",estado_actual);
            if(estado_actual==STATE_ERROR || estado_actual== STATE_OFF){
                exitModule=true;
            } 
            ros::spinOnce();
            break;
        default:
            break;
    }
  }
  cout << "Atica RANGE DATA FUSION :: Módulo finalizado" << endl;

  return 0;

}


/*******************************************************************************
 *******************************************************************************
 *                              SUSCRIPTORES
 * *****************************************************************************
 * ****************************************************************************/

// Suscriptor de habilitación de modulo (module_enable)
void fcn_sub_module_enable(const Common_files::msg_module_enable msg)
{
    
}

// Suscriptor de GPS
void fcn_sub_gps(const Common_files::msg_gps msg)
{
    
}

// Suscriptor de laser 
void fcn_sub_laser(const Common_files::msg_laser msg)
{
    
}

/*
 // Suscriptor de radiobalizas
 void fcn_sub_beacon(const Common_files::msg beacon msg)
 {
 
 }
 */