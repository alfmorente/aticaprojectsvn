#include "../include/Modulo_Camaras/camaras.h"

using namespace std;

// Publicadores
ros::Publisher pub_camaras;
ros::Publisher pub_errores;

// Variable de continuacion de modulo
 bool exitModule;

int main(int argc, char **argv)
{

  // Inicio de ROS
  ros::init(argc, argv, "Camaras");

  // Manejador ROS
  ros::NodeHandle n;

  // Espera activa de inicio de modulo
  int estado_actual=STATE_OFF;
  while(estado_actual!=STATE_CONF){
          n.getParam("estado_modulo_camaras",estado_actual);
  }
  cout << "Atica CAMARAS :: Iniciando configuración..." << endl;

  // Generación de publicadores
  pub_camaras = n.advertise<Modulo_Camaras::msg_camaras>("camera", 1000);
  pub_errores = n.advertise<Modulo_Camaras::msg_errores>("error", 1000);

  // Inicializacion de variable global de fin de modulo
  exitModule=false;

  // Conexion con dispositivo
  // TODO

  // Configuracion del dispositivo
  // TODO

  // Todo esta correcto, lo especificamos con el correspondiente parametro
  n.setParam("estado_modulo_camaras",STATE_OK);
  cout << "Atica CAMARAS :: Configurado y funcionando" << endl;

  while (ros::ok() && !exitModule)
  {
      if(isAlive()){
          if(checkStateCamera()){
              // Genera la imagen de la camara y lo mando por msg_camara

              n.getParam("estado_modulo_camaras",estado_actual);
              if(estado_actual== STATE_ERROR || estado_actual==STATE_OFF){
                  exitModule=true;
              }
          }else{
              Modulo_Camaras::msg_errores msg_err;
              msg_err.id_subsistema = SUBS_CAMARA;
              msg_err.id_error=0; // TODO por definir
              pub_errores.publish(msg_err);

              n.getParam("estado_modulo_camaras",estado_actual);
              if(estado_actual== STATE_ERROR || estado_actual==STATE_OFF){
                  exitModule=true;
              }
          }
      }else{
          Modulo_Camaras::msg_errores msg_err;
          msg_err.id_subsistema = SUBS_CAMARA;
          msg_err.id_error=0; // TODO por definir
          pub_errores.publish(msg_err);
          exitModule=true;
      }
  }
  cout << "Atica CAMARAS :: Módulo finalizado" << endl;
  return 0;
}

/*******************************************************************************
 *******************************************************************************
 *                              SUSCRIPTORES
 * *****************************************************************************
 * ****************************************************************************/

// No hay

/*******************************************************************************
 *******************************************************************************
 *                              FUNCIONES PROPIAS
 * *****************************************************************************
 * ****************************************************************************/

//Funciones propias

bool connect(){return true;}

bool disconnect(){return true;}

bool configure(){return true;}

bool sendData(){return true;}

bool recvData(){return true;}

bool isAlive(){return true;}

bool checkStateCamera(){return true;}
