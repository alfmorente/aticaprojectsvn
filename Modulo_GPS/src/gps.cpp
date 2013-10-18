#include "../include/Modulo_GPS/gps.h"

ros::Publisher pub_gps;
ros::Publisher pub_errores;

// Variable de continuacion de modulo
 bool exitModule;

using namespace std;

int main(int argc, char **argv)
{

  // Inicio de ROS
  ros::init(argc, argv, "GPS");

    // Manejador ROS
  ros::NodeHandle n;

  // Espera activa de inicio de modulo
  int estado_actual=STATE_OFF;
  while(estado_actual!=STATE_CONF){
          n.getParam("estado_modulo_GPS",estado_actual);
  }
  cout << "Atica GPS :: Iniciando configuración..." << endl;

  // Generación de publicadores
  pub_gps = n.advertise<Modulo_GPS::msg_gps>("gps", 1000);
  pub_errores = n.advertise<Modulo_GPS::msg_errores>("error", 1000);

  // Inicializacion de variable global de fin de modulo
  exitModule=false;

  // Conexion y configuracion del dispositivo
  connect();
  configure();

  // Todo esta correcto, lo especificamos con el correspondiente parametro
  n.setParam("estado_modulo_GPS",STATE_OK);
  cout << "Atica GPS :: Configurado y funcionando" << endl;

  while (ros::ok() && !exitModule)
  {
      if(isAlive()){
          if(checkStateGPS()){
              // Genera la informacion en msg_gps y se publica

              n.getParam("estado_modulo_GPS",estado_actual);
              if(estado_actual== STATE_ERROR || estado_actual==STATE_OFF){
                  exitModule=true;
              }
          }else{
              Modulo_GPS::msg_errores msg_err;
              msg_err.id_subsistema = SUBS_GPS;
              msg_err.id_error=0; // TODO por definir
              pub_errores.publish(msg_err);

              n.getParam("estado_modulo_GPS",estado_actual);
              if(estado_actual== STATE_ERROR || estado_actual==STATE_OFF){
                  exitModule=true;
              }
          }
      }else{
          Modulo_GPS::msg_errores msg_err;
          msg_err.id_subsistema = SUBS_GPS;
          msg_err.id_error=0; // TODO por definir
          pub_errores.publish(msg_err);
          exitModule=true;
      }
  }

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

bool checkStateGPS(){return true;}
