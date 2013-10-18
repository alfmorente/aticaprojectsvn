#include "../include/Modulo_Laser2D/laser2D.h"

// Publicadores
ros::Publisher pub_laser;
ros::Publisher pub_errores;

// Variable de continuacion de modulo
 bool exitModule;

using namespace std;

int main(int argc, char **argv)
{

  // Inicio de ROS
  ros::init(argc, argv, "laser2D");

    // Manejador ROS
  ros::NodeHandle n;

  // Espera activa de inicio de modulo
  int estado_actual=STATE_OFF;
  while(estado_actual!=STATE_CONF){
          n.getParam("estado_modulo_laser2D",estado_actual);
  }
  cout << "Atica Laser2D :: Iniciando configuración..." << endl;

  // Generación de publicadores
  pub_laser = n.advertise<Modulo_Laser2D::msg_laser>("laser", 1000);
  pub_errores = n.advertise<Modulo_Laser2D::msg_errores>("error", 1000);

    // Inicializacion de variable global de fin de modulo
  exitModule=false;

  // Conexion y configuracion del dispositivo
  connect();
  configure();

  // Todo esta correcto, lo especificamos con el correspondiente parametro
  n.setParam("estado_modulo_laser2D",STATE_OK);
  cout << "Atica Laser2D :: Configurado y funcionando" << endl;

  while (ros::ok() && !exitModule)
  {
      if(isAlive()){
          if(checkStateLaser()){
              // Genera la informacion en msg_gps y se publica

              n.getParam("estado_modulo_laser2D",estado_actual);
              if(estado_actual== STATE_ERROR || estado_actual==STATE_OFF){
                  exitModule=true;
              }
          }else{
              Modulo_Laser2D::msg_errores msg_err;
              msg_err.id_subsistema = SUBS_LASER_DELANTERO;
              msg_err.id_error=0; // TODO por definir
              pub_errores.publish(msg_err);

              n.getParam("estado_modulo_laser2D",estado_actual);
              if(estado_actual== STATE_ERROR || estado_actual==STATE_OFF){
                  exitModule=true;
              }
          }
      }else{
          Modulo_Laser2D::msg_errores msg_err;
          msg_err.id_subsistema = SUBS_LASER_DELANTERO;
          msg_err.id_error=0; // TODO por definir
          pub_errores.publish(msg_err);
          exitModule=true;
      }
  }
   cout << "Atica Laser2D :: Modulo finalizado" << endl;
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

bool checkStateLaser(){return true;}

