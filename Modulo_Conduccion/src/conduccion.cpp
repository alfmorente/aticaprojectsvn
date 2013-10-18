#include "../include/Modulo_Conduccion/conduccion.h"

// Publicadores
ros::Publisher pub_errores;

// Control de fin de programa
bool finDePrograma;

using namespace std;

int main(int argc, char **argv)
{

  // Inicio de ROS
  ros::init(argc, argv, "conduccion");

  // Manejador ROS
  ros::NodeHandle n;

  // Espera activa de inicio de modulo
  int estado_actual=STATE_OFF;
  while(estado_actual!=STATE_CONF){
          n.getParam("estado_modulo_conduccion",estado_actual);
  }
  cout << "Atica CONDUCCION :: Iniciando configuración..." << endl;

  // Inicializacion de variables globales
  finDePrograma=false;

  // Generación de publicadores
  //ros::Publisher pub_backup = n.advertise<Modulo_Conduccion::msg_backup>("pub_backup", 1000);
  pub_errores = n.advertise<Modulo_Conduccion::msg_errores>("error",1000);

  ros::Subscriber sub_navegacion = n.subscribe("pre_navigation", 1000, fcn_sub_navegacion);
  ros::Subscriber sub_com_teleop = n.subscribe("teleop",1000,fcn_sub_com_teleop);

  // Se crea la conexion con el automata
  connect();

  // Todo esta correcto, lo especificamos con el correspondiente parametro
  n.setParam("estado_modulo_conduccion",STATE_OK);
  cout << "Atica CONDUCCION :: Configurado y funcionando" << endl;

  while (ros::ok() && !finDePrograma)
  {
      if(checkConnection()){
          n.getParam("estado_modulo_conduccion",estado_actual);
          if(estado_actual==STATE_ERROR || estado_actual== STATE_OFF){
              disconnect();
              finDePrograma=true;
              // fin de programa
          }
      }else{
          finDePrograma=true;
          Modulo_Conduccion::msg_errores msg_err;
          msg_err.id_subsistema=SUBS_CONDUCCION;
          msg_err.id_error=0; // TODO, aun no definido
          pub_errores.publish(msg_err);
      }
  }
  cout << "Atica CONDUCCION :: Módulo finalizado" << endl;
  return 0;
}

/*******************************************************************************
 *******************************************************************************
 *                              SUSCRIPTORES
 * *****************************************************************************
 * ****************************************************************************/

// Suscriptor de errores
void fcn_sub_navegacion(const Modulo_Conduccion::msg_gest_navegacion msg)
{
  ROS_INFO("I heard a NAVIGATION message");
}

// Suscriptor de teleoperado
void fcn_sub_com_teleop(const Modulo_Conduccion::msg_com_teleoperado msg)
{
  ROS_INFO("I heard a TELEOP. message");
}

/*******************************************************************************
 *******************************************************************************
 *                              FUNCIONES PROPIAS
 * *****************************************************************************
 * ****************************************************************************/

//Funciones propias
bool connect(){return true;}

bool disconnect(){return true;}

bool sendData(){return true;}

bool recvData(){return true;}

bool checkConnection(){return true;}

bool convertROStoCAN(){return true;}

bool convertCANtoROS(){return true;}

