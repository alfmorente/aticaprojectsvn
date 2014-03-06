#include "../include/Modulo_GPS/gps.h"

ros::Publisher pub_gps;
ros::Publisher pub_errores;

using namespace std;

// what has to be done at program exit
void do_exit(int error)
{
  printf("finished GPS (%d).\n\n", error);
  exit(error);
}

// the signal handler for manual break Ctrl-C
void signal_handler(int signal)
{
  do_exit(0);
}

// what has to be done at program start
void init()
{
  /* install signal handlers */
  signal(SIGTERM, signal_handler);
  signal(SIGINT, signal_handler);

}

int main(int argc, char **argv)
{
  // Orden para la parada manual con CTtrl+C
  init();

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

  // Inicialización de suscriptores
  ros::Subscriber sub_moduleEnable = n.subscribe("moduleEnable", 1000, fcn_sub_enableModule);

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

void fcn_sub_enableModule(const Modulo_GPS::msg_module_enable msg){
    if (msg.id_module == ID_MOD_TEACH){
        if (msg.status == STATUS_ON){
            // Guarda los datos del GPS en archivo TEMP
        }
        else if (msg.status == STATUS_OFF){
            // Cierra archivo temporal. Filtra y lo envía
        }
    }
}
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
