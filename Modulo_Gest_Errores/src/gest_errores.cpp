#include "../include/Modulo_Gest_Errores/gest_errores.h"

  // Publicadores

  ros::Publisher pub_modo;
  ros::Publisher pub_errores;
  ros::Publisher pub_com_teleop;

  using namespace std;

  // Variable de control de modo
  short modoActual;
  // Variable de continuacion de modulo
  bool exitModule;

int main(int argc, char **argv)
{

  // Inicio de ROS
  ros::init(argc, argv, "gest_errores");

  // Manejador ROS
  ros::NodeHandle n;

  // Espera activa de inicio de modulo
  int estado_actual=STATE_OFF;
  while(estado_actual!=STATE_CONF){
          n.getParam("estado_modulo_gestErrores",estado_actual);
  }
  cout << "Atica GEST. ERRORES :: Iniciando configuración..." << endl;

  // Inicializacion de publicadores
  pub_modo = n.advertise<Modulo_Gest_Errores::msg_modo>("mode", 1000);
  pub_errores = n.advertise<Modulo_Gest_Errores::msg_errores>("error", 1000);
  pub_com_teleop = n.advertise<Modulo_Gest_Errores::msg_com_teleoperado>("teleop", 1000);

  // Creacion de suscriptores
  ros::Subscriber sub_errores = n.subscribe("error", 1000, fcn_sub_errores);
  ros::Subscriber sub_modo = n.subscribe("mode", 1000, fcn_sub_modo);

  // Variable de control de modo
  modoActual=0;
  // Variable de continuacion de modulo
  exitModule=false;

  // Todo esta correcto, lo especificamos con el correspondiente parametro
  n.setParam("estado_modulo_gestErrores",STATE_OK);
  cout << "Atica GEST. ERRORES :: Configurado y funcionando" << endl;

  while (ros::ok() && !exitModule)
  {
      n.getParam("estado_modulo_conduccion",estado_actual);
      if(estado_actual==STATE_ERROR || estado_actual== STATE_OFF){
          exitModule=true;
      }
      ros::spinOnce();
  }
  cout << "Atica GEST. ERRORES :: Módulo finalizado" << endl;
  return 0;
}

/*******************************************************************************
 *******************************************************************************
 *                              SUSCRIPTORES
 * *****************************************************************************
 * ****************************************************************************/

// Suscriptor de gps
void fcn_sub_modo(const Modulo_Gest_Errores::msg_modo msg)
{
    modoActual=msg.modo;
}

// Suscriptor de errores
void fcn_sub_errores(const Modulo_Gest_Errores::msg_errores msg)
{
    if(msg.tipo_error==ALARM_UNDEFINED){ // Evita leer mensajes que el mismo haya enviado
        short type_error = isWarningOrCritial(msg);
        Modulo_Gest_Errores::msg_errores msg_err;
        switch(type_error){
            case ALARM_CRITICAL:
                    // Generacion de mensaje de error y pasa a modo neutro
                    msg_err.id_subsistema=msg.id_subsistema;
                    msg_err.id_error=msg.id_error;
                    msg_err.tipo_error=ALARM_CRITICAL;
                    pub_errores.publish(msg_err);
                    switchNeutral();
                    break;
                case ALARM_EXIT:
                    // Pasa a modo neutro
                    switchNeutral();
                    break;
                case ALARM_WARNING:
                    // Generacion de mensaje de error
                    msg_err.id_subsistema=msg.id_subsistema;
                    msg_err.id_error=msg.id_error;
                    msg_err.tipo_error=ALARM_WARNING; 
                    pub_errores.publish(msg_err);
                    break;
                default:
                    cout << "Atica Gest. Errores :: Se ha recibido un error no clasificado" << endl;
        }
    }
}

/*******************************************************************************
 *******************************************************************************
 *                              FUNCIONES PROPIAS
 * *****************************************************************************
 * ****************************************************************************/
// Especifica que tipo de error se ha recibido y rellena el correspondiente
// campo del mensaje. Luego lo envia relleno.
short isWarningOrCritial(Modulo_Gest_Errores::msg_errores msg){
    // TODO !!!!!!!!!!!!!!!!!!!!!!
    return msg.tipo_error;
}
// Cambia a modo neutro
void switchNeutral(){
    Modulo_Gest_Errores::msg_modo msg_ch_neutral;
    msg_ch_neutral.modo=MODE_NEUTRAL;
    pub_modo.publish(msg_ch_neutral);
}
