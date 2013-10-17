#include "../include/Modulo_Comunicaciones/comunicaciones.h"

// Publicadores
// comentario
ros::Publisher pub_modo;
ros::Publisher pub_comteleop;
ros::Publisher pub_waypoint;
ros::Publisher pub_errores;

// Variables globales
bool comActiva;


using namespace std;
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

  // Inicio de ROS

  ros::init(argc, argv, "comunicaciones");

  // Manejador ROS
  ros::NodeHandle n;

  // Espera activa de inicio de modulo
  int estado_actual=STATE_OFF;
  while(estado_actual!=STATE_CONF){
          n.getParam("estado_modulo_comunicaciones",estado_actual);
  }
  cout << "Atica COMUNICACIONES :: Iniciando configuración..." << endl;

  // Creación de suscriptores
  ros::Subscriber sub_gps = n.subscribe("gps", 1000, fcn_sub_gps);
  ros::Subscriber sub_errores = n.subscribe("error", 1000, fcn_sub_errores);
  ros::Subscriber sub_camaras = n.subscribe("camera", 1000, fcn_sub_camaras);
  //ros::Subscriber sub_backup = n.subscribe("msg_backup", 1000, fcn_sub_backup);

  // Inicializacion de publicadores globales
  pub_modo = n.advertise<Modulo_Comunicaciones::msg_modo>("mode", 1000);
  pub_comteleop = n.advertise<Modulo_Comunicaciones::msg_com_teleoperado>("teleop", 1000);
  pub_waypoint = n.advertise<Modulo_Comunicaciones::msg_waypoint>("waypoint", 1000);
  pub_errores = n.advertise<Modulo_Comunicaciones::msg_errores>("error", 1000);

  // Variable comActiva para el chequeo de la conexion JAUS
  comActiva=false;

  // Variable para finalizacion de modulo
  bool exitCom = false;

  // Realización de la conexión JAUS
  connect();

  // Todo esta correcto, lo especificamos con el correspondiente parametro
  n.setParam("estado_modulo_comunicaciones",STATE_OK);
  cout << "Atica COMUNICACIONES :: Configurado y funcionando" << endl;

  while (ros::ok() && !exitCom){
      if(checkConnection()){
          comActiva=true;
          n.getParam("estado_modulo_comunicaciones",estado_actual);
          if(estado_actual==STATE_OFF || estado_actual==STATE_ERROR){
              disconnect();
              exitCom = true;
          }
      }else{
          comActiva=false;
          n.getParam("estado_modulo_comunicaciones",estado_actual);
          if(estado_actual==STATE_OFF || estado_actual==STATE_ERROR){
              exitCom = true;
          }else{
              connect();
          }
      }
      ros::spinOnce();
  }
  cout << "Atica COMUNICACIONES :: Módulo finalizado" << endl;
  return 0;
}


/*******************************************************************************
 *******************************************************************************
 *                              SUSCRIPTORES
 * *****************************************************************************
 * ****************************************************************************/

// Suscriptor de gps
void fcn_sub_gps(const Modulo_Comunicaciones::msg_gps msg)
{
    // Se genera el mensaje a enviar
    ROSmessage msg_ROS;
    msg_ROS.tipo_mensaje=TOM_GPS;
    msg_ROS.mens_gps=msg;

    // Espera que la comunicacion este activa
    while(!comActiva);
    // Se envia el mensaje por JAUS
    sendJAUSMessage(convertROStoJAUS(msg_ROS));
}

// Suscriptor de errores
void fcn_sub_errores(const Modulo_Comunicaciones::msg_errores msg)
{
    // Se genera el mensaje a enviar
    ROSmessage msg_ROS;
    msg_ROS.tipo_mensaje=TOM_ERROR;
    msg_ROS.mens_errores=msg;

    // Espera que la comunicacion este activa
    while(!comActiva);
    // Se envia el mensaje por JAUS
    sendJAUSMessage(convertROStoJAUS(msg_ROS));
}

// Suscriptor de camaras
void fcn_sub_camaras(const Modulo_Comunicaciones::msg_camaras msg)
{
    // Se genera el mensaje a enviar
    ROSmessage msg_ROS;
    msg_ROS.tipo_mensaje=TOM_CAMERA;
    msg_ROS.mens_cam=msg;

    // Espera que la comunicacion este activa
    while(!comActiva);
    // Se envia el mensaje por JAUS
    sendJAUSMessage(convertROStoJAUS(msg_ROS));
}

/*
// Suscriptor de backup
void fcn_sub_camaras(const Atica_COM::msg_backup msg)
{


    // Se genera el mensaje a enviar
    ROSmessage msg_ROS;
    msg_ROS.tipo_mensaje=TOM_BACKUP;
    msg_ROS.mens_backup=msg;

    // Espera que la comunicacion este activa
    while(!comActiva);
    // Se envia el mensaje por JAUS
    sendJAUSMessage(convertROStoJAUS(msg_ROS));
}*/

/*******************************************************************************
 *******************************************************************************
 *                       FUNCIONES PROPIAS DEL MODULO
 * *****************************************************************************
 * ****************************************************************************/

// Conexion JAUS

bool connect(){return true;}

// Desconexion JAUS

bool disconnect(){return true;}

// Comprobacion de la conexion

bool checkConnection(){return true;}

// Conversion de mensaje ROS a JAUS

JausMessage convertROStoJAUS(ROSmessage msg_ROS){
    JausMessage msg_JAUS;
    JausAddress destino;
    mensajeJAUS tipoMensajeJAUS;

    // Destino al que se envia el mensaje
    destino = jausAddressCreate(); // Destino.
    destino->subsystem = 1; // TODO a definir
    destino->node = 2; // TODO a definir
    destino->instance = 1; // TODO a definir
    
    switch (msg_ROS.tipo_mensaje){

        case TOM_CAMERA:
            destino->component = JAUS_VISUAL_SENSOR;
            tipoMensajeJAUS.imagen = reportImageMessageCreate();
            for(unsigned int i=0;i<msg_ROS.mens_cam.imagen.size();i++)
                tipoMensajeJAUS.imagen->data[i]=msg_ROS.mens_cam.imagen[i];
            msg_JAUS = reportImageMessageToJausMessage(tipoMensajeJAUS.imagen);
            reportImageMessageDestroy(tipoMensajeJAUS.imagen);
            break;

        case TOM_GPS:
            destino->component = JAUS_NAVEGATION_SENSOR;
            tipoMensajeJAUS.gps=reportGlobalPoseMessageCreate();
            jausAddressCopy(tipoMensajeJAUS.gps->destination, destino);
            tipoMensajeJAUS.gps->latitudeDegrees=msg_ROS.mens_gps.latitud;
            tipoMensajeJAUS.gps->longitudeDegrees=msg_ROS.mens_gps.longitud;
            tipoMensajeJAUS.gps->elevationMeters=msg_ROS.mens_gps.altitud;
            tipoMensajeJAUS.gps->rollRadians=msg_ROS.mens_gps.roll;
            tipoMensajeJAUS.gps->pitchRadians=msg_ROS.mens_gps.pitch;
            tipoMensajeJAUS.gps->yawRadians=msg_ROS.mens_gps.yaw;
            msg_JAUS = reportGlobalPoseMessageToJausMessage(tipoMensajeJAUS.gps);
            reportGlobalPoseMessageDestroy(tipoMensajeJAUS.gps);
            break;

        case TOM_ERROR:
            destino->component = JAUS_ALARMAS_SENSOR;
            tipoMensajeJAUS.alarma=reportAlarmaMessageCreate();
            jausAddressCopy(tipoMensajeJAUS.alarma->destination, destino);
            tipoMensajeJAUS.alarma->subsistema=msg_ROS.mens_errores.id_subsistema;
            tipoMensajeJAUS.alarma->idAlarma=msg_ROS.mens_errores.id_error;
            msg_JAUS = reportAlarmaMessageToJausMessage(tipoMensajeJAUS.alarma);
            reportAlarmaMessageDestroy(tipoMensajeJAUS.alarma);
            break;
        /*case TOM_BACKUP:*/
        default:
            // Imagen vacia, evita el warning
            tipoMensajeJAUS.imagen = reportImageMessageCreate();
            msg_JAUS = reportImageMessageToJausMessage(tipoMensajeJAUS.imagen);
            reportImageMessageDestroy(tipoMensajeJAUS.imagen);
            break;
    }
    return msg_JAUS;
}

// Conversion de mensaje JAUS a ROS

ROSmessage convertJAUStoROS(mensajeJAUS msg_JAUS){
    // TODO Conversion JAUS a ROS
    ROSmessage msg_ROS;
    return msg_ROS;
}

// Envio de mensaje JAUS

bool sendJAUSMessage(JausMessage msg_JAUS){
    return true;
}

// Recepcion de mensaje JAUS

bool rcvJAUSMessage(){
    return true;
}
