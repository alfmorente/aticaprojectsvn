/* 
 * File:   conduccion.cpp
 * Author: Sergio Doctor López
 *
 * Created on 5 de marzo de 2014, 10:03
 */

#include "../include/Modulo_Conduccion/conduccion.h"

using namespace std;

int main(int argc, char **argv)
{
  // Inicio de ROS
  ros::init(argc, argv, "conduccion");

  ros::NodeHandle n;        // Manejador ROS
  
  int estado_actual =  STATE_OK;    // Para hacer pruebas locales
  
  // Espera activa de inicio de modulo
  //int estado_actual=STATE_OFF;
  //while(estado_actual!=STATE_CONF){
  //        n.getParam("estado_modulo_conduccion",estado_actual);
  //}
  cout << "Atica CONDUCCION :: Iniciando configuración..." << endl;

  // Generación de publicadores
  pub_error = n.advertise<Modulo_Conduccion::msg_error>("error",1000);
  pub_switch = n.advertise<Modulo_Conduccion::msg_switch>("switch",1000);
  pub_backup = n.advertise<Modulo_Conduccion::msg_backup>("backup",1000);
  pub_info_stop = n.advertise<Modulo_Conduccion::msg_info_stop>("infoStop",1000);

  // Generación de subscriptores  
  sub_navigation = n.subscribe("pre_navigation", 1000, fcn_sub_navigation);
  sub_com_teleop = n.subscribe("clean",1000,fcn_sub_com_teleop);
  sub_engine_brake = n.subscribe("engBrakeSD",1000,fcn_sub_engine_brake);
  sub_emergency_stop = n.subscribe("emergency",1000,fcn_sub_emergency_stop);
  
  // Generacion 

  // Todo esta correcto, lo especificamos con el correspondiente parametro
  n.setParam("estado_modulo_conduccion",STATE_OK);
  cout << "Atica CONDUCCION :: Configurado y funcionando" << endl;

  // Inicializacion de la comunicacion CAN
  finDePrograma = createCommunication(); // Es true: si la comunicaion se crea correctamente y false: si no se crea bien o da fallos.
  
  msg_err.id_subsystem = SUBS_DRIVING;
  
  // Envío de erorr si no hay comunicacion CAN 
  if (!finDePrograma) {       
        msg_err.id_error = CONNECTION_CAN_FAIL; // Error en conduccion (0: comunicacion)
        pub_error.publish(msg_err);
  }
  
  while (ros::ok() && finDePrograma && conduccion->CONDUCCION_ACTIVE)
  {     
      n.getParam("estado_modulo_conduccion",estado_actual);
      if(estado_actual==STATE_ERROR || estado_actual== STATE_OFF)             
         finDePrograma=disconnectCommunication();   // Fin de programa = false (se cierra el CAN)
      else {
                   
          // Publicaciones....
          
          // Publicacion del mensaje de Backup
          msg_backup.throttle = conduccion->acelerador; // acelerador
          msg_backup.brake = conduccion->freno_servicio; // freno de servicio
          msg_backup.steer = (conduccion->sentido_marcha) * (conduccion->direccion) ; // direccion
          msg_backup.handbrake = conduccion->freno_estacionamiento; // freno de mano
          msg_backup.gear = conduccion->marcha_actual; // marcha
          msg_backup.engine = conduccion->arranque_parada; // arranque/parada
          msg_backup.speed = conduccion->velocidad; // velocidad
          pub_backup.publish(msg_backup);
          
          
          // Publicacion del mensaje switch
          if (conduccion->conmutador_m_a == 0)
                msg_switch.value = false;   // Manual
          else if (conduccion->conmutador_m_a == 1)
                msg_switch.value = true;    // Teleoperado
          pub_switch.publish(msg_switch);
          
          
          // Publicacion del mensaje info_stop
          if (conduccion->parada_emergencia_obstaculo == 1) {
              msg_info_stop.id_event = 0;
              msg_info_stop.value = true;
          }
          else if (conduccion->parada_emergencia_obstaculo == 0) {
              msg_info_stop.id_event = 0;
              msg_info_stop.value = false;
          }
          pub_info_stop.publish(msg_info_stop);
                    
          if (conduccion->parada_emergencia_remota == 1) {
              msg_info_stop.id_event = 1;
              msg_info_stop.value = true;
          }
          else if (conduccion->parada_emergencia_remota == 0) {
              msg_info_stop.id_event = 1;
              msg_info_stop.value = false;
          }
          pub_info_stop.publish(msg_info_stop);
          
          if (conduccion->parada_emergencia_local == 1) {
              msg_info_stop.id_event = 2;
              msg_info_stop.value = true;
          }
          else if (conduccion->parada_emergencia_local== 0) {
              msg_info_stop.id_event = 2;
              msg_info_stop.value = false;
          }
          pub_info_stop.publish(msg_info_stop);
          
          // Publicacion del mensaje error
          switch (conduccion->id_error_Conduccion){
              case 2:           // Fallo arranque/Parada
                  msg_err.id_error = START_STOP_FAILURE;
                  break;
              case 3:           // Fallo acelerador
                  msg_err.id_error = THROTTLE_FAILURE;
                  break;
              case 4:           // Fallo freno de estacionamiento
                  msg_err.id_error = HANDBRAKE_FAILURE;
                  break;
              case 5:           // Fallo freno de servicio
                  msg_err.id_error = BRAKE_FAILURE;
                  break;
              case 6:           // Fallo cambio de marcha
                  msg_err.id_error = GEAR_SHIFT_FAILURE;
                  break;
              case 7:           // Fallo de direccion
                  msg_err.id_error = STEER_FAILURE;
                  break;
              case 8:           // Fallo bloqueo de diferenciales
                  msg_err.id_error = DIFFERENTIAL_LOCK_FAILURE;
                  break;
              case -1:
                  break;
              default:
                  break;
          }
          pub_error.publish(msg_err);
      }

  }
  
  // Envío de erorr si no hay envio/recepcion de tramas CAN 
  if (!conduccion->CONDUCCION_ACTIVE){
        msg_err.id_error = CONNECTION_CAN_FAIL; // Error en conduccion (1: envio/recepcion tramas CAN)
        pub_error.publish(msg_err);                
  }
      
  cout << "Atica CONDUCCION :: Módulo finalizado" << endl;
  return 0;
}



/*******************************************************************************
 *******************************************************************************
 *                              SUSCRIPTORES
 * *****************************************************************************
 * ****************************************************************************/

// Suscriptor al Módulo de Navegacion
void fcn_sub_navigation(const Modulo_Conduccion::msg_navigation msg)
{
  ROS_INFO("I heard a NAVIGATION message");
}

// Suscriptor al Módulo de Remote
void fcn_sub_com_teleop(const Modulo_Conduccion::msg_com_teleop msg)
{
  ROS_INFO("I heard a TELEOP DEPURADO. message");

  switch (msg.id_element) {
        case 0:   // Acelerador
            conduccion->acelerador_tx = (short) msg.value;
            break;
        case 1:   // Freno de servicio
            conduccion->freno_servicio_tx = (short) msg.value;
            break;
        case 2:   // Direccion
            conduccion->valor_direccion = (short) msg.value;
            break;
        case 3:   // Marcha
            conduccion->valor_marcha = (short) msg.value;
            break;
        case 4:   // Freno de mano
            conduccion->valor_freno_estacionamiento = (short) msg.value;
            break;
        case 5:   // Encendido del motor
            conduccion->valor_arranque_parada = (short) msg.value;
            break;
        case 6:   // Luces IR
            conduccion->valor_luces_IR = (short) msg.value;
            break;
        case 7:   // Luces 
            conduccion->valor_luces = (short) msg.value;
            break;
        case 8:   // Diferenciales
            conduccion->valor_diferencial = (short) msg.value;
            break;
        case 9:   // Activacion Laser
            conduccion->valor_laser = (short) msg.value;
            break;
        default:          
            break;          
    }
        
  conduccion->valor_parada_emergencia = 0;

}

// Suscriptor al Módulo de System Management
void fcn_sub_engine_brake(const Modulo_Conduccion::msg_engine_brake msg)  {
    
    ROS_INFO("I heard a ENGINE BRAKE message from SYSTEM MANAGEMENT");
    
    if (msg.command) {          // brake
        if (msg.value)
            conduccion->valor_freno_estacionamiento = 1;  // 1 --> ON   
        else
            conduccion->valor_freno_estacionamiento = 0;  // 0 --> OFF    
    }
    else {                      // engine
        if (msg.value)
            conduccion->valor_arranque_parada = 1;      // 1 --> ON   
        else
            conduccion->valor_arranque_parada = 0;  // 0 --> OFF
    }       
    
}

// Suscriptor al Módulo de System Management
void fcn_sub_emergency_stop(const Modulo_Conduccion::msg_emergency_stop msg) {
    
    ROS_INFO("I heard a EMERGENCY STOP message from SYSTEM MANAGEMENT");
    
    if (msg.value)
        conduccion->valor_parada_emergencia = 1;
    else
        conduccion->valor_parada_emergencia = 1;
    
}


/*******************************************************************************
 *******************************************************************************
 *                              FUNCIONES PROPIAS
 * *****************************************************************************
 * ****************************************************************************/

bool createCommunication(){
    
    bool res = false;

    while(CANflag < 3 && CANflag != 0);
    
    can = new CANCommunication(false, false, HW_PCI, 0, 0, 0x031C, false, "Conduccion"); // Asegurarnos del BaudRate
    
    conduccion = new ConduccionThread(can);
    
     // Establecimiento y configuración de las comunicaciones
    
    do {
        if (can->EstablishCommunication() == true){
            can->ConfigureCommunication();
            CANflag = 0;
        }
        else{
            CANflag++;
            if (CANflag == 3){           // Para más de 3 errores se cierra todo
                can->~CANCommunication();
                return res = false;
            }
        }
    } while(CANflag < 3 && CANflag != 0);

    // Arranque de todos los manejadores
    
    can->Run();
    
    conduccion->Run();
    
    return res = true;
}

bool disconnectCommunication(){
     can->~CANCommunication();
     return false;
}

bool sendData(){return true;}

bool recvData(){return true;}

bool checkConnection(){return true;}

bool convertROStoCAN(){return true;}

bool convertCANtoROS(){return true;}

