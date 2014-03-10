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

  // Todo esta correcto, lo especificamos con el correspondiente parametro
  n.setParam("estado_modulo_conduccion",STATE_OK);
  cout << "Atica CONDUCCION :: Configurado y funcionando" << endl;

  // Inicializacion de la comunicacion CAN
  finDePrograma = createCommunication(); // Es true: si la comunicaion se crea correctamente y false: si no se crea bien o da fallos.
  
  // Envío de erorr si no hay comunicacion CAN 
  if (!finDePrograma) {
        Modulo_Conduccion::msg_error msg_err;
        msg_err.id_subsystem = SUBS_DRIVING;
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
          
          
      }

  }
  
  // Envío de erorr si no hay envio/recepcion de tramas CAN 
  if (!conduccion->CONDUCCION_ACTIVE){
        Modulo_Conduccion::msg_error msg_err;
        msg_err.id_subsystem = SUBS_DRIVING;
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

}

// Suscriptor al Módulo de System Management
void fcn_sub_engine_brake(const Modulo_Conduccion::msg_engine_brake msg)  {
    
}

// Suscriptor al Módulo de System Management
void fcn_sub_emergency_stop(const Modulo_Conduccion::msg_emergency_stop msg) {
    
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

