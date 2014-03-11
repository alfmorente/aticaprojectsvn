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
  // Obtencion del modo de operacion y comprobacion de que es correcto
  if ((operationMode = getOperationMode(argc, argv)) == 0) {
      return 1;
  }

  // Orden para la parada manual con CTtrl+C
  init_signals();
  
  // Inicio de ROS
  ros::init(argc, argv, "conduccion");

  ros::NodeHandle n;        // Manejador ROS
  
  int estado_actual = STATE_OK;    // Para hacer pruebas locales
  
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
  pub_emergency_stop = n.advertise<Modulo_Conduccion::msg_emergency_stop>("emergInfo",1000);

  // Generación de subscriptores  
  sub_navigation = n.subscribe("pre_navigation", 1000, fcn_sub_navigation);
  sub_com_teleop = n.subscribe("clean",1000,fcn_sub_com_teleop);
  sub_fcn_aux = n.subscribe("engBrake",1000,fcn_sub_engine_brake);
  sub_emergency_stop = n.subscribe("emergSet",1000,fcn_sub_emergency_stop);
  
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
        
        cout << "CONNECTION_CAN_FAIL \n";
  }
  
  switch (operationMode) {
        case OPERATION_MODE_DEBUG:
            
            cout << "FUNCIONAMIENTO EN MODO DEBUG \n\n";
            
            while (ros::ok() && finDePrograma && conduccion->CONDUCCION_ACTIVE) {     
                
                n.getParam("estado_modulo_conduccion",estado_actual);
                if(estado_actual==STATE_ERROR || estado_actual== STATE_OFF) {             
                   finDePrograma=disconnectCommunication();   // Fin de programa = false (se cierra el CAN)

                   cout << "Fin de programa --- Se cierra el CAN \n";
                }
                
                else {

                    // Publicaciones....

                    // Publicacion del mensaje de Backup
                    msg_backup.throttle = (short) conduccion->acelerador; // acelerador
                    msg_backup.brake = (short)conduccion->freno_servicio; // freno de servicio
                    msg_backup.steer = (short) (conduccion->sentido_marcha) * (conduccion->direccion) ; // direccion
                    msg_backup.handbrake = (short) conduccion->freno_estacionamiento; // freno de mano
                    msg_backup.gear = (short) conduccion->marcha_actual; // marcha
                    msg_backup.engine = (short) conduccion->arranque_parada; // arranque/parada
                    msg_backup.speed = (short) conduccion->velocidad; // velocidad
                    pub_backup.publish(msg_backup);

                    // Impresion del mensaje de Backup

                    cout << "********** Publicacion del mensaje BACKUP *********** \n";
                    cout << "Acelerador: " << msg_backup.throttle << "\n";
                    cout << "Freno de Servicio: " << msg_backup.brake << "\n";
                    cout << "Direccion: " << msg_backup.steer << "\n";
                    cout << "Freno de mano: " << msg_backup.handbrake << "\n";
                    cout << "Marcha: " << msg_backup.gear << "\n";
                    cout << "Arranque/Parada: " << msg_backup.engine << "\n";
                    cout << "Velocidad: " << msg_backup.speed << "\n";
                    cout << "***************************************************** \n\n\n";


                    // Publicacion del mensaje switch
                    if (conduccion->conmutador_m_a == 0)
                          msg_switch.value = false;   // Manual
                    else if (conduccion->conmutador_m_a == 1)
                          msg_switch.value = true;    // Teleoperado
                    pub_switch.publish(msg_switch);

                    // Impresion del mensaje switch

                    cout << "********** Publicacion del mensaje SWITCH *********** \n";
                    cout << "Si es 0 = Manual; Si es 1 = Teleoperado \n";
                    cout << "Comuntador Manual/Automático: " << (int) msg_switch.value << "\n";
                    cout << "***************************************************** \n\n\n";


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

                    // Impresion del mensaje info_stop

                    cout << "********** Publicacion del mensaje INFO_STOP 1 *********** \n";
                    cout << "Si es 0 = NADA ; Si es 1 = Parada de emergencia obstaculo;  \n";
                    cout << "Parada de emergencia obstaculo: " << (int) msg_info_stop.value << "\n";
                    cout << "********************************************************** \n\n\n";

                    if (conduccion->parada_emergencia_remota == 1) {
                        msg_info_stop.id_event = 1;
                        msg_info_stop.value = true;

                        // MANDAR ACK A SYSTEM MANAGEMENT

                        msg_emergency_stop.value = INFO;
                        pub_emergency_stop.publish(msg_emergency_stop);

                        cout << "******** Publicacion del mensaje EMERGENCY STOP 2 *********** \n";
                        cout << "Si es 1 = INFO;  \n";
                        cout << "Emergency Stop: " << msg_emergency_stop.value << "\n";
                        cout << "************************************************************* \n\n\n";

                    }
                    else if (conduccion->parada_emergencia_remota == 0) {
                        msg_info_stop.id_event = 1;
                        msg_info_stop.value = false;
                    }
                    pub_info_stop.publish(msg_info_stop);

                    // Impresion del mensaje info_stop

                    cout << "********** Publicacion del mensaje INFO_STOP 3 *********** \n";
                    cout << "Si es 0 = NADA ; Si es 1 = Parada de emergencia remota;  \n";
                    cout << "Parada de emergencia remota: " << (int) msg_info_stop.value << "\n";
                    cout << "********************************************************** \n\n\n";

                    if (conduccion->parada_emergencia_local == 1) {
                        msg_info_stop.id_event = 2;
                        msg_info_stop.value = true;
                    }
                    else if (conduccion->parada_emergencia_local== 0) {
                        msg_info_stop.id_event = 2;
                        msg_info_stop.value = false;
                    }               

                    pub_info_stop.publish(msg_info_stop);

                    // Impresion del mensaje info_stop

                    cout << "************ Publicacion del mensaje INFO_STOP *********** \n";
                    cout << "Si es 0 = NADA ; Si es 1 = Parada de emergencia local;  \n";
                    cout << "Parada de emergencia local: " << (int) msg_info_stop.value << "\n";
                    cout << "********************************************************** \n\n\n";

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

                    cout << "************ Publicacion del mensaje ERROR *************************** \n";
                    cout << "2 = FALLO ARRANQUE MOTOR; 3 = FALLO ACELERADOR; 4 = FALLO FRENO ESTAC. \n";
                    cout << "5 = FALLO FRENO SERVICIO; 6 = FALLO CAMBIO MARCHA; 7 = FALLO DIRECCION \n";
                    cout << "8 = FALLO DIFERENCIALES\n";
                    cout << "ERROR: " << msg_err.id_error << "\n";
                    cout << "********************************************************** \n\n\n";

                }

            }
        break;
            
        case OPERATION_MODE_RELEASE:
            
            cout << "FUNCIONAMIENTO EN MODO RELEASE \n\n";
            
        break;
        
        case OPERATION_MODE_SIMULATION:
            
            cout << "FUNCIONAMIENTO EN MODO SIMULATION \n\n";
            
        break;
        
      default:
            break;
    }
            
            
  // Envío de erorr si no hay envio/recepcion de tramas CAN 
  if (!conduccion->CONDUCCION_ACTIVE){
        msg_err.id_error = COMMUNICATION_CAN_FAIL; // Error en conduccion (1: envio/recepcion tramas CAN)
        pub_error.publish(msg_err); 
        
        cout << "COMMUNICATION_CAN_FAIL \n";
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
  ROS_INFO("I heard a NAVIGATION message \n");
}

// Suscriptor al Módulo de Remote
void fcn_sub_com_teleop(const Modulo_Conduccion::msg_com_teleop msg)
{
  ROS_INFO("I heard a TELEOP DEPURADO. message \n");

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
        
  conduccion->valor_parada_emergencia = OFF;

}

// Suscriptor al Módulo de System Management
void fcn_sub_engine_brake(const Modulo_Conduccion::msg_fcn_aux msg)  {
    
    ROS_INFO("I heard a FUNCTION AUX message from SYSTEM MANAGEMENT \n");
    
    if (msg.function == BRAKE) {          
        if (msg.value)
            conduccion->valor_freno_estacionamiento = ON;    
        else
            conduccion->valor_freno_estacionamiento = OFF;     
    }
    else if (msg.function == ENGINE) {                      
        if (msg.value)
            conduccion->valor_arranque_parada = ON;        
        else
            conduccion->valor_arranque_parada = OFF;  
    }       
    
}

// Suscriptor al Módulo de System Management
void fcn_sub_emergency_stop(const Modulo_Conduccion::msg_emergency_stop msg) {
    
    ROS_INFO("I heard a EMERGENCY STOP message from SYSTEM MANAGEMENT \n");
    
    if (msg.value)
        conduccion->valor_parada_emergencia = SET;
    else
        conduccion->valor_parada_emergencia = INFO;
    
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


/*******************************************************************************
 *******************************************************************************
 *                     FUNCIONES TRATAMIENTO DE SEÑALES
 * *****************************************************************************
 * ****************************************************************************/

// what has to be done at program exit
void do_exit(int error)
{
  printf("finished Driving (%d).\n\n", error);
  exit(error);
}

// the signal handler for manual break Ctrl-C
void signal_handler(int signal)
{
  do_exit(0);
}

// what has to be done at program start
void init_signals()
{
  /* install signal handlers */
  signal(SIGTERM, signal_handler);
  signal(SIGINT, signal_handler);
}


bool sendData(){return true;}

bool recvData(){return true;}

bool checkConnection(){return true;}

bool convertROStoCAN(){return true;}

bool convertCANtoROS(){return true;}

