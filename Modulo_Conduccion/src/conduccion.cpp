/* 
 * File:   conduccion.cpp
 * Author: Sergio Doctor López
 *
 * Created on 5 de marzo de 2014, 10:03
 */

#include "../include/Modulo_Conduccion/conduccion.h"

#define temporizador 10.0

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
  pub_error = n.advertise<Common_files::msg_error>("error",1000);
  pub_switch = n.advertise<Common_files::msg_switch>("switch",1000);
  pub_backup = n.advertise<Common_files::msg_backup>("backup",1000);
  pub_info_stop = n.advertise<Common_files::msg_info_stop>("infoStop",1000);
  pub_emergency_stop = n.advertise<Common_files::msg_emergency_stop>("emergInfo",1000);

  // Generación de subscriptores  
  sub_navigation = n.subscribe("pre_navigation", 1000, fcn_sub_navigation);
  sub_com_teleop = n.subscribe("commands_clean",1000,fcn_sub_com_teleop);
  sub_fcn_aux = n.subscribe("engBrake",1000,fcn_sub_engine_brake);
  sub_emergency_stop = n.subscribe("emergSet",1000,fcn_sub_emergency_stop);
  
  // Todo esta correcto, lo especificamos con el correspondiente parametro
  n.setParam("estado_modulo_conduccion",STATE_OK);
  cout << "Atica CONDUCCION :: Configurado y funcionando" << endl;

  // Inicializacion de la comunicacion CAN
  finDePrograma = createCommunication(); // Es true: si la comunicaion se crea correctamente y false: si no se crea bien o da fallos.
  
  inicializa_variables();   
  
  // Envío de erorr si no hay comunicacion CAN 
  if (!finDePrograma) {       
        msg_err.id_error = CONNECTION_CAN_FAIL; // Error en conduccion (0: comunicacion)
        pub_error.publish(msg_err);
        
        cout << "CONNECTION_CAN_FAIL \n";
  }
   
  switch (operationMode) {
        case OPERATION_MODE_DEBUG:
            
            cout << "FUNCIONAMIENTO EN MODO DEBUG \n\n";
            
            while (ros::ok() && finDePrograma && !can->errorWrite) { // && ((can->errorRead) || (can->errorWrite))) {     
                
                n.getParam("estado_modulo_conduccion",estado_actual);
                if(estado_actual==STATE_ERROR || estado_actual== STATE_OFF) {             
                   finDePrograma=disconnectCommunication();   // Fin de programa = false (se cierra el CAN)

                   cout << "Fin de programa --- Se cierra el CAN \n";
                }
                
                else {
                    
                    //checkEmergencyStop();
                    
                    //checkSwitch();
                        
                    //checkInfoStop();
                    
                    checkError();
                    
                    //publishBackup();
                                                          
                }
                
                ros::spinOnce();

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
  if ((can->errorWrite) || (can->errorRead)){
        msg_err.id_error = COMMUNICATION_CAN_FAIL; // Error en conduccion (1: envio/recepcion tramas CAN)
        pub_error.publish(msg_err); 
        conduccion->CONDUCCION_ACTIVE= false;
        
        cout << "COMMUNICATION_CAN_FAIL \n";
  }
      
  cout << "Atica CONDUCCION :: Módulo finalizado" << endl;
  return 0;

}



/*******************************************************************************
 *******************************************************************************
 *                              PUBLICADORES
 * *****************************************************************************
 * ****************************************************************************/

void publishEmergencyStop(){

    cout << "Publicacion de la parada de emergencia = INFORM \n";
    msg_emergency_stop.value = true;
    pub_emergency_stop.publish(msg_emergency_stop);
        
}


void publishBackup() {
    /*
                     cout << "********** Publicacion del mensaje BACKUP *********** \n";
                     cout << "Acelerador: " << conduccion->acelerador  << "\n";
                     cout << "Freno de Servicio: " << conduccion->freno_servicio << "\n";
                     cout << "Direccion: " << (conduccion->sentido_marcha) * (conduccion->direccion) << "\n";
                     cout << "Freno de mano: " << conduccion->freno_estacionamiento << "\n";
                     cout << "Marcha: " << conduccion->marcha_actual << "\n";
                     cout << "Arranque/Parada: " << conduccion->arranque_parada << "\n";
                     cout << "Velocidad: " << conduccion->velocidad << "\n";
                     cout << "***************************************************** \n\n\n";
     */
    // Publicacion del mensaje de Backup
    msg_backup.throttle = (short) conduccion->acelerador; // acelerador
    msg_backup.brake = (short) conduccion->freno_servicio; // freno de servicio
    msg_backup.steer = (short) (conduccion->sentido_marcha) * (conduccion->direccion); // direccion
    msg_backup.handbrake = (short) conduccion->freno_estacionamiento; // freno de mano
    msg_backup.gear = (short) conduccion->marcha_actual; // marcha
    msg_backup.engine = (short) conduccion->arranque_parada; // arranque/parada
    msg_backup.speed = (short) conduccion->velocidad; // velocidad
    pub_backup.publish(msg_backup);

    // Impresion del mensaje de Backup

    cout << "********** Publicacion del mensaje BACKUP *********** \n";
    cout << "Acelerador: " << (int) msg_backup.throttle << "\n";
    cout << "Freno de Servicio: " << (int) msg_backup.brake << "\n";
    cout << "Direccion: " << (int) msg_backup.steer << "\n";
    cout << "Freno de mano: " << (int) msg_backup.handbrake << "\n";
    cout << "Marcha: " << (int) msg_backup.gear << "\n";
    cout << "Arranque/Parada: " << (int) msg_backup.engine << "\n";
    cout << "Velocidad: " << (int) msg_backup.speed << "\n";
    cout << "***************************************************** \n\n\n";
  
}


void publishSwitch(){

    if (conduccion->conmutador_m_a == OFF)
        msg_switch.value = false; // Manual
    else if (conduccion->conmutador_m_a == ON)
        msg_switch.value = true; // Teleoperado

    pub_switch.publish(msg_switch);

    cout << "********** Publicacion del mensaje SWITCH *********** \n";
    cout << "Si es 0 = Manual; Si es 1 = Teleoperado \n";
    cout << "Comuntador Manual/Automático: " << (int) msg_switch.value << "\n";
    cout << "***************************************************** \n\n\n";
    
}
/* 
void publishInfoStopOsbtacule(short valor) {

    if (valor == ON) {
        msg_info_stop.id_event = 0;
        msg_info_stop.value = true;
    } else if (valor == OFF) {
        msg_info_stop.id_event = 0;
        msg_info_stop.value = false;
    }
    pub_info_stop.publish(msg_info_stop);

    cout << "****** Publicacion del mensaje INFO_STOP OBSTACULE  ****** \n";
    cout << "Si es 0 = NADA ; Si es 1 = Parada de emergencia obstaculo;  \n";
    cout << "Parada de emergencia obstaculo: " << (int) msg_info_stop.value << "\n";
    cout << "********************************************************** \n\n\n";
}


void publishInfoStopLocal (short valor) {

    if (valor == ON) {
        msg_info_stop.id_event = 2;
        msg_info_stop.value = true;
    } else if (valor == OFF) {
        msg_info_stop.id_event = 2;
        msg_info_stop.value = false;
    }

    pub_info_stop.publish(msg_info_stop);

    cout << "******* Publicacion del mensaje INFO_STOP LOCAL ********* \n";
    cout << "Si es 0 = NADA ; Si es 1 = Parada de emergencia local;  \n";
    cout << "Parada de emergencia local: " << (int) msg_info_stop.value << "\n";
    cout << "********************************************************** \n\n\n";
}


void publishInfoStopRemote (short valor) {

    if (valor == ON) {
        msg_info_stop.id_event = 1;
        msg_info_stop.value = true;

        // MANDAR ACK A SYSTEM MANAGEMENT

        msg_emergency_stop.value = INFO;
        pub_emergency_stop.publish(msg_emergency_stop);

        cout << "******** Publicacion del mensaje EMERGENCY STOP  ************ \n";
        cout << "Si es 1 = INFO;  \n";
        cout << "Emergency Stop: " << msg_emergency_stop.value << "\n";
        cout << "************************************************************* \n\n\n";

    } else if (valor == OFF) {
        msg_info_stop.id_event = 1;
        msg_info_stop.value = false;
    }
    pub_info_stop.publish(msg_info_stop);

    cout << "******* Publicacion del mensaje INFO_STOP REMOTE ********* \n";
    cout << "Si es 0 = NADA ; Si es 1 = Parada de emergencia remota;  \n";
    cout << "Parada de emergencia remota: " << (int) msg_info_stop.value << "\n";
    cout << "********************************************************** \n\n\n";
}
*/

void publishInfoStop (short valor, int i) {
    
    switch (i) {
        case 0:           // Obstaculo

            if (conduccion->conmutador_m_a == OFF)
                msg_switch.value = false; // Manual
            else if (conduccion->conmutador_m_a == ON)
                msg_switch.value = true; // Teleoperado

            pub_switch.publish(msg_switch);

            cout << "********** Publicacion del mensaje SWITCH *********** \n";
            cout << "Si es 0 = Manual; Si es 1 = Teleoperado \n";
            cout << "Comuntador Manual/Automático: " << (int) msg_switch.value << "\n";
            cout << "***************************************************** \n\n\n";
            
            break;
            
        case 1:         // Local

            if (valor == ON) {
                msg_info_stop.id_event = 2;
                msg_info_stop.value = true;
            } else if (valor == OFF) {
                msg_info_stop.id_event = 2;
                msg_info_stop.value = false;
            }

            pub_info_stop.publish(msg_info_stop);

            cout << "******* Publicacion del mensaje INFO_STOP LOCAL ********* \n";
            cout << "Si es 0 = NADA ; Si es 1 = Parada de emergencia local;  \n";
            cout << "Parada de emergencia local: " << (int) msg_info_stop.value << "\n";
            cout << "********************************************************** \n\n\n";
    
            break;
            
        case 2:         // Remote

            if (valor == ON) {
                msg_info_stop.id_event = 1;
                msg_info_stop.value = true;

                // MANDAR ACK A SYSTEM MANAGEMENT

                msg_emergency_stop.value = INFO;
                pub_emergency_stop.publish(msg_emergency_stop);

                cout << "******** Publicacion del mensaje EMERGENCY STOP  ************ \n";
                cout << "Si es 1 = INFO;  \n";
                cout << "Emergency Stop: " << msg_emergency_stop.value << "\n";
                cout << "************************************************************* \n\n\n";

            } else if (valor == OFF) {
                msg_info_stop.id_event = 1;
                msg_info_stop.value = false;
            }
            pub_info_stop.publish(msg_info_stop);

            cout << "******* Publicacion del mensaje INFO_STOP REMOTE ********* \n";
            cout << "Si es 0 = NADA ; Si es 1 = Parada de emergencia remota;  \n";
            cout << "Parada de emergencia remota: " << (int) msg_info_stop.value << "\n";
            cout << "********************************************************** \n\n\n";
            
            break;
            
        default:
            break;
    }
        
}


void publishError (short valor, int i) {
    // Publicacion del mensaje error
    if (valor == ON)
        msg_err.type_error = TOE_UNDEFINED;
    else if (valor == OFF)
        msg_err.type_error = TOE_END_ERROR;
                
    switch (i) {
        case START_STOP_FAILURE: // Fallo arranque/Parada
            msg_err.id_error = START_STOP_FAILURE;            
           break;
        case THROTTLE_FAILURE: // Fallo acelerador
            msg_err.id_error = THROTTLE_FAILURE;
            break;
        case HANDBRAKE_FAILURE: // Fallo freno de estacionamiento
            msg_err.id_error = HANDBRAKE_FAILURE;
            break;
        case BRAKE_FAILURE: // Fallo freno de servicio
            msg_err.id_error = BRAKE_FAILURE;
            break;
        case GEAR_SHIFT_FAILURE: // Fallo cambio de marcha
            msg_err.id_error = GEAR_SHIFT_FAILURE;
            break;
        case STEER_FAILURE: // Fallo de direccion
            msg_err.id_error = STEER_FAILURE;
            break;
        case DIFFERENTIAL_LOCK_FAILURE: // Fallo bloqueo de diferenciales
            msg_err.id_error = DIFFERENTIAL_LOCK_FAILURE;
            break;
        default:
            break;
    }
    pub_error.publish(msg_err);

    cout << "************ Publicacion del mensaje ERROR *************************** \n";
    cout << "2 = FALLO ARRANQUE MOTOR; 3 = FALLO ACELERADOR; 4 = FALLO FRENO ESTAC. \n";
    cout << "5 = FALLO FRENO SERVICIO; 6 = FALLO CAMBIO MARCHA; 7 = FALLO DIRECCION \n";
    cout << "8 = FALLO DIFERENCIALES\n";
    cout << "---------------------------------------------------------------------- \n";
    cout << "TIPO DE ERROR => 0 = TOE_UNDEFINED; 3 = TOE_END_ERROR \n";
    cout << "---------------------------------------------------------------------- \n";
    cout << "ERROR: " << (int) msg_err.id_error << "\n";
    cout << "TIPO ERROR: " << (int) msg_err.type_error << "\n";
    cout << "********************************************************************** \n\n\n";
    
}




/*******************************************************************************
 *******************************************************************************
 *                              SUSCRIPTORES
 * *****************************************************************************
 * ****************************************************************************/

// Suscriptor al Módulo de Navegacion
void fcn_sub_navigation(const Common_files::msg_navigation msg)
{
  ROS_INFO("I heard a NAVIGATION message \n");
}

// Suscriptor al Módulo de Remote
void fcn_sub_com_teleop(const Common_files::msg_com_teleop msg)
{
       // ROS_INFO("I heard a TELEOP DEPURADO. message \n");
        
    if (!parada_emergencia) {

        switch (msg.id_element) {
              case ID_REMOTE_THROTTLE:   // Acelerador
                  //cout << "acelerador = " << msg.value << "\n";
                  conduccion->acelerador_tx = (short) msg.value;
                  break;              
              case ID_REMOTE_BRAKE:   // Freno de servicio
                  //cout << "Freno de servicio = " << msg.value << "\n";
                  conduccion->freno_servicio_tx = (short) msg.value;
                  break;
              case ID_REMOTE_STEER:   // Direccion                  
                  //cout << "Direccion = " << msg.value << "\n";
                  conduccion->valor_direccion = (short) msg.value;
                  break;
              case ID_REMOTE_GEAR:   // Marcha
                  //cout << "Marcha = " << msg.value << "\n";
                  conduccion->valor_marcha = (short) msg.value;
                  break;
              case ID_REMOTE_HANDBRAKE:   // Freno de mano
                  //cout << "Freno de mano = " << msg.value << "\n";
                  conduccion->valor_freno_estacionamiento = (short) msg.value;
                  break;
              case ID_REMOTE_ENGINE:   // Encendido del motor
                  //cout << "Encendido del motor = " << msg.value << "\n";
                  conduccion->valor_arranque_parada = (short) msg.value;
                  break;
              case ID_REMOTE_LIGHT_IR:   // Luces IR
                  //cout << "Luces IR = " << msg.value << "\n";
                  conduccion->valor_luces_IR = (short) msg.value;
                  break;
              case ID_REMOTE_LIGHT_STANDARD:   // Luces 
                  //cout << "Luces = " << msg.value << "\n";
                  conduccion->valor_luces = (short) msg.value;
                  break;
              case ID_REMOTE_DIFF:   // Diferenciales
                  //cout << "Diferenciales = " << msg.value << "\n";
                  conduccion->valor_diferencial = (short) msg.value;
                  break;
              case ID_REMOTE_ACT_LASER2D:   // Activacion Laser
                  //cout << "Activacion del laser = " << msg.value << "\n";
                  conduccion->valor_laser = (short) msg.value;
                  break;
              default:          
                  break;          
          }                 
        conduccion->m_teleop_CAN_AUTOMATA(); 
        
    }
        
}

// Suscriptor al Módulo de System Management
void fcn_sub_engine_brake(const Common_files::msg_fcn_aux msg)  {
    
    ROS_INFO("I heard a FUNCTION AUX message from SYSTEM MANAGEMENT \n");
    
    if (msg.function == BRAKE) {          
        if (msg.value){
            cout << "Freno de estacionamiento = ON \n";
            conduccion->valor_freno_estacionamiento = ON;    
        }else{
            cout << "Freno de estacionamiento = OFF \n";
            conduccion->valor_freno_estacionamiento = OFF;     
        }
    }
    else if (msg.function == ENGINE) {                      
        if (msg.value){
            cout << "Arranque / Parada = ON \n";
            conduccion->valor_arranque_parada = ON;        
        }else {
            cout << "Arranque / Parada = OFF \n";
            conduccion->valor_arranque_parada = OFF;  
        }
    }       
    conduccion->m_engine_brake_CAN_AUTOMATA();
}

// Suscriptor al Módulo de System Management
void fcn_sub_emergency_stop(const Common_files::msg_emergency_stop msg) {

    ROS_INFO("I heard a EMERGENCY STOP message from SYSTEM MANAGEMENT \n");

    parada_emergencia = true;
    cout << "Parada de emergencia = SET \n";
    conduccion->valor_parada_emergencia = ON;
    conduccion->m_emergency_stop_CAN_AUTOMATA();
    conduccion->t.Enable();

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


void inicializa_variables() {
  msg_err.id_subsystem = SUBS_DRIVING;  // Flag que indica a errores que estamos en el subsistema Driving
  parada_emergencia = false;            // Al principio siempre es falsa la parada de emergencia. Se pondra a true cuando verdaderamente haya una parada.
  valor_conmutador = 9;                 // Al principio se le asigna un valor cualquiera para que en la primera itereacion cambio de valor y lo publique
  valor_parada_obstaculo = 0;           // Al principio se le asigna el valor 0 que indica que no hay ninguna parada de emergencia de obstaculo. 
  valor_parada_local = 0;               // Al principio se le asigna el valor 0 que indica que no hay ninguna parada de emergencia local. 
  valor_parada_remote = 0;              // Al principio se le asigna el valor 0 que indica que no hay ninguna parada de emergencia remota.
  error_a_p = 0;                        // Al principio se le asigna el valor 0 que indica que no hay ningun error en el arranque y la parada
  error_acelerador = 0;                 // Al principio se le asigna el valor 0 que indica que no hay ningun error en el acelerador
  error_freno_estacionamiento = 0;      // Al principio se le asigna el valor 0 que indica que no hay ningun error en el freno de estacionamiento
  error_freno_servicio = 0;             // Al principio se le asigna el valor 0 que indica que no hay ningun error en el freno de servicio
  error_cambio_marcha = 0;              // Al principio se le asigna el valor 0 que indica que no hay ningun error en el cambio de amrcha
  error_direccion = 0;                  // Al principio se le asigna el valor 0 que indica que no hay ningun error en la direccion
  error_diferenciales = 0;              // Al principio se le asigna el valor 0 que indica que no hay ningun error en los diferenciales
}


void checkEmergencyStop () {

    if ((conduccion->conf_parada_emergencia == 1) && (conduccion->paradaEmergencia)) {
        publishEmergencyStop();
        conduccion->paradaEmergencia = false;
    }

    if (parada_emergencia) {
        //cout << "inicio del timer a 10 segundos" << endl;
        if (conduccion->t.GetTimed() > temporizador) {
            parada_emergencia = false;
            conduccion->t.Disable();
            //cout << "Fin del timer a 10 segundos" << endl;

        }
    }
    
}


void checkSwitch() {

    if (valor_conmutador != conduccion->conmutador_m_a) {
        valor_conmutador = conduccion->conmutador_m_a;
        publishSwitch();
    }
    
}


void checkInfoStop() {
    
    if (valor_parada_obstaculo != conduccion->parada_emergencia_obstaculo) {
        valor_parada_obstaculo = conduccion->parada_emergencia_obstaculo;
        //publishInfoStopObstacule(conduccion->parada_emergencia_obstaculo);
        publishInfoStop(conduccion->parada_emergencia_obstaculo, 0);
    }

    if (valor_parada_local != conduccion->parada_emergencia_local) {
        valor_parada_local = conduccion->parada_emergencia_local;
        //publishInfoStopLocal(conduccion->parada_emergencia_obstaculo);
        publishInfoStop(conduccion->parada_emergencia_local, 1);
    }

    if (valor_parada_remote != conduccion->parada_emergencia_remota) {
        valor_parada_remote = conduccion->parada_emergencia_remota;
        //publishInfoStopRemote(conduccion->parada_emergencia_obstaculo);
        publishInfoStop(conduccion->parada_emergencia_remota, 2);
    }
    
}


void checkError() {
    
    if ((error_a_p != conduccion->error_arranque_parada) && (conduccion->error_arranque_parada >= 0) && (conduccion->error_arranque_parada <= 1))  {
        cout << "Valor 1: " << conduccion->error_arranque_parada << endl; 
        error_a_p = conduccion->error_arranque_parada;
        publishError(conduccion->error_arranque_parada, START_STOP_FAILURE);
    }
    
    if ((error_acelerador != conduccion->error_acelerador) && (conduccion->error_acelerador >= 0) && (conduccion->error_acelerador <= 1))  {
        cout << "Valor 2: " << conduccion->error_acelerador << endl;
        error_acelerador = conduccion->error_acelerador;
        publishError(conduccion->error_acelerador, THROTTLE_FAILURE);
    }
    
    if ((error_freno_estacionamiento != conduccion->error_freno_estacionamiento) && (conduccion->error_freno_estacionamiento >= 0) && (conduccion->error_freno_estacionamiento <= 1)) {
        cout << "Valor 3: " << conduccion->error_freno_estacionamiento << endl;
        error_freno_estacionamiento = conduccion->error_freno_estacionamiento;
        publishError(conduccion->error_freno_estacionamiento, HANDBRAKE_FAILURE);
    }
            
    if ((error_freno_servicio != conduccion->error_freno_servicio) && (conduccion->error_freno_servicio >= 0) && (conduccion->error_freno_servicio <= 1)) {
        cout << "Valor 4: " << conduccion->error_freno_servicio << endl;
        error_freno_servicio = conduccion->error_freno_servicio;
        publishError(conduccion->error_freno_servicio, BRAKE_FAILURE);
    }
    
    if ((error_cambio_marcha != conduccion->error_cambio_marcha) && (conduccion->error_cambio_marcha >= 0) && (conduccion->error_cambio_marcha <= 1)) {
        cout << "Valor 5: " << conduccion->error_cambio_marcha << endl;
        error_cambio_marcha = conduccion->error_cambio_marcha;
        publishError(conduccion->error_cambio_marcha, GEAR_SHIFT_FAILURE);
    }

    if ((error_direccion != conduccion->error_direccion) && (conduccion->error_direccion >= 0) && (conduccion->error_direccion <= 1)) {
        cout << "Valor 6: " << conduccion->error_direccion << endl;
        error_direccion = conduccion->error_direccion;
        publishError(conduccion->error_direccion, STEER_FAILURE);
    }

    if ((error_diferenciales != conduccion->error_bloqueo_diferenciales) && (conduccion->error_bloqueo_diferenciales >= 0) && (conduccion->error_bloqueo_diferenciales <= 1)) {
        cout << "Valor 7: " << conduccion->error_bloqueo_diferenciales << endl;
        error_diferenciales = conduccion->error_bloqueo_diferenciales;
        publishError(conduccion->error_bloqueo_diferenciales, DIFFERENTIAL_LOCK_FAILURE);
    }
    
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

