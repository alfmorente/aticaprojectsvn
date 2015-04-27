/** 
 * @file  ConduccionThread.hpp
 * @brief Implementación del main principal del Subsistema Driving
 * @author Sergio Doctor 
 * @date 2014
 * @addtogroup 
 * @{
 */

#include "../include/Modulo_Conduccion/conduccion.h"


// Tratamiento de señales (CTRL+C)
#include <signal.h>

using namespace std;


/*******************************************************************************
 *******************************************************************************
 *                     FUNCIONES TRATAMIENTO DE SEÑALES
 * *****************************************************************************
 * ****************************************************************************/

/**
 * Método que gestiona la entrada se señales por teclado
 * @param signal Tipo de señal
 */

// the signal handler for manual break Ctrl-C
void signal_handler(int signal)
{
  printf("finished Driving (%d).\n\n", signal);
  disconnectCommunication();
  exit(signal);
}

/*******************************************************************************
 *******************************************************************************
 *                                           MAIN
 * *****************************************************************************
 * ****************************************************************************/

/**
 * Método principal del nodo. 
 * @param[in] argc Número de argumentos
 * @param[in] argv Vector de argumentos
 * @return Entero distinto de 0 si ha habido problemas. 0 en caso contrario. 
 */

int main(int argc, char **argv)
{
  
    
  // Orden para la parada manual con CTtrl+C
  signal(SIGTERM, signal_handler);
  signal(SIGINT, signal_handler);  
    
  //Obtencion del modo de operacion y comprobacion de que es correcto
   
  if ((operationMode = getOperationMode(argc, argv)) == 0) {
      return 1;
  }
  
  // Inicio de ROS
  ros::init(argc, argv, "conduccion");
 
  ros::NodeHandle n;        // Manejador ROS
  
  //int estado_actual = STATE_OK;    // Para hacer pruebas locales
  
 
  //Espera activa de inicio de modulo
  int estado_actual=STATE_OFF; 
  while(estado_actual!=STATE_CONF){
          n.getParam("state_module_driving",estado_actual);
          usleep(50000);
  }
 
  cout << "Atica CONDUCCION :: Iniciando configuración..." << endl;

  initialize(n); 
  usleep(100000);
  
  cout << "Atica CONDUCCION :: Configurado y funcionando" << endl;

  // Inicializacion de la comunicacion CAN
  finDePrograma = createCommunication(); // Es true: si la comunicaion se crea correctamente y false: si no se crea bien o da fallos. 
  
  
  //Espera activa de inicio de operacion de cada modulo
  //int estado_sistema = STATE_SYSTEM_ON;
  
  int estado_sistema = STATE_SYSTEM_OFF;
  while(estado_sistema != STATE_SYSTEM_ON){
       n.getParam("state_system",estado_sistema);
       usleep(50000); 
       //can->inicio_read_write_CAN_frame = false; // Para que no empiece a contar tramas CAN //VALIDO PARA COMUNIACION CAN
                                            // antes de que se inicie el modulo
  }
  
  //can->inicio_read_write_CAN_frame = true;  //VALIDO PARA COMUNIACION CAN
    // Envío de erorr si no hay comunicacion CAN 
  if (!finDePrograma) {       
        msg_err->id_error = CONNECTION_CAN_FAIL; // Error en conduccion (0: comunicacion)
        pub_error.publish(msg_err);
        
        cout << "CONNECTION_CAN_FAIL \n";
        cout << "Atica CONDUCCION :: Módulo finalizado" << endl;
        return 0;
  }
  usleep(100000);
  
  conduccion->envio_trama_reinicio_CAN_AUTOMATA();  //VALIDO PARA COMUNIACION CAN
  
  ros::Rate loop_rate(40); //Equivale a 50 milisegundos
  
  switch (operationMode) {
        case OPERATION_MODE_DEBUG:
                       
            //cout << "FUNCIONAMIENTO EN MODO DEBUG \n\n";
            while (ros::ok() && finDePrograma && !(can->errorWrite) && !(can->errorRead)) {    // PARA COMUNIACCION CAN
            
                
                n.getParam("state_module_driving",estado_actual);
                if(estado_actual==STATE_ERROR || estado_actual== STATE_OFF) {             
                   finDePrograma=disconnectCommunication();   // Fin de programa = false (se cierra el CAN)

                   cout << "Fin de programa --- Se cierra el CAN \n";
                }             
                else {                      
                        checkEmergencyStop();
                    
                        checkSwitch();
                      
                        checkInfoStop();
                    
                        checkError();
                    
                        if (conduccion->flag_active_backup) {
                                publishBackup();
                                conduccion->flag_active_backup = false;
                        }        	                                 
                      
                     
                }
                
                ros::spinOnce();
                //usleep(25000);
                loop_rate.sleep();

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
        msg_err->id_error = COMMUNICATION_CAN_FAIL; // Error en conduccion (1: envio/recepcion tramas CAN)
        pub_error.publish(msg_err);
        conduccion->CONDUCCION_ACTIVE= false;
        cout << "COMMUNICATION_CAN_FAIL \n";
  }
  
  disconnectCommunication();
  usleep(100000);
  cout << "Atica CONDUCCION :: Módulo finalizado" << endl;
  return 0;

}



/*******************************************************************************
 *******************************************************************************
 *                              PUBLICADORES
 * *****************************************************************************
 * ****************************************************************************/


/**
 * Publicación de la parada de emergencia
 */

void publishEmergencyStop(){

    cout << "Publicacion de la parada de emergencia = INFORM \n";
    msg_emergency_stop->value = true;
    pub_emergency_stop.publish(msg_emergency_stop);
        
}

/**
 * Publicación del backup, el cual dispone de información procedente del vehículo
 */

void publishBackup() {
    // Publicacion del mensaje de Backup
    msg_backup->throttle = (short) conduccion->acelerador; // acelerador
    msg_backup->brake = (short) conduccion->freno_servicio; // freno de servicio
    msg_backup->steer = (short) (conduccion->sentido_marcha) * (conduccion->direccion); // direccion
    msg_backup->handbrake = (short) conduccion->freno_estacionamiento; // freno de mano
    msg_backup->gear = (short) conduccion->marcha_actual; // marcha
    msg_backup->engine = (short) conduccion->arranque_parada; // arranque/parada
    msg_backup->speed = (short) conduccion->velocidad; // velocidad
    pub_backup.publish(msg_backup);

    // Impresion del mensaje de Backup
/*
    cout << "********** Publicacion del mensaje BACKUP *********** \n";
    cout << "Acelerador: " << (int) msg_backup->throttle << "\n";
    cout << "Freno de Servicio: " << (int) msg_backup->brake << "\n";
    cout << "Direccion: " << (short) conduccion->direccion << "\n"; // direccion 
    cout << "Direccion con sentido: " << (int) msg_backup->steer << "\n";
    cout << "Freno de mano: " << (int) msg_backup->handbrake << "\n";
    cout << "Marcha: " << (int) msg_backup->gear << "\n";
    cout << "Arranque/Parada: " << (int) msg_backup->engine << "\n";
    cout << "Velocidad: " << (int) msg_backup->speed << "\n";
    cout << "Conf parada emergencia: " << (int) conduccion ->conf_parada_emergencia << "\n";
    cout << "***************************************************** \n\n\n";
  */
}

/**
 * Publicación del modo de conducción (automática o manual)
 */


void publishSwitch(){

    if (conduccion->conmutador_m_a == OFF)
        msg_switch->value = false; // Manual
    else if (conduccion->conmutador_m_a == ON)
        msg_switch->value = true; // Teleoperado

    pub_switch.publish(msg_switch);

    cout << "********** Publicacion del mensaje SWITCH *********** \n";
    cout << "Si es 0 = Manual; Si es 1 = Teleoperado \n";
    cout << "Comuntador Manual/Automático: " << (int) msg_switch->value << "\n";
    cout << "***************************************************** \n\n\n";
    
}

/**
 * Publicación del tipo de parada de emergencia
 * @param valor Si se ha activado o desactivado cualquier parada de emergencia 
 * @param i Tipo de parada de emergencia (obstáculo, seta remota o seta local)
 */


void publishInfoStop (short valor, int i) {
    
    switch (i) {
        case 0:    // Obstaculo

            if (valor == ON) {
                msg_info_stop->id_event = 0;
                msg_info_stop->value = true;

                /*
                // MANDAR ACK A SYSTEM MANAGEMENT

                msg_emergency_stop.value = INFO;
                pub_emergency_stop.publish(msg_emergency_stop);

                cout << "******** Publicacion del mensaje EMERGENCY STOP  ************ \n";
                cout << "Si es 1 = INFO;  \n";
                cout << "Emergency Stop: " << (int) msg_emergency_stop.value << "\n";
                cout << "************************************************************* \n\n\n";
*/
            } else if (valor == OFF) {
                msg_info_stop->id_event = 0;
                msg_info_stop->value = false;
            }
            pub_info_stop.publish(msg_info_stop);

            cout << "******* Publicacion del mensaje INFO_STOP ON ********* \n";
            cout << "Si es 0 = NADA ; Si es 1 = Parada de emergencia obstaculo;  \n";
            cout << "Parada de emergencia obstaculo: " << (int) msg_info_stop->value << "\n";
            cout << "********************************************************** \n\n\n";
            
            break;
                                   
        case 1:         // Local

            if (valor == ON) {
                msg_info_stop->id_event = 2;
                msg_info_stop->value = true;
            } else if (valor == OFF) {
                msg_info_stop->id_event = 2;
                msg_info_stop->value = false;
            }

            pub_info_stop.publish(msg_info_stop);

            cout << "******* Publicacion del mensaje INFO_STOP LOCAL ********* \n";
            cout << "Si es 0 = NADA ; Si es 1 = Parada de emergencia local;  \n";
            cout << "Parada de emergencia local: " << (int) msg_info_stop->value << "\n";
            cout << "********************************************************** \n\n\n";
    
            break;
            
        case 2:         // Remote

            if (valor == ON) {
                msg_info_stop->id_event = 1;
                msg_info_stop->value = true;
            } else if (valor == OFF) {
                msg_info_stop->id_event = 1;
                msg_info_stop->value = false;
            }

            pub_info_stop.publish(msg_info_stop);

            cout << "******* Publicacion del mensaje INFO_STOP LOCAL ********* \n";
            cout << "Si es 0 = NADA ; Si es 1 = Parada de emergencia remota;  \n";
            cout << "Parada de emergencia local: " << (int) msg_info_stop->value << "\n";
            cout << "********************************************************** \n\n\n";
            
            break;
            
        default:
            break;
    }
        
}

/**
 * Publicación del mensaje de error
 * @param valor Clasificación del error 
 * @param i Tipo de error (fallo en el acelerador, fallo en la dirección,...)
 */


void publishError (short valor, int i) {
    // Publicacion del mensaje error
    if (valor == ON)
        msg_err->type_error = TOE_UNDEFINED;
    else if (valor == OFF)
        msg_err->type_error = TOE_END_ERROR;
                
    switch (i) {
        case START_STOP_FAILURE: // Fallo arranque/Parada
            msg_err->id_error = START_STOP_FAILURE;            
           break;
        case THROTTLE_FAILURE: // Fallo acelerador
            msg_err->id_error = THROTTLE_FAILURE;
            break;
        case HANDBRAKE_FAILURE: // Fallo freno de estacionamiento
            msg_err->id_error = HANDBRAKE_FAILURE;
            break;
        case BRAKE_FAILURE: // Fallo freno de servicio
            msg_err->id_error = BRAKE_FAILURE;
            break;
        case GEAR_SHIFT_FAILURE: // Fallo cambio de marcha
            msg_err->id_error = GEAR_SHIFT_FAILURE;
            break;
        case STEER_FAILURE: // Fallo de direccion
            msg_err->id_error = STEER_FAILURE;
            break;
        case DIFFERENTIAL_LOCK_FAILURE: // Fallo bloqueo de diferenciales
            msg_err->id_error = DIFFERENTIAL_LOCK_FAILURE;
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
    cout << "ERROR: " << (int) msg_err->id_error << "\n";
    cout << "TIPO ERROR: " << (int) msg_err->type_error << "\n";
    cout << "********************************************************************** \n\n\n";
    
}




/*******************************************************************************
 *******************************************************************************
 *                              SUSCRIPTORES
 * *****************************************************************************
 * ****************************************************************************/

/**
 * Suscriptor de los comandos de navegación
 * Envío de mensajes de navegación al vehículo
 * @param msg Mensaje proveniente del publicador
 */

// Suscriptor al Módulo de Navegacion
void fcn_sub_navigation(const Common_files::msg_navigationPtr& msg)
{
  ROS_INFO("I heard a NAVIGATION message \n");
}


/**
 * Suscriptor de los comandos de teleoperación
 * Envío de mensajes (acelerador, freno, marcha,...) del modo remote al vehículo
 * @param msg Mensaje proveniente del publicador
 */

// Suscriptor al Módulo de Remote
void fcn_sub_com_teleop(const Common_files::msg_com_teleopPtr& msg)
{
       // ROS_INFO("I heard a TELEOP DEPURADO. message \n");
        
    if (!parada_emergencia) {

        switch (msg->id_element) {
              case ID_REMOTE_THROTTLE:   // Acelerador
                  //cout << "acelerador = " << msg.value << "\n";
                  conduccion->acelerador_tx = (short) msg->value;
                  break;              
              case ID_REMOTE_BRAKE:   // Freno de servicio
                  //cout << "Freno de servicio = " << msg.value << "\n";
                  conduccion->freno_servicio_tx = (short) msg->value;
                  break;
              case ID_REMOTE_STEER:   // Direccion                  
                  //cout << "Direccion = " << msg.value << "\n";
                  conduccion->valor_direccion = (short) msg->value;
                  break;
              case ID_REMOTE_GEAR:   // Marcha
                  //cout << "Marcha = " << msg.value << "\n";
                  conduccion->valor_marcha = (short) msg->value;
                  break;
              case ID_REMOTE_HANDBRAKE:   // Freno de mano
                  //cout << "Freno de mano = " << msg.value << "\n";
                  conduccion->valor_freno_estacionamiento = (short) msg->value;
                  break;
              case ID_REMOTE_ENGINE:   // Encendido del motor
                  //cout << "Encendido del motor = " << msg.value << "\n";
                  conduccion->valor_arranque_parada = (short) msg->value;
                  break;
              case ID_REMOTE_LIGHT_IR:   // Luces IR
                  //cout << "Luces IR = " << msg.value << "\n";
                  conduccion->valor_luces_IR = (short) msg->value;
                  break;
              case ID_REMOTE_LIGHT_CONVENTIONAL:   // Luces 
                  //cout << "Luces = " << msg.value << "\n";
                  conduccion->valor_luces = (short) msg->value;
                  break;
              case ID_REMOTE_DIFF:   // Diferenciales
                  //cout << "Diferenciales = " << msg.value << "\n";
                  conduccion->valor_diferencial = (short) msg->value;
                  break;
              case ID_REMOTE_ACT_LASER2D:   // Activacion Laser
                  //cout << "Activacion del laser = " << msg.value << "\n";
                  conduccion->valor_laser = (short) msg->value;
                  break;
              default:          
                  break;          
          }                 
        conduccion->m_teleop_CAN_AUTOMATA(); 
        
    }
        
}

/**
 * Suscriptor del engine/brake
 * Cuando le llega un engine/brake del módulo de gestión de sistema, se le envía un mensaje al vehículo para que se active/desactive el motor o para 
 * que ponga/quite el freno de estacionamiento
 * @param msg Mensaje proveniente del publicador
 */

// Suscriptor al Módulo de System Management
void fcn_sub_engine_brake(const Common_files::msg_fcn_auxPtr& msg)  {
    
    //ROS_INFO("I heard a FUNCTION AUX message from SYSTEM MANAGEMENT \n");
    
    if (msg->function == BRAKE) {          
        if (msg->value){
            cout << "Freno de estacionamiento = ON \n";
            conduccion->valor_freno_estacionamiento = ON;    
        }else{
            cout << "Freno de estacionamiento = OFF \n";
            conduccion->valor_freno_estacionamiento = OFF;     
        }
    }
    else if (msg->function == ENGINE) {                      
        if (msg->value){
            cout << "Arranque / Parada = ON \n";
            conduccion->valor_arranque_parada = ON;        
        }else {
            cout << "Arranque / Parada = OFF \n";
            conduccion->valor_arranque_parada = OFF;  
        }
    }       
    conduccion->m_engine_brake_CAN_AUTOMATA();
}


/**
 * Suscriptor de la parada de emergencia
 * Cuando le llega una parada de emergencia del módulo de gestión de sistema, se le envía un mensaje al vehículo para que se detenga
 * @param msg Mensaje proveniente del publicador
 */

// Suscriptor al Módulo de System Management
void fcn_sub_emergency_stop(const Common_files::msg_emergency_stopPtr& msg) {

    //ROS_INFO("I heard a EMERGENCY STOP message from SYSTEM MANAGEMENT \n");

    parada_emergencia = true;
    //cout << "Parada de emergencia = SET \n";
    conduccion->valor_parada_emergencia = ON;
    conduccion->m_emergency_stop_CAN_AUTOMATA();
    conduccion->t.Enable();

}


/**
 * Servicio de heartbeat para Gestion de sistema
 * @param req 
 * @param resp
 * @return 
 */

// Servicio de heartbeat con Gestion del sistema
bool fcn_heartbeat(Common_files::srv_data::Request &req, Common_files::srv_data::Response &resp)
{
    if(req.param==PARAM_ALIVE)
    {  
        resp.value=0;
        return true;
    }
    else 
        return false;
}





/*******************************************************************************
 *******************************************************************************
 *                              FUNCIONES PROPIAS
 * *****************************************************************************
 * ****************************************************************************/

/**
 * Método que crea las comunicaciones CAN
 * @return Devuelve verdadero o falso si se ha creado correctamente la conexión
 */

bool createCommunication(){
    
        
    /* CREAR COMUNICACION POR CAN*/
    
    bool res = false;

    while(CANflag < 3 && CANflag != 0);
    
    // Establecimiento y configuración de las comunicaciones

        can = new CANCommunication(false, false, HW_PCI, 0, 0, 0x031C, false, "Conduccion"); // Asegurarnos del BaudRate

        do {
            if (can->EstablishCommunication() == true) {
                can->ConfigureCommunication();
                CANflag = 0;
            } else {
                CANflag++;
                if (CANflag == 3) { // Para más de 3 errores se cierra todo
                    can->~CANCommunication();
                    return res = false;
                }
            }
        } while (CANflag < 3 && CANflag != 0);

        conduccion = new ConduccionThread(can);
    
        can->Run();
        conduccion->Run();

    
    return res = true;
    
}

/**
 * Método que desconecta las comunicaciones CAN
 * @return Devuelve verdadero o falso si se ha desconectado correctamente la conexión
 */

bool disconnectCommunication(){
    
    /*FIN COMUNICACION CAN */
    can->~CANCommunication();
      
    
    return false;
}

/**
 * Inicialización de variables del sistema
 * @param n Nodo de trabajo de ROS
 */
void initialize(ros::NodeHandle n) {
    
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

 
  server = n.advertiseService("module_alive_3", fcn_heartbeat);
  
  // Todo esta correcto, lo especificamos con el correspondiente parametro
  n.setParam("state_module_driving",STATE_OK);  
    
  msg_err->id_subsystem = SUBS_DRIVING;  // Flag que indica a errores que estamos en el subsistema Driving
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
    
  sleep(1);
}

/**
 * Método que comprueba constantemente si se produce una parada de emergencia
 */

void checkEmergencyStop () {

    if ((conduccion->conf_parada_emergencia == 1) && (conduccion->paradaEmergencia)) {
        publishEmergencyStop();
        conduccion->paradaEmergencia = false;
    }
    
    
    if (parada_emergencia) {
        //cout << "inicio del timer a 10 segundos" << endl;
        if (conduccion->t.GetTimed() > TIMER) {
            parada_emergencia = false;
            cout << "Desactivando parada de emergencia" << endl;
            conduccion->valor_parada_emergencia = OFF;
            conduccion->m_emergency_stop_CAN_AUTOMATA();
            conduccion->paradaEmergencia = false;
            conduccion->t.Disable();
            //cout << "Fin del timer a 10 segundos" << endl;

        }
    }
    
}

/**
 * Método que comprueba constantemente si se ha producido un cambio de manual a automático o viceversa
 */

void checkSwitch() {

    if (valor_conmutador != conduccion->conmutador_m_a) {
        valor_conmutador = conduccion->conmutador_m_a;
        publishSwitch();
    }
    
}

/**
 * Método que comprueba constantemente si se ha producido alguna parada de emergencia (por obstáculo, a través de la seta de emergencia remota o 
 * a través de la seta de emergencia del vehículo
 */

void checkInfoStop() {
    //cout << "parada_emergencia_obstaculo: " << conduccion->parada_emergencia_obstaculo << endl;
    if (valor_parada_obstaculo != conduccion->parada_emergencia_obstaculo) {
        valor_parada_obstaculo = conduccion->parada_emergencia_obstaculo;
        //publishInfoStopObstacule(conduccion->parada_emergencia_obstaculo);
        publishInfoStop(conduccion->parada_emergencia_obstaculo, 0);
    }
    //cout << "parada_emergencia_local: " << conduccion->parada_emergencia_local << endl;
    if (valor_parada_local != conduccion->parada_emergencia_local) {
        valor_parada_local = conduccion->parada_emergencia_local;
        //publishInfoStopLocal(conduccion->parada_emergencia_obstaculo);
        publishInfoStop(conduccion->parada_emergencia_local, 1);
    }
    //cout << "parada_emergencia_remota: " << conduccion->parada_emergencia_remota << endl;
    if (valor_parada_remote != conduccion->parada_emergencia_remota) {
        valor_parada_remote = conduccion->parada_emergencia_remota;
        //publishInfoStopRemote(conduccion->parada_emergencia_obstaculo);
        publishInfoStop(conduccion->parada_emergencia_remota, 2);
    }
    
}

/**
 * Método que comprueba constantemente si se ha producido algún error en el vehículo
 */

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