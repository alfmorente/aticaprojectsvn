/* 
 * File:   conduccion.cpp
 * Author: Sergio Doctor López
 *
 * Created on 5 de marzo de 2014, 10:03
 */
#include "../include/Modulo_Conduccion_Bobcat/conduccion.h"


// Tratamiento de señales (CTRL+C)
#include <signal.h>


using namespace std;




/*******************************************************************************
 *******************************************************************************
 *                     FUNCIONES TRATAMIENTO DE SEÑALES
 * *****************************************************************************
 * ****************************************************************************/

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
  

  
  //ros::Rate loop_rate(40); //Equivale a 50 milisegundos
  
  Timer *timer = new Timer();
  timer->Enable();
  
  switch (operationMode) {
        case OPERATION_MODE_DEBUG:
                       
            //cout << "FUNCIONAMIENTO EN MODO DEBUG \n\n";
            
  // PARA COMUNIACCION CAN
            while (ros::ok() && finDePrograma) {  // PARA COMUNICACION POR SOCKET
                
                n.getParam("state_module_driving",estado_actual);
                if(estado_actual==STATE_ERROR || estado_actual== STATE_OFF) {             
                   finDePrograma=disconnectCommunication();   // Fin de programa = false (se cierra el CAN)

                   cout << "Fin de programa --- Se cierra el CAN \n";
                }
                
                else {      
                    
                    //cout << "entra " << endl;
                    //driving->checkMessageTimeout();
                    driving->checkForVehicleMessages();
                    //driving->checkAlarms();
                    //driving->checkSwitcher();
                    
                    // Comprobación del temporizador y requerimiento de info
                    if (timer->GetTimed() >= FREC_2HZ) {
                        // Clear del timer
                        timer->Reset();
                        driving->reqVehicleInfo();
                        if(driving->checkSwitcher()){
                                publishSwitch(driving->getSwitcher());
                        }
                        publishBackupArduino(driving->getVehicleInfo(true));
                        
                    }
                }
                ros::spinOnce();
                usleep(1000);
                //loop_rate.sleep();

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

void publishBackupArduino(DrivingInfo info) {
      msg_backup->steer = info.steering;
      msg_backup->throttle = info.thottle;
      msg_backup->brake = info.brake;
      msg_backup->gear = info.gear;
      msg_backup->speed = info.speed;
      msg_backup->handbrake = info.parkingBrake;
      msg_backup->engine = info.rengine;
  
      pub_backup.publish(msg_backup);
   /*   
    cout << "********** Publicacion del mensaje BACKUP *********** \n";
    cout << "Acelerador: " << (int) msg_backup->throttle << "\n";
    cout << "Freno de Servicio: " << (int) msg_backup->brake << "\n";
    cout << "Direccion con sentido: " << (int) msg_backup->steer << "\n";
    cout << "Freno de mano: " << (int) msg_backup->handbrake << "\n";
    cout << "Marcha: " << (int) msg_backup->gear << "\n";
    cout << "Arranque/Parada: " << (int) msg_backup->engine << "\n";
    cout << "Velocidad: " << (int) msg_backup->speed << "\n";    
    cout << "***************************************************** \n\n\n";
 */
}

void publishSwitch(int position) {
    msg_switch->value = position;
    pub_switch.publish(msg_switch);
    
}




/*******************************************************************************
 *******************************************************************************
 *                              SUSCRIPTORES
 * *****************************************************************************
 * ****************************************************************************/

// Suscriptor al Módulo de Navegacion
void fcn_sub_navigation(const Common_files::msg_navigationPtr& msg)
{
  ROS_INFO("I heard a NAVIGATION message \n");
}

// Suscriptor al Módulo de Remote
void fcn_sub_com_teleop(const Common_files::msg_com_teleopPtr& msg)
{
       // ROS_INFO("I heard a TELEOP DEPURADO. message \n");
        
    FrameDriving fdr;
    fdr.instruction = static_cast<CommandID>(SET);
    fdr.value = msg->value;


        switch (msg->id_element) {
              case ID_REMOTE_THROTTLE:   // Acelerador
                  
                  fdr.element = static_cast<DeviceID>(THROTTLE);
                  cout << "acelerador = " << fdr.element << "\n";
                  //conduccion->acelerador_tx = (short) msg->value;
                  break;              
              case ID_REMOTE_BRAKE:   // Freno de servicio
                  
                  fdr.element = static_cast<DeviceID>(ABRAKE);
                  fdr.value = (fdr.value)*45/100;
                  cout << "Freno de servicio = " << fdr.element << "\n";
                  //conduccion->freno_servicio_tx = (short) msg->value;
                  break;
              case ID_REMOTE_STEER:   // Direccion   
                  
                  fdr.element = static_cast<DeviceID>(STEERING);
                  // Conversion a Honorio 
                  //(BORRAR)
                  fdr.value = (fdr.value+100)/2;
                  // (BORRAR)
                  cout << "Direccion = " << fdr.element << "\n";
                  //conduccion->valor_direccion = (short) msg->value;
                  break;
              case ID_REMOTE_GEAR:   // Marcha
                  
                  fdr.element = static_cast<DeviceID>(GEAR);
                  
                  cout << "Marcha = " << fdr.element << "\n";
                  //conduccion->valor_marcha = (short) msg->value;
                  break;
              case ID_REMOTE_HANDBRAKE:   // Freno de mano
                  
                  fdr.element = static_cast<DeviceID>(HANDBRAKE);
                  
                  cout << "Freno de mano = " << fdr.element << "\n";
                  //conduccion->valor_freno_estacionamiento = (short) msg->value;
                  break;
              case ID_REMOTE_ENGINE:   // Encendido del motor
                  fdr.element = static_cast<DeviceID>(ARRANQUE);
                  cout << "Encendido del motor = " << fdr.element << "\n";
                  //cout << "Encendido del motor = " << msg.value << "\n";
                  //conduccion->valor_arranque_parada = (short) msg->value;
                  break;
              case ID_REMOTE_LIGHT_IR:   // Luces IR
                  //cout << "Luces IR = " << msg.value << "\n";
                  //conduccion->valor_luces_IR = (short) msg->value;
                  break;
              case ID_REMOTE_LIGHT_CONVENTIONAL:   // Luces 
                  
                  fdr.element = static_cast<DeviceID>(DIPSS);
                  
                  cout << "Luces = " << fdr.element << "\n";
                  //conduccion->valor_luces = (short) msg->value;
                  break;
              case ID_REMOTE_DIFF:   // Diferenciales
                  //cout << "Diferenciales = " << msg.value << "\n";
                  fdr.element = static_cast<DeviceID>(RELEHIDRAULICO);
                  cout << "Hidraulico = " << fdr.element << "\n";
                  //conduccion->valor_diferencial = (short) msg->value;
                  break;
              case ID_REMOTE_ACT_LASER2D:   // Activacion Laser
                  //cout << "Activacion del laser = " << msg.value << "\n";
                  //conduccion->valor_laser = (short) msg->value;
                  break;
              default:          
                  break;          
          }
        driving->setCommand(fdr);

        
}


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

bool createCommunication(){
    
    driving = new DrivingConnectionManager();
    while(!driving->doConnect(DEVICE_DRIVING)){
        printf("No se pudo conectar con el vehiculo. Reintentando...\n");
        usleep(2000000);
    }
    ROS_INFO("Conexion establecida. Enviando RESET de sincronizacion...");
    driving->sendReset();
    printf("OK\n");
            
    
    return true;

}

bool disconnectCommunication(){
    
    driving->disconnect();
    
    return false;
}


void initialize(ros::NodeHandle n) {
    
  // Generación de publicadores
  pub_backup = n.advertise<Common_files::msg_backup>("backup",1000);
  pub_switch = n.advertise<Common_files::msg_switch>("switch",1000);
  
  // Generación de subscriptores  
  sub_navigation = n.subscribe("pre_navigation", 1000, fcn_sub_navigation);
  sub_com_teleop = n.subscribe("commands_clean",1000,fcn_sub_com_teleop);
 
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
