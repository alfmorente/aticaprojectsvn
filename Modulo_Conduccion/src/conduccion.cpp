/* 
 * File:   conduccion.cpp
 * Author: Sergio Doctor López
 *
 * Created on 5 de marzo de 2014, 10:03
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
  tipo_vehiculo = 2;  //(1 == ATICA y 2 == CAMION)
    
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
 
  int estado_sistema = STATE_SYSTEM_OFF;
  while(estado_sistema != STATE_SYSTEM_ON){
       n.getParam("state_system",estado_sistema);
       usleep(50000); 
       can->inicio_read_write_CAN_frame = false; // Para que no empiece a contar tramas CAN 
                                            // antes de que se inicie el modulo
  }
  can->inicio_read_write_CAN_frame = true;
    // Envío de erorr si no hay comunicacion CAN 
  if (!finDePrograma) {       
        msg_err->id_error = CONNECTION_CAN_FAIL; // Error en conduccion (0: comunicacion)
        pub_error.publish(msg_err);
        
        cout << "CONNECTION_CAN_FAIL \n";
        cout << "Atica CONDUCCION :: Módulo finalizado" << endl;
        return 0;
  }
  usleep(100000);
  
  ros::Rate loop_rate(40); //Equivale a 50 milisegundos
  
  switch (operationMode) {
        case OPERATION_MODE_DEBUG:
                       
            //cout << "FUNCIONAMIENTO EN MODO DEBUG \n\n";
             
            while (ros::ok() && finDePrograma && !(can->errorWrite) && !(can->errorRead)) {  
                
                n.getParam("state_module_driving",estado_actual);
                if(estado_actual==STATE_ERROR || estado_actual== STATE_OFF) {             
                   finDePrograma=disconnectCommunication();   // Fin de programa = false (se cierra el CAN)

                   cout << "Fin de programa --- Se cierra el CAN \n";
                }
                
                else {
                    
                    if (tipo_vehiculo == ATICA) {
                        checkEmergencyStop();
                    
                        checkSwitch();
                      
                        checkInfoStop();
                    
                        checkError();
                    
                        //publishBackup();
                    }   
                    else if (tipo_vehiculo == CAMION) {
                        
                        // Vuelca los valores de la estructura al mensaje ROS
		  	convertMessageToROS(conduccionCamion->ksm,msg);
				
			// Publica el mensaje ROS
		  	pub_camion.publish(msg);
                        
                        int nivel = SerialComBomba->nivelBomba();
                        msg_mensajeNivel->nivel = nivel;
                        		
			//printf ("Nivel de Agua %d\n", mensajeNivel.nivel);
			usleep(250000);
	  		pub_nivel_bomba.publish(msg_mensajeNivel);
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
        if (tipo_vehiculo == ATICA) {
            conduccion->CONDUCCION_ACTIVE= false;
        }
        else if (tipo_vehiculo == CAMION){
            conduccionCamion->CONDUCCION_CAMION_ACTIVE = false;
            SerialComBomba->serialFlagActive = false;
            SerialComMastil->serialFlagActive = false;
            datosBajarMastil();
	    //write(canal,bajarMastil, sizeof(bajarMastil)+1);
            SerialComMastil->serial_send((char *)bajarMastil, sizeof(bajarMastil)+1);
        }
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

void publishEmergencyStop(){

    cout << "Publicacion de la parada de emergencia = INFORM \n";
    msg_emergency_stop->value = true;
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
    msg_backup->throttle = (short) conduccion->acelerador; // acelerador
    msg_backup->brake = (short) conduccion->freno_servicio; // freno de servicio
    msg_backup->steer = (short) (conduccion->sentido_marcha) * (conduccion->direccion); // direccion
    msg_backup->handbrake = (short) conduccion->freno_estacionamiento; // freno de mano
    msg_backup->gear = (short) conduccion->marcha_actual; // marcha
    msg_backup->engine = (short) conduccion->arranque_parada; // arranque/parada
    msg_backup->speed = (short) conduccion->velocidad; // velocidad
    pub_backup.publish(msg_backup);

    // Impresion del mensaje de Backup

    cout << "********** Publicacion del mensaje BACKUP *********** \n";
    cout << "Acelerador: " << (int) msg_backup->throttle << "\n";
    cout << "Freno de Servicio: " << (int) msg_backup->brake << "\n";
    cout << "Direccion: " << (int) msg_backup->steer << "\n";
    cout << "Freno de mano: " << (int) msg_backup->handbrake << "\n";
    cout << "Marcha: " << (int) msg_backup->gear << "\n";
    cout << "Arranque/Parada: " << (int) msg_backup->engine << "\n";
    cout << "Velocidad: " << (int) msg_backup->speed << "\n";
    cout << "***************************************************** \n\n\n";
  
}


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
/*
            if (conduccion->conmutador_m_a == OFF)
                msg_switch.value = false; // Manual
            else if (conduccion->conmutador_m_a == ON)
                msg_switch.value = true; // Teleoperado

            pub_switch.publish(msg_switch);

            cout << "********** Publicacion del mensaje SWITCH *********** \n";
            cout << "Si es 0 = Manual; Si es 1 = Teleoperado \n";
            cout << "Comuntador Manual/Automático: " << (int) msg_switch.value << "\n";
            cout << "***************************************************** \n\n\n";
  */          
            break;
            
        default:
            break;
    }
        
}


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

// Suscriptor al Módulo de Navegacion
void fcn_sub_navigation(const Common_files::msg_navigationPtr& msg)
{
  ROS_INFO("I heard a NAVIGATION message \n");
}

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

// Suscriptor al Módulo de System Management
void fcn_sub_emergency_stop(const Common_files::msg_emergency_stopPtr& msg) {

    //ROS_INFO("I heard a EMERGENCY STOP message from SYSTEM MANAGEMENT \n");

    parada_emergencia = true;
    cout << "Parada de emergencia = SET \n";
    conduccion->valor_parada_emergencia = ON;
    conduccion->m_emergency_stop_CAN_AUTOMATA();
    conduccion->t.Enable();

}

void fcn_sub_bomba(const Modulo_Conduccion::bombaPtr& msg)
{
	int comando = msg->comando;
	int value = msg->value;

	switch(comando)
	{

		case 0: 
			{
				if (value == 0 ) {			//DESACELERAR
					SerialComBomba->abrir_cerrar(1, 0, true);
					sleep (1);
					SerialComBomba->abrir_cerrar(1, 0, false);
					ROS_INFO("Desacelerar"); 
					break;
				}
				else if (value == 1) {    //ACELERAR
					SerialComBomba->abrir_cerrar(1, 1, true);
					sleep (1);
					SerialComBomba->abrir_cerrar(1, 1, false);
					ROS_INFO("Acelerar");
					break;
				}
			}		
		case 1:
			{ 
				if (value == 0) {	//VÁLVULA CISTERNA
					SerialComBomba->abrir_cerrar(1, 2, false);
					ROS_INFO("Cerrar válvula cisterna");
					break;
				}
				else if (value == 1) {
					SerialComBomba->abrir_cerrar(1, 2, true);
					ROS_INFO("Abrir válvula cisterna");
					break;
				}	
			}	
		case 2:
			{ 
				if (value == 0) {	//VÁLVULA AUTOLLENADO
					SerialComBomba->abrir_cerrar(1, 3, false);
					ROS_INFO("Cerrar válvula autollenado");
					break;
				}
				else if (value == 1) {
					SerialComBomba->abrir_cerrar(1, 3, true);
					ROS_INFO("Abrir válvula autollenado");
					break;
				}	
			}
		case 3:
			{ 
				if (value == 0) {	//VÁLVULA MONITOR
					SerialComBomba->abrir_cerrar(2, 0, false);
					ROS_INFO("Cerrar válvula monitor");
					break;
				}
				else if (value == 1) {
					SerialComBomba->abrir_cerrar(2, 0, true);
					ROS_INFO("Abrir válvula monitor");
					break;
				}	
			}
		case 4: 
			{
				if (value == 0 ) {			//PAN IZQUIERDA
					SerialComBomba->abrir_cerrar(2, 2, true);
					//sleep (1);
					//abrir_cerrar(2, 2, false);
					break;
				}
				else if (value == 1) {    //PAN DERECHA
					SerialComBomba->abrir_cerrar(2, 1, true);
					//sleep (1);
					//abrir_cerrar(2, 1, false);
					break;
				}
				else if (value == 2){ //PAN STOP
					SerialComBomba->abrir_cerrar(2,2,false);
					SerialComBomba->abrir_cerrar(2,1,false);
					break;
				}
			}	
		case 5: 
			{
				if (value == 0 ) {			//TILT ABAJO
					SerialComBomba->abrir_cerrar(2, 3, true);
					//sleep (1);
					//abrir_cerrar(2, 3, false);
					break;
				}
				else if (value == 1) {    //TILT ARRIBA
					SerialComBomba->abrir_cerrar(2, 7, true);
					//sleep (1);
					//abrir_cerrar(2, 7, false);
					break;
				}
				else if (value == 2){ //TILT STOP
					SerialComBomba->abrir_cerrar(2,3,false);
					SerialComBomba->abrir_cerrar(2,7,false);
					break;
				}
			}	
		case 6: 
			{
				if (value == 0 ) {			//CHORRO MONITOR
					SerialComBomba->abrir_cerrar(2, 5, true);  // ---> abre chorro
					usleep (200000);
					SerialComBomba->abrir_cerrar(2, 5, false); //---> cierra chorro
					ROS_INFO("Chorro monitor");
					break;
				}
				else if (value == 1) {    //NIEBLA MONITOR
					SerialComBomba->abrir_cerrar(2, 6, true);  // ---> abre niebla
					usleep (200000);
					SerialComBomba->abrir_cerrar(2, 6, false); //---> cierra niebla
					ROS_INFO("Niebla monitor");
					break;
				}
			}
	}
}

void fcn_sub_mastil(const Modulo_Conduccion::mastilPtr& msg)
{

	int oM = msg->opcionMastil;
	switch (oM) {

		case 0:
			{
			datosFocoDerecho();
			//write(canal,focoDerecho, sizeof(focoDerecho)+1);	
                        SerialComMastil->serial_send((char *)focoDerecho, sizeof(focoDerecho)+1);
			ROS_INFO("Foco derecho apagado ");
			break;
			}
		case 1:
			{
			datosFocoDerecho();
			//write(canal,focoDerecho, sizeof(focoDerecho)+1);	
                        SerialComMastil->serial_send((char *)focoDerecho, sizeof(focoDerecho)+1);
			ROS_INFO("Foco derecho encendido "); 
			break;
			}
		case 2:
			{
			datosFocoIzquierdo();
			//write(canal,focoIzquierdo, sizeof(focoIzquierdo)+1);	
                        SerialComMastil->serial_send((char *)focoIzquierdo, sizeof(focoIzquierdo)+1);
			ROS_INFO("Foco izquierdo apagado ");
			break;
			}
		case 3:
			{
			datosFocoIzquierdo();
			//write(canal,focoIzquierdo, sizeof(focoIzquierdo)+1);	
                        SerialComMastil->serial_send((char *)focoIzquierdo, sizeof(focoIzquierdo)+1);
			ROS_INFO("Foco izquierdo encendido ");
			break;
			}
		case 4: 
			{
			datosTiltAbajo();
			//write(canal,tiltAbajo, sizeof(tiltAbajo)+1);	
                        SerialComMastil->serial_send((char *)tiltAbajo, sizeof(tiltAbajo)+1);
			ROS_INFO("Tilt abajo ");

			/**sleep(1);

			datosOff();
			write(canal,off, sizeof(off)+1);
			ROS_INFO("Fin tilt abajo ");**/
			break;
			}
		case 5: 
			{
			datosTiltArriba();
			//write(canal,tiltArriba, sizeof(tiltArriba)+1);
                        SerialComMastil->serial_send((char *)tiltArriba, sizeof(tiltArriba)+1);
			ROS_INFO("Tilt arriba ");

			/**sleep(1);

			datosOff();
			write(canal,off, sizeof(off)+1);
			ROS_INFO("Fin tilt arriba ");**/  			
			break;
			}
		case 6: 
			{
			datosOff();
			//write(canal,off, sizeof(off)+1);
                        SerialComMastil->serial_send((char *)off, sizeof(off)+1);
			ROS_INFO("Tilt stop ");  			
			break;
			}
		case 7: 
			{
			datosPanDerecha();
			//write(canal,panDerecha, sizeof(panDerecha)+1);
                        SerialComMastil->serial_send((char *)panDerecha, sizeof(panDerecha)+1);
			ROS_INFO("Pan derecha ");
						
			/**sleep(1);
			
			datosOff();
			write(canal,off, sizeof(off)+1);
			ROS_INFO("Fin pan derecha ");**/  						

			break;
			}
		case 8: 
			{
			datosPanIzquierda();
			//write(canal,panIzquierda, sizeof(panIzquierda)+1);
                        SerialComMastil->serial_send((char *)panIzquierda, sizeof(panIzquierda)+1);
			ROS_INFO("Pan izquierda "); 
						
			/**sleep(1);
			
			datosOff();
			write(canal,off, sizeof(off)+1);
			ROS_INFO("Fin pan izquierda ");**/

			break;
			}
		case 9: 
			{
			datosOff();
			//write(canal,off, sizeof(off)+1);
                        SerialComMastil->serial_send((char *)off, sizeof(off)+1);
			ROS_INFO("Pan Stop ");
			break;
			}
		case 10:
			{
			datosBajarMastil();
			//write(canal,bajarMastil, sizeof(bajarMastil)+1);
                        SerialComMastil->serial_send((char *)bajarMastil, sizeof(bajarMastil)+1);
			ROS_INFO("Abajo mástil "); 
			break;
			}
		case 11:
			{
			datosSubirMastil();
			//write(canal,subirMastil, sizeof(subirMastil)+1);
                        SerialComMastil->serial_send((char *)subirMastil, sizeof(subirMastil)+1);
			ROS_INFO("Arriba mástil ");
			break;
			}
		case 12:
			{
			datosStop();
			//write(canal,stop, sizeof(stop)+1);
                        SerialComMastil->serial_send((char *)stop, sizeof(stop)+1);
			ROS_INFO("STOP mástil "); 
			break;
			}
	}
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
    
    bool res = false;

    while(CANflag < 3 && CANflag != 0);
    
    // Establecimiento y configuración de las comunicaciones

    if (tipo_vehiculo == ATICA) {
        // Establecimiento de la comunicacion con ATICA

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
    }
    
    else if (tipo_vehiculo == CAMION) {
        // Establecimiento de la comunicacion con CAMION DE BOMBEROS   

        canCamion = new CANCommunication(false, false, HW_PCI, 0, 0, 0x011C, true, "Camion"); // Asegurarnos del BaudRate

        do {
            if (canCamion->EstablishCommunication() == true) {
                canCamion->ConfigureCommunication();
                CANflag = 0;
            } else {
                CANflag++;
                if (CANflag == 3) { // Para más de 3 errores se cierra todo
                    canCamion->~CANCommunication();
                    return res = false;
                }
            }
        } while (CANflag < 3 && CANflag != 0);

        // Manejador PuertoSerie (Arranque Motor)

    SerialComBomba = new SerialCommunication ("/dev/ttyUSB0", B9600);
    
    do {
        if (SerialComBomba->EstablishCommunication() == true){
            // Se configurará únicamente 1 vez, no en cada arranque
            Serialflag = 0;
        }
        else{
            Serialflag++;
            if (Serialflag == 3){           // Para más de 3 errores se cierra todo   
                SerialComBomba->~SerialCommunication();
                return res = false;
            }
        }
        
    } while(Serialflag < 3 && Serialflag != 0);
        
    SerialComMastil = new SerialCommunication ("/dev/ttyUSB1", B9600);
    
    do {
        if (SerialComMastil->EstablishCommunication() == true){
            // Se configurará únicamente 1 vez, no en cada arranque
            Serialflag = 0;
        }
        else{
            Serialflag++;
            if (Serialflag == 3){           // Para más de 3 errores se cierra todo   
                SerialComMastil->~SerialCommunication();
                return res = false;
            }
        }
        
    } while(Serialflag < 3 && Serialflag != 0);
    
        SerialComBomba->iniciar();
        SerialComBomba->Run();
        SerialComMastil->Run();
        canCamion->Run();
        conduccionCamion->Run();
    }
    
    return res = true;
}

bool disconnectCommunication(){
    
     if (tipo_vehiculo == ATICA)
        can->~CANCommunication();
     else if (tipo_vehiculo == CAMION){
        canCamion->~CANCommunication();
        SerialComBomba->serialFlagActive = false;
        SerialComMastil->serialFlagActive = false;
        datosBajarMastil();
	//write(canal,bajarMastil, sizeof(bajarMastil)+1);
        //write(canal,bajarMastil, sizeof(bajarMastil)+1);
        SerialComMastil->serial_send((char *)bajarMastil, sizeof(bajarMastil)+1);

     }
     return false;
}


void initialize(ros::NodeHandle n) {
    
  // Generación de publicadores
  pub_error = n.advertise<Common_files::msg_error>("error",1000);
  pub_switch = n.advertise<Common_files::msg_switch>("switch",1000);
  pub_backup = n.advertise<Common_files::msg_backup>("backup",1000);
  pub_info_stop = n.advertise<Common_files::msg_info_stop>("infoStop",1000);
  pub_emergency_stop = n.advertise<Common_files::msg_emergency_stop>("emergInfo",1000);

  
  // Generacion de publicadores para el camion de bomberos
  pub_camion = n.advertise<Modulo_Conduccion::messageCAN>("ksm",1000);
  pub_nivel_bomba = n.advertise<Modulo_Conduccion::nivelBomba>("pNivelBomba", 1000);
 

  // Generación de subscriptores  
  sub_navigation = n.subscribe("pre_navigation", 1000, fcn_sub_navigation);
  sub_com_teleop = n.subscribe("commands_clean",1000,fcn_sub_com_teleop);
  sub_fcn_aux = n.subscribe("engBrake",1000,fcn_sub_engine_brake);
  sub_emergency_stop = n.subscribe("emergSet",1000,fcn_sub_emergency_stop);

  
  // Generacion de subscriptores para el camion de bomberos
  subBomba = n.subscribe("bomba", 1000, fcn_sub_bomba);
  subMastil = n.subscribe("mastil", 2, fcn_sub_mastil);
  
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
  
  // variables para el camion de bomberos
  freno_Mano = "OK";
  rev = 100.00;
  
}


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


void checkSwitch() {

    if (valor_conmutador != conduccion->conmutador_m_a) {
        valor_conmutador = conduccion->conmutador_m_a;
        publishSwitch();
    }
    
}


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


void convertMessageToROS(struct ksmData ksm, Modulo_Conduccion::messageCANPtr& msg)
{
/*
  if (ksm.signal1 == 'S') {
	msg->senal = 1;    	
	msg->opcion_etc1 = ksm.elecTransController1.opcion;
	msg->revoluciones_etc1 = ksm.elecTransController1.revoluciones;
	msg->porcentaje_etc1 = ksm.elecTransController1.porcentaje;
	msg->revoluciones2_etc1 = ksm.elecTransController1.revoluciones2;
	ksm.signal1 = 'N';
	cont1++;

	fprintf(f1, "El estado de la cadena cinética está: %s\n",ksm.elecTransController1.opcion);
	fprintf(f1, "Las revoluciones de salida son de %.2f rpm\n",ksm.elecTransController1.revoluciones);

	//printf ("REVOlUCIONES %.2f rpm\n",ksm.elecTransController1.revoluciones);

	fprintf(f1, "La posición del pedal del embrague es de un: %.2f %\n",ksm.elecTransController1.porcentaje);
	fprintf(f1, "La revoluciones de entrada son de %.2f rpm\n",ksm.elecTransController1.revoluciones2);

	//printf("La revoluciones de entrada son de %.2f rpm\n",ksm.elecTransController1.revoluciones2);

	fprintf(f1, "CONTADOR = %d \n",cont1);

	fprintf(f1, "\n");
  }  
 
*/
  if (ksm.signal2 == 'S') {
	msg->senal = 2;
	msg->revoluciones_eec1 = ksm.elecEngineController1.revoluciones;
	msg->porcentaje_eec1 = ksm.elecEngineController1.porcentaje;
	ksm.signal2 = 'N';
	
	if (rev != ksm.elecEngineController1.revoluciones) {
		//printf("La revoluciones de entrada son de %.2f rpm\n", ksm.elecEngineController1.revoluciones);
		rev = ksm.elecEngineController1.revoluciones;
	}
	/*cont2++;
	
	fprintf(f2, "El par motor/caudal inyectado es de un %d %\n",ksm.elecEngineController1.porcentaje);
	fprintf(f2, "La revoluciones de entrada son de %.2f rpm\n", ksm.elecEngineController1.revoluciones);

	


	fprintf(f2, "CONTADOR = %d \n",cont2);
	fprintf(f2, "\n");
	*/ 
  }


  else if (ksm.signal3 == 'S') {
	msg->senal = 3;
	msg->marcha = ksm.elecTransController2.marcha;
	msg->marcha_2 = ksm.elecTransController2.marcha2;
	msg->marcha_3 = ksm.elecTransController2.marcha3;
	ksm.signal3 = 'S';
	/*cont3++;
	

	fprintf(f3, "La marcha seleccionada es la %d\n", ksm.elecTransController2.marcha);

	printf ("MARCHA SELECCIONADA %d\n", ksm.elecTransController2.marcha);

	fprintf(f3, "La relación entrada a revoluciones de salida de la caja es de %.2f \n", ksm.elecTransController2.marcha2);
	fprintf(f3, "La actual/última marcha seleccionada es la %d\n", ksm.elecTransController2.marcha3);
	fprintf(f3, "CONTADOR = %d \n",cont3);
        fprintf(f3, "\n");
*/
  }


  else if (ksm.signal4 == 'S') {
	msg->senal = 4;
	msg->opcion_ebc1 = ksm.elecBrakeController1.opcion;
	msg->porcentaje_ebc1 = ksm.elecBrakeController1.porcentaje;
	ksm.signal4 = 'N';
	/*cont4++;
	
	fprintf(f4, "El ABS está: %s\n",ksm.elecBrakeController1.opcion);
	fprintf(f4, "La posición del pedal de freno es de un %.2f %\n",ksm.elecBrakeController1.porcentaje);

	//printf("La posición del pedal de freno es de un %.2f %\n",ksm.elecBrakeController1.porcentaje);

	fprintf(f4, "CONTADOR = %d \n",cont4);
	fprintf(f4, "\n");
	*/
  }
 

  else if (ksm.signal5 == 'S') {
	msg->senal = 5;
	msg->opcion_ccvs = ksm.cruiseControlVehiSpeed.opcion;
	msg->km_h_ccvs = ksm.cruiseControlVehiSpeed.km_h;
	msg->opcion2_ccvs = ksm.cruiseControlVehiSpeed.opcion2;
	msg->opcion3_ccvs = ksm.cruiseControlVehiSpeed.opcion3;
	msg->opcion4_ccvs = ksm.cruiseControlVehiSpeed.opcion4;
	msg->pto_state_ccvs = ksm.cruiseControlVehiSpeed.pto_state;
	ksm.signal5 = 'N';


	if (freno_Mano != ksm.cruiseControlVehiSpeed.opcion) {
		printf("El freno de estacionamiento está %s\n", ksm.cruiseControlVehiSpeed.opcion);
		freno_Mano = ksm.cruiseControlVehiSpeed.opcion;
	}
	/*cont5++;

	fprintf(f5, "El freno de etacionamiento está %s\n",ksm.cruiseControlVehiSpeed.opcion);

	

	fprintf(f5, "La relación entrada a revoluciones de salida de la caja es de %.2f \n", ksm.cruiseControlVehiSpeed.km_h);
	fprintf(f5, "El control de crucero está %s\n",ksm.cruiseControlVehiSpeed.opcion2);
	fprintf(f5, "El pedal del freno está %s\n",ksm.cruiseControlVehiSpeed.opcion3);
	fprintf(f5, "El pedal del embrague está %s\n",ksm.cruiseControlVehiSpeed.opcion4);
	fprintf(f5, "El estado del PTO stá en %s\n",ksm.cruiseControlVehiSpeed.pto_state);
	fprintf(f5, "CONTADOR = %d \n",cont5);
	fprintf(f5, "\n");
	*/
  }

/*	
  else if (ksm.signal6 == 'S') {
	msg->senal = 6;
	msg->opcion_aux = ksm.auxiliaryState.opcion;
	msg->opcion2_aux = ksm.auxiliaryState.opcion2;
	msg->opcion3_aux = ksm.auxiliaryState.opcion3;
	ksm.signal6 = 'N';
	cont6++;

	fprintf(f6, "La información del nivel del combustible está %s\n",ksm.auxiliaryState.opcion);
	fprintf(f6, "La marcha atrás está %s\n",ksm.auxiliaryState.opcion2);
	fprintf(f6, "El info Not-Aus stá %s\n",ksm.auxiliaryState.opcion3 );
	fprintf(f6, "CONTADOR = %d \n",cont6); 
	fprintf(f6, "\n");
  }

*/

  else if (ksm.signal7 == 'S') {
	msg->senal = 7;
	msg->opcion_eec2 = ksm.elecEngineController2.opcion;
	msg->opcion2_eec2 = ksm.elecEngineController2.opcion2;
	msg->porcentaje_eec2 = ksm.elecEngineController2.porcentaje;
	msg->porcentaje2_eec2 = ksm.elecEngineController2.porcentaje2;
	ksm.signal7 = 'N';
	/*cont7++;

	fprintf(f7, "La posición del relantí está %s\n",ksm.elecEngineController2.opcion);
	fprintf(f7, "El pedal del acelerador está %s\n",ksm.elecEngineController2.opcion2);
	fprintf(f7, "La posición del pedal del acelerador es de un %.2f %\n",ksm.elecEngineController2.porcentaje);

	//printf("La posición del pedal del acelerador es de un %.2f %\n",ksm.elecEngineController2.porcentaje);

	fprintf(f7, "La carga de la velocidad actual es de un %.2f %\n",ksm.elecEngineController2.porcentaje2);
	fprintf(f7, "CONTADOR = %d \n",cont7);
	fprintf(f7, "\n");
	*/
  }


  else if (ksm.signal8 == 'S') {
	msg->senal = 8;
	msg->grados_et = ksm.engineTemperature.grados;
	msg->grados2_et = ksm.engineTemperature.grados2;
	msg->grados3_et = ksm.engineTemperature.grados3;
	ksm.signal8 = 'N';
	/*cont8++;

	fprintf(f8, "La temperatura del agua refrigerante es de %d ºC\n",ksm.engineTemperature.grados);
	fprintf(f8, "La temperatura del combustible es de %d ºC\n",ksm.engineTemperature.grados2);
	fprintf(f8, "La temperatura del aceite es de %.2f ºC\n", ksm.engineTemperature.grados3); 
	fprintf(f8, "CONTADOR = %d \n",cont8);
	fprintf(f8, "\n");
	*/
  }

  else if (ksm.signal9 == 'S') {
	msg->senal = 9;
	msg->bares_ef = ksm.engineFluid.bares;
	ksm.signal9 = 'N';
	/*cont9++;

	fprintf(f9, "La presión del aceite es de %.2f bares\n",ksm.engineFluid.bares);
	fprintf(f9, "CONTADOR = %d \n",cont9);
	fprintf(f9, "\n");
	*/
  }


  else if (ksm.signal10 == 'S') {
	msg->senal = 10;
	msg->bares_sp = ksm.supplyPressure.bares;
	msg->bares2_sp = ksm.supplyPressure.bares2;
	msg->bares3_sp = ksm.supplyPressure.bares3;
	msg->bares4_sp = ksm.supplyPressure.bares4;
	msg->bares5_sp = ksm.supplyPressure.bares5;
	msg->bares6_sp = ksm.supplyPressure.bares6;
	ksm.signal10 = 'N';
	/*cont10++;

	fprintf(f10, "La presión neumática es de %.2f bares\n",ksm.supplyPressure.bares);
	fprintf(f10, "La presión de aire del trailer es de %.2f bares\n",ksm.supplyPressure.bares2);
	fprintf(f10, "La presión del circuito #1 del freno de servicio es de %.2f bares\n",ksm.supplyPressure.bares3);
	fprintf(f10, "La presión del circuito #2 del freno de servicio es de %.2f bares\n",ksm.supplyPressure.bares4);
	fprintf(f10, "La presión del equipo auxiliar es de %.2f bares\n",ksm.supplyPressure.bares5);
	fprintf(f10, "La presión de la suspensión es de %.2f bares\n",ksm.supplyPressure.bares6);
	fprintf(f10, "CONTADOR = %d \n",cont10);
	fprintf(f10, "\n");
	*/
  }

/*
  else if (ksm.signal11 == 'S') {
	msg->senal = 11;
	msg->bares_ac = ksm.ambientConditions.bares;
	msg->grados_ac = ksm.ambientConditions.grados;
	ksm.signal11 = 'N';
	cont11++;

	fprintf(f11, "La presión barométrica es de %.2f bares\n",ksm.ambientConditions.bares);
	fprintf(f11, "La temperatura ambiente es de %.2f ºC\n", ksm.ambientConditions.grados); 
	fprintf(f11, "CONTADOR = %d \n",cont11);
	fprintf(f11, "\n");
  }


  else if (ksm.signal12 == 'S') {
	msg->senal = 12;
	msg->kilometros_vdhr = ksm.vehiculeDistance.kilometros;
	ksm.signal12 = 'N';
	cont12++;

	fprintf(f12, "El total de kilómetros es de %.2f km\n", ksm.vehiculeDistance.kilometros);
	fprintf(f12, "CONTADOR = %d \n",cont12);
	fprintf(f12, "\n");
  }


  else if (ksm.signal13 == 'S') {
	msg->senal = 13;
	msg->axle = ksm.vehiculeWeight.axle;
	msg->kilogramos_vheacs = ksm.vehiculeWeight.kilogramos;
	ksm.signal13 = 'N';
	cont13++;

	fprintf(f13, "La localización de los ejes es %s\n",ksm.vehiculeWeight.axle);
	fprintf(f13, "El peso de los ejes es de %.2f kg\n",ksm.vehiculeWeight.kilogramos);
	fprintf(f13, "CONTADOR = %d \n",cont13);
	fprintf(f13, "\n");
  }


  else if (ksm.signal14 == 'S') {
	msg->senal = 14;
	msg->horas = ksm.engineHours.horas;
	ksm.signal14 = 'N';
	cont14++;

	fprintf(f14, "Las horas totales del motor son de %.2f horas\n", ksm.engineHours.horas);
	fprintf(f14, "CONTADOR = %d \n",cont14);
	fprintf(f14, "\n");
  }


  else if (ksm.signal15 == 'S') {
	msg->senal = 15;
	msg->driverWorking1 = ksm.tacograph.driverWorking1;
	msg->driverWorking2 = ksm.tacograph.driverWorking2;
	msg->opcion_t = ksm.tacograph.opcion;
	msg->driverTime1 = ksm.tacograph.driverTime;	
	msg->opcion2_t = ksm.tacograph.opcion2;
	msg->opcion3_t = ksm.tacograph.opcion3;
	msg->driverTime2 = ksm.tacograph.driverTime;	
	msg->opcion4_t = ksm.tacograph.opcion4;
	msg->opcion5_t = ksm.tacograph.opcion5;
	msg->opcion6_t = ksm.tacograph.opcion6;
	msg->opcion7_t = ksm.tacograph.opcion7;
	msg->km_h_t = ksm.tacograph.km_h;
	ksm.signal15 = 'N';
	cont15++;

	fprintf(f15, "El estado de Drive 1 Working está %s\n",ksm.tacograph.driverWorking1);
	fprintf(f15, "El estado de Drive 2 Working está %s\n",ksm.tacograph.driverWorking2);
	fprintf(f15, "Driver recognition está %s\n",ksm.tacograph.opcion);
	fprintf(f15, "El estado de Drive 1 time related está %s\n",ksm.tacograph.driverTime);
	fprintf(f15, "El estado de Drive Card 1 está %s\n",ksm.tacograph.opcion2);
	fprintf(f15, "Overspeed está %s\n",ksm.tacograph.opcion3);
	fprintf(f15, "El estado de Drive 2 time related está %s\n",ksm.tacograph.driverTime2);
	fprintf(f15, "El estado de Drive Card 2 está %s\n",ksm.tacograph.opcion4);
	fprintf(f15, "El sistema de eventos está %s\n",ksm.tacograph.opcion5);
	fprintf(f15, "El manejo de la información está %s\n",ksm.tacograph.opcion6);
	fprintf(f15, "El sistema de rendimiento está %s\n",ksm.tacograph.opcion7);
	fprintf(f15, "La velocidad del tacógrafo es de %.2f km/h\n",ksm.tacograph.km_h);
	fprintf(f15, "CONTADOR = %d \n",cont15);
	fprintf(f15, "\n");
  }


  else if (ksm.signal16 == 'S') {
	msg->senal = 16;
	msg->porcentaje_erc = ksm.elecRetarder.porcentaje;
	ksm.signal16 = 'N';
	cont16++;

	fprintf(f16, "Actual retarder torque es de %d %	\n",ksm.elecRetarder.porcentaje);
	fprintf(f16, "CONTADOR = %d \n",cont16);
	fprintf(f16, "\n");
  }


  else if (ksm.signal17 == 'S') {
	msg->senal = 17;
	msg->opcion_auxstat = ksm.auxStat.opcion;
	msg->opcion2_auxstat = ksm.auxStat.opcion2;
	ksm.signal17 = 'N';
	cont17++;

	fprintf(f17, "El aviso de la temperatura excesiva del agua refrigerante está %s\n",ksm.auxStat.opcion);
	fprintf(f17, "El aviso de la presión del aceite está %s\n", ksm.auxStat.opcion2);
	fprintf(f17, "CONTADOR = %d \n",cont17);
	fprintf(f17, "\n");
  }


  else if (ksm.signal18 == 'S') {
	msg->senal = 18;
	msg->litros = ksm.fuelEconomy.litros;
	msg->km_l = ksm.fuelEconomy.km_l;
	ksm.signal18 = 'N';
	cont18++;

	fprintf(f18, "La tasa de combustible es de %.2f litros\n",ksm.fuelEconomy.litros);
	fprintf(f18, "El gasto instantáneo de fuel es de %.2f km/l\n",ksm.fuelEconomy.km_l);	
	fprintf(f18, "CONTADOR = %d \n",cont18);
	fprintf(f18, "\n");
  }

*/
  else if (ksm.signal19 == 'S') {
	msg->senal = 19;
	msg->opcion_etc3 = ksm.elecTransController3.opcion;
	msg->opcion2_etc3 = ksm.elecTransController3.opcion2;
	msg->pto1state = ksm.elecTransController3.pto1state;
	msg->pto2state = ksm.elecTransController3.pto2state;
	msg->nmvstate = ksm.elecTransController3.nmvstate;
	ksm.signal19 = 'N';
	/*cont19++;

	fprintf(f19, "La posición neutral de la caja está %s\n",ksm.elecTransController3.opcion);
	fprintf(f19, "La marcha está %s\n", ksm.elecTransController3.opcion2);
	fprintf(f19, "PTO1 State está %s\n", ksm.elecTransController3.pto1state);
	fprintf(f19, "PTO2 State está %s\n", ksm.elecTransController3.pto2state);
	fprintf(f19, "NMV State está %s\n", ksm.elecTransController3.nmvstate);
	fprintf(f19, "CONTADOR = %d \n",cont19);
	fprintf(f19, "\n");
	*/
  }
 
}


//DATOS FOCO DERECHO
void datosFocoDerecho () {
	focoDerecho [0] = 0x02;
	focoDerecho [1] = 0x01;
	focoDerecho [2] = 0x0C;
	focoDerecho [3] = 0x96;
	focoDerecho [4] = 0x00;
	focoDerecho [5] = 0x10;
	focoDerecho [6] = 0xB5;
	focoDerecho [7] = 0xFF;
	focoDerecho [8] = 0xFF;
	focoDerecho [9] = 0x03;
	focoDerecho [10] = 0x02;
	focoDerecho [11] = 0x01;
	focoDerecho [12] = 0x0C;
	focoDerecho [13] = 0x96;
	focoDerecho [14] = 0x10;
	focoDerecho [15] = 0x04;
	focoDerecho [16] = 0xB9;
	focoDerecho [17] = 0x00;
	focoDerecho [18] = 0x00;
	focoDerecho [19] = 0x03;
	focoDerecho [20] = 0x02;
	focoDerecho [21] = 0x01;
	focoDerecho [22] = 0x0C;
	focoDerecho [23] = 0x96;
	focoDerecho [24] = 0x00;
	focoDerecho [25] = 0x00;
	focoDerecho [26] = 0xA5;
	focoDerecho [27] = 0xFF;
	focoDerecho [28] = 0xFF;
	focoDerecho [29] = 0x03;
	focoDerecho [30] = 0x02;
	focoDerecho [31] = 0x01;
	focoDerecho [32] = 0x0C;
	focoDerecho [33] = 0x96;
	focoDerecho [34] = 0x00;
	focoDerecho [35] = 0x04;
	focoDerecho [36] = 0xA9;
	focoDerecho [37] = 0x00;
	focoDerecho [38] = 0x00;
	focoDerecho [39] = 0x03;
}


//DATOS FOCO IZQUIERDO
void datosFocoIzquierdo () {
	focoIzquierdo [0] = 0x02;
	focoIzquierdo [1] = 0x01;
	focoIzquierdo [2] = 0x0C;
	focoIzquierdo [3] = 0x96;
	focoIzquierdo [4] = 0x00;
	focoIzquierdo [5] = 0x20;
	focoIzquierdo [6] = 0xC5;
	focoIzquierdo [7] = 0xFF;
	focoIzquierdo [8] = 0xFF;
	focoIzquierdo [9] = 0x03;
	focoIzquierdo [10] = 0x02;
	focoIzquierdo [11] = 0x01;
	focoIzquierdo [12] = 0x0C;
	focoIzquierdo [13] = 0x96;
	focoIzquierdo [14] = 0x20;
	focoIzquierdo [15] = 0x04;
	focoIzquierdo [16] = 0xC9;
	focoIzquierdo [17] = 0x00;
	focoIzquierdo [18] = 0x00;
	focoIzquierdo [19] = 0x03;
	focoIzquierdo [20] = 0x02;
	focoIzquierdo [21] = 0x01;
	focoIzquierdo [22] = 0x0C;
	focoIzquierdo [23] = 0x96;
	focoIzquierdo [24] = 0x00;
	focoIzquierdo [25] = 0x00;
	focoIzquierdo [26] = 0xA5;
	focoIzquierdo [27] = 0xFF;
	focoIzquierdo [28] = 0xFF;
	focoIzquierdo [29] = 0x03;
	focoIzquierdo [30] = 0x02;
	focoIzquierdo [31] = 0x01;
	focoIzquierdo [32] = 0x0C;
	focoIzquierdo [33] = 0x96;
	focoIzquierdo [34] = 0x00;
	focoIzquierdo [35] = 0x04;
	focoIzquierdo [36] = 0xA9;
	focoIzquierdo [37] = 0x00;
	focoIzquierdo [38] = 0x00;
	focoIzquierdo [39] = 0x03;
}


//DATOS Tilt ABAJO
void datosTiltAbajo() {
	tiltAbajo [0] = 0x02;
	tiltAbajo [1] = 0x01;
	tiltAbajo [2] = 0x0C;
	tiltAbajo [3] = 0x96;
	tiltAbajo [4] = 0x04;
	tiltAbajo [5] = 0x00;
	tiltAbajo [6] = 0xA9;
	tiltAbajo [7] = 0xFF;
	tiltAbajo [8] = 0xFF;
	tiltAbajo [9] = 0x03;
	tiltAbajo [10] = 0x02;
	tiltAbajo [11] = 0x01;
	tiltAbajo [12] = 0x0C;
	tiltAbajo [13] = 0x96;
	tiltAbajo [14] = 0x04;
	tiltAbajo [15] = 0x04;
	tiltAbajo [16] = 0xAD;
	tiltAbajo [17] = 0x00;
	tiltAbajo [18] = 0x00;
	tiltAbajo [19] = 0x03;
}


//DATOS Tilt ARRIBA
void datosTiltArriba() {
	tiltArriba [0] = 0x02;
	tiltArriba [1] = 0x01;
	tiltArriba [2] = 0x0C;
	tiltArriba [3] = 0x96;
	tiltArriba [4] = 0x02;
	tiltArriba [5] = 0x00;
	tiltArriba [6] = 0xA7;
	tiltArriba [7] = 0xFF;
	tiltArriba [8] = 0xFF;
	tiltArriba [9] = 0x03;
	tiltArriba [10] = 0x02;
	tiltArriba [11] = 0x01;
	tiltArriba [12] = 0x0C;
	tiltArriba [13] = 0x96;
	tiltArriba [14] = 0x02;
	tiltArriba [15] = 0x04;
	tiltArriba [16] = 0xAB;
	tiltArriba [17] = 0x00;
	tiltArriba [18] = 0x00;
	tiltArriba [19] = 0x03;
}


//DATOS Pan IZQUIERDA
void datosPanIzquierda() {
	panIzquierda [0] = 0x02;
	panIzquierda [1] = 0x01;
	panIzquierda [2] = 0x0C;
	panIzquierda [3] = 0x96;
	panIzquierda [4] = 0x00;
	panIzquierda [5] = 0x04;
	panIzquierda [6] = 0xA9;
	panIzquierda [7] = 0xFF;
	panIzquierda [8] = 0xFF;
	panIzquierda [9] = 0x03;
	panIzquierda [10] = 0x02;
	panIzquierda [11] = 0x01;
	panIzquierda [12] = 0x0C;
	panIzquierda [13] = 0x96;
	panIzquierda [14] = 0x04;
	panIzquierda [15] = 0x04;
	panIzquierda [16] = 0xAD;
	panIzquierda [17] = 0x00;
	panIzquierda [18] = 0x00;
	panIzquierda [19] = 0x03;
}


//DATOS Pan DERECHA
void datosPanDerecha() {
	panDerecha [0] = 0x02;
	panDerecha [1] = 0x01;
	panDerecha [2] = 0x0C;
	panDerecha [3] = 0x96;
	panDerecha [4] = 0x00;
	panDerecha [5] = 0x40;
	panDerecha [6] = 0xE5;
	panDerecha [7] = 0xFF;
	panDerecha [8] = 0xFF;
	panDerecha [9] = 0x03;
	panDerecha [10] = 0x02;
	panDerecha [11] = 0x01;
	panDerecha [12] = 0x0C;
	panDerecha [13] = 0x96;
	panDerecha [14] = 0x40;
	panDerecha [15] = 0x04;
	panDerecha [16] = 0xE9;
	panDerecha [17] = 0x00;
	panDerecha [18] = 0x00;
	panDerecha [19] = 0x03;
}


//OFF --> valido para parar el tilt hacia arriba y abajo y el pan hacia la derecha y la izquierda
void datosOff() {
	off [0] = 0x02;
	off [1] = 0x01;
	off [2] = 0x0C;
	off [3] = 0x96;
	off [4] = 0x00;
	off [5] = 0x00;
	off [6] = 0xA5;
	off [7] = 0xFF;
	off [8] = 0xFF;
	off [9] = 0x03;
	off [10] = 0x02;
	off [11] = 0x01;
	off [12] = 0x0C;
	off [13] = 0x96;
	off [14] = 0x00;
	off [15] = 0x04;
	off [16] = 0xA9;
	off [17] = 0x00;
	off [18] = 0x00;
	off [19] = 0x03;
}


//DATOS BAJAR MASTIL
void datosBajarMastil () {
	bajarMastil [0] = 0x02;
	bajarMastil [1] = 0x01;
	bajarMastil [2] = 0x0C;
	bajarMastil [3] = 0x96;
	bajarMastil [4] = 0x01;
	bajarMastil [5] = 0x00;
	bajarMastil [6] = 0xA6;
	bajarMastil [7] = 0xFF;
	bajarMastil [8] = 0xFF;
	bajarMastil [9] = 0x03;
	bajarMastil [10] = 0x02;
	bajarMastil [11] = 0x01;
	bajarMastil [12] = 0x0C;
	bajarMastil [13] = 0x96;
	bajarMastil [14] = 0x01;
	bajarMastil [15] = 0x04;
	bajarMastil [16] = 0xAA;
	bajarMastil [17] = 0x00;
	bajarMastil [18] = 0x00;
	bajarMastil [19] = 0x03;
	bajarMastil [20] = 0x02;
	bajarMastil [21] = 0x01;
	bajarMastil [22] = 0x0C;
	bajarMastil [23] = 0x96;
	bajarMastil [24] = 0x00;
	bajarMastil [25] = 0x00;
	bajarMastil [26] = 0xA5;
	bajarMastil [27] = 0xFF;
	bajarMastil [28] = 0xFF;
	bajarMastil [29] = 0x03;
	bajarMastil [30] = 0x02;
	bajarMastil [31] = 0x01;
	bajarMastil [32] = 0x0C;
	bajarMastil [33] = 0x96;
	bajarMastil [34] = 0x00;
	bajarMastil [35] = 0x04;
	bajarMastil [36] = 0xA9;
	bajarMastil [37] = 0x00;
	bajarMastil [38] = 0x00;
	bajarMastil [39] = 0x03;
}


//DATOS SUBIR MASTIL
void datosSubirMastil () {
	subirMastil [0] = 0x02;
	subirMastil [1] = 0x01;
	subirMastil [2] = 0x0C;
	subirMastil [3] = 0x96;
	subirMastil [4] = 0x00;
	subirMastil [5] = 0x01;
	subirMastil [6] = 0xA6;
	subirMastil [7] = 0xFF;
	subirMastil [8] = 0xFF;
	subirMastil [9] = 0x03;
	subirMastil [10] = 0x02;
	subirMastil [11] = 0x01;
	subirMastil [12] = 0x0C;
	subirMastil [13] = 0x96;
	subirMastil [14] = 0x01;
	subirMastil [15] = 0x04;
	subirMastil [16] = 0xAA;
	subirMastil [17] = 0x00;
	subirMastil [18] = 0x00;
	subirMastil [19] = 0x03;
	subirMastil [20] = 0x02;
	subirMastil [21] = 0x01;
	subirMastil [22] = 0x0C;
	subirMastil [23] = 0x96;
	subirMastil [24] = 0x00;
	subirMastil [25] = 0x00;
	subirMastil [26] = 0xA5;
	subirMastil [27] = 0xFF;
	subirMastil [28] = 0xFF;
	subirMastil [29] = 0x03;
	subirMastil [30] = 0x02;
	subirMastil [31] = 0x01;
	subirMastil [32] = 0x0C;
	subirMastil [33] = 0x96;
	subirMastil [34] = 0x00;
	subirMastil [35] = 0x04;
	subirMastil [36] = 0xA9;
	subirMastil [37] = 0x00;
	subirMastil [38] = 0x00;
	subirMastil [39] = 0x03;
}


//STOP
void datosStop() {
	stop [0] = 0x02;
	stop [1] = 0x01;
	stop [2] = 0x0C;
	stop [3] = 0x96;
	stop [4] = 0x10;
	stop [5] = 0x00;
	stop [6] = 0xB5;
	stop [7] = 0xFF;
	stop [8] = 0xFF;
	stop [9] = 0x03;
}

