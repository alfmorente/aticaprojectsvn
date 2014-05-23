/* 
 * File:   ConduccionThread.cpp
 * Author: Sergio Doctor López
 *
 * Created on 6 de febrero de 2014
 */

#include "../include/Modulo_Conduccion/ConduccionThread.hpp"
#include <queue>
#include <iostream>



ConduccionThread::ConduccionThread(CANCommunication * canCOND) {
    CONDUCCION_ACTIVE = true;
    paradaEmergencia = false;
    CANCONDUCCION = canCOND;
}

ConduccionThread::~ConduccionThread() {
}

void ConduccionThread::DoWork(){

    //int QueueSize;
    TPCANRdMsg MsgAux;

    while (CONDUCCION_ACTIVE) {

        if (!CANCONDUCCION->ConduccionQueue.empty()) {
        // RX de mensajes
        //pthread_mutex_lock (&CANCONDUCCION->ConduccionQueue_mutex);
        //QueueSize = CANCONDUCCION->ConduccionQueue.size();
        //pthread_mutex_unlock (&CANCONDUCCION->ConduccionQueue_mutex);

        //for (int i=0;i<QueueSize; i++) {        
        
            pthread_mutex_lock (&CANCONDUCCION->ConduccionQueue_mutex);
            MsgAux=CANCONDUCCION->ConduccionQueue.front();
            pthread_mutex_unlock (&CANCONDUCCION->ConduccionQueue_mutex);
            //cout << "Desencolado mensaje con ID = " << hex << MsgAux.Msg.ID << "\n";
            switch (MsgAux.Msg.ID){
                    
                case 0x5:
                    //cout << "\n MENSAJE DE ESTADO DEL VEHÍCULO \n";
                    m_Status_Message_AUTOMATA_CAN(MsgAux);
                    CANCONDUCCION->ConduccionQueue.pop();
                    //cout << "\n Desencolado mensaje con ID = " << hex << MsgAux.Msg.ID << "\n";
                    break;
             
                case 0x00E:
                    //cout << "\n MENSAJE DE ERROR DEL VEHÍCULO \n";
                    m_Error_Message_AUTOMATA_CAN(MsgAux);
                    CANCONDUCCION->ConduccionQueue.pop();
                    //cout << "\n Desencolado mensaje con ID = " << hex << MsgAux.Msg.ID << "\n";
                    break;
                    
                case 0x80:
                    //cout << "\n TRAMA DE SINCRONISMO \n";
                    
                    CANCONDUCCION->ConduccionQueue.pop();
                    //cout << "\n Desencolado mensaje con ID = " << hex << MsgAux.Msg.ID << "\n";
                    break;
                
                default:                    
                    //cout << "\n Error en lectura \n";
                    break;
             }
                        
        }
        usleep(10000);
        // TX de mensajes
        //m_Change_Command_CAN_AUTOMATA();
        //cout << "Tiempo de envio: " << time1.GetTime() << "\n";
        
    }
        //usleep(150000); // TODO Revisar y eliminar/configurar "sleeps"
    
    cout << "\n Salida del hilo de Conducción \n"
            "\n ----------------------------- \n";
    

}
      

// Tratamiento del mensaje 005 (vehículo --> autómata --> CAN)

void ConduccionThread::m_Status_Message_AUTOMATA_CAN(TPCANRdMsg StatusMsg){   
         
    int sentido_giro;
    
    // BYTE 0 - Arranque/Parada - Freno estacionamiento
    if ((StatusMsg.Msg.DATA[0] & ARRANQUE_PARADA) == ARRANQUE_PARADA) 
        arranque_parada = 1;
    else
        arranque_parada = 0;
    
    if ((StatusMsg.Msg.DATA[0] & FRENO_ESTACIONAMIENTO) == FRENO_ESTACIONAMIENTO) 
        freno_estacionamiento = 1;
    else
        freno_estacionamiento = 0;
    
    // BYTE 1 - Posicion conmutador M/A
    conmutador_m_a = StatusMsg.Msg.DATA[1];
        
    // BYTE 2 - Velocidad
    velocidad = StatusMsg.Msg.DATA[2];
    
    // BYTE 3 - Freno de Servicio
    freno_servicio = StatusMsg.Msg.DATA[3];
    
    // BYTE 4 - Marcha 
    if ((StatusMsg.Msg.DATA[4] & MARCHA_H_) == MARCHA_H_) 
        marcha_actual = 0;
    if ((StatusMsg.Msg.DATA[4] & MARCHA_N_) == MARCHA_N_) 
        marcha_actual = 1;
    if ((StatusMsg.Msg.DATA[4] & MARCHA_R_) == MARCHA_R_) 
        marcha_actual = 2;
    if ((StatusMsg.Msg.DATA[4] & MARCHA_N1_) == MARCHA_N1_) 
        marcha_actual = 3;
    if ((StatusMsg.Msg.DATA[4] & MARCHA_L_) == MARCHA_L_)     
        marcha_actual = 4;
    
    // BYTE 5 - Sentido de la direccion
    sentido_giro = StatusMsg.Msg.DATA[5];
    switch (sentido_giro) {
        case 0:
            sentido_marcha = 0;  // NINGUNO
            break;
        case 1:
            sentido_marcha = 1;  // DERECHA
            break;
        case 16:
            sentido_marcha = -1; // IZQUIERDA
            break;
        default:
            break;
    }
    
    // BYTE 6 - Direccion
    direccion = StatusMsg.Msg.DATA[6];
        
    // BYTE 7 - Confirmacion parada de emergencia - parada de emergencia por obstaculo 
    //        - Parada de emergencia remota - parada de emergencia local
    if ((StatusMsg.Msg.DATA[7] & CONF_PARADA_EMERGENCIA) == CONF_PARADA_EMERGENCIA)
        conf_parada_emergencia = 1;
    else
        conf_parada_emergencia = 0;
    
    if ((StatusMsg.Msg.DATA[7] & PARADA_EMERGENCIA_OBSTACULO) == PARADA_EMERGENCIA_OBSTACULO)
        parada_emergencia_obstaculo = 1;
    else        
        parada_emergencia_obstaculo = 0;
    if ((StatusMsg.Msg.DATA[7] & PARADA_EMERGENCIA_REMOTA) == PARADA_EMERGENCIA_REMOTA)
        parada_emergencia_remota = 1;    
    else         
        parada_emergencia_remota = 0;
    if ((StatusMsg.Msg.DATA[7] & PARADA_EMERGENCIA_LOCAL) == PARADA_EMERGENCIA_LOCAL)
        parada_emergencia_local = 1;
    else        
        parada_emergencia_local = 0;
      
}

// Tratamiento del mensaje 00E (vehículo --> autómata --> CAN

void ConduccionThread::m_Error_Message_AUTOMATA_CAN(TPCANRdMsg StatusMsg){   
         
    // BYTE 0 - Fallo Arranque/Parada
    error_arranque_parada = StatusMsg.Msg.DATA[0];
    //cout << "ERROR A/P: " << error_arranque_parada << endl;
        
    // BYTE 1 - Fallo acelerador 
    error_acelerador = StatusMsg.Msg.DATA[1];
    //cout << "ERROR Acelerador: " << error_acelerador << endl;
        
    // BYTE 2 - Fallo Freno de estacionamiento
    error_freno_estacionamiento = StatusMsg.Msg.DATA[2];
    //cout << "ERROR Freno Estacionamiento: " << error_freno_estacionamiento << endl;
        
    // BYTE 3 - Fallo Freno de servicio
    error_freno_servicio = StatusMsg.Msg.DATA[3];
    //cout << "ERROR Freno Servicio: " << error_freno_servicio << endl;
            
    // BYTE 4 - Fallo Cambio de marchas
    error_cambio_marcha = StatusMsg.Msg.DATA[4];
    //cout << "ERROR Cambio Marcha: " << error_cambio_marcha << endl;
                
    // BYTE 5 - Fallo Direccion
    error_direccion = StatusMsg.Msg.DATA[5];
    //cout << "ERROR Direccion: " << error_direccion << endl;
               
    // BYTE 6 - Fallo Bloqueo de diferenciales
    error_bloqueo_diferenciales = StatusMsg.Msg.DATA[6];
    //cout << "ERROR Diferenciales: " << error_bloqueo_diferenciales << endl;
           
    // BYTE 7 - SIN USO

}



// Envío del mensaje 00A (CAN --> autómata --> vehículo)

void ConduccionThread::m_teleop_CAN_AUTOMATA(){
  
    TPCANMsg msgEx;
    msgEx.ID = 0x00A;
    msgEx.LEN = 8;
    msgEx.MSGTYPE = 0;
    
    uint8_t byte_1;

    if ((valor_arranque_parada == 1) && (valor_freno_estacionamiento == 1))
        byte_1 = 17;          
    else {
        byte_1 = 0;
    }
    if (valor_arranque_parada == 1)
        byte_1 = 1;
    if (valor_freno_estacionamiento == 1)
        byte_1 = 16;
    
       
    switch (valor_marcha) {
        case 0: // Marcha H-
            cambio_marcha_tx = 0;
            break;
        case 1: // Marcha N-
            cambio_marcha_tx = 1;
            break;
        case 2: // MArcha R-
            cambio_marcha_tx = 2;
            break;
        case 3: // Marcha N-
            cambio_marcha_tx = 4;
            break;
        case 4: // Marcha L-
            cambio_marcha_tx = 8;
            break;
        default:
            break;
    }
    
    
    if (valor_direccion == 0) {               // direccion = 0
        direccion_tx = 0;            
        sentido_direccion_tx = 0;
    }
    else if (valor_direccion > 0) {        // direccion 1...100
        direccion_tx = abs(valor_direccion);
        sentido_direccion_tx = 1;          // derecha
    }
    else if (valor_direccion < 0) {        // direccion -1...-100
        direccion_tx = abs(valor_direccion);
        sentido_direccion_tx = 16;         // izquierda
    }
    

    if (valor_diferencial == 1) 
        byte_7 = 1;
    else 
        byte_7 = 0;
    
    if (valor_luces == 1){        
        if (valor_diferencial == 1)
            byte_7 = 3;
        else 
            byte_7 = 2;
    }
    
    if (valor_luces_IR == 1){        
        if (valor_luces == 1){
            if (valor_diferencial == 1)
                byte_7 = 7;
            else
                byte_7 = 6;       
        } else {
            if (valor_diferencial == 1)
                byte_7 = 5;
            else
                byte_7 = 4;
        }
    }
    
    if (valor_laser == 1){        
        if (valor_luces_IR == 1){
            if (valor_luces == 1) {
                if (valor_diferencial == 1)
                        byte_7 = 15;
                else
                        byte_7 = 14;       
            }
            else {
                if (valor_diferencial == 1)
                        byte_7 = 13;
                else
                        byte_7 = 12;
            }
        }
        else {
            if (valor_luces == 1) {
                if (valor_diferencial == 1)
                        byte_7 = 11;
                else
                        byte_7 = 10;       
            }
            else {
                if (valor_diferencial == 1)
                        byte_7 = 9;
                else
                        byte_7 = 8;
            }    
        }
    }

    if (valor_parada_emergencia == 1){
        if (valor_laser == 1){        
                if (valor_luces_IR == 1){
                        if (valor_luces == 1) {
                                if (valor_diferencial == 1)
                                        byte_7 = 31;
                                else
                                        byte_7 = 30;       
                        }
                        else {
                                if (valor_diferencial == 1)
                                        byte_7 = 29;
                                else
                                        byte_7 = 28;
                        }
                }
                else {
                       if (valor_luces == 1) {
                                if (valor_diferencial == 1)
                                        byte_7 = 27;
                                else
                                        byte_7 = 26;       
                       }
                       else {
                                if (valor_diferencial == 1)
                                        byte_7 = 25;
                                else
                                        byte_7 = 24;
                       }    
                }
        }
        else{
                if (valor_luces_IR == 1){
                        if (valor_luces == 1) {
                                if (valor_diferencial == 1)
                                        byte_7 = 23;
                                else
                                        byte_7 = 22;       
                        }
                        else {
                                if (valor_diferencial == 1)
                                        byte_7 = 21;
                                else
                                        byte_7 = 20;
                        }
                }
                else {
                       if (valor_luces == 1) {
                                if (valor_diferencial == 1)
                                        byte_7 = 19;
                                else
                                        byte_7 = 18;       
                       }
                       else {
                                if (valor_diferencial == 1)
                                        byte_7 = 17;
                                else
                                        byte_7 = 16;
                       }    
                }
        }
    }

    msgEx.DATA[0] = byte_1;
    msgEx.DATA[1] = (uint8_t) acelerador_tx;
    msgEx.DATA[2] = (uint8_t) velocidad_tx;
    msgEx.DATA[3] = (uint8_t) freno_servicio_tx;
    msgEx.DATA[4] = (uint8_t) cambio_marcha_tx;
    msgEx.DATA[5] = (uint8_t) sentido_direccion_tx;
    msgEx.DATA[6] = (uint8_t) direccion_tx;
    msgEx.DATA[7] = byte_7;
    
        
    CANCONDUCCION->SendMessage(&msgEx);
    
}


void ConduccionThread::m_engine_brake_CAN_AUTOMATA(){
  
    TPCANMsg msgEx;
    msgEx.ID = 0x00A;
    msgEx.LEN = 8;
    msgEx.MSGTYPE = 0;
    
    uint8_t byte_1;
    
    //cout << "valor arranque: " << valor_arranque_parada << endl;
    //cout << "valor freno estacionamiento: " << valor_freno_estacionamiento << endl;
    
    if ((valor_arranque_parada == 0) && (valor_freno_estacionamiento == 0))
        byte_1 = 0; 
    else if ((valor_arranque_parada == 1) && (valor_freno_estacionamiento == 0))
        byte_1 = 1; 
    else if ((valor_arranque_parada == 0) && (valor_freno_estacionamiento == 1))
        byte_1 = 16; 
    else if ((valor_arranque_parada == 1) && (valor_freno_estacionamiento == 1))
        byte_1 = 17;          

    msgEx.DATA[0] = byte_1;
    msgEx.DATA[1] = (uint8_t) acelerador_tx;
    msgEx.DATA[2] = (uint8_t) velocidad_tx;
    msgEx.DATA[3] = (uint8_t) freno_servicio_tx;
    msgEx.DATA[4] = (uint8_t) cambio_marcha_tx;
    msgEx.DATA[5] = (uint8_t) sentido_direccion_tx;
    msgEx.DATA[6] = (uint8_t) direccion_tx;
    msgEx.DATA[7] = byte_7;
    
        
    CANCONDUCCION->SendMessage(&msgEx);
    
}



void ConduccionThread::m_emergency_stop_CAN_AUTOMATA(){
    
    paradaEmergencia = true;
    
    TPCANMsg msgEx;
    msgEx.ID = 0x00A;
    msgEx.LEN = 8;
    msgEx.MSGTYPE = 0;

    uint8_t byte_7;

    if (valor_parada_emergencia == 1)    
        byte_7 = 16;
    else if (valor_parada_emergencia == 0)
        byte_7 = 0;
    
    msgEx.DATA[0] = 0;         
    msgEx.DATA[1] = 0;
    msgEx.DATA[2] = 0;
    msgEx.DATA[3] = 0;
    msgEx.DATA[4] = 0;
    msgEx.DATA[5] = 0;
    msgEx.DATA[6] = 0;
    msgEx.DATA[7] = byte_7;         // Parada de emergencia ON
    
        
    CANCONDUCCION->SendMessage(&msgEx);
    
}

void temporizador() {
    
}