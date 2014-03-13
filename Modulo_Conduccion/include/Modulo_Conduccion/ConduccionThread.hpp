/* 
 * File:   ConduccionThread.hpp
 * Author: Sergio Doctor López
 *
 * Created on 6 de febrero de 2014
 */

#ifndef CONDUCCIONTHREAD_HPP
#define	CONDUCCIONTHREAD_HPP


// MASCARAS DE LAS TRAMAS DE INFO

        // BYTE 0
        #define ARRANQUE_PARADA 0x01                     // En binario (0000 0001)
        #define FRENO_ESTACIONAMIENTO 0x02              // En binario (0000 0010)
        
        // BYTE 4
        #define MARCHA_H_ 0x00                          // En binario (0000 0000)
        #define MARCHA_N_ 0x01                          // En binario (0000 0001)
        #define MARCHA_R_ 0x02                          // En binario (0000 0010)
        #define MARCHA_N1_ 0x04                         // En binario (0000 0100)
        #define MARCHA_L_ 0x08                          // En binario (0000 1000)

        // BYTE 7
        #define CONF_PARADA_EMERGENCIA 0x01             // En binario (0000 0001)
        #define PARADA_EMERGENCIA_OBSTACULO 0x02        // En binario (0000 0010)   ->>>> Señal de activación/desactivacion del laser
        #define PARADA_EMERGENCIA_REMOTA 0x04           // En binario (0000 0100)   ->>>> Señal de parada por la seta remota
        #define PARADA_EMERGENCIA_LOCAL 0x08            // En binario (0000 1000)   ->>>> Señal de parada por la seta del vehiculo

// FIN MASCARAS PARA LAS TRAMAS INFO

#include "Thread.hpp"
#include "CANCommunication.hpp"
#include "Timer.hpp"
#include <queue>

class ConduccionThread : public Thread {
    
public:
    ConduccionThread(CANCommunication * canCOND);
    virtual ~ConduccionThread();
    virtual void DoWork();
    
    void m_Change_Command_CAN_AUTOMATA();
    
    void m_Status_Message_AUTOMATA_CAN(TPCANRdMsg StatusMsg);
    void m_Error_Message_AUTOMATA_CAN(TPCANRdMsg StatusMsg);
    
    // Semáforos de acceso a datos comunes
    pthread_mutex_t ConduccionThread_mutex;
    
    //Timer time1;
    
    bool CONDUCCION_ACTIVE; // Flag de estado
    
    // Atributos para el mensaje SET (00A)
    short valor_arranque_parada;
    short valor_freno_estacionamiento;
    short acelerador_tx;
    short velocidad_tx;
    short freno_servicio_tx;
    short valor_marcha;
    short cambio_marcha_tx;
    short sentido_direccion_tx;
    short direccion_tx;
    short valor_direccion;
    short valor_luces;
    short valor_luces_IR;
    short valor_diferencial;
    short valor_laser;
    short valor_parada_emergencia;
    
    // Atributos para el mensaje INFO (005)
    short arranque_parada;
    short freno_estacionamiento;
    short conmutador_m_a;
    short acelerador;
    short velocidad;
    short freno_servicio;
    short marcha_actual;
    short sentido_marcha;
    short direccion;
    short conf_parada_emergencia;
    short parada_emergencia_obstaculo;
    short parada_emergencia_remota;
    short parada_emergencia_local;
    
    // Atributos para el mensaje de ERROR (00E)
    short error_arranque_parada;
    short error_acelerador;
    short error_freno_estacionamiento;
    short error_freno_servicio;
    short error_cambio_marcha;
    short error_direccion;
    short error_bloqueo_diferenciales;
    
    //atributo de error
    int id_error_Conduccion; 
    
private:
    
    CANCommunication * CANCONDUCCION;
    
    void CheckConnection();
    
};

#endif	/* CONDUCCIONTHREAD_HPP */

