/** 
 * @file  ConduccionThread.hpp
 * @brief Declara el tipo de la clase "ConduccionThread"
 * - La clase implementa el tratamiento de los mensajes CAN que se reciben/envían del Subsistema Driving
 * @author Sergio Doctor 
 * @date 2014
 * @addtogroup 
 * @{
 */

#ifndef CONDUCCIONTHREAD_HPP
#define	CONDUCCIONTHREAD_HPP


// MASCARAS DE LAS TRAMAS DE INFO

// BYTE 0
#define ARRANQUE_PARADA 0x01                 ///<Valor del arranque del vehículo        // En binario (0000 0001)
#define FRENO_ESTACIONAMIENTO 0x02           ///<Valor del freno de estacionamiento     // En binario (0000 0010)

// BYTE 2
#define MAN 0x00                             ///<Valor del freno de estacionamiento     // En binario (0000 0000)
#define AUTO 0x01                            ///<Valor del freno de estacionamiento     // En binario (0000 0001)
#define MARCHA_H_ 0x08                       ///<Valor de la Marcha H                   // En binario (0000 1000)
#define MARCHA_N_ 0x10                       ///<Valor de la Marcha N                   // En binario (0001 0000)
#define MARCHA_R_ 0x20                       ///<Valor de la Marcha R                   // En binario (0010 0000)
#define MARCHA_N1_ 0x40                      ///<Valor de la Marcha N1                  // En binario (0100 0000)
#define MARCHA_L_ 0x80                       ///<Valor de la Marcha L                   // En binario (1000 0000)

// BYTE 7
#define CONF_PARADA_EMERGENCIA 0x01          ///<Valor de la confirmación de la parada de emergencia                        // En binario (0000 0001)
#define PARADA_EMERGENCIA_OBSTACULO 0x02     ///<Valor de la señal de activación/desactivación del láser                    // En binario (0000 0010)   ->>>> Señal de activación/desactivacion del laser
#define PARADA_EMERGENCIA_REMOTA 0x04        ///<Valor de la señal de parada por activación de la seta remota               // En binario (0000 0100)   ->>>> Señal de parada por la seta remota
#define PARADA_EMERGENCIA_LOCAL 0x08         ///<Valor de la señal de parada por activación de la seta del vehículo         // En binario (0000 1000)   ->>>> Señal de parada por la seta del vehiculo

// FIN MASCARAS PARA LAS TRAMAS INFO

#include "Thread.hpp"
#include "CANCommunication.hpp"
#include "Timer.hpp"
#include <queue>


/**
 * \class ConduccionThread
 * \brief Clase que representa el tratamiento de los menajes CAN que se le envía/recibe el vehículo
 */

class ConduccionThread : public Thread {
    
public:
    ConduccionThread(CANCommunication * canCOND);
    virtual ~ConduccionThread();
    virtual void DoWork();
    
    void m_teleop_CAN_AUTOMATA();               // Escritura en el CAN cuando le llega un mensaje msg_com_teleop
    void m_engine_brake_CAN_AUTOMATA();         // Escritura en el CAN cuando le llega un mensaje msg_fcn_aux
    void m_emergency_stop_CAN_AUTOMATA();       // Escritura en el CAN cuando le llega un mensaje msg_emergency_stop
    void envio_trama_reinicio_CAN_AUTOMATA ();
    void inicializacion_valores_tx();
    
    void m_Status_Message_AUTOMATA_CAN(TPCANRdMsg StatusMsg);
    void m_Error_Message_AUTOMATA_CAN(TPCANRdMsg StatusMsg);
    
    // Semáforos de acceso a datos comunes
    pthread_mutex_t ConduccionThread_mutex;
    
    //Timer time1;
    
    bool CONDUCCION_ACTIVE; // Flag de estado
    bool flag_active_backup;

    
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
    uint8_t byte_7;
    
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
           
    bool paradaEmergencia;      // Flag para la parada de emergencia
    int tipo_parada;            // Flag que indica el tipo de parada producida dentro de 
    int tipo_parada_emergencia;     // Flag que indica el tipo de parada producida
    
    //Timer para esperar X segundos a que haga la parada de emergencia y volver a tener posibiildad de control del vehiculo
    Timer t;
    
private:
    
    CANCommunication * CANCONDUCCION;
   

    
};

#endif	/* CONDUCCIONTHREAD_HPP */

