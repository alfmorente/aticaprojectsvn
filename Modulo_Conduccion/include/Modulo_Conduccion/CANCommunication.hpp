/** 
 * @file  CANCommunication.hpp
 * @brief Declara el tipo de la clase "CANCommunication"
 * - La clase implementa la escritura/lectura de los mensajes CAN del Subsistema Driving
 * @author Sergio Doctor 
 * @date 2014
 * @addtogroup 
 * @{
 */

#ifndef CANCOMMUNICATION_HPP
#define	CANCOMMUNICATION_HPP

#include <libpcan.h>
#include <queue>
#include <errno.h>
#include <unistd.h>   // exit
#include <signal.h>
#include <stdio.h>
#include <string>
#include <iostream>
#include <stdlib.h>
#include <sstream>
#include <fcntl.h>    // O_RDWR
#include <stdint.h>

#include "Thread.hpp"
#include "Timer.hpp"

using namespace std;


//****************************************************************************
// DEFINES

#define DEFAULT_NODE "/dev/pcan0"  ///<Nombre del nodo en el que conectado el dispositivo CAN
#define ERROR_WRITE_FRAME 1000     ///<Número de tramas de escrituras erroneas a partir de las cuales saltaria un error de escritura
#define ERROR_READ_FRAME 1000      ///<Número de tramas de lectura erroneas a partir de las cuales saltaria un error de lectura



//****************************************************************************
// CLASS

/**
 * \class CANCommunication
 * \brief Clase que representa las comunicaciones CAN del subsistema driving
 */

class CANCommunication: public Thread{
    
public:
    CANCommunication(bool bDevNodeGiven, bool bTypeGiven, int nType,
                     __u32 dwPort, __u16 wIrq, uint32_t bitrate,
                     bool frame_extended, string id);
    virtual ~CANCommunication();
    virtual bool EstablishCommunication();
    virtual bool ConfigureCommunication ();
    bool CloseCommunication (HANDLE h);
    
    virtual bool SendMessage(TPCANMsg* msg);
    virtual int32_t ReceiveMessage(TPCANRdMsg* msgRx);  
     
    virtual void DoWork();
    
    void checkErrorWrite (int contWrite);
    void checkErrorRead (int contRead);

    
    // Creación de colas de mensajes para Ática y Camión
    queue<TPCANRdMsg> ConduccionQueue;
    queue<TPCANRdMsg> ConduccionCamionQueue;
    
    // Semáforos de acceso a colas para Ática y Camión
    pthread_mutex_t ConduccionQueue_mutex;
    pthread_mutex_t ConduccionCamionQueue_mutex;
    
    bool flagActive;        /* Usada para indicar si debe seguirse ejecutando */
    
    Timer CommunicationTimer;
    Timer CommunicationTimer2;
    
    //Comprueba el número de veces que se deja de escribir/leer
    int contWrite;
    int contRead;
    bool inicio_read_write_CAN_frame;
    
    bool errorWrite; // flag que controla si se ha producido error de escritura
    bool errorRead;  // flag que contrala si se ha producido error de lectura
    
private:
    int idBus;
        
    // Variables necesarias para la apertura del puerto CAN
    bool connected;
    bool bDevNodeGiven;                       // Value of the node --> Value = false;
    bool bTypeGiven;                          // Value of the type --> Value = false;
    int nType;                                // Can Configuration: CAN Type = HW_PCI
    __u32 dwPort;                             // Can Configuration: CAN Port --> Value = 0;
    __u16 wIrq;                               // Can Configuration: Irq --> Value = 0;
    int32_t lenRx;                            // # of CAN messages Rx
    int32_t lenTx;                            // # of CAN messages Tx
    uint32_t bitrate;                         // Configured CAN baudrate 
    bool frame_extended;                      // Frame Type: Standart o Extended
    string id; 
  
    // Errores en la comunicacion can
    int errno_can;
    
    
    //Mensajes
    TPCANRdMsg msgRx;                          // can Rx message buffer 
    TPCANMsg msgTx;                            // can Tx message buffer 
    
    HANDLE h;
   
    int i;                  /* loop counter */
    bool bDisplayOn;
    
    
    
};


#endif	/* CANCOMMUNICATION_HPP */

