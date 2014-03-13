/* 
 * File:   CANCommunication.hpp
 * Author: Sergio Doctor López
 *
 * Created on 5 de febrero de 2014
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

#define DEFAULT_NODE "/dev/pcan0"

#ifndef bool
  #define bool  int
  #define true  1
  #define false 0
#endif

//****************************************************************************
// GLOBALS





//****************************************************************************
// CLASS

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

    
    // Creación de colas de mensakes
    queue<TPCANRdMsg> ConduccionQueue;


    // Semáforos de acceso a colas
    pthread_mutex_t ConduccionQueue_mutex;
    
    // Semáforos de acceso a datos comunes
    pthread_mutex_t CANVCS_mutex;
    
    bool flagActive;        /* Usada para indicar si debe seguirse ejecutando */
    
    Timer CommunicationTimer;
    Timer CommunicationTimer2;
    
    //Comprueba el número de veces que se deja de escribir/leer
    int cont;

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

