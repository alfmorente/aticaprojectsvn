/* 
 * File:   CANCommunication.cpp
 * Author: Sergio Doctor López
 * 
 * Created on 05 de febrero de 2014
 */

#include "../include/Modulo_Conduccion/CANCommunication.hpp"
#include <libpcan.h>

CANCommunication::CANCommunication(bool bDevNodeGiven, bool bTypeGiven, int nType,
                                   __u32 dwPort, __u16 wIrq, uint32_t bitrate,
                                   bool frame_extended, string id) {

    connected = false;
    flagActive = true;

    this->bDevNodeGiven = bDevNodeGiven;
    this->bTypeGiven = bTypeGiven;
    this->nType = nType;
    this->dwPort = dwPort;
    this->wIrq = wIrq;
    this->bitrate = bitrate;
    this->frame_extended = frame_extended;
    this->id = id;
    
}

CANCommunication::~CANCommunication() {
    CloseCommunication(h);
}

bool CANCommunication::EstablishCommunication() {

    bool res = false;
    if ((bDevNodeGiven) || (!bDevNodeGiven && !bTypeGiven)) 
    {
       
      h = LINUX_CAN_Open(DEFAULT_NODE, O_RDWR);     
      if (!h)
      {	
         
        cout << "Error can't open " << DEFAULT_NODE << " \n";          
      }
      else {
        cout << "LINUX Can Open() returned OK !\n";
        res = true;
        connected = true;
      }
    }
    else 
    {     
      // please use what is appropriate: HW_DONGLE_SJA, HW_DONGLE_SJA_EPP, 
      //                                 HW_ISA_SJA, HW_PCI y HW_USB
      
        h = CAN_Open(nType, dwPort, wIrq);
      if (!h)
      {
        
	cout << "Error can't open %s device \n";
      }
      else {
        cout << "Can Open() returned OK !\n";
        res = true;
        connected = true;
      }
    }
    
    return res;
    
}
    

bool CANCommunication::ConfigureCommunication() {

    bool res = false;
    int type;
    
    if (frame_extended)
        type = CAN_INIT_TYPE_EX;
    else
        type = CAN_INIT_TYPE_ST;

    errno_can = CAN_Init(h, bitrate, type);
    if (errno_can == CAN_ERR_OK) // errno_configure = 0;
      {
        cout << "Configuration returned OK !\n";
        res = true;        
      }
    
    else{  //errno_configure = -1
        //Error que impide abrir una comunicación CAN necesaria
        
	cout << "CAN_Init() failed with error: The bitrate " << bitrate << " is not the correct! \n";
	CAN_Close(h); 
    }

    return res;


}

bool CANCommunication::CloseCommunication(HANDLE h) {
    
    bool res = false;

    errno_can = CAN_Close(h);

    if (errno_can != CAN_ERR_OK){ // CAN_ERR_OK = 0
        cout << "canClose() failed with error !\n";
        
    }
    else{ 
        cout << "Can Close() returned OK !\n";
        res = true;
    }

    return res;
}


bool CANCommunication::SendMessage(TPCANMsg* msgTx) {

    bool res = false;
    
    errno_can = CAN_Write(h, msgTx);
    if (errno_can != CAN_ERR_OK){
        cout << "canWrite() failed() with error " << errno_can << "!\n";
        cont++;  // Contador de tramas fail. Se usa en ConduccionThread para comprobar que la comunicacion no se ha caido
    }        
    else {
        //cout << "function canWrite() returned OK !\n";
        res = true;
        cont = 0;  // Se resetea el contador de tramas. Se usa en ConduccionThread para comprobar que la comunicacion no se ha caido
    }

    return res;
}

int32_t CANCommunication::ReceiveMessage(TPCANRdMsg* msgRx) {

    errno_can = LINUX_CAN_Read(h, msgRx);
    if (errno_can != CAN_ERR_OK) {
         cout << "canRead() failed with error " << errno_can << "!\n";
         cont++;
    } else {
        //cout << "function canRead() returned OK !\n";
        cont = 0;
    }
    
    return errno_can;

}


void CANCommunication::DoWork() {

    this->CommunicationTimer.Reset();
          
    while (flagActive) {
        
        i = ReceiveMessage(&msgRx);
        
        if (i==0) {   
            
            if (this->id == "Conduccion"){
                
                //printf("receivetest: 0x%08x   ", msgRx.Msg.ID); 
      
                if (!(msgRx.Msg.MSGTYPE & MSGTYPE_RTR)) {
                    
                    
                    switch (msgRx.Msg.ID) {
                        case 0x5:
                        case 0xE:
                        case 0x80:
                                pthread_mutex_lock (&ConduccionQueue_mutex);                  
                                ConduccionQueue.push (msgRx);
                                pthread_mutex_unlock (&ConduccionQueue_mutex);
                                this->CommunicationTimer.Reset(); // Se resetea cada vez que se reciba 1 mensaje   
                    
                    //for (v = 0; v < msgRx.Msg.LEN; v++)
                    //            printf("%02x ", msgRx.Msg.DATA[v]);
                    //printf("\n");
                                break;
                            
                        default:
                            cout << "\n Error en lectura ALT \n";
                
                    }
                }
            }
            else 
                cout << "\n Error en lectura en hilo de conducción\n";
        }
    }
}