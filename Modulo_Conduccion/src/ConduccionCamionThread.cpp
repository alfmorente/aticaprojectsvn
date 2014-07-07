/* 
 * File:   ConduccionCamionThread.cpp
 * Author: Sergio Doctor López
 *
 * Created on 6 de febrero de 2014
 */

#include <queue>
#include <iostream>
#include "../include/Modulo_Conduccion/ConduccionCamionThread.hpp"

  int i;
  int valor;
  int j;
  WORD c;
  BYTE a;
  BYTE b;
  BYTE c_x;
  BYTE d;
  int id0 = 418381772;
  int id1 = 217055747;
  int id2 = 217056256;
  int id3 = 418383107;
  int id4 = 418382091;
  int id5 = 419361024;
  int id6 = 419406113;
  int id7 = 217056000;
  int id8 = 419360256;
  int id9 = 419360512;
  int id10 = 419343920;
  int id11 = 419362048;
  int id12 = 419348974;
  int id13 = 419359243;
  int id14 = 419357991;
  int id15 = 218000622;
  int id16 = 418381865;
  int id17 = 419355133;
  int id18 = 419361319;
  int id19 = 486459139;

  
  unsigned char bits[7];
  unsigned char mask = 1;


ConduccionCamionThread::ConduccionCamionThread(CANCommunication * canCOND) {
    CONDUCCION_CAMION_ACTIVE = true; 
    CAN_CAMION_CONDUCCION = canCOND;
}

ConduccionCamionThread::~ConduccionCamionThread() {
}

void ConduccionCamionThread::DoWork(){

    //int QueueSize;
    TPCANRdMsg MsgAux;

    while (CONDUCCION_CAMION_ACTIVE) {

        if (!CAN_CAMION_CONDUCCION->ConduccionQueue.empty()) {
        // RX de mensajes
        //pthread_mutex_lock (&CANCONDUCCION->ConduccionQueue_mutex);
        //QueueSize = CANCONDUCCION->ConduccionQueue.size();
        //pthread_mutex_unlock (&CANCONDUCCION->ConduccionQueue_mutex);

        //for (int i=0;i<QueueSize; i++) {        
        
            pthread_mutex_lock (&CAN_CAMION_CONDUCCION->ConduccionCamionQueue_mutex);
            MsgAux=CAN_CAMION_CONDUCCION->ConduccionCamionQueue.front();
            pthread_mutex_unlock (&CAN_CAMION_CONDUCCION->ConduccionCamionQueue_mutex);
            //cout << "Desencolado mensaje con ID = " << hex << MsgAux.Msg.ID << "\n";
            switch (MsgAux.Msg.ID){
                case 0x18EFFFCC:
                case 0xCF00203:
                case 0xCF00400:
                case 0x18F00503:
                case 0x18F0010B:
                case 0x18FEF100:
                case 0x18FFA121:
                case 0xCF00300:
                case 0x18FEEE00:
                case 0x18FEEF00:
                case 0x18FEAE30:
                case 0x18FEF500:
                case 0x18FEC1EE:
                case 0x18FEEA0B:
                case 0x18FEE527:
                case 0xCFE6CEE:
                case 0x18F00029:
                case 0x18FED9FD:
                case 0x18FEF227:
                case 0x1CFEC703:
                    //cout << "\n MENSAJE DE ERROR DEL VEHÍCULO \n";
                    print_message(MsgAux);
                    CAN_CAMION_CONDUCCION->ConduccionQueue.pop();
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
      

void ConduccionCamionThread::print_message(TPCANRdMsg m) {


  ksm.signal1 = 'N';
  ksm.signal2 = 'N';
  ksm.signal3 = 'N';
  ksm.signal4 = 'N';
  ksm.signal5 = 'N';
  ksm.signal6 = 'N';
  ksm.signal7 = 'N';
  ksm.signal8 = 'N';
  ksm.signal9 = 'N';
  ksm.signal10 = 'N';
  ksm.signal11 = 'N';
  ksm.signal12 = 'N';
  ksm.signal13 = 'N';
  ksm.signal14 = 'N';
  ksm.signal15 = 'N';
  ksm.signal16 = 'N';
  ksm.signal17 = 'N';
  ksm.signal18 = 'N';
  ksm.signal19 = 'N';
  
  int idmsg = m.Msg.ID;


// don't print any telegram contents for remote frames 
 if (!(m.Msg.MSGTYPE & MSGTYPE_RTR)){      

/*
	if (idmsg == id1) {   //Electronic Transmission Controller #1 - CF00203

	  ksm.signal1 = 'S';

      	
	// DriverLine Engaged (Estado de la cadena cinética)
	  for (j = 0; j<8; j++) {
	    bits[7-j] = (m.Msg.DATA[0] >> 7-j) & mask ;// & (mask << 1);
	  }
	  valor = bits[1]*2 + bits[0]*1;
	  ksm.elecTransController1.opcion = opciones2(valor);

    
	// Output Speed TCU (Revoluciones de salida)
	  a = m.Msg.DATA[1];
	  b = m.Msg.DATA[2];
	  Lo(c) = a;
	  Hi(c) = b;
	  valor = c;
	  ksm.elecTransController1.revoluciones = calcularRPM (valor);

	  
	// Clutch Slip (Posicion del pedal del embrague)
	  a = m.Msg.DATA[3];
	  valor = a;
	  ksm.elecTransController1.porcentaje = calcularPorcentaje (valor);
	
	  
	// Input Speed (Velocidad de entrada)
	  a = m.Msg.DATA[5];
	  b = m.Msg.DATA[6];  
	  Lo(c) = a;
	  Hi(c) = b;
	  valor = c;
	  ksm.elecTransController1.revoluciones2 = calcularRPM (valor);

    }
*/

    if (idmsg == id2) {   // Electonic Engine Controller #1 - CF00400

	  ksm.signal2 = 'S';

	// Actual Engine Torque (Par motor/caudal inyectado) 
	  a = m.Msg.DATA[2];
	  valor = a;
	  ksm.elecEngineController1.porcentaje = calcularPorcentaje3  (valor);

	
	// Engine Speed (Revoluciones del motor)
	  a = m.Msg.DATA[3];
	  b = m.Msg.DATA[4];
	  Lo(c) = a;
	  Hi(c) = b;
	  valor = c;
	  ksm.elecEngineController1.revoluciones = calcularRPM (valor);

    } 
   
    else if (idmsg == id3) {   // Electronic Transmission Controller #2 - 18F00503
	  
	  ksm.signal3 = 'S';

    // Select Gear (Marcha seleccionada)
	  a = m.Msg.DATA[0];
	  valor = a;
  	  ksm.elecTransController2.marcha = calcularMarcha (valor);  

	
	// Actual Gear Ratio (Relación entrada a revoluciones de salida de la caja)
	  a = m.Msg.DATA[1];
	  b = m.Msg.DATA[2];
	  Lo(c) = a;
	  Hi(c) = b;
	  valor = c;
	  ksm.elecTransController2.marcha2 = calcularMarcha2 (valor);

		  
	// Current Gear (actual/Última marcha)
	  a = m.Msg.DATA[3];
	  valor = a;
	  ksm.elecTransController2.marcha3 = calcularMarcha (valor);

    } 


    else if (idmsg == id4) {   // Electronic Brake Controller - 18F0010B
	 
	  ksm.signal4 = 'S';

   	// ABS Active
	  for (j = 0; j<8; j++) {
	    bits[7-j] = (m.Msg.DATA[0] >> 7-j) & mask ;// & (mask << 1);
	  }
	  valor = bits[5]*2 + bits[4]*1;
	  ksm.elecBrakeController1.opcion = opciones(valor);	

    
    // Break Pedal (Posición pedal de freno)
	  a = m.Msg.DATA[1];
	  valor = a;
      ksm.elecBrakeController1.porcentaje = calcularPorcentaje (valor);

    } 


    else if (idmsg == id5) {   //Cruise Control Vehicule Speed - 18FEF100
	  
	  ksm.signal5 = 'S';

	// Parking Brake Switch (Accionamiento freno de estacionamiento)
	  for (j = 0; j<8; j++) {
	    bits[7-j] = (m.Msg.DATA[0] >> 7-j) & mask ;// & (mask << 1);
	  }	  
	  valor = bits[3]*2 + bits[2]*1;
	  ksm.cruiseControlVehiSpeed.opcion = opciones(valor);	
	  
      
	// Vehicule Speed FFR (Velocidad del vehículo) 
	  a = m.Msg.DATA[1];
	  b = m.Msg.DATA[2];
	  Lo(c) = a;
	  Hi(c) = b;
	  valor = c;
	  ksm.cruiseControlVehiSpeed.km_h = calcularKM_H (valor);	


	// Cruise Control Active - Brake switch (Accionamiento pedal del freno) - Clutch switch (Accionamiento pedal del embrague)
	  for (j = 0; j<8; j++) {
	    bits[7-j] = (m.Msg.DATA[3] >> 7-j) & mask ;// & (mask << 1);
	  }

  
	  // Cruise Control Active
	  valor = bits[1]*2 + bits[0]*1;
	  ksm.cruiseControlVehiSpeed.opcion2 = opciones(valor);

      
	  // Brake switch (Accionamiento pedal del freno)
	  int valor2 = bits[5]*2 + bits[4]*1;
	  ksm.cruiseControlVehiSpeed.opcion3 = opciones(valor2);


	  // Clutch switch (Accionamiento pedal del embrague)
	  int valor3 = bits[7]*2 + bits[6]*1;
	  ksm.cruiseControlVehiSpeed.opcion4 = opciones(valor3);


	// PTO State
	  for (j = 0; j<8; j++) {
	    bits[7-j] = (m.Msg.DATA[6] >> 7-j) & mask ;// & (mask << 1);
	  }
	  
	  valor = bits[4]*16 + bits[3]*8 + bits[2]*4 + bits[1]*2 + bits[0]*1;
	  ksm.cruiseControlVehiSpeed.pto_state = PTOState(valor);

    } 

/*
    else if (idmsg == id6) {   // Auxiliary State I/O Body Controller - 18FFA121
	  
	  ksm.signal6 = 'S';

	// Información nivel de combustible en tanque actual demasiado baja - Marcha atrás metida
	  for (j = 0; j<8; j++) {
	    bits[7-j] = (m.Msg.DATA[0] >> 7-j) & mask ;// & (mask << 1);
	  }

	  
	  // Información nivel de combustible en tanque actual demasiado baja
	  valor = bits[5]*2 + bits[4]*1;
	  ksm.auxiliaryState.opcion = opciones(valor);

      
	  // Marcha atrás metida
	  int valor2 = bits[7]*2 + bits[6]*1;
	  ksm.auxiliaryState.opcion2 = opciones(valor2);


	// Info Not-Aus
	  for (j = 0; j<8; j++) {
	    bits[7-j] = (m.Msg.DATA[1] >> 7-j) & mask ;// & (mask << 1);
	  }
	  valor = bits[1]*2 + bits[0]*1;
	  ksm.auxiliaryState.opcion3 = opciones(valor);

    }
*/

    else if (idmsg == id7) {   //Electronic Engine Controller #2 - CF00300
	  
	  ksm.signal7 = 'S';

	// Idle Position (Posición del relantí) - Accelerator Pedal
	  for (j = 0; j<8; j++) {
	    bits[7-j] = (m.Msg.DATA[0] >> 7-j) & mask ;// & (mask << 1);
	  }
	  
	  // Idle Position (Posición del relantí) - Accelerator Pedal
	  valor = bits[1]*2 + bits[0]*1;
	  ksm.elecEngineController2.opcion = opciones(valor);

      
	  // Accelerator Pedal
	  int valor2 = bits[3]*2 + bits[2]*1;
	  ksm.elecEngineController2.opcion2 = opciones(valor2);


	// Posición del pedal del acelerador
	  a = m.Msg.DATA[1];
	  valor = a;
	  ksm.elecEngineController2.porcentaje = calcularPorcentaje (valor);

	
	// Load at current speed
	  a = m.Msg.DATA[2];
	  valor = a;
	  ksm.elecEngineController2.porcentaje2 = calcularPorcentaje2 (valor);

    }  

    else if (idmsg == id8) {   //Engine Temperature - 18FEE00
	  
	  ksm.signal8 = 'S';

	// Temperatura agua refrigerante
	  a = m.Msg.DATA[0];
	  valor = a;
	  ksm.engineTemperature.grados = calcularGrados (valor);

	
	// Temperatura combustible 
	  a = m.Msg.DATA[1];
	  valor = a;
      ksm.engineTemperature.grados2 = calcularGrados (valor);


	// Temperatura aceite
	  a = m.Msg.DATA[2];
	  b = m.Msg.DATA[3];
	  Lo(c) = a;
	  Hi(c) = b;
	  valor = c;
	  ksm.engineTemperature.grados3 = calcularGrados2 (valor);

    }

  
    else if (idmsg == id9) {   //Engine Fluid Level/Pressure - 18FEEF00
	  
	  ksm.signal9 = 'S';
	
	// Presión aceite 
	  a = m.Msg.DATA[3];
	  valor = a;
	  ksm.engineFluid.bares = calcularBares (valor);

    }
   

    else if (idmsg == id10) {   //Suppy Pressure - 18FEAE30
	  
	  ksm.signal10 = 'S';
	
	// Presión neumática  
	  a = m.Msg.DATA[0];
	  valor = a;
	  ksm.supplyPressure.bares = calcularBares2 (valor);


	// Parking and/or trailer air pressure
	  a = m.Msg.DATA[1];
	  valor = a;
	  ksm.supplyPressure.bares2 = calcularBares2 (valor);


	// Presión del circuito #1 del freno de servicio 
	  a = m.Msg.DATA[2];
	  valor = a;
	  ksm.supplyPressure.bares3 = calcularBares2 (valor);


	// Presión del circuito #2 del freno de servicio 
	  a = m.Msg.DATA[3];
	  valor = a;
	  ksm.supplyPressure.bares4 = calcularBares2 (valor);


	// Presión del equipo auxiliar
	  a = m.Msg.DATA[4];
	  valor = a;
	  ksm.supplyPressure.bares5 = calcularBares2 (valor);


	// Presión de la suspensión 
	  a = m.Msg.DATA[5];
	  valor = a;
	  ksm.supplyPressure.bares6 = calcularBares2 (valor);	  

    }

/*
    else if (idmsg == id11) {   //Ambient Conditions - 18FEF500
	  
	  ksm.signal11 = 'S';

	// Presión barométrica
	  a = m.Msg.DATA[0];
	  valor = a;
	  ksm.ambientConditions.bares = calcularBares3 (valor);


	// Temperatura ambiente
	  a = m.Msg.DATA[3];
	  b = m.Msg.DATA[4];
	  Lo(c) = a;
	  Hi(c) = b;
	  valor = c;
	  ksm.ambientConditions.grados = calcularGrados2 (valor);

    }


    else if (idmsg == id12) {   //Vehicule Distance High Resolution - 18FEC1EE

	  ksm.signal12 = 'S';

	// Total kilómetros
  	  a = m.Msg.DATA[0];
	  b = m.Msg.DATA[1];
      c_x = m.Msg.DATA[2];
      d = m.Msg.DATA[3];
	  Lo(c) = a;
	  Hi(c) = b;
	  Higher(c) = c_x;
	  Highest(c) = d;
	  valor = c;
	  ksm.vehiculeDistance.kilometros = calcularKM(valor);
	 
    }

    
    else if (idmsg == id13) {   //Vehicule Weight EBS - 18FEEA0B
          
	  ksm.signal13 = 'S';

	// Localización de los ejes
	  for (j = 0; j<8; j++) {
	    bits[7-j] = (m.Msg.DATA[0] >> 7-j) & mask ;// & (mask << 1);
	  } 
	  valor = bits[7]*8 + bits[6]*4 + bits[5]*2 + bits[4]*1;
	  ksm.vehiculeWeight.axle = axleLocation(valor);

      
	// Peso de los ejes
	  a = m.Msg.DATA[1];
	  b = m.Msg.DATA[2];
	  Lo(c) = a;
	  Hi(c) = b;
	  valor = c;
	  ksm.vehiculeWeight.kilogramos = calcularKG (valor);

    }


    else if (idmsg == id14) {   //Engine Hours revolutions - 18FEE527

	  ksm.signal14 = 'S';

	// Total horas de motor
  	  a = m.Msg.DATA[0];
	  b = m.Msg.DATA[1];
      c_x = m.Msg.DATA[2];
      d = m.Msg.DATA[3];
	  Lo(c) = a;
	  Hi(c) = b;
	  Higher(c) = c_x;
	  Highest(c) = d;
	  valor = c;
	  ksm.engineHours.horas = calcularHoras(valor);

    }


    else if (idmsg == id15) {   //Tacógrafo - CFE6CEE

	  ksm.signal15 = 'S';

	// Driver 1 Working status - Driver 2 Working status - Driver recognition
	  for (j = 0; j<8; j++) {
	    bits[7-j] = (m.Msg.DATA[0] >> 7-j) & mask ;// & (mask << 1);
	  }
	  
	  // Driver 1 Working status
	  valor = bits[2]*4 +bits[1]*2 + bits[0]*1;
	  ksm.tacograph.driverWorking1 = driverWorking(valor);


	  // Driver 2 Working status
	  valor = bits[5]*4 + bits[4]*2 + bits[3]*1;
	  ksm.tacograph.driverWorking2 = driverWorking(valor);


	  // Driver recognition
	  valor = bits[7]*2 + bits[6]*1;
	  ksm.tacograph.opcion = opciones(valor);


	// Driver 1 time related status - Driver Card 1 - Overspeed
	  for (j = 0; j<8; j++) {
	    bits[7-j] = (m.Msg.DATA[1] >> 7-j) & mask ;// & (mask << 1);
	  }
	  
	  // Driver 1 time related status
	  valor = bits[3]*8 + bits[2]*4 +bits[1]*2 + bits[0]*1;
	  ksm.tacograph.driverTime = driverTime(valor);


	  // Driver Card 1
	  valor = bits[5]*2 + bits[4]*1;
	  ksm.tacograph.opcion2 = opciones(valor);


	  // Overspeed
	  valor = bits[7]*2 + bits[6]*1;
	  ksm.tacograph.opcion3 = opciones(valor);


	// Driver 2 time related status - Driver Card 2 - Overspeed
	  for (j = 0; j<8; j++) {
	    bits[7-j] = (m.Msg.DATA[2] >> 7-j) & mask ;// & (mask << 1);
	  }
	  
	  // Driver 2 time related status
	  valor = bits[3]*8 + bits[2]*4 +bits[1]*2 + bits[0]*1;
   	  ksm.tacograph.driverTime2 = driverTime(valor);


	  // Driver Card 1
	  valor = bits[5]*2 + bits[4]*1;
	  ksm.tacograph.opcion4 = opciones(valor);


	// System Event - Handling Information - System Performance
	  for (j = 0; j<8; j++) {
	    bits[7-j] = (m.Msg.DATA[3] >> 7-j) & mask ;// & (mask << 1);
	  }
	  
	  // System Event
	  valor = bits[1]*2 + bits[0]*1;
	  ksm.tacograph.opcion5 = opciones(valor);
	 

	  // Handling Information
	  valor = bits[3]*2 + bits[2]*1;
	  ksm.tacograph.opcion6 = opciones(valor);


	  // System Performance
	  valor = bits[5]*2 + bits[4]*1;
	  ksm.tacograph.opcion7 = opciones(valor);


	// Tacograph vehicle Speed
	  a = m.Msg.DATA[6];
	  b = m.Msg.DATA[7];
	  Lo(c) = a;
	  Hi(c) = b;
	  valor = c;
	  ksm.tacograph.km_h = calcularKM_H(valor);

    }


    else if (idmsg == id16) {   //Electronic Retarder Controller Exhaust - 18F00029

	  ksm.signal16 = 'S';

	// Actual retarder torque
	  a = m.Msg.DATA[1];
	  valor = a;
	  ksm.elecRetarder.porcentaje = calcularPorcentaje4 (valor);
	
    }


    else if (idmsg == id17) {   //Aux Stat KSM1 - 18FED9FD

	  ksm.signal17 = 'S';
	
	// Aviso temperatura excesiva del agua refrigerante - Aviso presión del aceite
	  for (j = 0; j<8; j++) {
	    bits[7-j] = (m.Msg.DATA[0] >> 7-j) & mask ;// & (mask << 1);
	  }
	  
	  // Aviso temperatura excesiva del agua refrigerante
	  valor = bits[1]*2 + bits[0]*1;
	  ksm.auxStat.opcion = opciones(valor);


	  // Aviso presión del aceite
	  valor = bits[3]*2 + bits[2]*1;
	  ksm.auxStat.opcion2 = opciones(valor);

    }

   
    else if (idmsg == id18) {   //Fuel Economy - 18FEF227

	  ksm.signal18 = 'S';

	// Fuel Rate
	  a = m.Msg.DATA[0];
	  b = m.Msg.DATA[1];
	  Lo(c) = a;
	  Hi(c) = b;
	  valor = c;
	  ksm.fuelEconomy.litros = calcularLitros (valor);


	// Instantaneous fuel economy
	  a = m.Msg.DATA[2];
	  b = m.Msg.DATA[3];
	  Lo(c) = a;
	  Hi(c) = b;
	  valor = c;
	  ksm.fuelEconomy.km_l = calcularKM_L (valor);

    }

*/
    else if (idmsg == id19) {   //Electronic Transmission Controller #3 - 1CFEC703
	
	ksm.signal19 = 'S';
	
	// Posición neutral de la caja - Marcha metida
	  for (j = 0; j<8; j++) {
	    bits[7-j] = (m.Msg.DATA[2] >> 7-j) & mask ;// & (mask << 1);
	  }
	  
	  // Posición neutral de la caja
	  valor = bits[1]*2 + bits[0]*1;
	  ksm.elecTransController3.opcion = opciones(valor);


	  // Marcha metida
	  valor = bits[3]*2 + bits[2]*1;
	  ksm.elecTransController3.opcion2 = opciones(valor);


	// PTO1 State - PTO2 State - NMV State
	  for (j = 0; j<8; j++) {
	    bits[7-j] = (m.Msg.DATA[6] >> 7-j) & mask ;// & (mask << 1);
	  }
	  
	  // PTO1 State
	  valor = bits[2]*4 +bits[1]*2 + bits[0]*1;
	  ksm.elecTransController3.pto1state = PTO1State(valor);


	  // PTO2 State
	  valor = bits[5]*4 + bits[4]*2 + bits[3]*1;
	  ksm.elecTransController3.pto2state = PTO2State(valor);


	  // NMV State
	  valor = bits[7]*2 + bits[6]*1;
	  ksm.elecTransController3.nmvstate = NMVState(valor);

    }
 
}

}

