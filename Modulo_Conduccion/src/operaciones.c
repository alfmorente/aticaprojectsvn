#include "../include/Modulo_Conduccion/operaciones.h"
#include <string.h>


int calcularGrados (int grados) {
  int factor = 1;
  int offset = -40;
  int resultado = (grados * factor) + offset; 
  
  if (!((resultado >= -40) && (resultado <= 210))) { 
	resultado = 0;
	return resultado;    
  }
  else 
      return resultado;
      
}


double calcularGrados2 (int grados) {
  double factor = 0.03125;
  int offset = -273;
  double grades = (double) grados;
  double resultado = (grades * factor) + offset; 
  
  if (!((resultado >= -273) && (resultado <= 1735))) {
	resultado = 0.0;
	return resultado;
  }
  else 
      return resultado;
      
}


double calcularRPM (int revoluciones) {
  double factor = 0.125;
  int offset = 0;
  double revol = (double) revoluciones;
  double resultado = (revol * factor) + offset; 
  
  if (!((resultado >= 0) && (resultado <= 8031.875))) {
	resultado = 0.0;
	return resultado;

  }
  else
      return resultado;
      
}


double calcularPorcentaje (int porcentaje) {
  double factor = 0.4;
  int offset = 0;
  double percent = (double) porcentaje;
  double resultado = (percent * factor) + offset; 
  
  if (!((resultado >= 0) && (resultado <= 100))) {
	resultado = 0.0;
	return resultado;
  }
  else
      return resultado;
      
}

double calcularPorcentaje2 (int porcentaje) {
  int factor = 1;
  int offset = 0;
  int resultado = (porcentaje * factor) + offset; 
  
  if (!((resultado >= 0) && (resultado <= 100))) {
	resultado = 0.0;
	return resultado;
  }
  else 
      return resultado;
  
      
}

int calcularPorcentaje3 (int porcentaje) {
  int factor = 1;
  int offset = 125;
  int resultado = (porcentaje * factor) + offset; 
  
  if (!((resultado >= -125) && (resultado <= 125))) {
	resultado = 0.0;
	return resultado;
  }
  else 
      return resultado;
  
      
}

int calcularPorcentaje4 (int porcentaje) {
  int factor = 1;
  int offset = 125;
  int resultado = (porcentaje * factor) + offset; 
  
  if (!((resultado >= -125) && (resultado <= 0))) {
	resultado = 0;
	return resultado;
  }
  else 
      return resultado;
  
      
}

int calcularMarcha (int marcha) {
  int factor = 1;
  int offset = -125;
  int resultado = (marcha * factor) + offset; 
  
  if (!((resultado >= -125) && (resultado <= 125))) {
	resultado = 0;
	return resultado;
  }
  else 
      return resultado;
      
}


double calcularMarcha2 (int marcha) {
  double factor = 0.001;
  int offset = 0;
  double resultado = (marcha * factor) + offset; 
  
  if (!((resultado >= 0) && (resultado <= 64.255))) {
	resultado = 0.0;
	return resultado;
  }
  else 
      return resultado;
      
}


double calcularBares (int bares) {
  double factor = 0.04;
  int offset = 0;
  double bar = (double) bares;
  double resultado = (bar * factor) + offset; 
  
  if (!((resultado >= 0) && (resultado <= 10))) {
	resultado = 0.0;
	return resultado;
  }
  else
      return resultado;
      
}


double calcularBares2 (int bares) {
  double factor = 0.08;
  int offset = 0;
  double bar = (double) bares;
  double resultado = (bar * factor) + offset; 
  
  if (!((resultado >= 0) && (resultado <= 20))) {
	resultado = 0.0;
	return resultado;
  }
  else
      return resultado;
      
}


double calcularBares3 (int bares) {
  double factor = 0.005;
  int offset = 0;
  double bar = (double) bares;
  double resultado = (bar * factor) + offset; 
  
  if (!((resultado >= 0) && (resultado <= 1.25))) {
	resultado = 0.0;
	return resultado;
  }
  else
      return resultado;
      
}


double calcularKM (int kilometros) {
  double factor = 0.005;
  int offset = 0;
  double km = (double) kilometros;
  double resultado = (km * factor) + offset; 
  
  if (!((resultado >= 0) && (resultado <= 21055406))) {
	resultado = 0.0;
	return resultado;
  }
  else
      return resultado;
      
}


double calcularKG (int kilogramos) {
  double factor = 0.5;
  int offset = 0;
  double kg = (double) kilogramos;
  double resultado = (kg * factor) + offset; 
  
  if (!((resultado >= 0) && (resultado <= 32127.5))) {
	resultado = 0.0;
	return resultado;
  }
  else
      return resultado;
      
}


double calcularHoras (int horas) {
  double factor = 0.05;
  int offset = 0;
  double hours = (double) horas;
  double resultado = (hours * factor) + offset; 
  
  if (!((resultado >= 0) && (resultado <= 210554060.75))) {
	resultado = 0.0;
	return resultado;
  }
  else
      return resultado;
      
}


double calcularLitros (int litros) {
  double factor = 0.05;
  int offset = 0;
  double hours = (double) litros;
  double resultado = (hours * factor) + offset; 
  
  if (!((resultado >= 0) && (resultado <= 210554060.75))) {
	resultado = 0.0;
	return resultado;
  }
  else
      return resultado;
      
}


double calcularKM_H (int kilometros_horas) {
  int parte1 = 1;
  int parte2 = 256;
  double factor = (double) parte1/parte2;
  int offset = 0;
  double km_h = (double) kilometros_horas;
  double resultado = (km_h * factor) + offset; 
  
  if (!((resultado >= 0) && (resultado <= 250.996))) {
	resultado = 0.0;
	return resultado;
  }
  else
      return resultado;
      
}


double calcularKM_L (int kilometros_litros) {
  int parte1 = 1;
  int parte2 = 512;
  double factor = (double) parte1/parte2;
  int offset = 0;
  double km_l = (double) kilometros_litros;
  double resultado = (km_l * factor) + offset; 
  
  if (!((resultado >= 0) && (resultado <= 125.5))) {
	resultado = 0.0;
	return resultado;
  }
  else
      return resultado;
      
}


char * opciones (int opcion) {
  
  char * estado;
  
  switch(opcion) {
    case 0:
      estado = "Off";
      break;
    case 1:
      estado = "On";
      break;
    case 2:
      estado = "Error";
      break;
    case 3:
      estado = "Not Available";
      break;
    default:
      estado = "Valor no disponible";	      

  }
  return estado;
}


char * opciones2 (int opcion) {
  
  char * estado;
  
  switch(opcion) {
    case 0:
      estado = "Dissengaged";
      break;
    case 1:
      estado = "Engaged";
      break;
    case 2:
      estado = "Error";
      break;
    case 3:
      estado = "Not Available";
      break;
    default:
      estado = "Valor no disponible";	      

  }
  return estado;
}


char * axleLocation (int opcion) {
  
  char * estado;
    
  switch(opcion) {
    case 0:
      estado = "Axle 1";
      break;
    case 1:
      estado = "Axle 2";
      break;
    case 2:
      estado = "Axle 3";
      break;
    case 3:
      estado = "Axle 4";
      break;
    case 4:
      estado = "Axle 5";
      break;
    case 5:
      estado = "Axle 6";
      break;      
    case 6:
      estado = "Axle 7";
      break;      
    case 7:
      estado = "Axle 8";
      break;      
    case 8:
      estado = "Axle 9";
      break;      
    case 9:
      estado = "Axle 10";
      break;      
    case 10:
      estado = "Axle 11";
      break;      
    case 11:
      estado = "Axle 12";
      break;          
    case 12:
      estado = "Axle 13";
      break;      
    case 13:
      estado = "Axle 14";
      break;      
    case 14:
      estado = "Axle 15";
      break;      
    case 15:
      estado = "Axle 16";
      break;
    default:
      estado = "Valor no disponible";	           
      
  }
  return estado;
}

char * driverWorking (int opcion) {
  
  char * estado;
    
  switch(opcion) {
    case 0:
      estado = "Reset";
      break;
    case 1:
      estado = "Avaliable";
      break;
    case 2:
      estado = "Work";
      break;
    case 3:
      estado = "Drive";
      break;
    case 4:
      estado = "Reserved";
      break;
    case 5:
      estado = "Reserved";
      break;      
    case 6:
      estado = "Reserved";
      break;      
    case 7:
      estado = "Not avaliable";
      break;
    default:
      estado = "Valor no disponible";	      
      
  }
  return estado;
}

char * driverTime (int opcion) {
  
  char * estado;
    
  switch(opcion) {
    case 0:
      estado = "No warning";
      break;
    case 1:
      estado = "Warning #1";
      break;
    case 2:
      estado = "Warning #2";
      break;
    case 3:
      estado = "Warning #3";
      break;
    case 4:
      estado = "Warning #4";
      break;
    case 5:
      estado = "Warning #5";
      break;      
    case 6:
      estado = "Reserved";
      break;      
    case 7:
      estado = "Reserved";
      break;      
    case 8:
      estado = "Reserved";
      break;      
    case 9:
      estado = "Reserved";
      break;      
    case 10:
      estado = "Reserved";
      break;      
    case 11:
      estado = "Reserved";
      break;          
    case 12:
      estado = "Reserved";
      break;      
    case 13:
      estado = "Reserved";
      break;      
    case 14:
      estado = "Error";
      break;      
    case 15:
      estado = "Not avaliable";
      break;
    default:
      estado = "Valor no disponible";	      
      
  }
  return estado;
}

char * PTOState (int opcion) {
  
  char * estado;

  switch(opcion) {
    case 0:
      estado = "Off";
      break;
    case 5:
      estado = "Set";
      break;
    case 31:
      estado = "Not avaliable";
      break;
    default:
      estado = "Valor no disponible";	      
}

  return estado;	

}


char * PTO1State (int opcion) {

 char * estado;

  switch(opcion) {
    case 0:
      estado = "Off";
      break;
    case 1:
      estado = "Activated";
      break;
    case 2:
      estado = "Enganged";
      break;
    case 3:
      estado = "On";
      break;
    default:
      estado = "No defined";	      
}

  return estado;

}


char * PTO2State (int opcion) {

 char * estado;

switch(opcion) {
    case 0:
      estado = "Off";
      break;
    case 1:
      estado = "Activated";
      break;
    case 2:
      estado = "Enganged";
      break;
    case 3:
      estado = "On";
      break;
    default:
      estado = "No defined";	      
}

return estado;

}


char * NMVState (int opcion) {

 char * estado;

 switch(opcion) {
    case 0:
      estado = "Off";
      break;
    case 1:
      estado = "Activated";
      break;
    case 2:
      estado = "Enganged";
      break;
    case 3:
      estado = "On";
      break;
    default:
      estado = "No defined";	      
}
return estado;
}

