/** 
 * @file  TurnOffAlright.cpp
 * @brief Implementacion de la clase "TurnOffAlright"
 * @author Carlos Amores
 * @date 2013, 2014
 */

#include "TurnOffAlright.h"

/**
 * Constructor de la clase
 */
TurnOffAlright::TurnOffAlright() {
  filename = "/home/ugv/catkin_ws/src/CITIUS_Control_Manager/bin/Operation_Status.txt";
}

/**
 * Método privado que obtiene fecha y hora en el formato definido para fichero 
 * de control de correcto apagado
 * @return Cadena con fecha y hora en el formato definido
 */
string TurnOffAlright::getDate() {
  time_t now = time(0);
  struct tm tstruct;
  char buf[80];
  tstruct = *localtime(&now);
  strftime(buf, sizeof (buf), "%d-%m-%Y :: %H-%M-%S :: ", &tstruct);
  return buf;
}

/**
 * Incluye una nueva línea en el fichero de control de correcto apagado
 * @param status Estado a incluir en la línea
 */
void TurnOffAlright::setStatusLine(string status) {
  ofstream myfile;
  myfile.open (filename.c_str(),ios::app);
  myfile << getDate() << status << endl;
  myfile.close();
}

bool TurnOffAlright::fileExists() {
  struct stat buf;
  if (stat(filename.c_str(), &buf) != -1) {
    return true;
  }
  return false;
}

/**
 * Comprueba que la anterior ejecución finalizó correctamente (de manera 
 * ordenada) consultando el fichero de control de correcto apagado
 * @return Booleano que indica si la anterior ejecución finalizó correctamente
 */
bool TurnOffAlright::checkCorrectTurnedOff() {
  if (fileExists()) {
    char line[100];
    ifstream myfile(filename.c_str());
    while (!myfile.eof()) {
      myfile >> line;
    }
    myfile.close();
    return (strcmp(line, "APAGANDO") == 0);
  } else {
    ofstream initFile(filename.c_str());
    initFile << "";
    initFile.close();
    return true;
  }
}

/**
 * Método público que borra el contenido anterior del fichero de control de 
 * correcto apagado y lo prepara para una nueva escritura
 */
void TurnOffAlright::clearFile() {
  ofstream initFile(filename.c_str());
  initFile << "";
  initFile.close();
}





