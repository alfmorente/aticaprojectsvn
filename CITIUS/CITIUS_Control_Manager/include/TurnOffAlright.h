
/** 
 * @file  TurnOffAlright.h
 * @brief Declara el tipo de la clase "TurnOffAlright"
 * - La clase implementa la gestión del fichero de gestión de correcto apagado
 * en la ejecución anterior
 * @author Carlos Amores
 * @date 2013, 2014
 * @addtogroup Manager
 * @{
 */

#ifndef TURNOFFALRIGHT_H
#define	TURNOFFALRIGHT_H

#include <string>
#include <time.h>
#include <stdio.h>
#include <fstream>
#include <sys/stat.h>
#include <ios>
#include <string.h>

using namespace std;

/**
 * \class TurnOffAlright
 * \brief Clase que representa la gestión del fichero de gestión de correcto
 * apagado
 */
class TurnOffAlright {
public:
  TurnOffAlright();
  void setStatusLine(string);
  bool checkCorrectTurnedOff();
private:
  bool fileExists();
  string getDate();
  string filename;
};

#endif	/* TURNOFFALRIGHT_H */

/**
 * @}
 */