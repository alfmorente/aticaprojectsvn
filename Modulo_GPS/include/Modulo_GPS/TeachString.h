
/** 
 * @file  TeachString.h
 * @brief Declara el tipo de la clase "TeachString"
 * - La clase implementa la gestión de Strings necesarios para activación e 
 * implementación del modo Teach del vehículo
 * @author Carlos Amores
 * @date 2013, 2014, 2015
 * @addtogroup GPS
 * @{
 */

#ifndef TEACHSTRING_H

#define	TEACHSTRING_H

#include <iostream>
#include <fstream>
#include <string>
#include <ctime>
#include <sstream>
#include <cstdlib>
#include <string.h>
#include <vector>

#define SIZE_MAX_TEACH (65000-50) ///< Indica el tamaño máximo de una página enviable del modo Teach

using namespace std;

/**
 * \class TeachString
 * \brief Clase que representa la gestión de Strings necesaria para la implementación del modo Teach
 */
class TeachString {
private:
  string teachRoute;
  vector<string> dividedTeach;
  string getDate();
public:
  TeachString();
  void includeNameOfTeachFileLine();
  string getString(float number);
  vector<string> divideTeach(string teach);
  string getNOfRoute();
  void increaseNOfRoute();
  void includeWPLine(double latitude, double longitude, bool first);
  void includeFinalLine();
  vector<string> getTeachStrings();
  void divideTeach();


};

#endif	/* TEACHSTRING_H */

/**
 * @}
 */