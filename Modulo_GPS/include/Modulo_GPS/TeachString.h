/* 
 * File:   TeachString.h
 * Author: atica
 *
 * Created on 14 de abril de 2015, 9:35
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

#define SIZE_MAX_TEACH (65000-50)

using namespace std;

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
  void includeWPLine(double latitude,double longitude, bool first);
  void includeFinalLine();
  vector<string> getTeachStrings();
  void divideTeach();
  

};

#endif	/* TEACHSTRING_H */

