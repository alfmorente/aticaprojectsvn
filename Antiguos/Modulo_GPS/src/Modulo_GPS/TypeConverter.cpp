#include "Modulo_GPS/TypeConverter.h"

using namespace std;

string toString(double num){
  stringstream ss;
  ss << num;
  return ss.str();
}

double stringToDouble(string s){
  stringstream ss;
  double d;
  ss << s;
  ss >> d;
  return d;
}

float stringToFloat(string s){
  stringstream ss;
  float f;
  ss << s;
  ss >> f;
  return f;
}

long stringToLong(string s){
  stringstream ss;
  long l;
  ss << s;
  ss >> l;
  return l;
}

int stringToInt(string s){
  stringstream ss;
  int i;
  ss << s;
  ss >> i;
  return i;
}
