/**
  @file TeachString.cpp
  @brief Implementación de la clase TeachString
  @author Carlos Amores
  @date 2013,2014,2015
*/

#include "Modulo_GPS/TeachString.h"

/**
 * Constructor de la clase (vacío)
 */
TeachString::TeachString(){
  
}

/**
 * Método privado que obtiene la fecha de sistema en el formato utilizado para
 * la obtención del nombre de la ruta a guardar en modo Teach
 * @return String con la fecha en el formato especificado
 */
string TeachString::getDate() {
  time_t t = time(0);
  struct tm * now = localtime(& t);
  string fecha;
  fecha.append(getString(now->tm_mday));
  fecha.append("-");
  fecha.append(getString(now->tm_mon+1));
  fecha.append("-");
  fecha.append(getString(now->tm_year+1900));
  fecha.append("_");
  fecha.append(getString(now->tm_hour));
  fecha.append(":");
  fecha.append(getString(now->tm_min));
  fecha.append(":");
  fecha.append(getString(now->tm_sec));
  return fecha;
}

/**
 * Método público que incluye una línea en el string de la ruta con el nombre 
 * del fichero
 */
void TeachString::includeNameOfTeachFileLine(){
  teachRoute.append("route_");
  teachRoute.append(getDate());
  teachRoute.append(".txt");
  teachRoute.append("\r\n");
}

/**
 * Método público que transforma un número de tipo float a un string
 * @param[in] number Número a transformar
 * @return String con el número convertido
 */
string TeachString::getString(float number){
  stringstream ss;
  ss << number;
  return ss.str();
}

/**
 * Método público que realiza la segmentación de la ruta completa en partes de
 * tamaño máximo (65kbytes)
 * @param[in] teach String con la ruta completa guardada
 * @return Vector con tantos segmentos como haya resultado de la división
 */
vector<string> TeachString::divideTeach(string teach) {
	printf("Dividiendo fichero\n");
  int numOfFiles = (teach.size() / (SIZE_MAX_TEACH))+1;

  cout << teach.size() << " bytes" << endl;
  cout << numOfFiles << " divisiones" << endl;
  vector<string> ret;

  int indexOfTeach = 0;
  int firstSub = 0;
  for (int i = 0; i < numOfFiles; i++) {
    firstSub += indexOfTeach;
    indexOfTeach = 0;
    if(i!=(numOfFiles-1)){
      indexOfTeach += SIZE_MAX_TEACH;
	printf("Buscando el barraN\n");
      while (teach[firstSub+indexOfTeach] != '\n') {
        indexOfTeach++;
      }
	printf("Encontrado\n");
      indexOfTeach++;
      ret.push_back(teach.substr(firstSub, indexOfTeach));
    } else {
      ret.push_back(teach.substr(firstSub, teach.size()));
    }
  }
  string pag;
  // Paginacion
  for (int i = 0; i < numOfFiles; i++) {
    switch (i) {
      case 0:
        indexOfTeach = 0;
        while (teach[indexOfTeach] != '\n') {
          indexOfTeach++;
        }
        indexOfTeach++;
        pag.append("P  ");
        pag.append(getString(i + 1));
        pag.append(" ");
        pag.append(getString(numOfFiles));
        pag.append(" ");
        pag.append(getNOfRoute());
        pag.append("\r\n");
        ret.at(i).insert(indexOfTeach, pag);
        break;
      default:
        pag.clear();
        pag.append("P  ");
        pag.append(getString(i + 1));
        pag.append(" ");
        pag.append(getString(numOfFiles));
        pag.append(" ");
        pag.append(getNOfRoute());
        pag.append("\r\n");
        ret.at(i).insert(0, pag);
        break;
    };
  }
  increaseNOfRoute();
  return ret;
}

/**
 * Método público que obtiene el contador de rutas para obtener el identificador
 * de la ruta actual sumando 1 a la última realizada
 * @return String con el identificador de la ruta actual
 */
string TeachString::getNOfRoute(){
  string ret = "RT";
  ifstream fin("/home/atica/catkin_ws/src/Modulo_GPS/bin/nOfRoute.txt"); 
  string aux;
  fin >> aux;
  fin.close();
  for (int i=0;i<(9-aux.length());i++)
        ret.insert(2,"0");
  ret.append(aux);
  return ret;
}

/**
 * Método público que incrementa el contador de rutas realizadas
 */
void TeachString::increaseNOfRoute() {
  int num;
  std::ifstream fin("/home/atica/catkin_ws/src/Modulo_GPS/bin/nOfRoute.txt");
  fin >> num;
  fin.clear();
  fin.close();
  num++;
  std::ofstream fout("/home/atica/catkin_ws/src/Modulo_GPS/bin/nOfRoute.txt");
  fout << num;
  fout.close();
}

/**
 * Método público que incluye un WP durante el guardado de la ruta en curso
 * @param[in] latitude Latitud del WP
 * @param[in] longitude Longitud del WP
 * @param[in] first Booleano que indica si el WP es el primero de la ruta
 */
void TeachString::includeWPLine(double latitude, double longitude, bool first) {
  teachRoute.append("W  ");
  teachRoute.append(getString(latitude));
  teachRoute.append(" ");
  teachRoute.append(getString(longitude));
  (first)? teachRoute.append(" n\r\n"):teachRoute.append(" s\r\n");
}

/**
 * Consultor del atributo dividedTeach
 * @return Atributo dividedTeach
 */
vector<string> TeachString::getTeachStrings() {
  return dividedTeach;
}

/**
 * Método público que incluye en la ruta actual de Teach la línea que indica el 
 * final de ruta
 */
void TeachString::includeFinalLine() {
  teachRoute.append("F  FIN\r\n");
}

/**
 * Método público que realiza la segmentación de la ruta completa en partes de
 * tamaño máximo (65kbytes) para el atriburo teachRoute de la clase
 */
void TeachString::divideTeach() {
  int numOfFiles = (teachRoute.size() / (SIZE_MAX_TEACH))+1;

  int indexOfTeach = 0;
  int firstSub = 0;
  for (int i = 0; i < numOfFiles; i++) {
    firstSub += indexOfTeach;
    indexOfTeach = 0;
    if(i!=(numOfFiles-1)){
      indexOfTeach += SIZE_MAX_TEACH;
      while (teachRoute[firstSub+indexOfTeach] != '\n') {
        indexOfTeach++;
      }
      indexOfTeach++;
      dividedTeach.push_back(teachRoute.substr(firstSub, indexOfTeach));
    } else {
      dividedTeach.push_back(teachRoute.substr(firstSub, teachRoute.size()));
    }
  }
  string pag;
  // Paginacion
  for (int i = 0; i < numOfFiles; i++) {
    switch (i) {
      case 0:
        indexOfTeach = 0;
        while (teachRoute[indexOfTeach] != '\n') {
          indexOfTeach++;
        }
        indexOfTeach++;
        pag.append("P  ");
        pag.append(getString(i + 1));
        pag.append(" ");
        pag.append(getString(numOfFiles));
        pag.append(" ");
        pag.append(getNOfRoute());
        pag.append("\r\n");
        dividedTeach.at(i).insert(indexOfTeach, pag);
        break;
      default:
        pag.clear();
        pag.append("P  ");
        pag.append(getString(i + 1));
        pag.append(" ");
        pag.append(getString(numOfFiles));
        pag.append(" ");
        pag.append(getNOfRoute());
        pag.append("\r\n");
        dividedTeach.at(i).insert(0, pag);
        break;
    };
  }
  increaseNOfRoute();
}

