
#include "Modulo_GPS/TeachString.h"


TeachString::TeachString(){
  
}

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

void TeachString::includeNameOfTeachFileLine(){
  teachRoute.append("route_");
  teachRoute.append(getDate());
  teachRoute.append(".txt");
  teachRoute.append("\r\n");
}

string TeachString::getString(float number){
  stringstream ss;
  ss << number;
  return ss.str();
}

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

void TeachString::includeWPLine(double latitude, double longitude, bool first) {
  teachRoute.append("W  ");
  teachRoute.append(getString(latitude));
  teachRoute.append(" ");
  teachRoute.append(getString(longitude));
  (first)? teachRoute.append(" n\r\n"):teachRoute.append(" s\r\n");
}

vector<string> TeachString::getTeachStrings() {
  return dividedTeach;
}

void TeachString::includeFinalLine() {
  teachRoute.append("F  FIN\r\n");
}

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

