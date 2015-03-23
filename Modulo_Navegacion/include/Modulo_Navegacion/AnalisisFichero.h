//Funciones de análisis de ficheros
#include <string>
#include <string.h>
#include <iostream>
#include <vector>
#include <stdio.h>
#include <sstream>

using namespace std;

typedef struct Body
{    
    int numWP;
    int currentPage;  
    vector <double> wpLat;
    vector <double> wpLon;  
    vector <string> type;      
}DataPage;

typedef struct Plan
{
    string NamePath;
    int numPages;
    string idPath;
    vector<DataPage>  vecDP;
}PathPlan;


vector<string> createVectorLineFromPage(string page);
vector<string> createVectorFromLine(string line);
bool analizeEndPage(vector<string> vEnd);
bool analizePage(string,int, PathPlan*);
bool analizeLineWaypoint(vector<string> vWaypoint, int page, PathPlan* newPlan);
bool analizeMainHead(vector<string> mainHead,PathPlan* newPlan);
bool analizePageHead(vector<string> pageHead,PathPlan* newPlan,int page);
void cleanPathPlan(PathPlan*);
int convertStringToInt(string s);
double convertStringToDouble(string s);

