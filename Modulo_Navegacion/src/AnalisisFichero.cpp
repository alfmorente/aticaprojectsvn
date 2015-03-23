#include <vector>

#include "Modulo_Navegacion/AnalisisFichero.h"


vector<string> createVectorLineFromPage(string page)
{
  vector <string> vecLine;
  char *ptr;
  ptr = strtok((char*)page.c_str(),"\n");//argumentos: frase, caracter delimitador
  while(ptr != NULL)
  {
      vecLine.push_back(ptr);
      ptr = strtok(NULL,"\n");
  }
  return vecLine;
}

vector<string> createVectorFromLine(string line)
{
  vector <string> vecData;
  char *ptr;
  ptr = strtok((char*)line.c_str()," ");//argumentos: frase, caracter delimitador
  while(ptr != NULL)
  {
      vecData.push_back(ptr);
      ptr = strtok(NULL," ");
  }
  return vecData;
}

bool analizeMainHead(vector<string> mainHead,PathPlan* newPlan)
{
    if(mainHead.size()< 2)
        return false;
    if(mainHead.at(0)!="N")
        return false;
    newPlan->NamePath=mainHead.at(1);
    return true;    
}
bool analizePageHead(vector<string> pageHead,PathPlan* newPlan,int page)
{
    DataPage Body;
    int totalPages;
    string idPath;
    if(pageHead.size()< 4)
        return false;
    if(pageHead.at(0)!="P")
        return false;
    
    Body.currentPage=convertStringToInt(pageHead.at(1));
    
    if(Body.currentPage!=page)
        return false;
    
    totalPages=convertStringToInt(pageHead.at(2));
    idPath=pageHead.at(3);
    if(page!=1)
    {
        if(totalPages!=newPlan->numPages)
                return false;
        if(idPath!=newPlan->idPath)
                return false;
    }
    else
    {
            newPlan->numPages=totalPages;
            newPlan->idPath=pageHead.at(3);   
    }
    newPlan->vecDP.push_back(Body);
    return true;    
}
bool analizeLineWaypoint(vector<string> vWaypoint, int page, PathPlan* newPlan)
{

    if(vWaypoint.size()<4)
        return false;    
    if(vWaypoint.at(0)!="W")
        return false;
    if(newPlan->vecDP.size()< page)
        return false;
    
    newPlan->vecDP.at(page-1).wpLat.push_back(convertStringToDouble(vWaypoint.at(1)));
    newPlan->vecDP.at(page-1).wpLon.push_back(convertStringToDouble(vWaypoint.at(2)));
    newPlan->vecDP.at(page-1).type.push_back(vWaypoint.at(3));    
    return true;   
}
bool analizeEndPage(vector<string> vEnd)
{
    if(vEnd.size()<2)
        return false;    
    if(vEnd.at(0)!="F")
        return false;
    if(vEnd.at(1)!="FIN")
        return false;    
    return true;   
}
bool analizePage(string page,int currentPage, PathPlan* newPlan)
{
    bool endPage=false;
    vector <string> vline;
    vector <string> vField;
    vline=createVectorLineFromPage(page);
    for(int i=0;i<vline.size();i++)
        cout << vline.at(i) <<endl;
    int line=0;
    int numLines=vline.size(); 
    int numWaypoints; 
    if(currentPage==1)
    {
        if(numLines<2)
            return false;          
        vField=createVectorFromLine(vline.at(line));
        cout <<"Linea "<<line<<endl;
        for(int i=0;i<vField.size();i++)
                cout << vField.at(i) <<endl;        
        if(!analizeMainHead(vField,newPlan))
            return false;
        line++;
    }
    vField=createVectorFromLine(vline.at(line)); 
    cout <<"Linea "<<line<<endl;
    for(int i=0;i<vField.size();i++)
           cout << vField.at(i) <<endl;     
    if(!analizePageHead(vField,newPlan,currentPage))
        return false;

    cout <<"Numero de paginas: "<<newPlan->numPages<<endl;    
    line++;
    if(currentPage==newPlan->numPages)
    {
        endPage=true;
        numWaypoints=(numLines-line)-1;
    }
    else
        numWaypoints=numLines-line; 
    cout <<"Numero de waypoints: "<< numWaypoints << endl;
    newPlan->vecDP.at(currentPage-1).numWP=numWaypoints;
    
    for(int i=0;i<numWaypoints;i++)
    {
        vField=createVectorFromLine(vline.at(line));
        cout <<"Linea "<<line<<endl;
        for(int i=0;i<vField.size();i++)
                cout << vField.at(i) <<endl; 
        if(!analizeLineWaypoint(vField,currentPage,newPlan))
            return false;
        line++;
  
    }
    
    if(endPage)
    {
        vField=createVectorFromLine(vline.at(line));   
        for(int i=0;i<vField.size();i++)
                cout << vField.at(i) <<endl;         
        if(!analizeEndPage(vField))
            return false;
    }
    return true;
    
}

void cleanPathPlan(PathPlan* oldPlan)
{
    oldPlan->NamePath.clear();
    oldPlan->idPath.clear();
    oldPlan->numPages=0;
    oldPlan->vecDP.clear();
}
int convertStringToInt(string s)
{
    int value;
    stringstream aux;
    aux << s;
    aux >> value;
    return value;    
}

double convertStringToDouble(string s)
{
    double value;
    stringstream aux;
    aux << s;
    aux >> value;
    return value;     
    
}