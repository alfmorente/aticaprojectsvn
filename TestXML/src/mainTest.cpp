/* 
 * File:   mainTest.cpp
 * Author: atica
 *
 * Created on 4 de marzo de 2014, 11:09
 */

#include <cstdlib>
#include <tinyxml.h>
#include "ros/ros.h"
#include <sstream>
#include <string>

using namespace std;

// Obtain date and hour (2 type of formats)
string getDate(bool type);
void printCorrectSyntax();

int main(int argc, char** argv) {
    
    int a = atoi(argv[1]);
    cout << a << endl;
    switch(a){
        case 1:
            cout << "Mode DEBUG enabled" << endl;
            break;
        case 2:
            cout << "Mode RELEASE enabled" << endl;
            break;
        case 3:
            cout << "Mode SIMULATION enabled" << endl;
            break;
        default:
            printCorrectSyntax();
            return 0;
    }

    TiXmlDocument doc;
    TiXmlDeclaration * decl = new TiXmlDeclaration("1.0", "UTF-8", "no");
    doc.LinkEndChild(decl);
    
    TiXmlElement * elementGPX = new TiXmlElement("gpx");
    elementGPX->SetAttribute("version","1.1");
    elementGPX->SetAttribute("creator","Carlos Amores");
    doc.LinkEndChild(elementGPX);
    
    /* METADATA */
    
    TiXmlElement * elementMetadata = new TiXmlElement("metadata");
    elementGPX->LinkEndChild(elementMetadata);
    
    // Name
    TiXmlElement * elementName = new TiXmlElement("name");
    TiXmlText* text = new TiXmlText( "Carlos Amores" );
    elementName->LinkEndChild(text);
    elementMetadata->LinkEndChild(elementName);
    // Description
    elementName = new TiXmlElement("desc");
    text = new TiXmlText( "Fichero generado con TinyXML" );
    elementName->LinkEndChild(text);
    elementMetadata->LinkEndChild(elementName);
    // Author
    elementName = new TiXmlElement("author");
    elementMetadata->LinkEndChild(elementName);
    
    TiXmlElement * elementNameAux = new TiXmlElement("name");
    text = new TiXmlText( "Carlos Amores" );
    elementNameAux->LinkEndChild(text);
    elementName->LinkEndChild(elementNameAux);
    elementNameAux = new TiXmlElement("email");
    text = new TiXmlText( "caramoalv@gmail.com" );
    elementNameAux->LinkEndChild(text);
    elementName->LinkEndChild(elementNameAux);
    // Copyright
    elementName = new TiXmlElement("copyright");
    text = new TiXmlText( "" );
    elementName->LinkEndChild(text);
    elementMetadata->LinkEndChild(elementName);
    // Link
    elementName = new TiXmlElement("link");
    text = new TiXmlText("href=\"http://grvc.us.es\"");
    elementName->LinkEndChild(text);
    elementMetadata->LinkEndChild(elementName);
    
    /* TRK */
    
    TiXmlElement * elementTRK = new TiXmlElement("trk");
    elementGPX->LinkEndChild(elementTRK);
    
    // Name of path (based time)
    elementName = new TiXmlElement("name");
    string nameOfPath = getDate(true).c_str();
    text = new TiXmlText(nameOfPath.c_str());
    elementName->LinkEndChild(text);
    elementTRK->LinkEndChild(elementName);
    
    // Segment
    TiXmlElement * elementSegment = new TiXmlElement("trkseg");
    elementTRK->LinkEndChild(elementSegment);
    
    for (int i=1;i<11;i++){
        // trkpt
        elementName = new TiXmlElement("trkpt");
        stringstream ss,ss2;
        ss << (-90 + rand() % (181)) ;
        ss2 << (-180 + rand() % (361)) ;
        elementName->SetAttribute("lat",ss.str().c_str());
        elementName->SetAttribute("lon",ss2.str().c_str());
        elementSegment->LinkEndChild(elementName);
        // ele
        elementNameAux = new TiXmlElement("ele");
        ss.str("");
        ss << i;
        text = new TiXmlText(ss.str().c_str());
        elementNameAux->LinkEndChild(text);
        elementName->LinkEndChild(elementNameAux);
        // time
        elementNameAux = new TiXmlElement("time");
        text = new TiXmlText(getDate(false).c_str());
        elementNameAux->LinkEndChild(text);
        elementName->LinkEndChild(elementNameAux);
        sleep(1);
    }
    stringstream ssFinal;
    ssFinal << nameOfPath << ".gpx";
    doc.SaveFile(ssFinal.str().c_str());
    
    return 0;
}

string getDate(bool type){
    
    if(type){ // For name of file
        time_t     now;
        struct tm *ts;
        char       buf[80];
        stringstream ss;
        now = time(0);

        ts = localtime(&now);
        strftime(buf, sizeof(buf), "%Y_%m_%d_%H_%M_%S", ts);
        ss << "Ruta TEM_TEACH_" << buf;
        return ss.str();
    }else{ // For each waypoint
        time_t     now;
        struct tm *ts;
        char       buf[80];
        stringstream ss;
        now = time(0);

        ts = localtime(&now);
        strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", ts);
        ss << buf;
        return ss.str();
    }
}

void printCorrectSyntax() {
    cout << "Invalid option. Syntax: ./GPS [mode option]" << endl;
    cout << "Options: " << endl;
    cout << "1: Debug" << endl;
    cout << "2: Release" << endl;
    cout << "3: Simulation" << endl;
    cout << "---------------" << endl;
    cout << "Try again" << endl;
}