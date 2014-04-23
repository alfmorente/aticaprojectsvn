/* 
 * File:   ConduccionThread.cpp
 * Author: Sergio Doctor López
 *
 * Created on 6 de febrero de 2014
 */

#include "Modulo_GPS/TeachThread.hpp"
#include <queue>
#include <iostream>

using namespace std;

TeachThread::TeachThread(){
    this->mode_active=true;
}

TeachThread::~TeachThread() {
    
}

void TeachThread::DoWork(){
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
    string nameOfPath = this->getDate(true).c_str();
    text = new TiXmlText(nameOfPath.c_str());
    elementName->LinkEndChild(text);
    elementTRK->LinkEndChild(elementName);
    
    // Segment
    TiXmlElement * elementSegment = new TiXmlElement("trkseg");
    elementTRK->LinkEndChild(elementSegment);
    // Elementos auxiliares para el tratamiento de los datos
    int countData = 0;
    TeachData presentElement;
    stringstream ss;
    while(mode_active){
        if(!queueGPSdata.empty()){
            countData++;
            // trkpt
            presentElement = queueGPSdata.front();
            queueGPSdata.pop();
            elementName = new TiXmlElement("trkpt");
            ss << presentElement.latitude ;
            elementName->SetAttribute("lat",ss.str().c_str());
            ss.str("");
            ss << presentElement.longitude ;
            elementName->SetAttribute("lon",ss.str().c_str());
            ss.str("");
            elementSegment->LinkEndChild(elementName);
            // ele
            elementNameAux = new TiXmlElement("ele");
            ss << countData ;
            text = new TiXmlText(ss.str().c_str());
            ss.str("");
            elementNameAux->LinkEndChild(text);
            elementName->LinkEndChild(elementNameAux);
            // time
            elementNameAux = new TiXmlElement("time");
            text = new TiXmlText(this->getDate(false).c_str());
            elementNameAux->LinkEndChild(text);
            elementName->LinkEndChild(elementNameAux);
        }
    }
    ss << nameOfPath << ".gpx";
    doc.SaveFile(ss.str().c_str());
    ss.str("");
}


string TeachThread::getDate(bool type){
    
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
       
void TeachThread::setMode(bool in){
    this->mode_active=in;
}