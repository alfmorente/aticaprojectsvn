
#include "AxisP3364LveDriver.h"

/*******************************************************************************
                               CONSTRUCTOR
 ******************************************************************************/

AxisP3364LveDriver::AxisP3364LveDriver() {

}

/*******************************************************************************
                                DESTRUCTOR
 ******************************************************************************/
AxisP3364LveDriver::~AxisP3364LveDriver() {

}
    
/*******************************************************************************
                        ENVIAR COMANDO DE CONTROL
 ******************************************************************************/

bool AxisP3364LveDriver::sentSetToDevice(short order, float value){
    socketDescriptor = socket(AF_INET, SOCK_STREAM, 0);

    if (socketDescriptor < 0) {

        return false;

    } else {

        if ((he = gethostbyname(IP_CAMERA)) == NULL) {
            return false;
        }

        if ((socketDescriptor = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
            return false;
        }

        server.sin_family = AF_INET;
        server.sin_port = htons(PORT_CAMERA);
        server.sin_addr = *((struct in_addr *) he->h_addr);

        bzero(&(server.sin_zero), 8);

        if (connect(socketDescriptor, (struct sockaddr *) &server, sizeof (struct sockaddr)) == -1) {

            return false;

        } else {

            stringstream stream;
            stream << "GET http://" /*<< AUTH_CAM_USER << "@" << AUTH_CAM_PASS << ":" */ << IP_CAMERA << PTZ_ROUTE;
            switch (order) {
                case ORDER_ZOOM:
                    stream << "zoom=";
                    break;
                case ORDER_TILT:
                    stream << "tilt=";
                    break;
                case ORDER_PAN:
                    stream << "pan=";
                    break;
                default:
                    stream.str().clear();
                    break;
            }

            if (stream.str().size() == 0) {
                return false;
            }

            stream << value << "\nConnection: keep-alive\r\n";


            int nBytesSent = send(socketDescriptor, stream.str().c_str(), strlen(stream.str().c_str()), 0);

            if (nBytesSent < 0) {
                return false;
            }

            // Lectura y obtencion del ack
            char respuesta[256];

            int nBytesRead = 0;
            while (nBytesRead == 0) {
                nBytesRead = recv(socketDescriptor, respuesta, 256, 0);
            }
            
            LensPosition lenspos = getPosition();
            
            if(lenspos.state){
                
                printf("PAN: %f\n",lenspos.pan);
                printf("TILT: %f\n",lenspos.tilt);
                printf("ZOOM: %f\n",lenspos.zoom);
                
            }else{
                printf("No se ha podido obtener la posicion\n");
            }
            
            

            return true;

        }

        return false;
    }

}

/*******************************************************************************
                         LECTURA DE LA POSICION
 ******************************************************************************/

LensPosition AxisP3364LveDriver::getPosition() {

    LensPosition pos;
    pos.state = false;
    pos.pan = 0;
    pos.tilt = 0;
    pos.zoom = 0;

    socketDescriptor = socket(AF_INET, SOCK_STREAM, 0);

    if (socketDescriptor >= 0) {

        if ((he = gethostbyname(IP_CAMERA)) != NULL) {
            server.sin_family = AF_INET;
            server.sin_port = htons(PORT_CAMERA);
            server.sin_addr = *((struct in_addr *) he->h_addr);

            bzero(&(server.sin_zero), 8);

            if (connect(socketDescriptor, (struct sockaddr *) &server, sizeof (struct sockaddr)) != -1) {

                stringstream stream;
                stream << "GET http://" << AUTH_CAM_USER << "@" << AUTH_CAM_PASS << ":" << IP_CAMERA << PTZ_ROUTE << "query=position\r\n";
                //printf("Comando generado: %s", stream.str().c_str());

                int nBytesSent = send(socketDescriptor, stream.str().c_str(), strlen(stream.str().c_str()), 0);

                if (nBytesSent < 0) {
                    printf("Error en la escritura\n");
                    pos.state = false;
                }

                // Lectura y obtencion de los datos
                char respuesta[256];
                int nBytesRead = recv(socketDescriptor, respuesta, 256, 0);
                respuesta[nBytesRead] = '\0';

                if (nBytesRead > 0) {

                    pos.state = true;
                    pos.pan = extractPan(respuesta);
                    pos.tilt = extractTilt(respuesta);
                    pos.zoom = extractZoom(respuesta);

                }

            }
            
        }
        
    }
    
    return pos;
}

/*******************************************************************************
                        EXTRACCION DE VALORES EN CADENA
 ******************************************************************************/

float AxisP3364LveDriver::extractZoom(char cadena[256]) {
    string resp = cadena;
    int index = resp.find("zoom=");
    string value;
    for (unsigned int i = index + 5; i < resp.size(); i++) {
        if (resp.at(i) == '\r') {
            i = resp.size();
        } else {
            value.push_back(resp.at(i));
        }
    }
    return (float) atof(value.c_str());
}

float AxisP3364LveDriver::extractTilt(char cadena[256]) {
    string resp = cadena;
    int index = resp.find("tilt=");
    string value;
    for (unsigned int i = index + 5; i < resp.size(); i++) {
        if (resp.at(i) == '\r') {
            i = resp.size();
        } else {
            value.push_back(resp.at(i));
        }
    }
    return (float) atof(value.c_str());
}

float AxisP3364LveDriver::extractPan(char cadena[256]) {
    string resp = cadena;
    int index = resp.find("pan=");
    string value;
    for (unsigned int i = index + 4; i < resp.size(); i++) {
        if (resp.at(i) == '\r') {
            i = resp.size();
        } else {
            value.push_back(resp.at(i));
        }
    }
    return (float) atof(value.c_str());
}
