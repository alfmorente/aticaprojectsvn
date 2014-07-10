
#include "conversionTypes.h"

using namespace std;

float hexa2float( char * buffer){
    union
    {
        float value;
        unsigned char buffer[4];

    }floatUnion;

    floatUnion.buffer[0] = buffer[3];
    floatUnion.buffer[1] = buffer[2];
    floatUnion.buffer[2] = buffer[1];
    floatUnion.buffer[3] = buffer[0];

    return floatUnion.value;
}

int hexa2int(unsigned char * buffer){
    union
    {
        int value;
        unsigned char buffer[4];

    }floatUnion;

    floatUnion.buffer[0] = buffer[3];
    floatUnion.buffer[1] = buffer[2];
    floatUnion.buffer[2] = buffer[1];
    floatUnion.buffer[3] = buffer[0];

    return floatUnion.value;
}
short hexa2short(char buffer[2]){
    union
    {
        short value;
        unsigned char buffer[2];

    }shortUnion;

    shortUnion.buffer[0] = buffer[1];
    shortUnion.buffer[1] = buffer[0];

    return shortUnion.value;
}
double hexa2double(unsigned char * buffer){
    union{
        double value;
        unsigned char buffer[8];
    }floatUnion;

    floatUnion.buffer[0] = buffer[7];
    floatUnion.buffer[1] = buffer[6];
    floatUnion.buffer[2] = buffer[5];
    floatUnion.buffer[3] = buffer[4];
    floatUnion.buffer[4] = buffer[3];
    floatUnion.buffer[5] = buffer[2];
    floatUnion.buffer[6] = buffer[1];
    floatUnion.buffer[7] = buffer[0];

    return floatUnion.value;
}
char *shortToHexa(short s){
    char *buf = (char *) malloc(2);
    memcpy(buf,&s,2);
    char aux = buf[1];
    buf[1] = buf[0];
    buf[0] = aux;
    return buf;

}

