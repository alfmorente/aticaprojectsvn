
#include "conversionTypes.h"

using namespace std;

float hexa2float( vector<char> buffer){
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

int hexa2int(std::vector<unsigned char> buffer){
    union
    {
        int value;
        unsigned char buffer[4];

    }intUnion;

    intUnion.buffer[0] = buffer[3];
    intUnion.buffer[1] = buffer[2];
    intUnion.buffer[2] = buffer[1];
    intUnion.buffer[3] = buffer[0];

    return intUnion.value;
}
short hexa2short(vector<char> buffer){
    union
    {
        short value;
        unsigned char buffer[2];

    }shortUnion;

    shortUnion.buffer[0] = buffer[1];
    shortUnion.buffer[1] = buffer[0];

    return shortUnion.value;
}

double hexa2double(std::vector<unsigned char> buffer){
    union{
        double value;
        unsigned char buffer[8];
    }doubleUnion;

    doubleUnion.buffer[0] = buffer[7];
    doubleUnion.buffer[1] = buffer[6];
    doubleUnion.buffer[2] = buffer[5];
    doubleUnion.buffer[3] = buffer[4];
    doubleUnion.buffer[4] = buffer[3];
    doubleUnion.buffer[5] = buffer[2];
    doubleUnion.buffer[6] = buffer[1];
    doubleUnion.buffer[7] = buffer[0];

    return doubleUnion.value;
}

vector<char> shortToHexa(short s){
    char *buf = (char *) malloc(2);
    vector<char> out;
    memcpy(buf,&s,2);
    out.push_back(buf[1]);
    out.push_back(buf[0]);
    free(buf);
    return out;

}

