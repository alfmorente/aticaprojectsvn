#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <sys/time.h>

#define SERIAL_OK 0
#define SERIAL_BAD 1
#define SERIAL_TIMEOUT 2
#define FRAME_FAILED 3

class PortSerial
{

	private:
		int descriptorSerie;
		struct termios oldtio,newtio;

	public:
		PortSerial();
		bool send(char* bufferOut,int tam);
		int recv(char* bufferIn, int tam,int timeout);
		bool openSerial(char* nombre);
		void closeSerial();
		void clean();
		void configura(int velocity);
};