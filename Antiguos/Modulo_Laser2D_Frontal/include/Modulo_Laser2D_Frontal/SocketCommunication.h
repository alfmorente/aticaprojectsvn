#include <Modulo_Laser2D_Frontal/TypeConverter.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>



//SocketCommunication: Clase que se encarga de todo lo relativo al manejo de  de las tramas que se intercambian con el Laser
class SocketCommunication
{

	public:
		int socketLaser;
		string senderMessage;     //Mensaje a enviar
		string receiverMessage;   //mensaje recibido

		string type;		  //tipo recibido
		string command;		  //comando recibido
		string data[4000];	  //tabla de datos recibidos	
	public:
		SocketCommunication(bool*);
		bool connectLaser(short puertoLaser,string ipLaser);  
		void disconnectLaser();
		void createMessage(string,string,string[],int);
		bool sendMessage();
		bool recvMessage(float tiempoMaximoEspera);
		bool compruebaRespuesta(string typeWish,string commandWish,int numParametros);
};

