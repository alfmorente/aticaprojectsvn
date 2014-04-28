#include <Modulo_Laser2D_Frontal/SocketCommunication.h>




SocketCommunication::SocketCommunication(bool* error)
{

	*error=false;
	//Se crea el socket de comunicaciones
	socketLaser = socket(AF_INET, SOCK_STREAM, 0);
        if (socketLaser < 0) 
	{
                cout <<"Socket creation failed"<<endl;
                *error=true;
        }
	

}

/**
	connectLaser: conecta con el laser
	Parametros de entrada:
		-puertoLaser: Puerto del laser al que se conecta
		-ipLaser: Ip del laser
	Parametro de salida: Booleano indicando si la conexión fue posible.
**/	
bool SocketCommunication::connectLaser(short puertoLaser,string ipLaser)
{
	struct  sockaddr_in socketAddr;
	
	socketAddr.sin_family=AF_INET;
	socketAddr.sin_addr.s_addr=inet_addr(ipLaser.c_str());
	socketAddr.sin_port=htons(puertoLaser);

	if (connect(socketLaser, (struct sockaddr *)&socketAddr, sizeof(socketAddr)) < 0) 
	{
                cout <<"Conection failed"<<endl;
                return false;
        }
	else
	{
		cout <<"Conexion establecida"<<endl;
		return true;
	}

}

/**
	disconnectLaser: desconecta con el laser y cierra el socket
	Parametros de entrada:Ninguno
	Parametro de salida: Ninguno
**/	
void SocketCommunication::disconnectLaser()
{
	close(socketLaser);
	shutdown(socketLaser,2);

}

/**
	createMessage: Crea el comando que se enviará al laser y lo guarda en la variable senderMessage
	Parametros de entrada:
		-typeCommand: tipo de comando
		-command: comando
		-parametros: parametros del comando
		-numParametros: numero de parametros
	Parametro de salida: Ninguno.
**/
void SocketCommunication::createMessage(string typeCommand, string command, string parametros[],int numParametros)
{
	senderMessage +=0x02;
	senderMessage +=typeCommand;
	senderMessage +=" ";
	senderMessage +=command;
	for(int i=0;i< numParametros;i++)
	{
		senderMessage += " ";
		senderMessage += parametros[i];
	}
	senderMessage += 0x03;
}


/**
	sendMessage: Envia el comando guardado en senderMessage al laser
	Parametros de entrada: Ninguno
	Parametro de salida: booleano que indica si se envió correctamente.
**/
bool SocketCommunication::sendMessage()
{
	cout << senderMessage <<endl;
	if(send(socketLaser,senderMessage.c_str(), senderMessage.length(),0)< 0)
	{
		cout <<"Error al enviar el mensaje"<<endl;
		return false;
	}

	senderMessage.clear();
	return true;
}

/**
	recvMessage: Recibe la respuesta del laser y lo guarda en receiverMessage
	Parametros de entrada: 
		- timeMaxWait: Tiempo máximo de espera de la respuesta
	Parametro de salida: booleano que indica si se recibió correctamente.
**/
bool SocketCommunication::recvMessage(float timeMaxWait)
{
	

	struct timeval Timeout;
	fd_set readfs;    
	FD_ZERO( &readfs);
	FD_SET(socketLaser, &readfs);  
	


	// fija el valor del temporizador en el bucle de entrada  
	Timeout.tv_usec = (timeMaxWait-(int)timeMaxWait)*100000.0;  /* microseconds */
	Timeout.tv_sec  = (int)timeMaxWait;  /* seconds */

	  char message;

	  receiverMessage.clear();
	  while(message!=0x02)
	  {
		if(select(socketLaser+1, &readfs, NULL, NULL, &Timeout)>0)
	  		recv(socketLaser,&message,1, 0);
		else
			return false;
	  }
	  receiverMessage += message;
	  while(message!=0x03)
	  {
		if(select(socketLaser+1, &readfs, NULL, NULL, &Timeout)>0)
		{
			recv(socketLaser,&message,1, 0);
			receiverMessage += message;
		}
		else
			return false;
		
	  }
	  //if(receiverMessage.length()>100)
      //cout <<receiverMessage<<endl;
	  return true;
}


/**
	compruebaRespuesta: Comprueba la respuesta del laser
	Parametros de entrada: 
		- typeWish: Tipo de comando deseado en la respuesta
		- commandWish: Comando deseado en la respuesta.
		- numParametros: numero de parametros deseado en la respuesta 
	Parametro de salida: booleano que indica si la respuesta es correcta.

	Nota: Para la trama de de medidas del lase como es variable no se comprueba el numero de parametros
**/
bool SocketCommunication::compruebaRespuesta(string typeWish,string commandWish,int numParametros)
{


	type.clear();
	command.clear();
	for(int i=0;i<4000;i++)
		data[i].clear();
	
	int tam=receiverMessage.length();
	string campoMensaje;
	if(receiverMessage[0]!=0x02)
		return false;

	

	int i=1;
	while(!isspace(receiverMessage[i]))
	{	
		type +=receiverMessage[i];
		i++;
	}
	if(type!=typeWish)
		return false;
	
	i++;
	while(!isspace(receiverMessage[i]) && receiverMessage[i]!=0x03)
	{
		command +=receiverMessage[i];
		i++;
	}
	

	
	if(command!=commandWish)
		return false;
	
	int indiceData=0;
	if(receiverMessage[i]!=0x03)
	{
		if(numParametros!=0)
		{
			i++;
			while(receiverMessage[i]!=0x03)
			{
				if(!isspace(receiverMessage[i]))
					data[indiceData] +=receiverMessage[i];
				else
					indiceData++;
				i++;
			}
		}
		else
			return false;
	}
	
	if(numParametros>0)
	{
		if(numParametros!=(indiceData+1))
			return false;
	}
	return true;
}
