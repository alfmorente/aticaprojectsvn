/** 
 * @file  Thread.cpp
 * @brief Implementación de la clase "Thread"
 * @author Sergio Doctor
 * @date 2014
 */


#include "../include/Modulo_Conduccion/Thread.hpp"


/**
 * Constructor de la clase ThreadException
 * @param message Mensaje de error que aparacería si hubiera una excepción
 * @param inclSysMsg Variable que indica si se ha producido un error
 */

ThreadException::ThreadException(const string &message, bool inclSysMsg) throw () : userMessage(message) {
    if (inclSysMsg) {
        userMessage.append(": ");
        userMessage.append(strerror(errno));
    }
}

/**
 * Método que lanza la excepcion
 */

ThreadException::~ThreadException() throw () {
}

/**
 * Método que devuelve el mensaje de excepción
 * @return Devuelve el mensaje de excepción
 */

const char *ThreadException::what() const throw () {
    return userMessage.c_str();
}


/**
 * Destructor de la clase
 */

Thread::~Thread() {

}

/**
 * Método que indica si el hilo de ejecución está activo
 * @return Devuelve si el hilo está activo
 */
bool Thread::IsActive() {
    return active;
}

/**
 * Método que hace correr el hilo de ejecución
 */

void Thread::Run() throw (ThreadException) {

    if (pthread_create(&handler, NULL, Thread::Executer, (void*) this) != 0) {
        throw ThreadException("Thread could not be created", true);
    }

    active = true;
}


/**
 * Método que hace terminar el hilo de ejecución
 */

void Thread::Terminate() throw (ThreadException) {

    if (pthread_cancel(handler) == -1) {
        throw ThreadException("Thread could not be terminated", true);
    }

    active = false;
}

/**
 * Método que realiza la ejecución del hijo 
 * @param param Variable que se le pasa al hilo para realizar su ejecución
 * @return Devuelve la dirección de memoria donde está alojada el hilo
 */

void* Thread::Executer(void* param) {
    Thread *thread = (Thread*) param;
    thread->DoWork();
    
}