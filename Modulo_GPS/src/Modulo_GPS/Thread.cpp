/**
  @file Thread.cpp
  @brief Implementación de la clase TeachThread y ThreadException
  @author Carlos Amores
  @date 2013,2014,2015
*/

#include "Modulo_GPS/Thread.hpp"

/**
 * Constructor de la clase ThreadException
 * @param message Mensaje a mostrar al producirse la excepción
 * @param inclSysMsg Mensaje que indica si la excepción se ha producido por el sistema
 */
ThreadException::ThreadException(const string &message, bool inclSysMsg) throw () : userMessage(message) {
    if (inclSysMsg) {
        userMessage.append(": ");
        userMessage.append(strerror(errno));
    }
}

/**
 * Destructor de la clase ThreadException
 */
ThreadException::~ThreadException() throw () {
}

/**
 * Método público que indica la causa de la excepción
 * @return Cadena a mostrar como causa
 */
const char *ThreadException::what() const throw () {
    return userMessage.c_str();
}

/**
 * Destructor de la clase Thread
 */
Thread::~Thread() {

}

/**
 * Método público consultor del atributo active de la clase Thread que indica si
 * el hilo se encuentra en ejecución
 * @return Atributo active de la clase Thread
 */
bool Thread::IsActive() {
    return active;
}

/**
 * Método principal de ejecución del hilo
 */
void Thread::Run() throw (ThreadException) {

    if (pthread_create(&handler, NULL, Thread::Executer, (void*) this) != 0) {
        throw ThreadException("Thread could not be created", true);
    }

    active = true;
}

/**
 * Método público que cancela la ejecución del hilo
 */
void Thread::Terminate() throw (ThreadException) {

    if (pthread_cancel(handler) == -1) {
        throw ThreadException("Thread could not be terminated", true);
    }

    active = false;
}

/**
 * Método público que lanza la ejecución del método principal del hilo
 * @param param Parámetros de entrada del hilo (si los hubiera)
 * @return Retorno del hilo (si lo hubiera)
 */
void* Thread::Executer(void* param) {
    Thread *thread = (Thread*) param;
    thread->DoWork();
}