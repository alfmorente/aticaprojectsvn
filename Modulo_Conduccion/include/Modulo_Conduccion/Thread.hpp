/** 
 * @file  Thread.hpp
 * @brief Declara el tipo de la clase "Thread"
 * - La clase implementa el hilo de ejecución de la clase CANCommunication
 * @author Sergio Doctor 
 * @date 2014
 * @addtogroup 
 * @{
 */


#ifndef _THREAD_HPP
#define _THREAD_HPP

#include <pthread.h>
#include <errno.h>
#include <string.h> 
#include <exception>
#include <string>

using namespace std;

/**
 * \class ThreadException
 * \brief Clase que representa un mensaje de excepción por si la ejecución del hilo diera un errror
 */

class ThreadException : public exception {
public:
    ThreadException(const string &message, bool inclSysMsg = false) throw ();

    ~ThreadException() throw ();

    const char *what() const throw ();

private:
    string userMessage;
};


/**
 * \class Thread
 * \brief Clase que representa el hilo de ejecución a través del cual se van a escribir/leer mensajes CAN para luego procesarlos.
 */

class Thread {
private:
    static void* Executer(void* param);

protected:
    bool active;

    virtual void DoWork() = 0;

public:
    pthread_t handler;

    virtual ~Thread();

    void Run() throw (ThreadException);

    void Terminate() throw (ThreadException);

    bool IsActive();
};

#endif
