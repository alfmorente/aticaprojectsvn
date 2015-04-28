
/** 
 * @file  Thread.hpp
 * @brief Declara el tipo de la clase "Thread" y "ThreadException"
 * - La clase declara la funcionalidad de un hilo de ejecución paralelo y su
 * excepción asociada a posibles errores
 * @author Carlos Amores
 * @date 2013, 2014, 2015
 * @addtogroup GPS
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
 * \brief Clase que representa la excepción generada ante problemas en la ejecución
 * del hilo
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
 * \brief Clase que representa un hilo de ejecución
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

/**
 * @}
 */