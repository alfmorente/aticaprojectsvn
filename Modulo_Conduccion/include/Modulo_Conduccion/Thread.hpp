#ifndef _THREAD_HPP
#define _THREAD_HPP

#include <pthread.h>
#include <errno.h>
#include <string.h> 
#include <exception>
#include <string>

using namespace std;

class ThreadException : public exception {
public:
    ThreadException(const string &message, bool inclSysMsg = false) throw ();

    ~ThreadException() throw ();

    const char *what() const throw ();

private:
    string userMessage;
};

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
