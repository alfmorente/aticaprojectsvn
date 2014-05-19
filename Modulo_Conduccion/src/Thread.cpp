#include "../include/Modulo_Conduccion/Thread.hpp"

ThreadException::ThreadException(const string &message, bool inclSysMsg) throw () : userMessage(message) {
    if (inclSysMsg) {
        userMessage.append(": ");
        userMessage.append(strerror(errno));
    }
}

ThreadException::~ThreadException() throw () {
}

const char *ThreadException::what() const throw () {
    return userMessage.c_str();
}

Thread::~Thread() {

}

bool Thread::IsActive() {
    return active;
}

void Thread::Run() throw (ThreadException) {

    if (pthread_create(&handler, NULL, Thread::Executer, (void*) this) != 0) {
        throw ThreadException("Thread could not be created", true);
    }

    active = true;
}

void Thread::Terminate() throw (ThreadException) {

    if (pthread_cancel(handler) == -1) {
        throw ThreadException("Thread could not be terminated", true);
    }

    active = false;
}

void* Thread::Executer(void* param) {
    Thread *thread = (Thread*) param;
    thread->DoWork();
    
}