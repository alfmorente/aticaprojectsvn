
/** 
 * @file  TeachThread.h
 * @brief Declara el tipo de la clase "TeachThread"
 * - La clase hereda de la clase Thread y representa un hilo de ejecución
 * alternativo al principal de programa que se comunica con el mismo para la 
 * obtención de puntos del modo teach y si almacenamiento en caso de estar activo.
 * @author Carlos Amores
 * @date 2013, 2014, 2015
 * @addtogroup GPS
 * @{
 */

#ifndef TEACHTHREAD_HPP
#define	TEACHTHREAD_HPP

#include "Thread.hpp"
#include <queue>
#include <cstdlib>
#include <tinyxml.h>
#include <sstream>
#include <string>
#include "TeachString.h"

/**
 * \struct TeachData
 * \brief Estructura para almacenamiento coordenadas
 */
typedef struct {
    float latitude;
    float longitude;
} TeachData;

/**
 * \class TeachThread
 * \brief Clase que representa el hilo de ejecución del modo Teach
 */
class TeachThread : public Thread {
public:
    TeachThread();
    virtual ~TeachThread();
    virtual void DoWork();
    void setMode(bool);
    bool dataReady();
    // Atributos cola de datos
    queue<TeachData> queueGPSdata;
    vector<string> getTeaches();
private:
    TeachString *teachSt;
    bool mode_active; // Flag de estado
    vector<string> teaches;
    bool ready;

};

#endif	/* TEACHTHREAD_HPP */

/**
 * @}
 */