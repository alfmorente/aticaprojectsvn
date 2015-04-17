/**
  @file external_signals.cpp
  @brief Implementación de la colección de funciones de tratamiento de señales
  @author Carlos Amores
  @date 2013,2014,2015
*/

#include "../../include/Modulo_GPS/external_signals.h"

/**
 * Método que realiza las acciones necesarias previas a la finalización del hilo
 * @param[in] error Causa de ejecución de la rutina de tratamiento de la señal
 */
void do_exit(int error)
{
  printf("finished GPS (%d).\n\n", error);
  exit(error);
}

/**
 * Método capturador de la señal de interrupción
 * @param[in] signal Causa de la interrupción
 */
void signal_handler(int signal)
{
  do_exit(0);
}

/**
 * Método de asignación de señales a sus respectivas rutinas de incialización
 */
void init_signals()
{
  /* install signal handlers */
  signal(SIGTERM, signal_handler);
  signal(SIGINT, signal_handler);
}

