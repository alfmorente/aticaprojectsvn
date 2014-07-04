#include "../../include/Modulo_GPS/external_signals.h"

// Funciones tratamiento de señales
// what has to be done at program exit
void do_exit(int error)
{
  printf("finished GPS (%d).\n\n", error);
  exit(error);
}

// the signal handler for manual break Ctrl-C
void signal_handler(int signal)
{
  do_exit(0);
}

// what has to be done at program start
void init_signals()
{
  /* install signal handlers */
  signal(SIGTERM, signal_handler);
  signal(SIGINT, signal_handler);
}

