#include <stdio.h>
#include <iostream>
#include <fstream>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h> 
#include <string.h>

#include "../include/Modulo_Conduccion/mastil.hpp"

using namespace std;


//DATOS FOCO DERECHO
void datosFocoDerecho () {
	focoDerecho [0] = 0x02;
	focoDerecho [1] = 0x01;
	focoDerecho [2] = 0x0C;
	focoDerecho [3] = 0x96;
	focoDerecho [4] = 0x00;
	focoDerecho [5] = 0x10;
	focoDerecho [6] = 0xB5;
	focoDerecho [7] = 0xFF;
	focoDerecho [8] = 0xFF;
	focoDerecho [9] = 0x03;
	focoDerecho [10] = 0x02;
	focoDerecho [11] = 0x01;
	focoDerecho [12] = 0x0C;
	focoDerecho [13] = 0x96;
	focoDerecho [14] = 0x10;
	focoDerecho [15] = 0x04;
	focoDerecho [16] = 0xB9;
	focoDerecho [17] = 0x00;
	focoDerecho [18] = 0x00;
	focoDerecho [19] = 0x03;
	focoDerecho [20] = 0x02;
	focoDerecho [21] = 0x01;
	focoDerecho [22] = 0x0C;
	focoDerecho [23] = 0x96;
	focoDerecho [24] = 0x00;
	focoDerecho [25] = 0x00;
	focoDerecho [26] = 0xA5;
	focoDerecho [27] = 0xFF;
	focoDerecho [28] = 0xFF;
	focoDerecho [29] = 0x03;
	focoDerecho [30] = 0x02;
	focoDerecho [31] = 0x01;
	focoDerecho [32] = 0x0C;
	focoDerecho [33] = 0x96;
	focoDerecho [34] = 0x00;
	focoDerecho [35] = 0x04;
	focoDerecho [36] = 0xA9;
	focoDerecho [37] = 0x00;
	focoDerecho [38] = 0x00;
	focoDerecho [39] = 0x03;
}


//DATOS FOCO IZQUIERDO
void datosFocoIzquierdo () {
	focoIzquierdo [0] = 0x02;
	focoIzquierdo [1] = 0x01;
	focoIzquierdo [2] = 0x0C;
	focoIzquierdo [3] = 0x96;
	focoIzquierdo [4] = 0x00;
	focoIzquierdo [5] = 0x20;
	focoIzquierdo [6] = 0xC5;
	focoIzquierdo [7] = 0xFF;
	focoIzquierdo [8] = 0xFF;
	focoIzquierdo [9] = 0x03;
	focoIzquierdo [10] = 0x02;
	focoIzquierdo [11] = 0x01;
	focoIzquierdo [12] = 0x0C;
	focoIzquierdo [13] = 0x96;
	focoIzquierdo [14] = 0x20;
	focoIzquierdo [15] = 0x04;
	focoIzquierdo [16] = 0xC9;
	focoIzquierdo [17] = 0x00;
	focoIzquierdo [18] = 0x00;
	focoIzquierdo [19] = 0x03;
	focoIzquierdo [20] = 0x02;
	focoIzquierdo [21] = 0x01;
	focoIzquierdo [22] = 0x0C;
	focoIzquierdo [23] = 0x96;
	focoIzquierdo [24] = 0x00;
	focoIzquierdo [25] = 0x00;
	focoIzquierdo [26] = 0xA5;
	focoIzquierdo [27] = 0xFF;
	focoIzquierdo [28] = 0xFF;
	focoIzquierdo [29] = 0x03;
	focoIzquierdo [30] = 0x02;
	focoIzquierdo [31] = 0x01;
	focoIzquierdo [32] = 0x0C;
	focoIzquierdo [33] = 0x96;
	focoIzquierdo [34] = 0x00;
	focoIzquierdo [35] = 0x04;
	focoIzquierdo [36] = 0xA9;
	focoIzquierdo [37] = 0x00;
	focoIzquierdo [38] = 0x00;
	focoIzquierdo [39] = 0x03;
}


//DATOS Tilt ABAJO
void datosTiltAbajo() {
	tiltAbajo [0] = 0x02;
	tiltAbajo [1] = 0x01;
	tiltAbajo [2] = 0x0C;
	tiltAbajo [3] = 0x96;
	tiltAbajo [4] = 0x04;
	tiltAbajo [5] = 0x00;
	tiltAbajo [6] = 0xA9;
	tiltAbajo [7] = 0xFF;
	tiltAbajo [8] = 0xFF;
	tiltAbajo [9] = 0x03;
	tiltAbajo [10] = 0x02;
	tiltAbajo [11] = 0x01;
	tiltAbajo [12] = 0x0C;
	tiltAbajo [13] = 0x96;
	tiltAbajo [14] = 0x04;
	tiltAbajo [15] = 0x04;
	tiltAbajo [16] = 0xAD;
	tiltAbajo [17] = 0x00;
	tiltAbajo [18] = 0x00;
	tiltAbajo [19] = 0x03;
}


//DATOS Tilt ARRIBA
void datosTiltArriba() {
	tiltArriba [0] = 0x02;
	tiltArriba [1] = 0x01;
	tiltArriba [2] = 0x0C;
	tiltArriba [3] = 0x96;
	tiltArriba [4] = 0x02;
	tiltArriba [5] = 0x00;
	tiltArriba [6] = 0xA7;
	tiltArriba [7] = 0xFF;
	tiltArriba [8] = 0xFF;
	tiltArriba [9] = 0x03;
	tiltArriba [10] = 0x02;
	tiltArriba [11] = 0x01;
	tiltArriba [12] = 0x0C;
	tiltArriba [13] = 0x96;
	tiltArriba [14] = 0x02;
	tiltArriba [15] = 0x04;
	tiltArriba [16] = 0xAB;
	tiltArriba [17] = 0x00;
	tiltArriba [18] = 0x00;
	tiltArriba [19] = 0x03;
}


//DATOS Pan IZQUIERDA
void datosPanIzquierda() {
	panIzquierda [0] = 0x02;
	panIzquierda [1] = 0x01;
	panIzquierda [2] = 0x0C;
	panIzquierda [3] = 0x96;
	panIzquierda [4] = 0x00;
	panIzquierda [5] = 0x04;
	panIzquierda [6] = 0xA9;
	panIzquierda [7] = 0xFF;
	panIzquierda [8] = 0xFF;
	panIzquierda [9] = 0x03;
	panIzquierda [10] = 0x02;
	panIzquierda [11] = 0x01;
	panIzquierda [12] = 0x0C;
	panIzquierda [13] = 0x96;
	panIzquierda [14] = 0x04;
	panIzquierda [15] = 0x04;
	panIzquierda [16] = 0xAD;
	panIzquierda [17] = 0x00;
	panIzquierda [18] = 0x00;
	panIzquierda [19] = 0x03;
}


//DATOS Pan DERECHA
void datosPanDerecha() {
	panDerecha [0] = 0x02;
	panDerecha [1] = 0x01;
	panDerecha [2] = 0x0C;
	panDerecha [3] = 0x96;
	panDerecha [4] = 0x00;
	panDerecha [5] = 0x40;
	panDerecha [6] = 0xE5;
	panDerecha [7] = 0xFF;
	panDerecha [8] = 0xFF;
	panDerecha [9] = 0x03;
	panDerecha [10] = 0x02;
	panDerecha [11] = 0x01;
	panDerecha [12] = 0x0C;
	panDerecha [13] = 0x96;
	panDerecha [14] = 0x40;
	panDerecha [15] = 0x04;
	panDerecha [16] = 0xE9;
	panDerecha [17] = 0x00;
	panDerecha [18] = 0x00;
	panDerecha [19] = 0x03;
}


//OFF --> valido para parar el tilt hacia arriba y abajo y el pan hacia la derecha y la izquierda
void datosOff() {
	off [0] = 0x02;
	off [1] = 0x01;
	off [2] = 0x0C;
	off [3] = 0x96;
	off [4] = 0x00;
	off [5] = 0x00;
	off [6] = 0xA5;
	off [7] = 0xFF;
	off [8] = 0xFF;
	off [9] = 0x03;
	off [10] = 0x02;
	off [11] = 0x01;
	off [12] = 0x0C;
	off [13] = 0x96;
	off [14] = 0x00;
	off [15] = 0x04;
	off [16] = 0xA9;
	off [17] = 0x00;
	off [18] = 0x00;
	off [19] = 0x03;
}


//DATOS BAJAR MASTIL
void datosBajarMastil () {
	bajarMastil [0] = 0x02;
	bajarMastil [1] = 0x01;
	bajarMastil [2] = 0x0C;
	bajarMastil [3] = 0x96;
	bajarMastil [4] = 0x01;
	bajarMastil [5] = 0x00;
	bajarMastil [6] = 0xA6;
	bajarMastil [7] = 0xFF;
	bajarMastil [8] = 0xFF;
	bajarMastil [9] = 0x03;
	bajarMastil [10] = 0x02;
	bajarMastil [11] = 0x01;
	bajarMastil [12] = 0x0C;
	bajarMastil [13] = 0x96;
	bajarMastil [14] = 0x01;
	bajarMastil [15] = 0x04;
	bajarMastil [16] = 0xAA;
	bajarMastil [17] = 0x00;
	bajarMastil [18] = 0x00;
	bajarMastil [19] = 0x03;
	bajarMastil [20] = 0x02;
	bajarMastil [21] = 0x01;
	bajarMastil [22] = 0x0C;
	bajarMastil [23] = 0x96;
	bajarMastil [24] = 0x00;
	bajarMastil [25] = 0x00;
	bajarMastil [26] = 0xA5;
	bajarMastil [27] = 0xFF;
	bajarMastil [28] = 0xFF;
	bajarMastil [29] = 0x03;
	bajarMastil [30] = 0x02;
	bajarMastil [31] = 0x01;
	bajarMastil [32] = 0x0C;
	bajarMastil [33] = 0x96;
	bajarMastil [34] = 0x00;
	bajarMastil [35] = 0x04;
	bajarMastil [36] = 0xA9;
	bajarMastil [37] = 0x00;
	bajarMastil [38] = 0x00;
	bajarMastil [39] = 0x03;
}


//DATOS SUBIR MASTIL
void datosSubirMastil () {
	subirMastil [0] = 0x02;
	subirMastil [1] = 0x01;
	subirMastil [2] = 0x0C;
	subirMastil [3] = 0x96;
	subirMastil [4] = 0x00;
	subirMastil [5] = 0x01;
	subirMastil [6] = 0xA6;
	subirMastil [7] = 0xFF;
	subirMastil [8] = 0xFF;
	subirMastil [9] = 0x03;
	subirMastil [10] = 0x02;
	subirMastil [11] = 0x01;
	subirMastil [12] = 0x0C;
	subirMastil [13] = 0x96;
	subirMastil [14] = 0x01;
	subirMastil [15] = 0x04;
	subirMastil [16] = 0xAA;
	subirMastil [17] = 0x00;
	subirMastil [18] = 0x00;
	subirMastil [19] = 0x03;
	subirMastil [20] = 0x02;
	subirMastil [21] = 0x01;
	subirMastil [22] = 0x0C;
	subirMastil [23] = 0x96;
	subirMastil [24] = 0x00;
	subirMastil [25] = 0x00;
	subirMastil [26] = 0xA5;
	subirMastil [27] = 0xFF;
	subirMastil [28] = 0xFF;
	subirMastil [29] = 0x03;
	subirMastil [30] = 0x02;
	subirMastil [31] = 0x01;
	subirMastil [32] = 0x0C;
	subirMastil [33] = 0x96;
	subirMastil [34] = 0x00;
	subirMastil [35] = 0x04;
	subirMastil [36] = 0xA9;
	subirMastil [37] = 0x00;
	subirMastil [38] = 0x00;
	subirMastil [39] = 0x03;
}


//STOP
void datosStop() {
	stop [0] = 0x02;
	stop [1] = 0x01;
	stop [2] = 0x0C;
	stop [3] = 0x96;
	stop [4] = 0x10;
	stop [5] = 0x00;
	stop [6] = 0xB5;
	stop [7] = 0xFF;
	stop [8] = 0xFF;
	stop [9] = 0x03;
}