/**
 *	Author: Andrea Grandi
 *
 *	Funzioni per la simulazione di eventi del mouse.
 *	Ogni funzione richiede le coordinate x,y del punto sullo 
 *	schermo in cui si verifica l'evento
 */


#include <ApplicationServices/ApplicationServices.h>

void rightButtonPressed(int x, int y);
void rightButtonReleased(int x, int y);

void leftButtonPressed(int x, int y);
void leftButtonReleased(int x, int y);

void verticalScroll(int x, int y);
void moveCursor(int x, int y);


