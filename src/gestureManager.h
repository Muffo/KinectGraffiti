/**
 * Author: Andrea Grandi
 */


/**
 *	A partire dalla pointcloud contenente la mano effettua una nuova
 *	stima sulla posizione e sull'orientamento
 */
void updateHandPosition(PointCloud handPcl);

/**
 *	Stampa le informazioni sulla posizione della mano su un'immagine
 */
void imagePrintHandInfo(IplImage *image, CvFont font);

/**
 *	Restituisce 1 o 0 a seconda che la mano sia aperta o chiusa
 *	La stima viene fatta sulla base del fattore di forma
 */
int openHand(PointCloud handPcl);


/**
 *	Scatena gli eventi per il mouse in base alla posizione corrente della mano
 */
void moveMouse();