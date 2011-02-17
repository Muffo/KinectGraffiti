/**
 * libSkin.h
 *
 * Author: Andrea Grandi
 */

 
#include "libMyKinect.h"

#define GLOBAL_SKIN_BLUE 207
#define GLOBAL_SKIN_GREEN 208
#define GLOBAL_SKIN_RED 239

#define GLOBAL_SKIN_COLOR_THRESHOLD 200

#define SAMPLE_VEC_SIZE 100000


/**
 *	Calcola la posizione del seed point utilizzando la soglia globale
 *	per il colore della pelle
 */
void getSkinSeed(PointCloud pcl, int* seedSkinX, int* seedSkinY);


/**
 *	Algoritmo di region growing: espande la point cloud aggiungendo i punti
 *	vicini che si trovano ad una distanza (calcolata con il metodo di Mahlanobis)
 *	inferiore alla soglia specificata
 */
void regionGrowing(PointCloud src, PointCloud dst, int threshold);



/**
 *	Acquisisce un campione di punti della mano per determinare la distribuzione
 *	statistica del colore della pelle.
 *	Il campione è centrato nel seedpoint ed è costituito da un rettangolo con lato
 *	di dimensione pari a 2 * sampleSize + 1
 *	I punti vengono aggiunti agli eventuali punti precedentementemente acquisiti.
 */
int grabSkinSample(PointCloud pcl, int seedSkinX, int seedSkinY, int sampleSize);


/**
 *	Resetta il vettore che contiene i punti di tutti i campioni fino ad ora acquisiti
 */
void resetSkinSample();



/** 
 *	Stampa le informazioni statistiche sul campione di punti fino ad ora acquisiti
 */
void printSampleInfo();
