/*
 * libMyKinect.h
 *
 *  Created on: Dec 23, 2010
 *  Author: Andrea Grandi
 */

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "libfreenect_sync.h"
#include <libfreenect/libfreenect.h>

#include <stdio.h>
#include <stdlib.h>
#include <opencv/cv.h>
#include <math.h>

#ifndef LIBMYKINECT_H_
#define LIBMYKINECT_H_

typedef struct {
	int valid;
	float x;
	float y;
	float z;
	unsigned char blue;
	unsigned char green;
	unsigned char red;
	float Y;
	float U;
	float V;
} Point3d;

typedef Point3d PointCloud[480][640];

/**
 *	Carica i paramtetri di calibrazione del Kinect
 */
void loadParameters();

/**
 *	Inizializzazione della look-up table che associa ad ogni valore
 *	della depth una distanza in metri
 */
void initDepthLut();


/**
 *	Acquisisce i dati dal kinect e costruisce la pointcloud colorata
 *	proiettando nello spazio 3d i punti depth e riproiettandoli poi sull'immagine
 *	a colori.
 */
int createPointCloud(PointCloud dest);


/**
 *	Costruisce un'immagine 2d a colori a partire da una pointcloud
 *	Ad ogni punto associa il colore RGB del rispettivo punto sulla pointcloud
 */
void rgbImage(PointCloud src, IplImage *dst);

/**
 *	Costruisce un'immagine 2d in scala di grigi a partire da una pointcloud
 *	Ad ogni punto associa un'intensità proporzionale a quella del 
 *	rispettivo punto sulla pointcloud
 */
void depthImage(PointCloud src, IplImage *dst);


/**
 *	Costruisce un'immagine 2d in scala di grigi a partire da una pointcloud
 *	Ad ogni punto associa il valore 255 o 0 a seconda che il 
 *	rispettivo punto sulla pointcloud sia valido o meno
 */
void binaryImage(PointCloud src, IplImage *dst);

/**
 *	Restituisce la distanza del punto più vicino alla camera
 */
float getMinDistance(PointCloud pcl);


/**
 *	Invalida tutti i punti della pointcloud che si trovano ad una distanza
 *	minore di minDist e superiore a maxDist
 */
void pclDistThreshold(PointCloud pcl, float minDist, float maxDist);

/**
 *	Invalida tutti i punti della pointcloud
 */ 
void invalidatePcl(PointCloud pcl);

/**
 * Calcola la distanza euclidea tra le coordinate spaziali di due punti
 */
float spaceDistance(Point3d pt1, Point3d pt2);

/**
 * Calcola la distanza euclidea tra i colori RGB di due punti
 */
float colorRgbDistance(Point3d pt1, Point3d pt2);

/**
 *	Calcola la distanza euclidea tra i colori UV di due punti
 *	I valori di Y non sono considerati
 */
float colorUvDistance(Point3d pt1, Point3d pt2);

/**
 *	Calcola la distanza di Mahalanobis tra il colore del punto e il colore medio stimato,
 *	utilizzando l'inverso della matrice di covarianza precedentemente calcolata
 */
float colorMahalanobisDistance(Point3d point, double meanColorVec[], double inverseCovarMat[3][3]);

/**
 *	Calcola l'inverso di una matrice di dimensione 3x3
 */
double invertMat(double A[3][3], double X[3][3]);

/**
 *	Stampa i valori di una matrice su console
 */
void printMatrix(char* name, float** matrix, int order);


/**
 *	Calcola le coordinate spaziali del baricentro di una poincloud
 *	Tutti i punti hanno massa unitaria
 */
int barycenter(PointCloud src, double* x, double* y, double* z);


#endif /* LIBMYKINECT_H_ */



