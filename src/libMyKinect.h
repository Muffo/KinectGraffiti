/*
 * libMyKinect.h
 *
 *  Created on: Dec 23, 2010
 *      Author: muffo
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

typedef Point3d PclImage[480][640];

int createPclImage(PclImage dest);
void initUndistortMaps();
void initDepthLut();

void rgbImage(PclImage src, IplImage *dst);
void depthImage(PclImage src, IplImage *dst);
void bwImage(PclImage src, IplImage *dst);


float getMinDistance(PclImage pcl);
void pclDistThreshold(PclImage pcl, float minDist, float maxDist);
void invalidatePcl(PclImage pcl);


float spaceDistance(Point3d pt1, Point3d pt2);
float colorRgbDistance(Point3d pt1, Point3d pt2);
float colorUvDistance(Point3d pt1, Point3d pt2);

float colorMahalanobisDistance(Point3d point, double meanColorVec[], double inverseCovarMat[3][3]);
double invertMat(double A[3][3], double X[3][3]);
void printMatrix(char* name, float** matrix, int order);


int barycenter(PclImage src, double* x, double* y, double* z);


#endif /* LIBMYKINECT_H_ */



