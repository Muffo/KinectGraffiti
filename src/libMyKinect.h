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

typedef Point3d PointCloud[480][640];

int createPointCloud(PointCloud dest);
void loadParameters();
void initDepthLut();

void rgbImage(PointCloud src, IplImage *dst);
void depthImage(PointCloud src, IplImage *dst);
void binaryImage(PointCloud src, IplImage *dst);


float getMinDistance(PointCloud pcl);
void pclDistThreshold(PointCloud pcl, float minDist, float maxDist);
void invalidatePcl(PointCloud pcl);


float spaceDistance(Point3d pt1, Point3d pt2);
float colorRgbDistance(Point3d pt1, Point3d pt2);
float colorUvDistance(Point3d pt1, Point3d pt2);

float colorMahalanobisDistance(Point3d point, double meanColorVec[], double inverseCovarMat[3][3]);
double invertMat(double A[3][3], double X[3][3]);
void printMatrix(char* name, float** matrix, int order);


int barycenter(PointCloud src, double* x, double* y, double* z);


#endif /* LIBMYKINECT_H_ */



