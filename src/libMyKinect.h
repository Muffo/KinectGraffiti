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
float spaceDistance(Point3d pt1, Point3d pt2);
float colorRgbDistance(Point3d pt1, Point3d pt2);
float colorUvDistance(Point3d pt1, Point3d pt2);

void barycenter(PclImage src, float* x, float* y, float* z);


#endif /* LIBMYKINECT_H_ */



