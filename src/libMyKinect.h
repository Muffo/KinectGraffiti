/*
 * libMyKinect.h
 *
 *  Created on: Dec 23, 2010
 *      Author: muffo
 */


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
} Point3d;

typedef Point3d PclImage[480][640];

int buildPclImage(PclImage dest, char *dataRgb, uint16_t *dataDepth);
void initDepthLut();
void rgbImage(PclImage src, IplImage *dst);
void depthImage(PclImage src, IplImage *dst);


#endif /* LIBMYKINECT_H_ */



