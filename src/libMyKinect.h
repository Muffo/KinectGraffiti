/*
 * libMyKinect.h
 *
 *  Created on: Dec 23, 2010
 *      Author: muffo
 */

#ifndef LIBMYKINECT_H_
#define LIBMYKINECT_H_

#include "libfreenect.h"
#include <stdio.h>
#include <stdlib.h>
#include <opencv/cv.h>
#include <math.h>
#include "libfreenect_sync.h"


typedef struct {
	int valid;
	float x;
	float y;
	float z;

	unsigned char blue;
	unsigned char green;
	unsigned char red;
} Point3d;

typedef Point3d PclImage[FREENECT_FRAME_H][FREENECT_FRAME_W];


int buildPclImage(PclImage dest);
void initDepthLut();
void rgbImage(PclImage src, IplImage *dst);
void depthImage(PclImage src, IplImage *dst);


#endif /* LIBMYKINECT_H_ */
