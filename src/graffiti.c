/*
 * graffiti.c
 *
 *  Created on: Dec 23, 2010
 *      Author: Andrea Grandi
 */


#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <libfreenect/libfreenect.h>
#include "libfreenect_sync.h"
#include "libMyKinect.h"

freenect_context *f_ctx;
freenect_device *f_dev;
char *dataRgb;
uint16_t *dataDepth;
uint32_t timestamp;

double freenect_angle = 0;

PclImage curPcl;
PclImage bgPcl;

#define INIT_LEN 10
PclImage bgPclArray[INIT_LEN];



int getKinectData() {
	if (freenect_sync_get_rgb(&dataRgb, &timestamp) < 0) {
		printf("Errore: freenect_sync_get_rgb\n");
		return -1;
	}

	if (freenect_sync_get_depth(&dataDepth, &timestamp) < 0) {
		printf("Errore: freenect_sync_get_depth\n");
		return -1;
	}
	return 0;
}

void initBackground() {
	int i, u, v;

	printf("Background initialization: capture images");

	for (i=0; i<INIT_LEN; i++) {
		if (getKinectData()<0) {
			i--;
			continue;
		}
		buildPclImage(bgPclArray[i], dataRgb, dataDepth);
		free(dataDepth);
		free(dataRgb);

		cvWaitKey(10);
	}

	printf("\tDONE\nBackground initialization: elaboration");


	for(v=0; v<FREENECT_FRAME_H; v++) {
		for(u=0; u<FREENECT_FRAME_W; u++) {

			int validCount = 0;
			float blueAcc = 0;
			float redAcc = 0;
			float greenAcc = 0;
			float xAcc = 0;
			float yAcc = 0;
			float zAcc = 0;

			for (i=0; i<INIT_LEN; i++) {
				if (bgPclArray[i][v][u].valid) {
					validCount++;
					blueAcc += bgPclArray[i][v][u].blue;
					redAcc += bgPclArray[i][v][u].red;
					greenAcc += bgPclArray[i][v][u].green;
					xAcc += bgPclArray[i][v][u].x;
					yAcc += bgPclArray[i][v][u].y;
					zAcc += bgPclArray[i][v][u].z;
				}
			}

			if (validCount > (0.8 * INIT_LEN)) {
				bgPcl[v][u].valid = 1;
				bgPcl[v][u].blue = round(blueAcc/validCount);
				bgPcl[v][u].red = round(redAcc/validCount);
				bgPcl[v][u].green = round(greenAcc/validCount);
				bgPcl[v][u].x = xAcc/validCount;
				bgPcl[v][u].y = yAcc/validCount;
				bgPcl[v][u].z = zAcc/validCount;
			}
			else {
				bgPcl[v][u].valid = 0;
			}

		}
	}

	printf("\t\tDONE\n");

}


int main(int argc, char **argv)
{

	IplImage *imageMyRgb = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 3);
	IplImage *imageDepth = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 1);
	IplImage *spaceChange = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 1);
	IplImage *colorChange = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 1);
	IplImage *imageGraffiti = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 1);

	cvNamedWindow("MyRGB", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("Depth", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("SpaceChange", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("ColorChange", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("Graffiti", CV_WINDOW_AUTOSIZE);


	cvMoveWindow("MyRGB", 0, 0);
	cvMoveWindow("Depth", 0, 500);
	cvMoveWindow("SpaceChange", 640, 500);
	cvMoveWindow("ColorChange", 640, 0);
	cvMoveWindow("Graffiti", 1280, 0);


	initDepthLut();

 	freenect_sync_set_tilt_degs(freenect_angle);
 	freenect_sync_set_led(LED_RED);

 	initBackground();

	while (1) {
		int keyPressed = cvWaitKey(10);


		if (keyPressed==27) break;
		if (keyPressed > 0) {
			switch (keyPressed) {
				case 'w':
					freenect_angle++;
					if (freenect_angle > 30) {
						freenect_angle = 30;
					}
					break;
				case 's':
					freenect_angle--;
					if (freenect_angle < -30) {
						freenect_angle = -30;
					}
					break;
				case 'x':
					initBackground();
					break;
			}
			freenect_sync_set_tilt_degs(freenect_angle);
		}



		if (getKinectData()<0)
			continue;

		buildPclImage(curPcl, dataRgb, dataDepth);

		free(dataDepth);
		free(dataRgb);


		cvZero(spaceChange);
		cvZero(colorChange);
		cvZero(imageGraffiti);

		int u, v;
		for(v=0; v<FREENECT_FRAME_H; v++) {
			for(u=0; u<FREENECT_FRAME_W; u++) {
				if (bgPcl[v][u].valid && curPcl[v][u].valid) {

					int colorChanged = 0;
					int spaceChanged = 0;
					if(spaceDistance(bgPcl[v][u], curPcl[v][u]) > 0.10) {
						((unsigned char *) spaceChange->imageData)[v*FREENECT_FRAME_W + u] = 255;
						spaceChanged = 1;
					}

					if(colorDistance(bgPcl[v][u], curPcl[v][u]) > 100) {
						((unsigned char *) colorChange->imageData)[v*FREENECT_FRAME_W + u] = 255;
						colorChanged = 1;
					}

					if (colorChanged && !spaceChanged) {
						((unsigned char *) imageGraffiti->imageData)[v*FREENECT_FRAME_W + u] = 255;
					}
				}
			}
		}

		rgbImage(curPcl, imageMyRgb);
		depthImage(curPcl, imageDepth);


		cvShowImage("MyRGB", imageMyRgb);
		cvShowImage("Depth", imageDepth);
		cvShowImage("ColorChange", colorChange);
		cvShowImage("SpaceChange", spaceChange);
		cvShowImage("Graffiti", imageGraffiti);

    }

	freenect_sync_set_led(LED_YELLOW);

	cvReleaseImage(&imageMyRgb);
	cvReleaseImage(&imageDepth);
	cvDestroyAllWindows();

    freenect_sync_stop();
	freenect_sync_set_led(LED_BLINK_GREEN);

}
