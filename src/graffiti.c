/*
 * graffiti.c
 *
 *  Created on: Dec 23, 2010
 *      Author: muffo
 */


#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <libfreenect/libfreenect.h>
#include "libfreenect_sync.h"
#include "libMyKinect.h"


double freenect_angle = 0;

int main(int argc, char **argv)
{

	IplImage *imageRgb = cvCreateImageHeader(cvSize(FREENECT_FRAME_W, FREENECT_FRAME_H), 8, 3);
	IplImage *imageMyRgb = cvCreateImage(cvSize(FREENECT_FRAME_W, FREENECT_FRAME_H), 8, 3);
	IplImage *imageDepth = cvCreateImage(cvSize(FREENECT_FRAME_W, FREENECT_FRAME_H), 8, 1);

	PclImage bgPcl, curPcl;

	// cvNamedWindow("RGB", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("MyRGB", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("Depth", CV_WINDOW_AUTOSIZE);

	// cvMoveWindow("RGB", 0, 0);
	cvMoveWindow("MyRGB", 0, 0);
	cvMoveWindow("Depth", 0, 500);


	initDepthLut();
	int backgroundInit = 1;

 	// freenect_sync_set_tilt_degs(freenect_angle);
 	// freenect_sync_set_led(LED_RED);

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
					backgroundInit = 1;
					break;
			}
			freenect_sync_set_tilt_degs(freenect_angle);
		}


		if (backgroundInit) {
			buildPclImage(bgPcl);
			backgroundInit = 0;
		}

		buildPclImage(curPcl);

		int u, v;
		for(v=0; v<FREENECT_FRAME_H; v++) {
			for(u=0; u<FREENECT_FRAME_W; u++) {
				if (bgPcl[v][u].valid && curPcl[v][u].valid) {
					if(abs(bgPcl[v][u].blue - curPcl[v][u].blue) > 100) {
						curPcl[v][u].blue = 0;
						curPcl[v][u].red = 255;
						curPcl[v][u].green = 0;
					}
				}
			}
		}

		// immagine rgb
		// cvSetData(imageRgb, dataRgb, 640*3);
		// cvCvtColor(imageRgb, imageRgb, CV_RGB2BGR);


		rgbImage(curPcl, imageMyRgb);
		depthImage(curPcl, imageDepth);


		// cvShowImage("RGB", imageRgb);
		cvShowImage("MyRGB", imageMyRgb);
		cvShowImage("Depth", imageDepth);

    }

	freenect_sync_set_led(LED_YELLOW);

	cvReleaseImage(&imageMyRgb);
	cvReleaseImage(&imageDepth);
	cvDestroyAllWindows();

    freenect_sync_stop();
	freenect_sync_set_led(LED_BLINK_GREEN);

}
