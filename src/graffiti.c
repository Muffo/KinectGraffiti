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

double freenect_angle = 0;

PclImage curPcl;
PclImage bgPcl;

int main(int argc, char **argv)
{

	IplImage *imageMyRgb = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 3);
	IplImage *imageDepth = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 1);


	cvNamedWindow("MyRGB", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("Depth", CV_WINDOW_AUTOSIZE);

	cvMoveWindow("MyRGB", 0, 0);
	cvMoveWindow("Depth", 0, 500);


	initDepthLut();
	int backgroundInit = 1;

 	freenect_sync_set_tilt_degs(freenect_angle);
 	freenect_sync_set_led(LED_RED);

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


		char *dataRgb;
		uint16_t *dataDepth;
		uint32_t timestamp;

		// ottengo i dati dal kinect
		if (freenect_sync_get_rgb(&dataRgb, &timestamp) < 0) {
			printf("Errore: freenect_sync_get_rgb\n");
			return -1;
		}

		if (freenect_sync_get_depth(&dataDepth, &timestamp) < 0) {
			printf("Errore: freenect_sync_get_depth\n");
			return -1;
		}


		if (backgroundInit) {
			buildPclImage(bgPcl, dataRgb, dataDepth);
			backgroundInit = 0;
		}

		buildPclImage(curPcl, dataRgb, dataDepth);

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

		rgbImage(curPcl, imageMyRgb);
		depthImage(curPcl, imageDepth);


		// cvShowImage("RGB", imageRgb);
		cvShowImage("MyRGB", imageMyRgb);
		cvShowImage("Depth", imageDepth);

		free(dataDepth);
		free(dataRgb);
    }

	freenect_sync_set_led(LED_YELLOW);

	cvReleaseImage(&imageMyRgb);
	cvReleaseImage(&imageDepth);
	cvDestroyAllWindows();

    freenect_sync_stop();
	freenect_sync_set_led(LED_BLINK_GREEN);

}
