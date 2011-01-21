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
#include "libMyKinect.h"

int saveFlag;		
double freenect_angle = 0;

PclImage curPcl;
PclImage bgPcl;

#define INIT_LEN 10


#define DISTANCE_THRESHOLD 0.04
#define COLOR_RGB_THRESHOLD 100
#define COLOR_UV_THRESHOLD 0.1
#define CHANGE_COUNT_THRESHOLD 5
#define AREA_THRESHOLD 30
unsigned char changeCount[FREENECT_FRAME_H][FREENECT_FRAME_W];




void initBackground() {
	int i, u, v;
	
	PclImage *bgPclArray = (PclImage *)malloc(INIT_LEN * sizeof(PclImage));

	printf("Background initialization: capture images\n");
	fflush(stdout);

	for (i=0; i<INIT_LEN; i++) {
		createPclImage(bgPclArray[i]);
		cvWaitKey(10);
	}

	printf("Background initialization: capture images - DONE\nBackground initialization: elaboration\n");
	fflush(stdout);


	for(v=0; v<FREENECT_FRAME_H; v++) {
		for(u=0; u<FREENECT_FRAME_W; u++) {

			int validCount = 0;
			float blueAcc = 0;
			float redAcc = 0;
			float greenAcc = 0;
			float xAcc = 0;
			float yAcc = 0;
			float zAcc = 0;
			float YAcc = 0;
			float UAcc = 0;
			float VAcc = 0;

			for (i=0; i<INIT_LEN; i++) {
				if (bgPclArray[i][v][u].valid) {
					validCount++;
					blueAcc += bgPclArray[i][v][u].blue;
					redAcc += bgPclArray[i][v][u].red;
					greenAcc += bgPclArray[i][v][u].green;
					xAcc += bgPclArray[i][v][u].x;
					yAcc += bgPclArray[i][v][u].y;
					zAcc += bgPclArray[i][v][u].z;
					YAcc += bgPclArray[i][v][u].Y;
					UAcc += bgPclArray[i][v][u].U;
					VAcc += bgPclArray[i][v][u].V;
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
				bgPcl[v][u].Y = YAcc/validCount;
				bgPcl[v][u].U = UAcc/validCount;
				bgPcl[v][u].V = VAcc/validCount;
			}
			else {
				bgPcl[v][u].valid = 0;
			}

			
			changeCount[v][u] = 0;
		}
	}

	free(bgPclArray);
	printf("Background initialization: elaboration - DONE\n");
	fflush(stdout);
	

}

int processKeyPressed(int keyPressed) {
	if (keyPressed==27) return 0;

	if (keyPressed > 0) {
		switch (keyPressed) {
			case 'w':
				freenect_angle++;
				if (freenect_angle > 30) {
					freenect_angle = 30;
				}
				freenect_sync_set_tilt_degs(freenect_angle);
				break;
			case 's':
				freenect_angle--;
				if (freenect_angle < -30) {
					freenect_angle = -30;
				}
				freenect_sync_set_tilt_degs(freenect_angle);
				break;
			case 'x':
				initBackground();
				break;
			case 'c':
				saveFlag = 1;
				break;
		}
	}
	return 1;
}



void initWindows() {	
	cvNamedWindow("MyRGB", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("Depth", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("SpaceChange", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("ColorChange", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("Graffiti", CV_WINDOW_AUTOSIZE);
	
	
	cvMoveWindow("MyRGB", 0, 0);
	cvMoveWindow("Depth", 0, 500);
	cvMoveWindow("SpaceChange", 640, 500);
	cvMoveWindow("ColorChange", 640, 0);
	cvMoveWindow("Graffiti", 640, 0);
	
}

int main(int argc, char **argv) {

	IplImage *imageMyRgb = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 3);
	IplImage *imageDepth = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 1);
	IplImage *spaceChange = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 1);
	IplImage *colorChange = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 1);
	IplImage *imageGraffiti = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 1);

	
//	freenect_sync_set_tilt_degs(freenect_angle);
 	freenect_sync_set_led(LED_RED);
	
	initWindows();
	initDepthLut();
	initUndistortMaps();
 	initBackground();
	
	while (processKeyPressed(cvWaitKey(10))) {
		
		uint32_t timestamp = createPclImage(curPcl);
	
		rgbImage(curPcl, imageMyRgb);
		depthImage(curPcl, imageDepth);
		
		cvZero(spaceChange);
		cvZero(colorChange);
		cvZero(imageGraffiti);

		int u, v;
		for(v=0; v<FREENECT_FRAME_H; v++) {
			for(u=0; u<FREENECT_FRAME_W; u++) {
				if (bgPcl[v][u].valid && curPcl[v][u].valid) {

					int colorChanged = 0;
					int spaceChanged = 0;
					if(spaceDistance(bgPcl[v][u], curPcl[v][u]) > DISTANCE_THRESHOLD) {
						((unsigned char *) spaceChange->imageData)[v*FREENECT_FRAME_W + u] = 255;
						spaceChanged = 1;
					}

					if(colorUvDistance(bgPcl[v][u], curPcl[v][u]) > COLOR_UV_THRESHOLD) {
						((unsigned char *) colorChange->imageData)[v*FREENECT_FRAME_W + u] = 255;
						colorChanged = 1;
					}

					if (colorChanged && !spaceChanged) {
						changeCount[v][u]++;
					}
					else {
						changeCount[v][u] = 0;
					}
					
					if(changeCount[v][u] > CHANGE_COUNT_THRESHOLD) {
						((unsigned char *) imageGraffiti->imageData)[v*FREENECT_FRAME_W + u] = 255;
					}
					else {
						((unsigned char *) imageGraffiti->imageData)[v*FREENECT_FRAME_W + u] = 0;
					}
					
				}
			}
		}
		
		cvDilate(imageGraffiti, imageGraffiti, NULL, 1);
		
		
		// labeling
		unsigned char label = 255;
		for(v=0; v<FREENECT_FRAME_H; v++) {
			for(u=0; u<FREENECT_FRAME_W; u++) {
				if (((unsigned char *) imageGraffiti->imageData)[v*FREENECT_FRAME_W + u] == 255) {
					label--;
					
					((unsigned char *) imageGraffiti->imageData)[v*FREENECT_FRAME_W + u] = label;
					int i,j,m,n,again;
					
					do {
						again = 0;
						for(j=0; j<FREENECT_FRAME_H; j++) {
							for(i=0; i<FREENECT_FRAME_W; i++) {
								if (((unsigned char *) imageGraffiti->imageData)[j*FREENECT_FRAME_W + i] == label) {
									for (n=j-1; n<=j+1; n++) {
										for (m=i-1; m<=i+1; m++) {
											if (((unsigned char *) imageGraffiti->imageData)[n*FREENECT_FRAME_W + m] == 255) {
												((unsigned char *) imageGraffiti->imageData)[n*FREENECT_FRAME_W + m] = label;
												again = 1;
											}
										}
									}
								}
							}
						}
						
						for(j=FREENECT_FRAME_H-1; j>=0; j--) {
							for(i=FREENECT_FRAME_W-1; i>=0; i--) {
								if (((unsigned char *) imageGraffiti->imageData)[j*FREENECT_FRAME_W + i] == label) {
									for (n=j-1; n<=j+1; n++) {
										for (m=i-1; m<=i+1; m++) {
											if (((unsigned char *) imageGraffiti->imageData)[n*FREENECT_FRAME_W + m] == 255) {
												((unsigned char *) imageGraffiti->imageData)[n*FREENECT_FRAME_W + m] = label;
												again = 1;
											}
										}
									}
								}
							}
						}
						
					} while (again);
					
				}
			}
		}
		
		int i, uMin, uMax, vMin, vMax, area;
		int graffitiCount = 0;
		for (i=254; i>=label; i--) {
			area = 0;
			uMin = FREENECT_FRAME_W;
			uMax = 0;
			vMin = FREENECT_FRAME_H;
			vMax = 0;
			for(v=0; v<FREENECT_FRAME_H; v++) {
				for(u=0; u<FREENECT_FRAME_W; u++) {
					if (((unsigned char *) imageGraffiti->imageData)[v*FREENECT_FRAME_W + u] == i) {
						area++;
						if (v > vMax) vMax = v;
						if (v < vMin) vMin = v;
						if (u > uMax) uMax = u;
						if (u < uMin) uMin = u;
					}
				}
			}
			if (area > AREA_THRESHOLD) {
				cvRectangle(imageMyRgb, cvPoint(uMin, vMin), cvPoint(uMax, vMax), cvScalar(0, 255, 0, 0), 2, 8, 0);
				graffitiCount++;
			}
		}
		
		CvFont font;
		cvInitFont(&font, CV_FONT_HERSHEY_PLAIN, 1, 1, 0, 1, 8);
		
		char outString[50];
		sprintf(outString, "%lu - Graffiti count: %d", timestamp, graffitiCount);
		cvPutText(imageMyRgb, outString, cvPoint(30,30), &font, cvScalar(0,255, 0, 0));
	
		
		cvShowImage("MyRGB", imageMyRgb);
//		cvShowImage("Depth", imageDepth);
//		cvShowImage("ColorChange", colorChange);
//		cvShowImage("SpaceChange", spaceChange);
		cvShowImage("Graffiti", imageGraffiti);
		
		
		if (saveFlag) {
			saveFlag = 0;
			char fileName[30];
			sprintf(fileName, "img-%lu.jpg", timestamp);
			cvSaveImage(fileName, imageMyRgb, NULL);
		}

    }
	

	freenect_sync_set_led(LED_YELLOW);

	cvReleaseImage(&imageMyRgb);
	cvReleaseImage(&imageDepth);
	cvDestroyAllWindows();

    freenect_sync_stop();
	freenect_sync_set_led(LED_BLINK_GREEN);
	
	return 1;

}
