/*
 * hand.c
 *
 *  Created on: Dec 23, 2010
 *      Author: Andrea Grandi
 */


#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <ApplicationServices/ApplicationServices.h>
#include <unistd.h>
#include "libMyKinect.h"
#include "libSkin.h"
#include "eig3.h"
#include "gestureManager.h"

#define DEFAULT_SAMPLE_SIZE 30
#define DISTANCE_THRESHOLD 0.2


int saveFlag;
int displayGuideFlag = 1;
int sampleSize = DEFAULT_SAMPLE_SIZE;
int grabSkinSampleFlag = 0;
int handDetectionFlag = 0;
int gestureFlag = 0;

int specSkinColorThreshold = 8;

double freenect_angle = 0;

PointCloud curPcl;
PointCloud handPcl;

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
			case 'c':
				saveFlag = 1;
				break;
			case 'd':
				displayGuideFlag = !displayGuideFlag;
				break;
			case 'x':
				grabSkinSampleFlag = 1;
				break;
			case 'z':
				resetSkinSample();
				handDetectionFlag = 0;
				break;
			case 'm':
				gestureFlag = !gestureFlag;
				break;
		}
	}

	return 1;
}


void initWindows() {	
	cvNamedWindow("MyRGB", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("Depth", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("Hand", CV_WINDOW_AUTOSIZE);
	
	
	cvMoveWindow("MyRGB", 0, 0);
	cvMoveWindow("Depth", 0, 300);
	cvMoveWindow("Hand", 640, 0);
	
	cvCreateTrackbar("SampleSize", "MyRGB", &sampleSize, 100, NULL);
	cvCreateTrackbar("Threshold", "Hand", &specSkinColorThreshold, 30, NULL);
}


#pragma mark -

int main(int argc, char **argv) {

	IplImage *imageMyRgb = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 3);
	IplImage *imageDepth = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 1);
	IplImage *imageHand = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 3);
	
	
	
	CvFont font;
	cvInitFont(&font, CV_FONT_HERSHEY_PLAIN, 1, 1, 0, 1, 8);
	char outString[50];


	
//	freenect_sync_set_tilt_degs(freenect_angle);
 	freenect_sync_set_led(LED_RED);
	
	initWindows();
	initDepthLut();
	loadParameters();

	while (processKeyPressed(cvWaitKey(10))) {
		
		uint32_t timestamp = createPointCloud(curPcl);
		
		float minDist = getMinDistance(curPcl);
		pclDistThreshold(curPcl, minDist, minDist + DISTANCE_THRESHOLD);
		
		int seedSkinX, seedSkinY;
		getSkinSeed(curPcl, &seedSkinX, &seedSkinY);
		invalidatePcl(handPcl);
		handPcl[seedSkinY][seedSkinX] = curPcl[seedSkinY][seedSkinX];

		
		if (handDetectionFlag) { 						
			regionGrowing(curPcl, handPcl, specSkinColorThreshold);
			rgbImage(handPcl, imageHand);
			
			if (gestureFlag) {
				updateHandPosition(handPcl);
				printHandInfoOnImage(imageHand, font);
			}
		}
						
		rgbImage(curPcl, imageMyRgb);
		
		
		sprintf(outString, "%lu - minDist: %f m", timestamp, minDist);
		cvPutText(imageMyRgb, outString, cvPoint(30,30), &font, cvScalar(0,200, 0, 0));
		
		
		if (grabSkinSampleFlag) {
			grabSkinSampleFlag = 0;
				
			// auto start hand detection
			handDetectionFlag = grabSkinSample(curPcl, seedSkinX, seedSkinY, sampleSize);
			printSampleInfo();
		}
		
		if (displayGuideFlag) {
			cvRectangle(imageMyRgb, cvPoint(seedSkinX-sampleSize, seedSkinY-sampleSize), 
									cvPoint(seedSkinX+sampleSize, seedSkinY+sampleSize), 
									cvScalar(0, 255, 0, 0), 2, 8, 0);	
			
			sprintf(outString, "Seed X: %d - Y: %d", seedSkinX, seedSkinY);
			cvPutText(imageMyRgb, outString, cvPoint(30,50), &font, cvScalar(0,200, 0, 0));
		}
		
		cvShowImage("MyRGB", imageMyRgb);
		//cvShowImage("Depth", imageDepth);
		cvShowImage("Hand", imageHand);
		
		
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
