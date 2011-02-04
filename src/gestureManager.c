#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include "eig3.h"
#include "libMyKinect.h"
#include "gestureManager.h"
#include "libMouseOSX.h"


#define MIN_HAND_POINT_NUMBER 5
#define OPEN_HAND_FORM_FACTOR 0.55


double barycX, barycY, barycZ;
double prevBarycX, prevBarycY, prevBarycZ;
double majorAxeX, majorAxeY, majorAxeZ;
double normVecX, normVecY, normVecZ;
double prevNormVecX, prevNormVecY, prevNormVecZ;
float formFactor;

int isOpenHand = 0;
int prevAngleX = 0;


#define MOUSE_CLICK_UP 0
#define MOUSE_CLICK_DOWN 1
int leftButtonState = MOUSE_CLICK_UP;
int rightButtonState = MOUSE_CLICK_UP;

#define HAND_POS_BUFFER_SIZE 3
double barycBufferX[HAND_POS_BUFFER_SIZE];
double barycBufferY[HAND_POS_BUFFER_SIZE];
double barycBufferZ[HAND_POS_BUFFER_SIZE];
int barycBufferIndex = 0;

int mousePosX, mousePosY;


void updateHandPosition(PointCloud handPcl) {
	
	int i, j;
	
	int pointCount = barycenter(handPcl, &barycX, &barycY, &barycZ) ;
	if (pointCount < MIN_HAND_POINT_NUMBER) {
		return;
	}
		

	isOpenHand = openHand(handPcl);
	
	double posCovarMat[3][3];
	double posCovarMatEigVec[3][3];
	double posCovarMatEigVal[3];
	
	
	for (i=0; i<3; i++) {
		for (j=0; j<3; j++) {
			posCovarMat[i][j] = 0;
		}
	}
	
	
	for(j=0; j<FREENECT_FRAME_H; j++) {
		for(i=0; i<FREENECT_FRAME_W; i++) {
			if (handPcl[j][i].valid) {
				posCovarMat[0][0] += (handPcl[j][i].x - barycX)*(handPcl[j][i].x - barycX);
				posCovarMat[0][1] += (handPcl[j][i].x - barycX)*(handPcl[j][i].y - barycY);
				posCovarMat[0][2] += (handPcl[j][i].x - barycX)*(handPcl[j][i].z - barycZ);
				
				posCovarMat[1][0] += (handPcl[j][i].y - barycY)*(handPcl[j][i].x - barycX);
				posCovarMat[1][1] += (handPcl[j][i].y - barycY)*(handPcl[j][i].y - barycY);
				posCovarMat[1][2] += (handPcl[j][i].y - barycY)*(handPcl[j][i].z - barycZ);
				
				posCovarMat[2][0] += (handPcl[j][i].z - barycZ)*(handPcl[j][i].x - barycX);
				posCovarMat[2][1] += (handPcl[j][i].z - barycZ)*(handPcl[j][i].y - barycY);
				posCovarMat[2][2] += (handPcl[j][i].z - barycZ)*(handPcl[j][i].z - barycZ);
			}
		}
	}
	
	for (i=0; i<3; i++) {
		for (j=0; j<3; j++) {
			posCovarMat[i][j] = posCovarMat[i][j] / pointCount;
		}
	}
	
	eigen_decomposition(posCovarMat, posCovarMatEigVec, posCovarMatEigVal);
	
//	printMatrix("Covar Mat", posCovarMat, 3);
//	printMatrix("Pos Eigen Vec", posCovarMatEigVec, 3);
	
	
	majorAxeX = posCovarMatEigVec[1][2] > 0 ? posCovarMatEigVec[0][2] : - posCovarMatEigVec[0][2]; 
	majorAxeY = posCovarMatEigVec[1][2] > 0 ? posCovarMatEigVec[1][2] : - posCovarMatEigVec[1][2];
	majorAxeZ = posCovarMatEigVec[1][2] > 0 ? posCovarMatEigVec[2][2] : - posCovarMatEigVec[2][2];
	
	normVecX = posCovarMatEigVec[2][0] > 0 ? posCovarMatEigVec[0][0] : - posCovarMatEigVec[0][0]; 
	normVecY = posCovarMatEigVec[2][0] > 0 ? posCovarMatEigVec[1][0] : - posCovarMatEigVec[1][0];
	normVecZ = posCovarMatEigVec[2][0] > 0 ? posCovarMatEigVec[2][0] : - posCovarMatEigVec[2][0];
	
	//arduinoCmd();
	//printf("Barycenter: X: %.4f Y: %.4f  Z: %.4f\tDirection: X: %.4f Y: %.4f  Z: %.4f\n",  barycX, barycY, barycZ, normVecX, normVecY, normVecZ);
	
	moveMouse();
}


int openHand(PointCloud handPcl) {
	
	IplImage *imageHandBw = cvCreateImage(cvSize(640,480), 8, 1);
	
	binaryImage(handPcl, imageHandBw);
	cvDilate(imageHandBw, imageHandBw, NULL, 5);
	cvErode(imageHandBw, imageHandBw, NULL, 3);

	
	int perimeter = 0, area = 0;
	int i;
	for (i=0; i<FREENECT_FRAME_PIX; i++) {
		if(((unsigned char*)(imageHandBw->imageData))[i]) {
			area++;
			
			if (((unsigned char*)(imageHandBw->imageData))[i+1] == 0 ||
				((unsigned char*)(imageHandBw->imageData))[i-1] == 0 ||
				((unsigned char*)(imageHandBw->imageData))[i+FREENECT_FRAME_W] == 0 ||
				((unsigned char*)(imageHandBw->imageData))[i-FREENECT_FRAME_W] == 0)
				perimeter++;
		}
	}
	
	formFactor= 4 * M_PI * area / (perimeter * perimeter);
	
	//printf("PointCount: %d - Perimeter: %d - FF: %f\n", area, perimeter, formFactor);
	// cvShowImage("Depth", imageHandBw);

	cvReleaseImage(&imageHandBw);
	return (formFactor < OPEN_HAND_FORM_FACTOR);
	
}

void moveMouse() {
	// gesture analysis
	
	barycBufferX[barycBufferIndex] = barycX;
	barycBufferY[barycBufferIndex] = barycY;
	barycBufferZ[barycBufferIndex] = barycZ;
	barycBufferIndex = (barycBufferIndex + 1) % HAND_POS_BUFFER_SIZE;

	
	
	if (isOpenHand) {
		
		int prevBarycBufferIndex = (barycBufferIndex > 0) ? barycBufferIndex-1 : HAND_POS_BUFFER_SIZE-1;
		
		int countX = -round((barycBufferX[barycBufferIndex] - barycBufferX[prevBarycBufferIndex]) * 600);
		int countY = round((barycBufferY[barycBufferIndex] - barycBufferY[prevBarycBufferIndex]) * 600);
		
		verticalScroll(countX, countY);
		
		printf("Scrolling > countX: %d\n", countX);
		return;
	}
	
	
	double meanBarycX = 0;
	double meanBarycY = 0;
	double meanBarycZ = 0;
	
	int i;
	for (i=0; i<HAND_POS_BUFFER_SIZE; i++) {
		meanBarycX += barycBufferX[i] / HAND_POS_BUFFER_SIZE;
		meanBarycY += barycBufferY[i] / HAND_POS_BUFFER_SIZE;
		meanBarycZ += barycBufferZ[i] / HAND_POS_BUFFER_SIZE;
	}
	
	
	mousePosX = round(-meanBarycX * 1280 + 640);
	mousePosY = round(meanBarycY * 1280 + 400);
	

	if (leftButtonState == MOUSE_CLICK_UP && normVecY < -0.45) {
		leftButtonPressed(mousePosX, mousePosY);
		leftButtonState = MOUSE_CLICK_DOWN;
		printf("Mouse click left down\n");
		
	}
	else if (leftButtonState == MOUSE_CLICK_DOWN && normVecY >= -0.45 ) {
		leftButtonReleased(mousePosX, mousePosY);
		leftButtonState = MOUSE_CLICK_UP;
		printf("Mouse click left up\n");
	}
	else if (rightButtonState == MOUSE_CLICK_UP && normVecX < -0.5) {
		rightButtonPressed(mousePosX, mousePosY);
		rightButtonState = MOUSE_CLICK_DOWN;
		printf("Mouse click right down\n");	
	}
	else if (rightButtonState == MOUSE_CLICK_DOWN && normVecX >= -0.5 ) {
		rightButtonReleased(mousePosX, mousePosY);
		rightButtonState = MOUSE_CLICK_UP;	
		printf("Mouse click right up\n");
	}	
	else {
		moveCursor(mousePosX, mousePosY);
	}

	
}


void printHandInfoOnImage(IplImage *image, CvFont font) {
	char outString[50];
	
	sprintf(outString, "Barycenter: X: %.4f Y: %.4f  Z: %.4f",  barycX, barycY, barycZ);
	cvPutText(image, outString, cvPoint(30,30), &font, cvScalar(0,200, 0, 0));
	
	sprintf(outString, "Normal Vec: X: %.4f Y: %.4f  Z: %.4f", normVecX, normVecY, normVecZ);
	cvPutText(image, outString, cvPoint(30,50), &font, cvScalar(0,200, 0, 0));
	
	sprintf(outString, "Button state > Left %d - Right %d", leftButtonState, rightButtonState);
	cvPutText(image, outString, cvPoint(30,70), &font, cvScalar(0,200, 0, 0));
	
	if (isOpenHand) {
		sprintf(outString, "Form Factor: %.4f > Open", formFactor);
	}
	else {
		sprintf(outString, "Form Factor: %.4f > Closed", formFactor);
	}
	cvPutText(image, outString, cvPoint(30,90), &font, cvScalar(0,200, 0, 0));


}


void arduinoCmd() {
	int angleX = round(acos(normVecX) * 180 / M_PI);
	
	if (abs(angleX-prevAngleX) > 5) {
		char serialCommand[10];
		sprintf(serialCommand, "%ds", angleX);
		
		printf("Angle X: %d > %s\n",  angleX, serialCommand);
		
		prevAngleX = angleX;
		
	}
	
}
