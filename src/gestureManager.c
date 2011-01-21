/*
 * graffiti.c
 *
 *  Created on: Dec 23, 2010
 *      Author: Andrea Grandi
 */


#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <ApplicationServices/ApplicationServices.h>
#include <unistd.h>



#define MOUSE_CLICK_UP 0
#define MOUSE_CLICK_DOWN 1
int mouseLeftState = MOUSE_CLICK_UP;
int mouseRightState = MOUSE_CLICK_UP;

#define HAND_POS_BUFFER_SIZE 3
double handBufferX[HAND_POS_BUFFER_SIZE];
double handBufferY[HAND_POS_BUFFER_SIZE];
double handBufferZ[HAND_POS_BUFFER_SIZE];
int handBufferIndex = 0;

void setHandPosition(double barycX, double barycY, double barycZ, double normVecX, double normVecY, double normVecZ) {
	
	if (barycX != NAN && barycY != NAN) {
		handBufferX[handBufferIndex] = barycX;
		handBufferY[handBufferIndex] = barycY;
		handBufferZ[handBufferIndex] = barycZ;
		handBufferIndex = (handBufferIndex + 1) % HAND_POS_BUFFER_SIZE;
	}
	
	double meanHandX = 0;
	double meanHandY = 0;
	double meanHandZ = 0;

	int i;
	for (i=0; i<HAND_POS_BUFFER_SIZE; i++) {
		meanHandX += handBufferX[i] / HAND_POS_BUFFER_SIZE;
		meanHandY += handBufferY[i] / HAND_POS_BUFFER_SIZE;
		meanHandZ += handBufferZ[i] / HAND_POS_BUFFER_SIZE;
	}
	
	int mousePosX = round(meanHandX * 1280 + 640);
	int mousePosY = -round(meanHandY * 1280 + 400);
	
	if (mouseLeftState == MOUSE_CLICK_UP && normVecY < -0.4) {
		CGEventRef clickLeftDown = CGEventCreateMouseEvent(
														 NULL, kCGEventLeftMouseDown,
														 CGPointMake(mousePosX, mousePosY),
														 kCGMouseButtonLeft
														 );
		
		CGEventPost(kCGHIDEventTap, clickLeftDown);
		CFRelease(clickLeftDown);
		mouseLeftState = MOUSE_CLICK_DOWN;
		printf("Mouse click left down\n");

	}
	else if (mouseLeftState == MOUSE_CLICK_DOWN && normVecY >= -0.4 ) {		
		// Left button up at 250x250
		CGEventRef clickLeftUp = CGEventCreateMouseEvent(
													   NULL, kCGEventLeftMouseUp,
													   CGPointMake(mousePosX, mousePosY),
													   kCGMouseButtonLeft
													   );
		
		CGEventPost(kCGHIDEventTap, clickLeftUp);
		CFRelease(clickLeftUp);
		mouseLeftState = MOUSE_CLICK_UP;
		printf("Mouse click left up\n");
	}
	else if (mouseRightState == MOUSE_CLICK_UP && normVecX < -0.4) {
		CGEventRef clickRightDown = CGEventCreateMouseEvent(
														   NULL, kCGEventRightMouseDown,
														   CGPointMake(mousePosX, mousePosY),
														   kCGMouseButtonRight
														   );
		
		CGEventPost(kCGHIDEventTap, clickRightDown);
		CFRelease(clickRightDown);
		mouseRightState = MOUSE_CLICK_DOWN;
		printf("Mouse click right down\n");
		
	}
	else if (mouseLeftState == MOUSE_CLICK_DOWN && normVecX >= -0.4 ) {		
		// Left button up at 250x250
		CGEventRef clickRightUp = CGEventCreateMouseEvent(
														 NULL, kCGEventRightMouseDown,
														 CGPointMake(mousePosX, mousePosY),
														 kCGMouseButtonRight
														 );
		
		CGEventPost(kCGHIDEventTap, clickRightUp);
		CFRelease(clickRightUp);
		mouseRightState = MOUSE_CLICK_UP;
		printf("Mouse click right up\n");
	}
	
	else {
		CGEventRef move = CGEventCreateMouseEvent(
												  NULL, kCGEventMouseMoved,
												  CGPointMake(mousePosX, mousePosY),
												  kCGMouseButtonLeft // ignored
												  );
		
		CGEventPost(kCGHIDEventTap, move);
		CFRelease(move);
	}


}