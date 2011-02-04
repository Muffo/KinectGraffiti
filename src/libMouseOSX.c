#include "libMouseOSX.h"
#include <ApplicationServices/ApplicationServices.h>


void leftButtonPressed(int x, int y) {
	CGEventRef pressLeftButton = CGEventCreateMouseEvent(
														 NULL, kCGEventLeftMouseDown,
														 CGPointMake(x, y),
														 kCGMouseButtonLeft
														 );
	
	CGEventPost(kCGHIDEventTap, pressLeftButton);
	CFRelease(pressLeftButton);
}


void leftButtonReleased(int x, int y) {
	CGEventRef releaseLeftButton = CGEventCreateMouseEvent(
														   NULL, kCGEventLeftMouseUp,
														   CGPointMake(x, y),
														   kCGMouseButtonLeft
														   );
	
	CGEventPost(kCGHIDEventTap, releaseLeftButton);
	CFRelease(releaseLeftButton);	
}

void rightButtonPressed(int x, int y) {
	CGEventRef pressRightButton = CGEventCreateMouseEvent(
														NULL, kCGEventRightMouseDown,
														CGPointMake(x, y),
														kCGMouseButtonRight
														);
	
	CGEventPost(kCGHIDEventTap, pressRightButton);
	CFRelease(pressRightButton);
}


void rightButtonReleased(int x, int y){
	CGEventRef releaseRightButton = CGEventCreateMouseEvent(
													  NULL, kCGEventRightMouseUp,
													  CGPointMake(x, y),
													  kCGMouseButtonRight
													  );
	
	CGEventPost(kCGHIDEventTap, releaseRightButton);
	CFRelease(releaseRightButton);
}

void verticalScroll(int x, int y) {
	CGEventRef event = CGEventCreateScrollWheelEvent(NULL, kCGScrollEventUnitPixel, 2, y, x);
	CGEventSetType(event, kCGEventScrollWheel);
	CGEventPost(kCGSessionEventTap, event);
	CFRelease(event);
}


void moveCursor(int x, int y) {
	CGEventRef moveCursor = CGEventCreateMouseEvent(
											  NULL, kCGEventMouseMoved,
											  CGPointMake(x, y),
											  kCGMouseButtonLeft // ignored
											  );
	
	CGEventPost(kCGHIDEventTap, moveCursor);
	CFRelease(moveCursor);
}