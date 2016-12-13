#ifndef MOVEMENT_H
#define MOVEMENT_H


#include "variables.h"
#include <avr\io.h>
#include <avr\interrupt.h>
#include "behaviors.h"
#include "speed_control.h"


//turns 90 degrees to the left
void turnLeft();

/*turns 90 degress to the right
*/
void turnRight();

/*
turn 180 degrees and face the direction it came from
*/
void turn180();

/*move forwared 1 grid step
*/
void moveForwardOne();

/*move forwared x amount grid step, the user can specify how 
many grids they want the robot to move forward.
*/
void moveForward(int gridSteps);

/*
stop where robot is and set motor speed to 0, wait if stop = 1. Needs 
to be continuously called for the robot to stop
*/
void stopWait(char stop);


/* tell if the front ground sensors detect a gridEdge. 
Returns 1 if and edge is detected and returns 0 if no edge detected.
*/
char gridEdgeDetected();


/* Tell how many grid lines the robot crossed. Outputs 0 if no gird 
lines seen and 1 if robot crosses one line.
Outputs 2 if robot crosses 2 lines.
*/
int gridEdgeCount(); 

/* if the robot crosses from black surface to white surface, outputs 1.
*/
unsigned char blackToWhiteEdgeDetect();


#endif
