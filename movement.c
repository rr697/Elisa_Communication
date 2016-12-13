#include "motors.h"
#include "movement.h"

/*----variables for file-----*/
static uint16_t turn90count = 26500; //nonmagnetic surface counter
//static uint16_t gridMoveCount = 33000;
static uint16_t gridEdgeThresh = 540; 

/*temporary local variables for counting */
volatile uint32_t turnCounter = 0;
volatile uint8_t gridStepCounter = 0;

volatile char reachedNextGrid;
//helper function variables 
volatile int edgeCount = 0;
volatile int numEdge = 0; 
volatile unsigned char blackToWhiteEdge = 0; 
unsigned char whiteToBlackEdge = 0; 
volatile unsigned char colorEdge = 0;
volatile int groundColor = 0; 

/*turns 90 degrees to the left
*/
void turnLeft() {
		GREEN_LED7_ON;
		//spin for 90 degrees
		for (turnCounter = 0; turnCounter<turn90count; 	turnCounter++) {
			setLeftSpeed(-10);
			setRightSpeed(10);
			handleMotorsWithSpeedController();  
		}
		GREEN_LED7_OFF;
}

/*turns 90 degrees to the right
*/
void turnRight() {
		GREEN_LED1_ON;
		//spin for 90 degrees
		for (turnCounter = 0; turnCounter<turn90count; 	turnCounter++) {
			setLeftSpeed(10);
			setRightSpeed(-10);
			handleMotorsWithSpeedController();  
		}
		GREEN_LED1_OFF;
}

/*
turn 180 degrees and face the direction it came from
*/
void turn180() {
		GREEN_LED3_ON;
		GREEN_LED5_ON;
		//spin for 90 degrees
		for (turnCounter = 0; turnCounter<turn90count*2; 	turnCounter++) {
			setLeftSpeed(10);
			setRightSpeed(-10);
			handleMotorsWithSpeedController();  
		}		
		GREEN_LED3_OFF;
		GREEN_LED5_OFF;

}



/*stop where robot is and set motor speed to 0, wait if stop = 1
*/
void stopWait(char stop) {
	if(stop) {
			setLeftSpeed(0);
			setRightSpeed(0);
			handleMotorsWithSpeedController();
	}

}


/*move forward x amount grid step
*/
void moveForward(int gridSteps) {
	for (gridStepCounter = 0; gridStepCounter < gridSteps; gridStepCounter++) {
		moveForwardOne();
	}
}


/*----------helper functions ------------------*/




/*move forwared 1 grid step
*/
void moveForwardOne(){
	reachedNextGrid = 0;
	while(reachedNextGrid==0) {
	//for (uint16_t gridMoveCounter = 0; gridMoveCounter < gridMoveCount; gridMoveCounter ++ ){
			setLeftSpeed(15);
			setRightSpeed(15);
			handleMotorsWithSpeedController();
			reachedNextGrid = gridEdgeDetected();
	}

	//backup into the grid itself so the Robot doesn't stop on the line
	while(reachedNextGrid == 1) {
//	while(proximityResult[9] > gridEdgeThresh && proximityResult[10] > gridEdgeThresh && reachedNextGrid == 1) { //backup until prox sensors see Black
			setLeftSpeed(-15);
			setRightSpeed(-15);
			handleMotorsWithSpeedController();
			reachedNextGrid = gridEdgeDetected();

	}

}


/* tell if the front ground sensors detect the edge of the grid. 
Returns 1 if and edge is detected and returns 0 if no edge detected.
*/

char gridEdgeDetected() {
	numEdge = gridEdgeCount(); 
	// tell whether a gridEdge is detected or not
	if(numEdge==2) {
		return 1; //it sees the white line
	} else {
		return 0;
	}
}



/* Counts the number of edges the robot passes over. This is used to know when the Robot 
moves fully into the next grid square. The Robot needs to cross 2 edges.
*/
int gridEdgeCount() {
	blackToWhiteEdge = blackToWhiteEdgeDetect();
	
	switch(edgeCount) {
		
		case 0: 
			if(blackToWhiteEdge) {
				edgeCount = 1;
			}
			else{
				edgeCount = 0;
			}
			break;

		case 1: 
			if(blackToWhiteEdge) {
				edgeCount = 2;
			}
			else{
				edgeCount = 1;
			}
			break;

		case 2:
			whiteToBlackEdge = (proximityResult[9] < gridEdgeThresh && proximityResult[10] < gridEdgeThresh);
			if(whiteToBlackEdge) {
				edgeCount = 0;
			}
			else{
				edgeCount = 2;
			}
			break;
		
	}

	return edgeCount;


}

unsigned char blackToWhiteEdgeDetect() {

	switch(groundColor){//0 is black and 1 is white
		
		case 0: //groundColor is black
			if (proximityResult[9] > gridEdgeThresh && proximityResult[10] > gridEdgeThresh) {//if current values are white
				groundColor = 1;
				colorEdge = 1; 
			}
			else {
				groundColor = 0;
				colorEdge = 0; 
			}
			break;

		case 1: //groundColor is white
			if (proximityResult[9] < gridEdgeThresh && proximityResult[10] < gridEdgeThresh) {//if current values are black
				groundColor = 0;
				colorEdge = 0; 
			}
			else {
				groundColor = 1;
				colorEdge = 0; 
			}
			break;
			
	}

	return colorEdge; 

}
