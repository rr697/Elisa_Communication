#include "demoComm.h"
#include "irCommunication.h"
#include "variables.h"
#include "utility.h"
#include "speed_control.h"
#include "nRF24L01.h"
#include "behaviors.h"
#include "sensors.h"
#include "irCommunication.h"
#include "motors.h"

void demoComm() {
	readAccelXYZ();						
    computeAngle();
    handleIRRemoteCommands();
    handleRFCommands();
    //float targetAngle=0;

	while (1) {

		switch(demoState) {
					case 0:
						irCommInit();
						demoState = 1;
						break;

					case 1:						
						enableObstacleAvoidance();
						setLeftSpeed(10);
						setRightSpeed(10);
						demoState = 2;
						demoStartTime = getTime100MicroSec();
						break;
						}
          	 
			
         
	   	 					
		}
		handleMotorsWithSpeedController();

}


void rotate360() {

	setLeftSpeed(10);
	setRightSpeed(-10);
	handleMotorsWithSpeedController();  
}
