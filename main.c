
#include <avr\io.h>
#include <avr\interrupt.h>

#include <stdlib.h>
#include "variables.h"
#include "utility.h"
#include "speed_control.h"
#include "nRF24L01.h"
#include "behaviors.h"
#include "sensors.h"
#include "irCommunication.h"
#include "motors.h"
#include "elisaComm.h"
#include "demoComm.h"
#include "movement.h"
#include "pathing.h"
#include "gridNavigation.h"

int main(void) {

	initPeripherals();
	calibrateSensors();
	initBehaviors();
	
	GREEN_LED0_OFF;
	GREEN_LED1_OFF;
	GREEN_LED2_OFF;
	GREEN_LED3_OFF;
	GREEN_LED4_OFF;
	GREEN_LED5_OFF;
	GREEN_LED6_OFF;
	GREEN_LED7_OFF; 

   	

	while(1) {

	//elisa_communication();
    demoComm();
	//getMap();
	//gridNavigation();

	} 
	
	return(0);// while(1)

}
