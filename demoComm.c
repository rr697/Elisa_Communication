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
#include "movement.h"
#include "pathing.h"

void demoComm() {
	readAccelXYZ();						
    computeAngle();
    handleIRRemoteCommands();
    handleRFCommands();
    unsigned int i=0;
	unsigned int currRand=0, currRand2=0;
    //float targetAngle=0;

	while (1) {

	switch(demoState) {
					case 0:
						irCommInit();
						enableObstacleAvoidance();
						setLeftSpeed(10);
						setRightSpeed(10);
						demoState = 1;
						i = 0;
						break;

					case 1:						
						irCommSendData(irCommRxByteExpected);
																		
						demoState = 2;
						break;

					case 2:
						irCommTasks();
						if(irCommDataSent()==1) {							
							demoState = 1;
							if(irCommRxByteExpected<255) {
								irCommRxByteExpected++;
							} else {
								irCommRxByteExpected = 0;
							}
						}
						if(irCommDataAvailable()==1) {
							irCommReadData();
							if(i<7) {
								i++;
							} else {
								i = 0;
							}
							switch(i) {
								case 0: 
									updateRedLed(255);
									updateGreenLed(255);
									updateBlueLed(255);
									break;
								case 1: 
									updateRedLed(0);
									updateGreenLed(255);
									updateBlueLed(255);
									break;	
								case 2: 
									updateRedLed(255);
									updateGreenLed(0);
									updateBlueLed(255);
									break;
								case 3: 
									updateRedLed(255);
									updateGreenLed(255);
									updateBlueLed(0);
									break;
								case 4: 
									updateRedLed(0);
									updateGreenLed(0);
									updateBlueLed(255);
									break;
								case 5: 
									updateRedLed(0);
									updateGreenLed(255);
									updateBlueLed(0);
									break;
								case 6: 
									updateRedLed(255);
									updateGreenLed(0);
									updateBlueLed(0);
									break;
								case 7: 
									updateRedLed(0);
									updateGreenLed(0);
									updateBlueLed(0);
									break;
								default:
									break;
							}
						}
						break;
				}
				break;
	disableObstacleAvoidance();
    disableCliffAvoidance();
			GREEN_LED0_OFF;
			GREEN_LED1_OFF;
			GREEN_LED2_OFF;
			GREEN_LED3_OFF;
			GREEN_LED4_OFF;
			GREEN_LED5_OFF;
			GREEN_LED6_OFF;
			GREEN_LED7_OFF;
			pwm_red = 255;
			pwm_green = 255;
			pwm_blue = 255;
			updateRedLed(pwm_red);
			updateGreenLed(pwm_green);
			updateBlueLed(pwm_blue);
			setRightSpeed(0);
			setLeftSpeed(0);
			rgbState = 0;
			calibrationWritten = 0;
			demoState = 0;

			irCommState = 0;
}
	handleMotorsWithSpeedController();  
}


