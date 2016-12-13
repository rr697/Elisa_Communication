#include "elisaComm.h"
#include "irCommunication.h"
#include "variables.h"
#include "utility.h"
		
void elisa_communication(){	
	unsigned long int startTime = 0, endTime = 0, turnOffLedsTime = 0;
	unsigned char prevSelector=0;
	unsigned int i=0;
	unsigned int currRand=0, currRand2=0;
	float targetAngle=0;

	initPeripherals();

	initBehaviors();

	speedStepCounter = getTime100MicroSec();

	startTime = getTime100MicroSec();
	while((getTime100MicroSec() - startTime) < PAUSE_300_MSEC);
	calibrateSensors();

	startTime = getTime100MicroSec();

	while(1) {

		currentSelector = getSelector();	// update selector position

		readAccelXYZ();						// update accelerometer values to compute the angle

		computeAngle();
        handleIRRemoteCommands();
	    handleRFCommands();
	
  
		
		switch(currentSelector) {
    


			case 11: 	// sync-react demo using the IR local communication (2 robots only): the robots move around with obstacle avoidance
						// enabled and when they detect each other they try to align towards the same direction rotating in place
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

					case 2:
						irCommTasks();
						if(irCommDataSent()==1) {
							angleDegEncode = (unsigned char)((float)angleDeg*0.7084);
							irCommSendData(angleDegEncode);
						}
						if(irCommDataAvailable()==1) {
							demoStartTime = getTime100MicroSec();
							irCommLastData = irCommReadData();
							irCommLastData = (int)((float)irCommLastData*1.411);
							irCommLastSensor = irCommReceivingSensor();	
							disableObstacleAvoidance();										
							angleDeg = getBearing(irCommLastSensor);
							if(angleDeg < 0) {
								angleDeg += 360;
							}
							angleError = angleDeg - irCommLastData;
							angleError += 180;
							if(angleError > 180) {
								angleError -= 360;
							}
							if(angleError < -180) {
								angleError += 360;
							}
							if(abs(angleError) < 10) {
								setLeftSpeed(0);
								setRightSpeed(0);
							} else {
								resetOdometry();
								if(angleError > 0) {	// turn left
									setLeftSpeed(-7);
									setRightSpeed(7);
									targetAngle = 0.09;	// about -10 degrees
									demoState = 4;
								} else {	// turn right
									setLeftSpeed(7);
									setRightSpeed(-7);
									targetAngle = -0.09;	// about 10 degrees
									demoState = 3;
								}								
							}

						}
						if((getTime100MicroSec()-demoStartTime) >= (PAUSE_5_SEC)) {
							demoState = 1;
						}
						break;

					case 3:	// rotate right
						irCommTasks();
						if(theta <= targetAngle) {
							setLeftSpeed(0);
							setRightSpeed(0);
							demoState = 2;
							demoStartTime = getTime100MicroSec();
						}
						break;

					case 4:	// rotate left
						irCommTasks();
						if(theta >= targetAngle) {
							setLeftSpeed(0);
							setRightSpeed(0);
							demoState = 2;
							demoStartTime = getTime100MicroSec();
						}
						break;
				}
				break;

			case 12:	// IR local communication: 2 or more robots pass information between them in sequence
				switch(demoState) {
					case 0:
						irCommInit();
						irCommRobotsNum = 4;	// total number of robots that exchange data
						irCommRobotId = 3;		// choose the id of the robot (a different id for each robot), from 0 to 254 (255 is reserved)
						if(irCommRobotId==0) {	// robot with id 0 starts the transmission of information
							irCommSendData(0x01);
							updateBlueLed(255);
							updateRedLed(255);
							updateGreenLed(128);
							demoState = 1;
						} else {
							demoState = 2;
						}
						break;

					case 1: // led is on
							// transmission: continuously send the next robot id to tell him to turn on the led
							// reception: listen for 0xFF and then turn off the led; when I receive 0xFF it means the next robot has
							// received its id, thus it has turned on its led
						irCommTasks();
						if(irCommDataSent()==1) {
							if(irCommRobotId < (irCommRobotsNum-1)) {
								irCommSendData(irCommRobotId+1);
							} else {
								irCommSendData(0x00);
							}							
						}
						if(irCommDataAvailable()==1) {
							if(irCommLedToggle==0) {
								irCommLedToggle = 1;
								updateBlueLed(255);
								updateRedLed(255);
								updateGreenLed(128);
							} else {
								irCommLedToggle = 0;
								updateBlueLed(255);
								updateRedLed(255);
								updateGreenLed(235);
							}
							i = irCommReadData();
							if(i == 0xFF) {
								demoState = 2;
								updateBlueLed(255);		
								updateRedLed(255);
								updateGreenLed(255);		
							}
						}	
						break;

					case 2: // led is off
							// transmission: continuously send my robot id to tell the next robot I turned off the led
							// reception: listen for my robot id and then turn on the led				
						irCommTasks();
						if(irCommDataSent()==1) {
							irCommSendData(irCommRobotId);							
						}
						if(irCommDataAvailable()==1) {
							if(irCommLedToggle==0) {
								irCommLedToggle = 1;
								updateRedLed(255);
							} else {
								irCommLedToggle = 0;
								updateRedLed(235);
							}
							i = irCommReadData();
							if(i == irCommRobotId) {
								updateBlueLed(255);
								updateRedLed(255);
								updateGreenLed(128);
								irCommSendData(0xFF);
								demoState = 3;					
							}							
						}
						break;

					case 3:	// led is on
							// transmission: send 0xFF untill the previous robot is turned off
							// reception: I know that it is turned off when I receive the previous robot id 
						irCommTasks();
						if(irCommDataSent()==1) {
							if(irCommDataAvailable()==1) {
								if(irCommLedToggle==0) {
									irCommLedToggle = 1;
									updateBlueLed(255);
									updateRedLed(255);
									updateGreenLed(128);
								} else {
									irCommLedToggle = 0;
									updateBlueLed(255);
									updateRedLed(255);
									updateGreenLed(235);
								}
								i = irCommReadData();
								if(irCommRobotId == 0) {
									if(i == (unsigned int)(irCommRobotsNum-1)) {
										demoState = 1;
									} else {
										irCommSendData(0xFF);
									}
								} else {
									if(i == (unsigned int)(irCommRobotId-1)) {
										demoState = 1;
									} else {
										irCommSendData(0xFF);
									}
								}
								
							}
						}						
						break;

				}
				break;

			case 13: // IR local communication: listen and transmit continuously
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
			
			case 14: // Multirobots communication: continuously change current color and make it change also for other robots
				switch(demoState) {
					case 0:
						irCommInit();
						demoState = 1;
						irCommRxByteExpected = 1;
						irCommMsgCount = 0;
						break;

					case 1:	
						irCommTasks();
						if(irCommDataSent()==1) {	
							irCommSendData(irCommRxByteExpected);					
						}
						if(irCommDataAvailable()==1) {							
							i = irCommReadData();
							if(i == irCommRxByteExpected) {
								irCommMsgCount++;
								if(irCommMsgCount >= 4) {
									irCommMsgCount = 0;
									if(irCommRxByteExpected < 7) {
										irCommRxByteExpected++;
									} else {
										irCommRxByteExpected = 0;
									}
								}
							} else {
								if(irCommRxByteExpected==7) {
									if(i==0) {
										irCommRxByteExpected = 0;
										irCommMsgCount = 0;
									}
								} else {
									if(irCommRxByteExpected==(i-1)) {
										irCommRxByteExpected = i;
										irCommMsgCount = 0;
									}
								}
							}
							switch(irCommRxByteExpected) {
								case 0: 
									updateRedLed(255);
									updateGreenLed(255);
									updateBlueLed(255);
									break;
								case 1: 
									updateRedLed(235);
									updateGreenLed(255);
									updateBlueLed(255);
									break;	
								case 2: 
									updateRedLed(255);
									updateGreenLed(235);
									updateBlueLed(255);
									break;
								case 3: 
									updateRedLed(255);
									updateGreenLed(255);
									updateBlueLed(235);
									break;
								case 4: 
									updateRedLed(235);
									updateGreenLed(235);
									updateBlueLed(255);
									break;
								case 5: 
									updateRedLed(235);
									updateGreenLed(255);
									updateBlueLed(235);
									break;
								case 6: 
									updateRedLed(255);
									updateGreenLed(235);
									updateBlueLed(235);
									break;
								case 7: 
									updateRedLed(235);
									updateGreenLed(235);
									updateBlueLed(235);
									break;
								default:
									break;
							}
						}		
						break;
				}									
				break;

			case 15:// clock calibration
					//usart0Transmit(irCommand,1);
					//currentOsccal = OSCCAL;
					//usart0Transmit(currentOsccal,1);
					break;

		}

			if(currentSelector!=0) {
			handleMotorsWithSpeedController();  
		}

		if(prevSelector != currentSelector) {	// in case the selector is changed, reset the robot state
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
		prevSelector = currentSelector;


	} // while(1)

}
