#include "pathing.h"
#include <util/delay.h>
#include "motors.h"
#include "movement.h"

#define rows 4
#define cols 4

 
                        	
int myArray[cols][rows] = { 	{0, 0, 0, 2},
                        		{0, 0, 0, 0},
                        		{0, 0, 0, 0},
                        		{0, 0, 0, 3}  };							

void getMap() {
    

   for (int i = 0; i < cols; i++) {
    	for (int j = 0; j < rows; j++) {
	      	   
    		if (myArray[i][j] == 0)
				{
       			    	
					moveForward(1);
					GREEN_LED1_ON;
								
							
	 			}                 
    		else  if (myArray[i][j] == 1)
				{
	    			
							    
			        turnRight();
					moveForward(1);
					GREEN_LED2_ON;										
								
	 			}
            else if (myArray[i][j] == 2)
				{
	    			
							    
			        turn180();
					moveForward(1);
					GREEN_LED3_ON;										
								
	 			}
             else // if 3
			  {
			  	    while(1){
					GREEN_LED3_ON;
					stopWait(1);
		            }

			   }

			stopWait(1); 	 		
		    	
        }
			
	

      }
	  	 
    
}
