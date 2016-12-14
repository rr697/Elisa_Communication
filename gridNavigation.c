#include "movement.h"
#include "gridNavigation.h"

void gridNavigation() {

		turnLeft();
		moveForward(1);
		turnRight();
		moveForward(1);
		turn180(); 
		moveForward(1);
		turn180();
		for(int temp = 0 ; temp<1000; temp++){
			stopWait(1);
		}
		moveForward(3);

		while(1){
			stopWait(1);
		}
}
