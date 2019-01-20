#include "9DOF.h"
#include <stdio.h>

int main(){
	NineDOF sensor = NineDOF();

	int q = 0;

	while(q != 1){
		sensor.pollSensor();

		const float* a = sensor.getAccelerationVector();
		printf("Accel: %f, %f, %f\n\n",a[0],a[1],a[2]);
	}
return 0;
}
