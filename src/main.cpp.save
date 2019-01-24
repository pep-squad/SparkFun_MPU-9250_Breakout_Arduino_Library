#include "9DOF.h"
#include <stdio.h>

int main(){
	NineDOF sensor = NineDOF();

	int q = 0;

	while(q != 1){
		sensor.pollSensor();
/*
		const float* a = sensor.getAccelerationVector();
		printf("Accel: %f, %f, %f\n",a[0],a[1],a[2]);
*/
                const float* v = sensor.getVelocityVector();
                printf("Velocity: %f, %f, %f\n",v[0],v[1],v[2]);
/*
                const float* m = sensor.getMagneticFieldVector();
                printf("Mag: %f, %f, %f\n\n",m[0],m[1],m[2]);

                const float* g = sensor.getGravityVector();
                printf("Grav: %f, %f, %f\n\n",g[0],g[1],g[2]);
*/
		delay(1000);
	}
return 0;
}
