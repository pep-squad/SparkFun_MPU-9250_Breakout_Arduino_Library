#include "9DOF.h"
#include <stdio.h>
#include <iostream>
#include <fstream>

int main(){
	NineDOF sensor = NineDOF();
std::ofstream file;
file.open("Rz22.csv");
	for(int i =0;i<100;i++){
		sensor.pollSensor();
/*
		const float* a = sensor.getAccelerationVector();
		printf("Accel: %f, %f, %f\n",a[0],a[1],a[2]);

                const float* v = sensor.getVelocityVector();
                printf("Velocity: %f, %f, %f\n",v[0],v[1],v[2]);

                const float* m = sensor.getMagneticFieldVector();
                printf("Mag: %f, %f, %f\n\n",m[0],m[1],m[2]);

                const float* g = sensor.getGravityVector();
                printf("Grav: %f, %f, %f\n\n",g[0],g[1],g[2]);

		const float* z = sensor.getZOrientation();
		printf("Z Angle: %f\n",*z);
*/
 		const float* q = sensor.getQuarternions();
                //printf("Quarternion: %f, %f, %f, %f\n",q[0],q[1],q[2],q[3]);
		printf("%f\n",q[3]);
		//file << q[3];
	}
file.close();
return 0;
}
