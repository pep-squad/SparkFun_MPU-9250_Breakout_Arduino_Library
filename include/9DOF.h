/*
Public interface for the 9DOF sensor: MPU-9250
Relying on custom port of Sparkfun's MPU-9250 Breakout Arduino library for use on a Raspberry Pi

UoGuelph W19
41X - Group 25
Brad Scott

Initializes breakout board. Exposes interface for quaternions and velocity vector.
*/
#ifndef _NineDOF_
#define _NineDOF_

#include "MPU9250.h"
#include "quaternionFilters.h"

class NineDOF{

		MPU9250 device;      //sensor object derived from breakout library

		float quaternion[4]; //[Qw,Qx,Qy,Qz]
		float velocity[3];   //[Vx,Vy,Vz] measured in m/s
		float accel[3];      //[Ax,Ay,Az] measured in m/s/s
		float gyro[3];       //[Gx,Gy,Gz] measured in g
		float mag[3];        //[Mx,My,Mz] measured in milligauss

		int fd; 	         //file descriptor used by wiringPiI2C
		int _polling_rate;

	public:
		NineDOF(int polling_rate = 25);
		void pollSensor();   //poll the sensor and update data if new data available
		const float* getQuarternions();
		const float* getVelocityVector();
		const float* getAccelerationVector();
		const float* getMagneticFieldVector();
		const float* getGravityVector();
		//const float* getZOrientation();
};

#endif
