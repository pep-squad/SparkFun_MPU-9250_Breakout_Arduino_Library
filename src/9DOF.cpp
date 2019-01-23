#include "9DOF.h"
#include "MPU9250.h"
#include "quaternionFilters.h"
#include <stdio.h>

/*9DOF Constructor*/
NineDOF::NineDOF(){

    int fd_MPU9250 = wiringPiI2CSetup(MPU9250_ADDRESS_AD0);
    int fd_AK8962 = wiringPiI2CSetup(AK8963_ADDRESS);

    delay(1000);

    device = MPU9250(fd_MPU9250,fd_AK8962);

    device.MPU9250SelfTest(device.selfTest);

    delay(1000);

    /*calibrate and init gyroscope/accel*/
    device.calibrateMPU9250(device.gyroBias,device.accelBias);
    device.initMPU9250();
    device.initAK8963(device.factoryMagCalibration); //calibrate magnitometer

    delay(1000);

    /*Get sensor resolution*/
    device.getAres();
    device.getGres();
    device.getMres();
}

void NineDOF::pollSensor(){

float accel_avg[3]; //need to collect 1 hundred samples and average (needed to denoise)
float gyro_avg[3];
float mag_avg[3];


	for(int i = 0;i<10;i++){
		/*Check for new data*/
		if(device.readByte(fd,INT_STATUS) & 0x01){

			/*Get Acceleration Data*/
			device.readAccelData(device.accelCount);
			device.ax = (float)device.accelCount[0] * device.aRes;
			device.ay = (float)device.accelCount[1] * device.aRes;
			device.az = (float)device.accelCount[2] * device.aRes;

			accel_avg[0] += device.ax;
			accel_avg[1] += device.ay;
			accel_avg[2] += device.az;

			/*Gyro data*/
			device.readGyroData(device.gyroCount);
			device.gx = (float)device.gyroCount[0] * device.gRes;
			device.gy = (float)device.gyroCount[1] * device.gRes;
			device.gz = (float)device.gyroCount[2] * device.gRes;

			gyro_avg[0] += device.gx;
			gyro_avg[1] += device.gy;
			gyro_avg[2] += device.gz;

			/*Magnetometer*/
			device.readMagData(device.magCount);
			// Calculate the magnetometer values in milliGauss
			device.mx = (float)device.magCount[0] * device.mRes
			           * device.factoryMagCalibration[0] - device.magBias[0];
			device.my = (float)device.magCount[1] * device.mRes
			           * device.factoryMagCalibration[1] - device.magBias[1];
			device.mz = (float)device.magCount[2] * device.mRes
			           * device.factoryMagCalibration[2] - device.magBias[2];

		//printf("Acceleration: %f %f %f\nGyro: %f %f %f\nMagnetometer: %f %f %f \n",device.ax,device.ay,device.az,device.gx,device.gy,device.gz,device.mx,device.my,device.mz);

			mag_avg[0] += device.mx;
			mag_avg[1] += device.my;
			mag_avg[2] += device.mz;
		}
	}

	/*denoise*/
	device.ax = accel_avg[0]/100;
	device.ay = accel_avg[1]/100;
        device.az = accel_avg[2]/100;

	device.gx = gyro_avg[0]/100;
	device.gy = gyro_avg[1]/100;
	device.gz = gyro_avg[2]/100;

	device.mx = mag_avg[0]/100;
	device.my = mag_avg[1]/100;
	device.mz = mag_avg[2]/100;

	/*Quaternions*/
	device.updateTime();
	MadgwickQuaternionUpdate(device.ax, device.ay, device.az, device.gx * DEG_TO_RAD,device.gy * DEG_TO_RAD, device.gz * DEG_TO_RAD, device.my,device.mx, device.mz, device.deltat);

	const float* q = getQ();
	quaternion[0] = q[0];
	quaternion[1] = q[1];
	quaternion[2] = q[2];
	quaternion[3] = q[3];

	/*Calculate velocity vectors*/
	velocity[0] = device.ax / device.deltat;
	velocity[1] = device.ay / device.deltat;
	velocity[2] = device.az / device.deltat;
	//printf("Quaternion - qA: %f, qx: %f, qy: %f, qz: %f\n\n",q[0],q[1],q[2],q[3]);

}

/*Getters*/
const float* NineDOF::getQuarternions(){return quaternion;}
const float* NineDOF::getVelocityVector(){return velocity;}
const float* NineDOF::getAccelerationVector(){return accel;}
const float* NineDOF::getMagneticFieldVector(){return mag;}
const float* NineDOF::getGravityVector(){return gyro;}
const float* NineDOF::getZOrientation(){return &quaternion[3];}
