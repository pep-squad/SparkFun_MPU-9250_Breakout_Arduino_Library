#include "9DOF.h"
#include "MPU9250.h"
#include "quaternionFilters.h"
#include <stdio.h>

/*9DOF Constructor*/
NineDOF::NineDOF(){

    int fd_MPU9250 = wiringPiI2CSetup(MPU9250_ADDRESS_AD0);
    int fd_AK8962 = wiringPiI2CSetup(AK8963_ADDRESS);

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

    delay(1000);
}

void NineDOF::pollSensor(){
/*
	accel[0] = 0;
	accel[1] = 0;
	accel[2] = 0;

	gyro[0] = 0;
	gyro[1] = 0;
	gyro[2] = 0;

	mag[0] = 0;
	mag[1] = 0;
	mag[2] = 0;
*/
	float samples = 0.0f;
	float q_temp[4];

	for(int i = 0;i<25;i++){
		/*Check for new data*/
		if(device.readByte(fd,INT_STATUS) & 0x01){

			/*Get Acceleration Data*/
			device.readAccelData(device.accelCount);
			device.ax = (float)device.accelCount[0] * device.aRes;
			device.ay = (float)device.accelCount[1] * device.aRes;
			device.az = (float)device.accelCount[2] * device.aRes;
			
			accel[0] = device.ax;
			accel[1] = device.ay;
			accel[2] = device.az;

			/*Gyro data*/
			device.readGyroData(device.gyroCount);
			device.gx = (float)device.gyroCount[0] * device.gRes;
			device.gy = (float)device.gyroCount[1] * device.gRes;
			device.gz = (float)device.gyroCount[2] * device.gRes;

			gyro[0] = device.gx;
			gyro[1] = device.gy;
			gyro[2] = device.gz;

			/*Magnetometer*/
			device.readMagData(device.magCount);
			// Calculate the magnetometer values in milliGauss
			device.mx = (float)device.magCount[0] * device.mRes
			           * device.factoryMagCalibration[0] - device.magBias[0];
			device.my = (float)device.magCount[1] * device.mRes
			           * device.factoryMagCalibration[1] - device.magBias[1];
			device.mz = (float)device.magCount[2] * device.mRes
			           * device.factoryMagCalibration[2] - device.magBias[2];

			mag[0] = device.mx;
			mag[1] = device.my;
			mag[2] = device.mz;

			samples = samples + 1.0f;
	
	/*denoise*/
	/*
	device.ax = accel[0] / samples;
	device.ay = accel[1] / samples;
        device.az = accel[2] / samples;

	device.gx = gyro[0] / samples;
	device.gy = gyro[1] / samples;
	device.gz = gyro[2] / samples;

	device.mx = mag[0] / samples;
	device.my = mag[1] / samples;
	device.mz = mag[2] / samples;*/

			/*Quaternions*/
			device.updateTime();
			MahonyQuaternionUpdate(device.ax, device.ay, device.az, device.gx * DEG_TO_RAD,device.gy * DEG_TO_RAD, device.gz * DEG_TO_RAD, device.my,device.mx, device.mz, device.deltat);
	
			/*Summations for sampling*/
			const float* qtemp = getQ();
			q_temp[0] += qtemp[0];
			q_temp[1] += qtemp[1];
			q_temp[2] += qtemp[2];
			q_temp[3] += qtemp[3];
	
			}
		}

	//const float* q = getQ();
	quaternion[0] = q_temp[0] / samples;
	quaternion[1] = q_temp[1] / samples;
	quaternion[2] = q_temp[2] / samples;
	quaternion[3] = q_temp[3] / samples;

	/*Calculate velocity vectors*/
	velocity[0] = device.ax / device.deltat;
	velocity[1] = device.ay / device.deltat;
	velocity[2] = device.az / device.deltat;

}

/*Getters*/
const float* NineDOF::getQuarternions(){return quaternion;}
const float* NineDOF::getVelocityVector(){return velocity;}
const float* NineDOF::getAccelerationVector(){return accel;}
const float* NineDOF::getMagneticFieldVector(){return mag;}
const float* NineDOF::getGravityVector(){return gyro;}
const float* NineDOF::getZOrientation(){return &quaternion[3];}
