#include "MPU9250.h"
#include <stdio.h>

int main(){

    int fd = wiringPiI2CSetup(MPU9250_ADDRESS_AD0);
    printf("wiringPiI2C Result: %d\n",fd);

    delay(4);

    MPU9250 device(fd,7,8,100000);

    device.MPU9250SelfTest(device.selfTest);

    /*calibrate and init gyroscope/accel*/
    device.calibrateMPU9250(device.gyroBias,device.accelBias);
    device.initMPU9250();

    device.initAK8963(device.factoryMagCalibration); //calibrate magnitometer

    /*Get sensor resolution*/
    device.getAres();
    device.getGres();
    device.getMres();

    int q = 0;

    while (q != 1){

	/*Check for new data*/
        if(device.readByte(fd,INT_STATUS) & 0x01){

	    /*Get Acceleration Data*/
	    device.readAccelData(device.accelCount);
            device.ax = (float)device.accelCount[0] * device.aRes;
	    device.ay = (float)device.accelCount[1] * device.aRes;
	    device.az = (float)device.accelCount[2] * device.aRes;

	    /*Gyro data*/
            device.readGyroData(device.gyroCount);
	    device.gx = (float)device.gyroCount[0] * device.gRes;
    	    device.gy = (float)device.gyroCount[1] * device.gRes;
	    device.gz = (float)device.gyroCount[2] * device.gRes;

	    /*Magnetometer*/
	    device.readMagData(device.magCount);
	    // Calculate the magnetometer values in milliGauss
	    device.mx = (float)device.magCount[0] * device.mRes
	               * device.factoryMagCalibration[0] - device.magBias[0];
	    device.my = (float)device.magCount[1] * device.mRes
	               * device.factoryMagCalibration[1] - device.magBias[1];
	    device.mz = (float)device.magCount[2] * device.mRes
	               * device.factoryMagCalibration[2] - device.magBias[2];

            printf("Acceleration: %f %f %f\nGyro: %f %f %f\nMagnetometer: %f %f %f \n\n",device.ax,device.ay,device.az,device.gx,device.gy,device.gz,device.mx,device.my,device.mz);
	}
    }
    return 0;
}
