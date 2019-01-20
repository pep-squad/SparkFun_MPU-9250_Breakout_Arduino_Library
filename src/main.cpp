#include "9DOF.h"
#include <stdio.h>
#include <iostream>
#include <string>
#include <sstream>
using namespace std;

int main(){
	NineDOF sensor = NineDOF();

	int q = 0;

	while(q != 1){
		sensor.pollSensor();

		cout << sensor.getVelocityVector() << endl;
	}
}