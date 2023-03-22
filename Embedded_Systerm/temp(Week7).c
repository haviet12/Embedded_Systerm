#include<stdio.h>
#include<wiringPiI2C.h>
#include<stdint.h>
#include<wiringPi.h>
uint8_t who;
int mpu;
uint8_t high, low;
float temp;
void initMpu(void){
	// sample rate:100Hz
	wiringPiI2CWriteReg8(mpu, 0x19,9);
	// DLPF : fc=44Hz
	wiringPiI2CWriteReg8(mpu, 0x1A,0x03);
	// Gyro configuration :+-500
	wiringPiI2CWriteReg8(mpu, 0x1B,0x08);
	//Acc configuration: +-8
	wiringPiI2CWriteReg8(mpu, 0x1C,0x10);
	// interrupt: enabel data read interrupt
	wiringPiI2CWriteReg8(mpu, 0x38,1);
	// power manegment:
	wiringPiI2CWriteReg8(mpu, 0x6B,1);
}
int16_t readSensor(uint8_t address){
	uint8_t high, low;
	int16_t value;
	high= wiringPiI2cReadReg8(mpu,address);
	low=	wiringPiI2cReadReg8(mpu, address+1);
	valu=(high<<8)|low;
	return value;
}
	
int main(){
	// set up i2c interface
	mpu = wiringPiI2CSetUp(0x68);
	// test i2c connection
	who= wiringPiI2cReadReg8(mpu, 0x75);
	if (who!=0x68){
		printf("connection fail.\n");
		break;}
	
	initMpu();
	// read temprature cpu
	temp=readSensor(0x41);
	temp=temp/340.0+36.53;
	return 0;
	
	
}