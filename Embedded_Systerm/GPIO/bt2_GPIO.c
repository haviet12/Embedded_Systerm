#include<stdio.h>
#include<stdint.h>
#include<softPwm.h>
#include<wiringPi.h>

#define led 0
uint8_t bt_pin[4]={2,3,12,13};

void bt_1(void){
	if (digitalRead(bt_pin[0]==1){
		softPwmWrite(led,25);
	}
}
void bt_2(void){
	if (digitalRead(bt_pin[1]==1){
		softPwmWrite(led,50);
	}
}
void bt_3(void){
	if (digitalRead(bt_pin[2]==1){
		softPwmWrite(led,75);
	}
}
void bt_4(void){
	if (digitalRead(bt_pin[3]==1){
		softPwmWrite(led,100);
	}
}
int main(void)
{
	// khai bao chan
	wiringPiSetupGpio();
	//khai bao output cho led
	pinMode(led, OUTPUT);
	//khai bao input cho nut nhan
	for(int i =0;i<4;i++)
	{
		pinMode(bt_pin[i],INPUT);
		pullUpDnControl(bt_pin[i],PUD_UP);
	}
	//khai bao softPwm
	softPwmCreate(led,0,100);
	// khai bao ngat cho nut nhan
	wiringPiIRS(bt_pin[0],INT_EDGE_BOTH, &bt_1);
	wiringPiIRS(bt_pin[1],INT_EDGE_BOTH, &bt_2);
	wiringPiIRS(bt_pin[2],INT_EDGE_BOTH, &bt_3);
	wiringPiIRS(bt_pin[3],INT_EDGE_BOTH, &bt_4);
	while(1){}
	
}