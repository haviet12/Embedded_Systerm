#include<stdio.h>
#include<stdint.h>
#include<wiringPi.h>
#include<softPwm.h>
#include<stdlib.h>
uint8_t bt_pin[3]={0,2,3};
uint8_t led[3]={21,22,23};

// tao ham ngat cho 3 nut nhan 

void bt_1(void){
	int dem=0;
	if (digitalRead(bt_pin[0]==1){
		dem=dem+1;
		if (dem>3){dem=0;}
	}
void bt_2(void){
	int clk=25;
	if (digitalRead(bt_pin[1]==1){
		clk=clk+25;
		if(clk>100){clk=100;)
	}
}
}
void bt_3(void){
	int clk=100;
	if (digitalRead(bt_pin[2]==1){
		clk=clk-25;
		if(clk<0){clk=0;}
	}
}


int main(void){
	// khai bao chan 
	wiringPiSetUpGpio();
	//khai bao nat nhan la ngo vao, led la ngo ra
	for (int i=0; i<3;i++){
		pinMode(bt_pin[i], INPUT);
		pullUpDnControl(bt_pin[i], PUD_UP);
		pinMode(led[i], OUTPUT);
		softPwmCreate(led[i],0,100);
		
	}
	// khai bao interrupt
	
	wiringPiIRS(bt_pin[0], INT_EDGE_BOTH,&bt_1);
	wiringPiIRS(bt_pin[1], INT_EDGE_BOTH,&bt_2);
	wiringPiIRS(bt_pin[2], INT_EDGE_BOTH,&bt_3);
	
	// khai bao PWM
	
	
	while(1){
		if (dem==1)
		{
			softPwmWrite(led[0], clk);
			softPwmWrite(led[1],0);
			softPwmWrite(led[2],0);
	}
	if (dem==2)
		{
			softPwmWrite(led[0], 0);
			softPwmWrite(led[1],clk);
			softPwmWrite(led[2],0);
		}
	if (dem==3)
		{
			softPwmWrite(led[0], 0);
			softPwmWrite(led[1],0);
			softPwmWrite(led[2],clk);
		}
}