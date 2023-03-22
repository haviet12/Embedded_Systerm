#include<stdio.h>
#include<stdint.h>
#include<wiringPi.h>
#include<softPwm.h>
#include<stdlib.h>
#include<time.h>


uint8_t led_pin[3]={0,2,3};
#define bt 22
int count;
uint8_t state;
int time;
int loop=0;

int GetRandom(int min, int max){
	 return min + (int)(rand()*(max-min+1.0)/(1.0+RAND_MAX));
}
void ngaat_bt(void){
	if (digitalRead(bt)==1){
		state=1;
		
	}
}
int main(void){
		// khai bao chan cam GPIO
		wiringPiSetUpGpio();
		// khai ba ngo ra cho led + ngo vao cho button
		for(int i=0; i<3;i++){
			pinMode(led_pin[i],OUTPUT);
			
		}
		pinMode(bt,INPUT);
		
		// khai bao ham ngat cho nut nhan
		wiringPiIRS(bt,INT_EDGE_BOTH,&ngat_bt);
		while(loop<10){
			count=0;
			//////////////////////////////////////////// lam moi so dau moi khi random
			srand((unsigned) time(NULL));
			///////////////////////////////////////////
			int r=GetRandom(0,2);
			for (int i =0; i<10;i++){
				digitalWrite(led_pin[r], HIGH);
				count++;
				delay(100);
				if (state==1){
					digitalWrite(led_pin[r], LOW);
					time=count*100;
					printf("thoi gian nhan nut tu khi den sang"%d, time)
				}
				else{continue;}
			}
			if (state=0){
				
				digitalWrite(led_pin[r], LOW);
				printf("......");
				
			int t= GetRandom(10,30);
			for (int j=0;j<t;j++){
				delay(100);
			}
			loop=loop+1;
				
			}
			
			
		}
}