#include<stdio.h>
#include<stdint.h>
#include<wiringPi.h>
#include<softPwm.h>
#include<stdlib.h>
#include<time.h>

uint8_t led_pin[3]={0,2,3};
uint8_t bt_pin[3]={21,22,23};
uint8_t state=0;
// tao ham random
int GetRandom(int min, int max){
    return min + (int)(rand()*(max-min+1.0)/(1.0+RAND_MAX));
}
// ngat cho button red
void bt_red(void){
	if(digitalRead(bt_pin[0]==1){
		state=0;
	}
}
// ngat cho button green
void bt_green(void){
	if(digitalRead(bt_pin[1]==1){
		state=1;
	}
}
// ngta cho button blue
void bt_blue(void){
	if(digitalRead(bt_pin[2]==1){
		state=2;
	}
}
int main(void){
	
	// khai bao chan cam rasp
	wiringPiSetUpGpio();
	
	//khai bao ngo ra led_pin
	for(int i=0;i<3;i++){
		pinMode(led_pin[i],OUTPUT);
		pinMode(bt_pin[i], INPUT);
		pullUpDnControl(bt_pin[i], PUD_UP);
	}
	
	// khai bao ngat cho nut nhan
	wiringPiIRS(bt_pin[0], INT_EDGE_BOTH,&bt_red);
	wiringPiIRS(bt_pin[1], INT_EDGE_BOTH,&bt_green);
	wiringPiIRS(bt_pin[2], INT_EDGE_BOTH,&bt_blue);
	
	while(1){
	int dem=0;
	// lam moi thong so dau vao cho moi lan random 
	srand((unsigned) time(NULL));
	int n=GetRandom(0,2);
	digitalWrite(led_pin[n],HIGH);
	for(int i =0; i<30;i++){
		delay(100);
		dem++;
		if (dem<=5){
			if(state==n){
				digitalWrite[led_pin[n], LOW);
				for (int j =0; j<10;j++){
					digitalWrite[led_pin[0],HIGH);
					digitalWrite[led_pin[1],HIGH);
					digitalWrite[led_pin[2],HIGH);
					delay(50);
					digitalWrite[led_pin[0],LOW);
					digitalWrite[led_pin[1],LOW);
					digitalWrite[led_pin[2],LOW);
					delay(50);
					
				}
				printf("you win");
				
				break;
			}
		}
		else{
			printf("you lose");
			digitalWrite[led_pin[n], LOW);
			
		}
		
	}
	}
}