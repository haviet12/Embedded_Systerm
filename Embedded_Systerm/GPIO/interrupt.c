#include<wiringPI.h>
#include<stdio.h>

#define Led 12
#define Bt 11

void interrupt_button(void)
{
	if (digitalRead(Bt)==1)
	{
		digitalWrite(Led,HIGH);
	}
	else{
		digitalWrite(Led,LOW);
	}
}
int main()
{
	// khai bao setup thu vien wiringpi
	wiringPiSetupPhys();
	// khai bao pinmode
	pinMode(Led, OUTPUT);
	pinMode(Bt, INPUT);
	
	// khai bao interrupt
	wiringPiISR(Bt,INT_EDGE_BOTH,&interrupt_button);
	
	while(1)
	{
		
	}
	return 0;
}
