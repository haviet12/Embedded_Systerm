#include<stdio.h>
#include<wiringPi.h>
#include<softPwm.h>
#include<stdint.h>

/*#define led_R 13
#define led_G 14
#define led_B  15*/
uint8_t list_pin[3]={13,14,15};
uint16_t duty=0;
int main(void)
{
	// khai bao chan theo GPIO
	wiringPiSetupGpio();
	// khai bao ngo ra va tao xung PWM
	for ( int i=0; i<3; i++)
	{
		pinMode(led_pin[i], OUTPUT);
		softPwmCreate(led_pin[i],0,100);
	}
	while(1)
	{
		// sang dan
		for (int i=0;i<3;i++)
		{
			for (int j=0; j<20;j++)
			{
				softPwmWrite(led_pin[i],j*5);
				delay(100);
			}
				// tat dan
			for (int m=19;m>=0;m--)
			{
				softPwmWrite(led_pin[i],m*5);
				delay(100);
			}
		}
	
		
		
	}
}
