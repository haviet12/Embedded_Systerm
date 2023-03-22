#include<stdio.h>
#include<unistd.h> // sleep();
#include<stdlib.h> // rand();
int main()
{
    while(1)
    {
        float avg =0;
        for (int i = 0; i <10;i++) 
        {
            avg =avg +rand()%100;
        }
        avg=avg/10.0;
        printf("avg= %.2f\n", avg);
        sleep(1);
    }
    return 0;
}
