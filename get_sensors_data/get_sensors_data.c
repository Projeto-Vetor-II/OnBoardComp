/**
 * @file rc_altitude.c
 * @example    rc_altitude
 *
 * This serves as an example of how to read the barometer and IMU together to
 * estimate altitude
 *
 * @author     James Strawson
 * @date       3/14/2018
 */
#include <stdio.h>
#include <signal.h>
#include <string.h>
#include <math.h> // for M_PI
#include <rc/time.h>
#include <rc/gpio.h>


int main(void)
{    
	printf("Testando\n");
	fflush(stdout);
    rc_gpio_init(2,3,GPIOHANDLE_REQUEST_OUTPUT);

	 for(int i = 0; i < 20; i++)
	 {
		printf("%d\n",i);
		fflush(stdout);
		rc_gpio_set_value(2,3,1);
		rc_usleep(1000000);
		rc_gpio_set_value(2,3,0);
		rc_usleep(1000000);
	 }

	 return 0;
}

