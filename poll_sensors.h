#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <mbed.h>
#include <string.h>
#include <stdlib.h>

int altitude_sensor;                          /* added '_sensor' to show this is a sensor value */
int acceleration_sensor[3];
bool altitude_updated;
bool acceleration_updated;
double x, y, z;
double b = 0;
char buffer[7]; 				 // declare buffer 
int ndx = 0; 
memset(&buffer, 0, sizeof(buffer)); 		 // assign memory locations to value zero
      

// function prototypes
void init();

void poll_sensors(float *altitude, float *acceleration, 
	bool *altitude_updated, bool *acceleration_updated);

// initialize hardware connections
void init(){

	Serial pc(USBTX, USBRX); // tx, rx
	Serial device(p13,p14);// tx, rx
	DigitalOut myled(LED1);
	uLCD_4DGL uLCD(p9,p10,p11);
	MMA8452 acc(p28, p27, 100000);
	
	}

// poll sensor code
void poll sensors(int *altitude, double *acceleration, bool *altitude_updated, bool *acceleration_updated){

	*altitude = 0;            /* Setting the value that the pointer is pointing to */
	acceleration[0] = 0;
	acceleration[1] = 0;
	acceleration[2] = 0;

	printf("poll_sensors\n");
	
	while(1) {
		
	    if(device.readable()) {
            char a = device.getc(); 		 	 // read char from altimeter
            //pc.putc(a); 				 // write char to computer
            if ((a != 0x0D) && (ndx < sizeof(buffer))){  // if not equal to new line char and less than the size of the buffer
              buffer[ndx] = a; 				 // assign char to buffer at ndx
              ndx = ndx + 1; 				 // increment index
              }
            else { 					 // when new line char
              altitude_sensor = atoi(buffer); 	         // write the altitude to an int 
              printf("the altitude in ft is: %d \n" , altitude);           
              memset(&buffer, 0, sizeof(buffer));
              ndx = 0;  
                
            	}
    			}

	     // Change altitude value & report that it has been updated
	     if(altitude_sensor != *altitude){
		&altitude = altitude_sensor;
		*altitude_updated = true;
		}	

	      acc.readXYZGravity(&x,&y,&z);
              led1 = abs(x);
              led2 = abs(y);
              led3 = abs(z);
	       
              // Change acceleration value & report that it has been updated
              if( (x != acceleration[0]) || (y != acceleration[1]) || (z != acceleration[2])){

			acceleration[0] = x;
			acceleration[1] = y;
			acceleration[2] = z;
			*acceleration_updated = true;		
		}	
		
              //wait(.01);
              uLCD.locate(1,1);
              uLCD.printf("x:%lf\ny:%lf\nz:%lf\r\n",x,y,z); //Default Green on black text  
	
	
}


