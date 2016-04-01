/*
 * Example flight software using a scheduler function
 * to call blocks of code at different rates.
 *
 * The variable names are irritatingly long for clarity. May want to change them
 * so stuff will better fit on single lines (ie accel vs acceleration)
 *
 * All functions return an integer success code 0 and -1 for failure. This is
 * most likely overkill for this applicaiton, but I included it here anyways.
 *
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/*
 * Time between running blocks of code. Not based on time but rather how much
 * each block will run relative to the others. For example, the sensors will be
 * polled once for every 5 times the state estimator runs.
 */
static uint16_t POLL_SENSOR_RATE 	= 5;
static uint16_t STATE_ESTIMATE_RATE     = 1;
static uint16_t CONROLLER_RATE 		= 100;

/*
 * Variables for holding sensor values, and flags if they have been updated
 * (Once they are used by the state estimator, they will be set back to false)
 */


//float altitude_sensor; 			/* added '_sensor' to show this is a sensor value */
//float acceleration_sensor[3];
//bool altitude_updated;
//bool acceleration_updated;


/* Estimations of position, velocity, and acceleration */
float position_est[3]; /* added '_est' to show this is an estimation */
float velocity_est[3];
float acceleration_est[3];

/* True if pins are currently deployed, false otherwise */
bool control;

/* 
 * Count how many times loop has run. Use to determine when to call different
 * blocks. unsigned so an overflow won't cause it to go negative.
 */
uint16_t schedule_counter = 0;

/* Function prototypes. Normally will go in the header file with details */
//int init();

int scheduler();

//int poll_sensors(float *altitude, float *acceleration,
//	bool *altitude_updated, bool *acceleration_updated);

int state_estimate(float altitude, float *acceleration,
	bool altitude_updated, bool acceleration_updated,
	float *position_est, float *velocity_est, float *acceleration_est);

int controller(float *position_est, float *velocity_est,
	float *acceleration_est, bool *control);

int actuator(bool control);

/*
 * Point of program entry.
 */
int main()
{
	/* Initalize system */
	if (init() < 0) {
		printf("Failed to initalize.\n");
		return -1;
	}

	/* Run the scheduler endlessly. Minor performance increase
	 * from including the loop inside the function and avoiding
	 * repeated function call overhead.
	 */
	if (scheduler() < 0) {
		printf("Error.\n");
		return -1;
	}
	return 0;
}

int scheduler()
{

	/* Normally for ( ;; ) or while (1), but this allows you to see output */
	int i = 0;
	for (i = 0; i < 2000; i++) {
		/*
		 * The mod of schedule_counter and the poll rate will be zero when the
		 * functions should be called. For example, if POLL_SENSOR_RATE is 5,
		 * schedule_counter % POLL_SENSOR_RATE will only equal zero when
		 * schedule_counter is 5, 10, 15, ...
		 */
		if (!(schedule_counter % POLL_SENSOR_RATE)) {
			/*
			 * This is a standard way of calling functions that may be error
			 * prone. If the return is -1 handle the error, otherwise the
			 * funciton just ran normally
			 */
			if (poll_sensors(&altitude_sensor, acceleration_sensor,
								&altitude_updated, &acceleration_updated) < 0) {
				/* Error handling stuff here */
				printf("Error polling sensors.\n");
				return -1;
			}
		}
		if (!(schedule_counter % STATE_ESTIMATE_RATE)) {
			if (state_estimate(altitude_sensor, acceleration_sensor,
								altitude_updated, acceleration_updated,
								position_est, velocity_est,
								acceleration_est) < 0) {
				/* Error handling stuff here */
				printf("Error estimating state.\n");
				return -1;
			}
		}
		if (!(schedule_counter % CONROLLER_RATE)) {
			if (controller(position_est, velocity_est, acceleration_est,
								&control) < 0) {
				/* Error handling stuff here */
				printf("Error in controller.\n");
				return -1;
			}
			if (actuator(control) < 0) {
				/* Error handling stuff here */
				printf("Error in actuator.\n");
				return -1;
			}
		}
		/* Note that schedule_counter is an unsigned integer. Once it hits
		 * limit it will just go right back to zero, and zero causes no errors,
		 * so no need for bound checks here.
		 */
		schedule_counter++;
	}
	return 0;
}

/*
 * Code blocks. Implement per function prototype requirements. You could just as
 * easily break these up into different files for each block (ie controller) and
 * then it might be easier to maintain and add supporting functions. C will see
 * both the same though.
 */

int init()
{
	/* Do stuff here. Setup objects. Set pins. Flip LEDs. */
	return 0;
}

//int poll_sensors(float *altitude, float *acceleration,
//	bool *altitude_updated, bool *acceleration_updated)
//{
//	/* Logic here */
//	*altitude = 0; /* Setting the value that the pointer is pointing to */
//	acceleration[0] = 0;
//	acceleration[1] = 0;
//	acceleration[2] = 9.81;
//
//	*altitude_updated = true; /* If the sensor had new data, do this */
//	*acceleration_updated = true;
//	printf("poll_sensors\n");
//      /* Successful return */
//	return 0;
//}

int state_estimate(float altitude, float *acceleration,
	bool altitude_updated, bool acceleration_updated,
	float *position_est, float *velocity_est, float *acceleration_est)
{
	printf("state_estimate\n");
	/*
	 * Kalman filter here
	 * NOTE: Kalman filter will have logic to handle updated sensor values and
	 * this functionw will be called more often without updated values. (Hense
	 * the altitude_updated and acceleration_updated flags)
	 */
	return 0;
}

int controller(float *position_est, float *velocity_est,
	float *acceleration_est, bool *control)
{
	printf("controller\n");
	*control = true; /* DEPLOY AIRBRAKES!!!! */
	/* Control algorithm. May use memory or timer */
	return 0;
}

int actuator(bool control)
{
	printf("actuator\n");
	/* Do stuff with electricity */
	return 0;
}
