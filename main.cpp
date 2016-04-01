/**
 * Georgia Tech Student Launch Flight Software, 2016
 *
 * Matthew Allen 	mallen72@gatech.edu
 * Jack Ridderhof 	jridderhof3@gatech.edu
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>         /* atoi */
#include <cmath>
#include <string.h>         /* memset */
#include "tables.h"
#include "mbed.h"
#include <MMA8452.h>
#include <uLCD_4DGL.h>
#include "Servo.h"
#include "intheloop.h"

/* ****************************************************************************/
/* Prototypes */

/* Functions supporting Kalman filter */
void matScaleMultiplication(int row, int column, float **mat, float scale,
    float** result);
void matMultiplication(int row1, int column1, float **mat1, int row2,
    int column2, float **mat2, float** result);
void transpose(int row_in, int column_in, float **mat_in, float **mat_out);
void mat2Inverse(float **a, float **ainv);
void matSum(int row,  int column, float **mat1, float **mat2,
    float** result);

/* General functions */
int init();
int scheduler();
void state_estimate(float h_meas_m, float h_prev_meas_m, float a_meas_ms2,
    float dt_meas_s, float *h_est_m, float *v_est_ms);

/* Controller functions */
int controller(float h_est_m, float v_est_ms, float t_flt_s);
int get_index_time(float time1);
float sign(float u);
float get_hp(float time);
float get_vp(float time);
float get_ap(float time);

/* Sensor functions */
float poll_alt();
float poll_acc(uint8_t axis, uint8_t is_flip);

/* Actuator functions */
void actuator(int control);

/* ****************************************************************************/
/* Definitions */

/*
 * Time between running blocks of code. Not based on time but rather how much
 * each block will run relative to the others. For example, the sensors will be
 * polled once for every 5 times the state estimator runs.
 */

/**
 * \def Flag set to true for flight. Else for ground debug.
 */
#define IS_FLIGHT				(0)

#define POLL_SENSOR_RATE        (1)
#define STATE_ESTIMATE_RATE     (1)
#define CONROLLER_RATE          (10)

#define LAUNCH_DETECT_ACCEL_MS2 (2 * 9.81)
#define LAUNCH_DETECT_COUNT     (10)
#define ACCEL_AXIS              (2)
#define IS_ACCEL_FLIP			(0)		/* True if flip sign of accel values */
#define IS_ACCEL_CONNECTED		(1)

#define ALT_SIM_NOISE_MAG		(10)
#define ACCEL_SIM_NOISE_MAG		(10)

/* Conversions for table lookup values */
#define HP_CONV_TO_M			(1 / 1000.0 * 1609.34) 		/* mi*1000 -> m */
#define VP_CONV_TO_MS			(1 / 1000.0 * 1609.34)
#define AP_CONV_TO_MS2			(1)

/* Hardware in the loop functions */
#if (!IS_FLIGHT)
int get_index_time_sim(float time1);
float get_h_sim(float time1);
float get_a_sim(float time1);
#endif

/* ****************************************************************************/
/* Variables */

/* constant needed for the control part */
float Crocket;                      /* drag coefficient of the rocket */
float Cflaps;                       /* drag coefficient of the flaps */
float mass_rocket;
float surface_rocket;
float Spo;

float t_launch_s;                   /* Time of launch in seconds */
Timer main_timer;

#if (!IS_FLIGHT)
Timer hil_timer;					/* Timer for hardware in the loop ops */
#endif

/* Physical devices */
Serial pc(USBTX,USBRX);
Serial alt_dev(p13,p14);            /* Altimeter (tx, rx) */ 
#if(IS_ACCEL_CONNECTED)
MMA8452 acc_dev(p9, p10, 100000);  	/* Accelerometer */ //9&10
#endif
Servo servo_dev_1(p21);             /* Servo motor */ //21-24
Servo servo_dev_2(p22);
Servo servo_dev_3(p23);
Servo servo_dev_4(p24);
DigitalOut led1_dev(LED1);

//KALMAN FILTER VARIABLES
float **P_Kalman = new float*[2];
float ** R_Kalman = new float*[2];
float ** Q_Kalman = new float*[2];
float **A = new float*[2];
float ** Atr = new float*[2];
float ** B = new float*[2];
float ** x_minus = new float*[2];
float ** xvec_est = new float*[2];
float ** P_minus = new float*[2];
float ** K = new float*[2];
float ** temp1 = new float*[2];
float ** temp2 = new float*[2];
float ** temp3 = new float*[2];
float ** temp4 = new float*[2];
float ** temp5 = new float*[2];
float ** temp6 = new float*[2];
float ** temp7 = new float*[2];
float ** temp8 = new float*[2];
float ** inv_temp = new float*[2];
int unpwr_asc = 0;
// END KALMAN FILTER VARIABLES

/* ****************************************************************************/
/* General functions */

int main()
{
	/* Record program start for hardware in the loop */
	#if (!IS_FLIGHT)
	hil_timer.start();
	#endif

    pc.printf("Hello world!\n");
    /* Initalize system */
    if (init() < 0) {
        pc.printf("Failed to initalize.\n");
        return -1;
    }
    /* Wait until launch detected to continue */
    float a_meas_ms2;
    int launch_detect_counter = 0;

    while (true) {

        a_meas_ms2 = poll_acc(ACCEL_AXIS, IS_ACCEL_FLIP);
        if (a_meas_ms2 >= LAUNCH_DETECT_ACCEL_MS2) {
            /* Read launch time if this is the first trigger signal */
            if (!launch_detect_counter)
                t_launch_s = main_timer.read();

            /* If we pass the count threshold exit loop */
            if (++launch_detect_counter > LAUNCH_DETECT_COUNT)
                break;
        } else {
            /* Reset counter */
            launch_detect_counter = 0;
        }
    }
    pc.printf("Launch!!\n");
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

/**
 * Initialize program
 * 
 * @return 0 on success, -1 on failure
 */
int init()
{
    /* Start timer */
    main_timer.start();
    
    /* Retract ATS */
    actuator(0);

    /* KALMAN FILTER VARIABLES */
    for (int i=0;i<2;i++) {
         P_Kalman[i] = new float[2];  //ADDED BY DD.
         R_Kalman[i] = new float[2];  //ADDED BY DD.
         Q_Kalman[i] = new float[2];  //ADDED BY DD.
         A[i] = new float[2];
         Atr[i] = new float[2];
         B[i] = new float[1];
         x_minus[i] = new float[1];
         xvec_est[i] = new float[1];
         P_minus[i] = new float[2];
         K[i] = new float[2];
         temp1[i] = new float[2];
         temp2[i] = new float[2];
         temp3[i] = new float[2];
         temp4[i] = new float[2];
         temp5[i] = new float[1];
         temp6[i] = new float[1];
         temp7[i] = new float[1];
         temp8[i] = new float[1];
         inv_temp[i] = new float[2];
     }

    R_Kalman[0][0] = 43.37*43.37;
    R_Kalman[1][1] = 99.41*99.41;
    R_Kalman[0][1] = 0.0;
    R_Kalman[1][0] = 0.0;
    Q_Kalman[0][0] = 0.0;
    Q_Kalman[1][1] = 3.0*3.0;
    Q_Kalman[0][1] = 0.0;
    Q_Kalman[1][0] = 0.0;
    P_Kalman[0][0] = 1.0*1.0;
    P_Kalman[1][1] = 1.0*1.0;
    P_Kalman[0][1] = 0.0;
    P_Kalman[1][0] = 0.0;
    /* END KALMAN FILTER VARIABLES */

    return 0;
}

/**
 * Main program scheduler
 * 
 * @return -1 on failure
 */
int scheduler()
{
    uint16_t schedule_counter = 0;

    /* Sensor measurements */
    float h_meas_m          = 0;
    float h_prev_meas_m     = 0;
    float dt_meas_s         = 0;
    float a_meas_ms2        = 0;

    /* Flags to be set when sensor values are updated */
    uint8_t is_h_update     = 0;
    uint8_t is_a_update     = 0;

    /* Estimation of rocket's state, from Kalman filter */
    float h_est_m           = 0;
    float v_est_ms          = 0;

    /* Timer for difference in measurement times */
    Timer meas_timer;
    meas_timer.start();

    /* IMPORTANT: CHANGE TO "while (1)' BEFORE FLIGHT */
    while (1) {
        if (!(schedule_counter % POLL_SENSOR_RATE)) {
            float new_h_m = poll_alt();
            if (new_h_m > 0) {
                dt_meas_s = meas_timer.read(); 
                h_prev_meas_m = h_meas_m;
                h_meas_m = new_h_m;
                meas_timer.reset();
            }

            float new_a_ms2 = poll_acc(ACCEL_AXIS, IS_ACCEL_FLIP);
            if (new_a_ms2 > 0) a_meas_ms2 = new_a_ms2;
        }
        if (!(schedule_counter % STATE_ESTIMATE_RATE)) {
        	if (dt_meas_s)
            state_estimate(h_meas_m, h_prev_meas_m, a_meas_ms2, dt_meas_s,
                &h_est_m, &v_est_ms);
        }
        if (!(schedule_counter % CONROLLER_RATE)) {
            float t_flt_s = main_timer.read() - t_launch_s;
            // if (t_flt_s > 2.5) {
	            int control = controller(h_est_m, v_est_ms, t_flt_s);
	            actuator(control);
        	// }
            if (led1_dev == 1) led1_dev = 0; else led1_dev = 1;
        }
        schedule_counter++;
        wait_ms(100);
    }
    return 0;
}

/* ****************************************************************************/
/* Sensor functions */

#if(IS_FLIGHT)
/**
 * Poll the altimeter and return an altitude in meters, else return -1.
 * 
 * @return Altitude in meters, if no update then -1
 */
float poll_alt()
{
	pc.printf("poll_alt...");
    char a;
    const char lf = 0x0A;       /* Line feed character */
    uint8_t is_h_update = 0;    /* True if new altitude value returned */

    char buffer[20];            /* Allocate buffer */
    memset(&buffer, 0, sizeof(buffer)); /* Clear buffer */
    int ndx = 0; 

    int32_t h_read_ft = 0;      /* Altitude read from sensor */
    int32_t n_h_read_ft = 0;    /* New altitude read for comparison */
    float h_read_m;
    const float ft2m = 0.3048;  /* Feet to meters conversion */

    if(alt_dev.readable()) {
        /* Read through buffer */
        while(alt_dev.readable()) {
            a = alt_dev.getc();

            /* If end of line or the buffer is full */
            if ((a == 0x0A) || (ndx == sizeof(buffer))) {
            /* TODO: Handle full buffer as special case. Here it will give us
             * an erronious altitude if we convert an unfinished reading to an
             * integer. Will be serious problem if buffer overflows. */
                n_h_read_ft = atoi(buffer);
                if (n_h_read_ft == h_read_ft)
                    is_h_update = 0;
                else
                    is_h_update = 1;
                /* Assign the new reading to the current reading */
                h_read_ft = n_h_read_ft;
            } else
                /* Character is NOT a LF, so put it in the buffer */
                buffer[ndx++] = a;
        }
        /* Finished reading through buffer, return read value */
        h_read_m = h_read_ft * ft2m;
        pc.printf("h: %f \n" , h_read_m);
        return h_read_m;
    } 
    /* If altimeter is not readable, return not updated */
    pc.printf("not readable\n");
    return -1;
}

/**
 * Poll accelerometer. Return acceleration in m/s2
 * 
 * @param axis The axis of acceleration of interest. x:1 y:2 z:3
 * @param is_flip 0 if not change sign of value, else change
 * @return Absolute value of acceleration in m/s2 
 */
float poll_acc(uint8_t axis, uint8_t is_flip)
{
	pc.printf("poll_acc...");
    double x, y, z, new_a_ms2;

    #if(IS_ACCEL_CONNECTED)
		acc_dev.readXYZGravity(&x,&y,&z);
	#else
	    x=1;
	    y=1;
	    z=1;
	#endif

    /* Select axis we are interested in */
    switch (axis) {
        case 1:
            new_a_ms2 = x;
            break;
        case 2:
            new_a_ms2 = y;
            break;
        case 3:
            new_a_ms2 = z;
            break;
    }

    float new_a_ms2_flip = is_flip ? -new_a_ms2 : new_a_ms2;
    pc.printf("%f\n", new_a_ms2_flip);
    return new_a_ms2_flip;
}

#else /* ! IS_FLIGHT ( GROUND TEST )*/

/**
 * GROUND TEST ONLY
 * Mimic polling of altimeter through table lookup 
 *
 * NOTE Function uses the main_timer and t_launch_s to get a flight time for
 * use in table lookup
 * @return Altitude in meters
 */
float poll_alt()
{
	/* Get time into program in seconds */
	float t_prog_s = hil_timer.read();
	/* Lookup simulated trajectory altitude (with noise) for given time */
	float h_lookup_m = get_h_sim(t_prog_s);
	pc.printf("poll alt...h: %f \n" , h_lookup_m);
	return h_lookup_m;
}

/**
 * GROUND TEST ONLY
 * Mimic polling of accelerometer through table lookup
 *
 * NOTE Function uses main_timer and t_launch_s to get time into flight for
 * table lookup
 * @param  axis The axis of acceleration of interest. x:1 y:2 z:3
 * @return Absolute value of acceleration in m/s2 
 */
float poll_acc(uint8_t axis, uint8_t is_flip)
{
	/* Get time into program in seconds */
	float t_prog_s = hil_timer.read();
	/* Lookup simulated trajectory acceleration (with noise) for given time */
	float a_lookup_ms2 = get_a_sim(t_prog_s);
	pc.printf("a: %f\n", a_lookup_ms2);
	return a_lookup_ms2;
}
#endif

/**
 * Kalman filter state estimator
 *
 * Estimates the state of the rocket, defined by the state vector xvec with the
 * contents [height, velocity]. This function uses all metric units.
 *
 * Input current and previously measured altitude (meters) h_meas_m,
 * h_prev_meas_m, currently measured acceleration (meters/sec2) a_meas_ms2, and
 * the time difference between the altitude measurements (sec).
 *
 * Output will be an estimation of altitude (meters) h_est_m, and an estimation
 * of vertical velocity (meters/sec) v_est_ms.
 * 
 * @param h_meas_m Measured altitude in meters
 * @param h_prev_meas_m Previously measured altitude in meters
 * @param a_meas_ms2 Measured vertical acceleration in meters per second squared
 * @param dt_meas_s Time difference between altitude measurements in seconds
 * @param *h_est_m Reference to place estimation solition for altitude (m)
 * @param *v_est_ms Reference to place estimation solition for velocity (m/s)
 */
void state_estimate(float h_meas_m, float h_prev_meas_m, float a_meas_ms2,
    float dt_meas_s, float *h_est_m, float *v_est_ms)
{
    //pc.printf("state_estimate\n");

    /* KALMAN ALGORITHM: (for reference)
        x_minus = A*x_est(:,i) + B*u(i+1);
        P_minus = A*P_Kalman*A' + Q;
        K = P_minus*inv(P_minus + R);
        x_est(:,i+1) = x_minus + K*([h_act(i+1);v_act(i+1)] - x_minus);
        P_Kalman = (eye(2) - K)*P_minus;
    */

    A[0][0] = 1.0;
    A[1][1] = 1.0;
    A[0][1] = dt_meas_s;
    A[1][0] = 0.0;
    B[0][0] = 0.0;
    B[1][0] = dt_meas_s;

    /* Estimate the velocity from change in altitude */
    float v_meas_ms = (h_meas_m - h_prev_meas_m) / dt_meas_s;

	pc.printf("kf...meas: dt %f; %f, %f...", dt_meas_s, h_meas_m, v_meas_ms);

    /* Populate the state matrix with measurements */
    xvec_est[0][0] = h_meas_m;
    xvec_est[1][0] = v_meas_ms;

    /* Run Kalman algorithm */
    matMultiplication(2, 2, A, 2, 1, xvec_est, temp5);
    matScaleMultiplication(2, 1, B, a_meas_ms2, temp6);
    matSum(2, 1, temp5, temp6, x_minus);

    matMultiplication(2, 2, A, 2, 2, P_Kalman, temp1);
    transpose(2, 2, A, Atr);
    matMultiplication(2, 2, temp1, 2, 2, Atr, temp2);
    matSum(2, 2, temp2, Q_Kalman, P_minus);

    matSum(2, 2, P_minus, R_Kalman, temp3);
    mat2Inverse(temp3, inv_temp);
    matMultiplication(2, 2, P_minus, 2, 2, inv_temp, K);

    temp7[0][0] = h_meas_m;
    temp7[1][0] = v_meas_ms;
    matScaleMultiplication(2, 1, x_minus, -1, temp8);
    matSum(2, 1, temp7, temp8, temp5);
    matMultiplication(2, 2, K, 2, 1, temp5, temp6);
    matSum(2, 1, temp6, x_minus, xvec_est);

    /* Unpack state estimate vector */
    *h_est_m = xvec_est[0][0];
    *v_est_ms = xvec_est[1][0];

    /* Updates to Kalman sate variables */
    temp3[0][0] = 1.0;
    temp3[0][1] = 0.0;
    temp3[1][0] = 0.0;
    temp3[1][1] = 1.0;
    matScaleMultiplication(2, 2, K, -1, temp4);
    matSum(2, 2, temp3, temp4, temp1);
    matMultiplication(2, 2, temp1, 2, 2, P_minus, P_Kalman);
    pc.printf("est: %f, %f\n", *h_est_m, *v_est_ms);
}

/* ****************************************************************************/
/* Controller functions */

/**
 * Sliding mode nonlinear controller
 * 
 * @param h_est_m Estimated altitude in meters
 * @param v_est_ms Estimated vertical velocity in meters per second
 * @param t_flt_s Time of flight (since launch) in seconds
 * @return Boolean control command
 */
 
int controller(float h_est_m, float v_est_ms, float t_flt_s)
{
	pc.printf("cont...");
    float control;
    float delta_h;
    float delta_v;
    float s;
    float lambda = 0.1;
    delta_h = h_est_m - get_hp(t_flt_s); /* TODO fix time units */
    delta_v = v_est_ms - get_vp(t_flt_s);
    s = lambda * delta_h + delta_v;
    control = (sign(s) + 1)/2;
    pc.printf("done\n");  
    return control;
}

/**
 * Get index in lookup tables for a time from launch
 * @param  time Time from lift off in seconds
 * @return      Index in lookup tables
 */
int get_index_time(float time1)
{
    int i = 0;
    if (time1 > 19)
        return 50;
    else {
        while (time_100[i]/100.0 < time1)
            i++;
        return i - 1;
    }
}

float sign(float u)
{
    if (u > 0)
        return 1.0;
    if (u < 0)
        return -1.0;
    return 0;
}

/**
 * Return height from nominal table in meters
 * @param  time Time from lift off in seconds
 * @return      Altitude in meters
 */
float get_hp(float time)
{
	/* Table lookup. Units of mile * 1000 */
	int hp = table_hp[get_index_time(time)];
    /* Convert to meters */
    float hp_m = HP_CONV_TO_M * hp;
    return hp_m;
}

/**
 * Return velocity from nominal table in meters per second
 * @param  time Time from lift off in seconds
 * @return      Veclocity in m/s
 */
float get_vp(float time)
{
    int vp = table_vp[get_index_time(time)];
    /* Convert to m/s */
    float vp_ms = VP_CONV_TO_MS * vp;
    return vp_ms;
}

#if (!IS_FLIGHT)

/**
 * GROUND TEST ONLY
 * Get index in lookup tables for a time after program start
 * @param  time Time from program start in seconds
 * @return      Index in lookup tables for hardware in loop tables
 */
int get_index_time_sim(float time1)
{
    int i = 0;
    if (time1 > 19)
        return 50;
    else {
        while (time_100_sim[i] / 100.0 < time1)
            i++;
        return i - 1;
    }
}

/**
 * GROUND TEST ONLY
 * Altitude from simulation lookup table
 * @param  time Time from program start
 * @return      Altitude in meters
 */
float get_h_sim(float time1)
{
	int16_t h_sim = table_h_sim[get_index_time_sim(time1)];
	float h_sim_m = HP_CONV_TO_M * h_sim;
	return h_sim_m;
}

/**
 * GROUND TEST ONLY
 * Acceleration from simulation talbe lookup
 * @param  time Time from program start
 * @return      Acceleration in m/s2
 */
float get_a_sim(float time1)
{
	int16_t a_sim = table_a_sim[get_index_time_sim(time1)];
	float a_sim_ms2 = a_sim / 100.0;
	return a_sim_ms2;
}

#endif

/**
 * Command servo motors
 * @param control 1 if extend drag flaps, else retract
 */
void actuator(int control)
{
    //pc.printf("actuator\n");
    if(control == 1) {
		servo_dev_1 = 0;
		servo_dev_2 = 0;
		servo_dev_3 = 0;
		servo_dev_4 = 0;
    } else {
		servo_dev_1 = 1;
		servo_dev_2 = 1;
		servo_dev_3 = 1;
		servo_dev_4 = 1;
	} 
}

/* ****************************************************************************/
/* Matrix algebra supporting Kalman filter */

void matScaleMultiplication(int row, int column, float **mat, float scale,
    float** result) {
    for (int i = 0; i < row; i++) {
        for (int j = 0; j < column; j++) {
            result[i][j] = mat[i][j] * scale;
        }
    }
}

void matMultiplication(int row1, int column1, float **mat1, int row2,
    int column2, float **mat2, float** result) {

    if (column1 != row2) {
      //  Serial.println("Error: matrix dimension mismatch!");
    } else {
        float sum;
        for (int i = 0; i < row1; i++) {
            for (int j = 0; j < column2; j++) {
                sum = 0;
                for (int k = 0; k < column1; k++) {
                    sum += mat1[i][k] * mat2[k][j];
                }
                result[i][j] = sum;
            }
        }
    }
}

void matSum(int row,  int column, float **mat1, float **mat2,
    float** result) {

    for(int i=0; i<row; i++) {
        for(int j=0; j<column; j++) {
            result[i][j] = mat1[i][j] + mat2[i][j];
        }
    }
}

void transpose(int row_in, int column_in, float **mat_in, float **mat_out) {
    float temp;
    for (int j = 0; j<column_in; j++) {
        for (int i = 0; i<row_in; i++) {
            temp = mat_in[i][j];
            mat_out[j][i] = temp;
        }
    }
}

void mat2Inverse(float **a, float **ainv) {

  float a_ = a[0][0];
  float b_ = a[0][1];
  float c_ = a[1][0];
  float d_ = a[1][1];

  ainv[0][0] = d_/(a_*d_* - b_*c_);
  ainv[0][1] = -b_/(a_*d_* - b_*c_);
  ainv[1][0] = -c_/(a_*d_* - b_*c_);
  ainv[1][1] = a_/(a_*d_* - b_*c_);
}
