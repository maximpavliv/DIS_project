/*****************************************************************************/
/* File:         raynolds2.c                                                 */
/* Version:      2.0                                                         */
/* Date:         06-Oct-15                                                   */
/* Description:  Reynolds flocking with relative positions		     */
/*                                                                           */
/* Author: 	 06-Oct-15 by Ali Marjovi				     */
/* Last revision:12-Oct-15 by Florian Maushart				     */
/*****************************************************************************/


#include <stdio.h>
#include <math.h>
#include <string.h>
#include <time.h>

//Here we add the libraries (robot, motor, differential_wheels, distance_sensor, emitter, receiver)
#include <ircom/e_ad_conv.h>
#include <epfl/e_init_port.h>
#include <epfl/e_epuck_ports.h>
#include <epfl/e_uart_char.h>
#include <epfl/e_led.h>

#include <epfl/e_led.h>
#include <epfl/e_motors.h>
#include <epfl/e_agenda.h>

#include <ircom/ircom.h>
#include <btcom/btcom.h> //libraries from provided test code. It should be enough 


#include <stdlib.h>

#define DEBUG             1       // flag to indicate if epuck 136 is in debugging mode
#define DEBUG_ROBOT_ID    136     // id of the robot that is connected to minicom for debugging

#define NB_SENSORS	  8	  // Number of distance sensors
#define MIN_SENS          200 // 350     // Minimum sensibility value
#define MAX_SENS          4096    // Maximum sensibility value
#define MAX_SPEED         150     // Maximum speed
#define FLOCK_SIZE	  5	  // Size of flock
//#define TIME_STEP	  64	  // [ms] Length of time step
#define M_PI       3.14159265359

#define AXLE_LENGTH 		0.052	// Distance between wheels of robot (meters)
#define SPEED_UNIT_RADS		0.00628	// Conversion factor from speed unit to radian per second
#define WHEEL_RADIUS		0.0205	// Wheel radius (meters)


#define RULE1_THRESHOLD     0.05 // Threshold to activate aggregation rule. default 0.20
#define RULE1_WEIGHT        (1.9/10)	   // Weight of aggregation rule. default 0.6/10

#define RULE2_THRESHOLD     0.05 // Threshold to activate dispersion rule. default 0.15
#define RULE2_WEIGHT        (0.01/10)	   // Weight of dispersion rule. default 0.02/10

#define RULE3_WEIGHT        (0.02/10)   // Weight of consistency rule. default 1.0/10
#define MIGRATION_WEIGHT    (0.05/10)   // Wheight of attraction towards the common goal. default 0.01/10

#define BRT_WEIGHT           40
#define REYNOLDS	1
#define MIGRATORY_URGE	1 // Tells the robots if they should just go forward or move towards a specific migratory direction
#define BRAITENBERG	1 // Tells the robots to avoid obstacles with Braitenberg

#define ABS(x) ((x>=0)?(x):-(x))

#define TRIBE_A 1  //from 0 to 4
#define TRIBE_B 2  // from 5 to 9

int e_puck_ids[] = {127, 108, 136, 60, 54, 112};
int e_puck_matrix[16] = {17,29,34,10,8,-38,-56,-76,-72,-58,-36,8,10,36,28,18}; // Maze
//			float sensorDir[NB_SENSORS] = {0.2967, 0.8727, 1.5708, 2.6180, 3.6652, 4.7124, 5.4105, 5.9865}; probably we gonna reuse this

int didReceive[FLOCK_SIZE];

int robot_id;	// Unique robot ID
int real_robot_id;

int receivedBR[FLOCK_SIZE];             // indicates if a signal was received from the associated robot
float relative_pos[FLOCK_SIZE][3];	// relative X, Z, Theta of all robots of our tribe
// float prev_relative_pos[FLOCK_SIZE][3];	// Previous relative  X, Z, Theta values of our tribe
// long int relative_pos_time[FLOCK_SIZE];	// relative X, Z, Theta of all robots of our tribe
// long int prev_relative_pos_time[FLOCK_SIZE];	// Previous relative  X, Z, Theta values of our tribe
float my_position[3];     		// X, Z, Theta of the current robot
float prev_my_position[3];  		// X, Z, Theta of the current robot in the previous time step
// long int my_position_time;
// long int prev_my_position_time;
// float speed[FLOCK_SIZE][2];		// Speeds calculated with Reynold's rules of our tribe
// float relative_speed[FLOCK_SIZE][2];	// Speeds calculated with Reynold's rules of our tribe
//int initialized[FLOCK_SIZE];		// != 0 if initial positions have been received of our tribe
float my_speed[2];
float migr[2] = {0.0,10.0};	        // Migration vector

double time_step;

//float theta_robots[FLOCK_SIZE];

int my_tribe;

// variables from main
int msl, msr;			// Wheel speeds

int bmsl, bmsr, sum_sensors;	// Braitenberg parameters
int distances[NB_SENSORS];	// Array for the distance sensor readings
int max_sens;			// Store highest sensor value

// obstacle avoidance
float sensorDir[NB_IR_SENSORS] = {0.2967, 0.8727, 1.5708, 2.6180, 3.6652, 4.7124, 5.4105, 5.9865};

// Weights for the Braitenberg obstacle avoidance algorithm
int weightleft[8] = {-10, -10, -5, 0, 0, 5, 10, 10};
int weightright[8] = {10, 10, 5, 0, 0, -5, -10, -10};

int getselector()
{
    return SELECTOR0 + 2*SELECTOR1 + 4*SELECTOR2 + 8*SELECTOR3;
}

int getrealrobotid()
{
    return e_puck_ids[getselector()];
}

int getrobotidfromreal(int realrobotid) {
   int i=0;
   while(1==1) {
      if (realrobotid==e_puck_ids[i]) {
         return i;
      }
      i++;
   }
}

/*
 * Reset the robot's devices and get its ID
 */

void reset() 
{
	// Initialization functions from the provided 
	e_init_port();
	e_init_ad_scan();
	e_init_uart1();
	e_led_clear();	
	e_init_motors();
	e_start_agendas_processing();

	int selector_value = getselector();
	if(selector_value>=0 && selector_value<=2*FLOCK_SIZE-1)
	{
		robot_id = selector_value;
                real_robot_id = getrealrobotid();
	}	
	else
	{
		btcomSendString("ERROR: SELECTOR NOT BETWEEN 0 AND 2*FLOCK_SIZE-1!!!\n\n");
	}
//        if ((DEBUG==1)&&(real_robot_id==DEBUG_ROBOT_ID))
        {
//		btcomWaitForCommand('s');
		btcomSendString("==== READY - COMMUNICATION ESTABLISHED ====\n\n");
        }
	
	if(robot_id < FLOCK_SIZE)	//NEED TO CHANGE THIS A LITTLE maybe
               my_tribe = TRIBE_A;
           else
               my_tribe = TRIBE_B;
  
/*	for(i=0; i<FLOCK_SIZE; i++) 
	{
		initialized[i] = 0;		  // Set initialization to 0 (= not yet initialized)
	}
*/	
        e_calibrate_ir(); 

	ircomStart();
	ircomEnableContinuousListening();
	ircomListen();

	// Reset if Power on (some problem for few robots)
	if (RCONbits.POR) {
		RCONbits.POR = 0;
		__asm__ volatile ("reset");
	}

        int i;
        for (i=0;i<FLOCK_SIZE;i++) {
            receivedBR[i] = 0;
        }
}

//WORKS
void limit_duo_proportional(int *number_1, int *number_2, int limit) {

        float number_1_abs = (float) ABS(*number_1);
        float number_2_abs = (float) ABS(*number_2);
        float limited_1 = 0;
        float limited_2 = 0;

        if(number_1_abs > (float)limit || number_2_abs > (float)limit)
        {
                  if(number_1_abs > number_2_abs)
                  {
                        limited_1 = ((float)*number_1)*limit/number_1_abs;
                        limited_2 = ((float)*number_2)*limit/number_1_abs;
                  }
                  else
                  {
                        limited_1 = ((float)*number_1)*limit/number_2_abs;
                        limited_2 = ((float)*number_2)*limit/number_2_abs;
                  }
                  *number_1 = limited_1;
                  *number_2 = limited_2;
        }        
}
/*
void compute_time_step() {
	time_step = (double)ircomGetTime()*81.3/1000000.0;
	ircomResetTime();
}
*/


//WORKS
void compute_my_speed() {
	double speed_left = (double)msl*2*M_PI*WHEEL_RADIUS/1000;
	double speed_right = (double)msr*2*M_PI*WHEEL_RADIUS/1000;
	double speed_center = (speed_left + speed_right)/2;
	my_speed[0] = speed_center*sin(my_position[2]); 
	my_speed[1] = speed_center*cos(my_position[2]);
}


/*
 * Updates robot position with wheel speeds
 */
//WORKS!
void update_self_motion(int msl, int msr) { 
	float theta = my_position[2];
	// Compute deltas of the robot
//	float dr = (float)msr * SPEED_UNIT_RADS * WHEEL_RADIUS * TIME_STEP/1000;
//	float dl = (float)msl * SPEED_UNIT_RADS * WHEEL_RADIUS * TIME_STEP/1000;
	long int steps_left = e_get_steps_left();
	long int steps_right = e_get_steps_right();
	float dr = (float)steps_right*WHEEL_RADIUS*2*M_PI/1000;
	float dl = (float)steps_left*WHEEL_RADIUS*2*M_PI/1000;

	float du = (dr + dl)/2.0;
	float dtheta = (dl - dr)/AXLE_LENGTH;
  
	// Compute deltas in the environment
	float dx = du * sinf(theta);
	float dz = du * cosf(theta);
  
	// Update position
	my_position[0] += dx;
	my_position[1] += dz;
	my_position[2] += dtheta;
  
	// Keep orientation within 0, 2pi
	if (my_position[2] > 2*M_PI) my_position[2] -= 2.0*M_PI;
	if (my_position[2] < 0) my_position[2] += 2.0*M_PI;

	e_set_steps_left(0);		// set motor steps counter // modified by bahr
	e_set_steps_right(0);		// set motor steps counter // modified by bahr
}

/*
 * Computes wheel speed given a certain X,Z speed
 */
void compute_wheel_speeds(int *msl, int *msr) 
{
	// Compute wanted position from Reynold's speed and current location
	//float x = speed[robot_id][0]*cosf(loc[robot_id][2]) - speed[robot_id][1]*sinf(loc[robot_id][2]); // x in robot coordinates
	//float z = -speed[robot_id][0]*sinf(loc[robot_id][2]) - speed[robot_id][1]*cosf(loc[robot_id][2]); // z in robot coordinates
	
	float x = my_speed[0]*cosf(my_position[2]) - my_speed[1]*sinf(my_position[2]); // x in robot coordinates
	float z = my_speed[0]*sinf(my_position[2]) + my_speed[1]*cosf(my_position[2]); // z in robot coordinates  //WWWarning recheck sign here

	float Ku = 0.2;   // Forward control coefficient 0.2
	float Kw = 1;  // Rotational control coefficient 1
	float range = sqrtf(x*x + z*z);	  // Distance to the wanted position
	float bearing = atan2(x, z);	  // Orientation of the wanted position
	
	// Compute forward control
	float u = Ku*range*cosf(bearing);
	// Compute rotational control
	float w = Kw*bearing;
	
	// Convert to wheel speeds!
	
	*msl = (u + AXLE_LENGTH*w/2.0) * (1000.0 / WHEEL_RADIUS);
	*msr = (u - AXLE_LENGTH*w/2.0) * (1000.0 / WHEEL_RADIUS);
}




/*
 *  Update speed according to Reynold's rules
 */

void reynolds_rules() {
	int i, j, k;			// Loop counters
	float rel_avg_loc[2] = {0,0};	// Flock average positions
	// float rel_avg_speed[2] = {0,0};	// Flock average speeds
	float cohesion[2] = {0,0};
	float dispersion[2] = {0,0};
	// float consistency[2] = {0,0};
	
	/* Compute averages over the whole flock */
        int nbRobots = 0;
	for(i=0;i<FLOCK_SIZE;i++){
            if (i==robot_id) // ||(receivedBR[i]<=0))
          	continue; // dont consider yourself or robots from which we haven't hear recently for average
            for (j=0;j<2;j++) {
              // rel_avg_speed[j] += relative_speed[i][j] ;
              rel_avg_loc[j]   += relative_pos[i][j];
              nbRobots++;
            }
  	}
        if (nbRobots>0) {
		for (j=0;j<2;j++) {
		   // rel_avg_speed[j] /= nbRobots ;
		   rel_avg_loc[j]   /= nbRobots;
		}
	 
		/* Rule 1 - Aggregation/Cohesion: move towards the center of mass */
	    
		for (j=0;j<2;j++) 
		{	
		    cohesion[j] = rel_avg_loc[j];
		}

		/* Rule 2 - Dispersion/Separation: keep far enough from flockmates */
		for(k=0;k<FLOCK_SIZE;k++){
		   if(k!=robot_id){ //loop on flockmates only
		     //if neighbor k is too close (Euclidean distance )
		         if (pow(relative_pos[k][0],2)+pow(relative_pos[k][1],2)<RULE2_THRESHOLD){
		             for (j=0;j<2;j++) {
		  	   dispersion[j] -= 1/(1+relative_pos[k][j]); //relative distance to k 
		      	   }
		         }
		     }
		}
		
		/* Rule 3 - Consistency/Alignment: match the speeds of flockmates */
		/*for (j=0;j<2;j++) {
			consistency[j] = rel_avg_speed[j];
		 }*/

		if (REYNOLDS==1) {
			 //aggregation of all behaviors with relative influence determined by weights
			 for (j=0;j<2;j++) 
			{
				 my_speed[j] = cohesion[j] * RULE1_WEIGHT;
				 my_speed[j] +=  dispersion[j] * RULE2_WEIGHT;
				 // speed[robot_id][j] +=  consistency[j] * RULE3_WEIGHT;
			 }
			 // speed[robot_id][1] *= -1; //y axis of webots is inverted
		} else {
			 for (j=0;j<2;j++) 
			{
				 my_speed[j] = 0;
			 }
		}
        } else {
		 for (j=0;j<2;j++) 
		{
		         my_speed[j] = 0;
		 }
	}
        
        
        //move the robot according to some migration rule
        if(MIGRATORY_URGE == 0){
          //speed[robot_id][0] += 0.01*cos(my_position[2] + M_PI/2);
          //speed[robot_id][1] += 0.01*sin(my_position[2] + M_PI/2);
        }
        else {
		my_speed[0] += (migr[0]-my_position[0]) * MIGRATION_WEIGHT;
 		my_speed[1] += (migr[1]-my_position[1]) * MIGRATION_WEIGHT;
        }
}

/*
 *  each robot sends a ping message, so the other robots can measure relative range and bearing to the sender.
 *  the message contains the robot's name
 *  the range and bearing will be measured directly out of message RSSI and direction
*/
void send_ping(void)  
{
        long int j;
	int i;
        for (i=0;i<2;++i) {
        	for(j = 0; j < 50000; j++)	asm("nop");
		ircomSend(real_robot_id);	    
		while (ircomSendDone() == 0);
	}
}

void resetDidReceive() {
	int i;
	for (i=0;i<FLOCK_SIZE;i++) {
		didReceive[i] = 0;
	}
}

long int getCurrentTime() {
    	time_t rawtime;

    	time ( &rawtime );
	return (long int) rawtime;
}

/*
 * processing all the received ping messages, and calculate range and bearing to the other robots
 * the range and bearing are measured directly out of message RSSI and direction
*/
void process_received_ping_messages(void)
{
	// btcomSendString("start receive com\n");
	int i = 0;
        long int j;
        for(j = 0; j < 10000; j++)	asm("nop");
	while (i < 2)
	{
		IrcomMessage imsg;
		ircomPopMessage(&imsg);
		if (imsg.error == 0)
		{
			e_set_led(1, 2);
			char tmp[256];
			// sprintf(tmp, "Received from %d\n", (int) imsg.value);
			// btcomSendString(tmp);
			int other_robot_id = getrobotidfromreal((int) imsg.value);
			if(((other_robot_id<FLOCK_SIZE) && my_tribe==TRIBE_A) || ((other_robot_id>=FLOCK_SIZE) && my_tribe==TRIBE_B))
			{
				//if (didReceive[other_robot_id%FLOCK_SIZE]==0) {
					didReceive[other_robot_id%FLOCK_SIZE] = 1;
                                	receivedBR[other_robot_id%FLOCK_SIZE] = 10; // count of 10 - after 10 loops where we didn't hear of the robot, remove it from reynolds
					double range = imsg.distance/100.;
					double theta = imsg.direction;
					theta = (my_position[2] - theta);

					// prev_relative_pos[other_robot_id%FLOCK_SIZE][0] = relative_pos[other_robot_id%FLOCK_SIZE][0];
					// prev_relative_pos[other_robot_id%FLOCK_SIZE][1] = relative_pos[other_robot_id%FLOCK_SIZE][1];
					// prev_relative_pos_time[other_robot_id%FLOCK_SIZE] = relative_pos_time[other_robot_id%FLOCK_SIZE];

					relative_pos[other_robot_id%FLOCK_SIZE][0] = range*sin(theta);  // relative x pos
					relative_pos[other_robot_id%FLOCK_SIZE][1] = 1.0 * range*cos(theta);   // relative y pos
					// relative_pos_time[other_robot_id%FLOCK_SIZE] = getCurrentTime();
				
			    		// relative_speed[other_robot_id%FLOCK_SIZE][0] = relative_speed[other_robot_id%FLOCK_SIZE][0]*0.0 + 1.0*(1000/time_step)*(relative_pos[other_robot_id%FLOCK_SIZE][0]-prev_relative_pos[other_robot_id%FLOCK_SIZE][0]);
			    		// relative_speed[other_robot_id%FLOCK_SIZE][1] = relative_speed[other_robot_id%FLOCK_SIZE][1]*0.0 + 1.0*(1000/time_step)*(relative_pos[other_robot_id%FLOCK_SIZE][1]-prev_relative_pos[other_robot_id%FLOCK_SIZE][1]);		

					/* Send Value*/
					sprintf(tmp, "Receive: %d (%d) - x=%f \t z=%f \t theta=%f\n", other_robot_id, e_puck_ids[other_robot_id], (double) relative_pos[other_robot_id%FLOCK_SIZE][0], (double) relative_pos[other_robot_id%FLOCK_SIZE][1],(double) theta);
//					sprintf(tmp, "Receive: %d (%d) - dist=%f \t dir=%f \n", other_robot_id, e_puck_ids[other_robot_id], (double)imsg.distance, (double)imsg.direction);
					btcomSendString(tmp);
				//}
			}
		}
		else if (imsg.error > 0)
		{
			//btcomSendString("Receive failed \n");		
		}
		else if (imsg.error == -1)
		{
                        // -> no message available in the queue
			i++;
                }
	}
        for (i=0;i<FLOCK_SIZE;i++) {
           receivedBR[i] -= 1;
        }

}

void braitenberg()
{
	int n;
	int i;
	double sensorMean[8];
	double sensors[NB_SENSORS];	// Array for the distance sensor readings
	int numberOfSamples=1;
	// check if an obstacle is perceived 
	sum_sensors=0;
	bmsl=0;
	bmsr=0;	// Braitenberg parameters
	// Braitenberg parameters

		for (i=0;i<8;i++)
			sensorMean[i]=0;	
		//Compute an average value of each sensor on multiple samples to reduce noise	
		for (n=0;n<numberOfSamples;n++)
		{	
			// Get sensor values
			for (i = 0; i < 8; i++) {
				// Use the sensorzero[i] value generated in sensor_calibrate() to zero sensorvalues
				sensors[i] = e_get_prox(i);
				//linearize the sensor output and compute the average
				sensorMean[i]+=sensors[i]; //12.1514*log((double)sensors[i])/(double)numberOfSamples;
			}
		}
		for (i = 0; i < 8; i++) sensorMean[i] /= numberOfSamples;

		// Add the weighted sensors values
		for (i = 0; i < 8; i++) {
			bmsr += e_puck_matrix[i] * (int)sensorMean[i];
			bmsl += e_puck_matrix[i+NB_SENSORS] * (int)sensorMean[i];
			sum_sensors += sensorMean[i]; // Add up sensor values
			max_sens = max_sens>sensorMean[i]?max_sens:sensorMean[i]; // Check if new highest sensor value
		}
		bmsl/=MIN_SENS; bmsr/=MIN_SENS;
		bmsl+=66; bmsr+=72;


		// Speed bounds, to avoid setting to high speeds to the motor
		limit_duo_proportional(&bmsl, &bmsr, 1000);
}

int obstacleAvoidanceThreshold = 30.0;
int obstacleAvoidanceSpeed = 500.0;
void obstacleAvoidance()
{    
    // check if an obstacle is perceived 
    double reading = 0.0;
    int obstaclePerceived = 0;
    int i=0;
    double x = 0.0, y = 0.0;
    for (i = 0; i < 8; i++)
    {
        reading = e_get_calibrated_prox(i);
	// if signal above noise
	if(reading >= obstacleAvoidanceThreshold)
	{
	    obstaclePerceived = 1;
	    
	    // compute direction to escape
	    double signal = reading - obstacleAvoidanceThreshold;
	    x += -cos(sensorDir[i]) * signal / 8.0;
	    y += sin(sensorDir[i]) * signal / 8.0;
	}
    }
    
    // no obstacles to avoid, return immediately
    if (obstaclePerceived == 0)
    {
	// go straight forward
	// change movement direction
	e_set_speed_left(obstacleAvoidanceSpeed);
	e_set_speed_right(obstacleAvoidanceSpeed);
	// return obstaclePerceived;
	return;
    }
    
    double desiredAngle = atan2 (y, x);
    
    double leftSpeed = 0.0;
    double rightSpeed = 0.0;
    
    // turn left
    if (desiredAngle >= 0.0)
    {
	leftSpeed  = cos(desiredAngle);
	rightSpeed = 1;
    }
    // turn right
    else
    {
	leftSpeed = 1;
	rightSpeed = cos(desiredAngle);
    }
    
    // rescale values
    leftSpeed *= obstacleAvoidanceSpeed;
    rightSpeed *= obstacleAvoidanceSpeed;
    
    // change movement direction
    // e_set_speed_left(leftSpeed);
    // e_set_speed_right(rightSpeed);
    
    // advertise obstacle avoidance in progress
    // return 1;

    bmsl = leftSpeed;
    bmsr = rightSpeed;
}

  
// the main function
int main(){ 
	int step;

 	reset();			// Resetting the robot

//	e_set_led(1, 2);
        char tmp[128];
        sprintf(tmp, "%d : Reset done\n\n",real_robot_id);
	btcomSendString(tmp);

	// Forever
	for(;;){

        	msl = 0;
		msr = 0; 
	
		bmsl = 0;
                bmsr = 0;
                sum_sensors = 0;
		max_sens = 0;
                
                braitenberg();
		// obstacleAvoidance();

		e_set_led(1, 0);
		resetDidReceive();
		int i;
		for (i=0;i<2;i++) {
			/* Send and get information */
			send_ping();

			process_received_ping_messages();
		}

		/// Compute self position
		prev_my_position[0] = my_position[0];
		prev_my_position[1] = my_position[1];

		// update relative position of robots from which we havent heard according to the distance we have covered
		/*for (i=0;i<FLOCK_SIZE;i++) {
			if (didReceive[i]==0) {
				relative_pos[i][0] -= prev_my_position[0]-my_position[0];  // relative x pos
				relative_pos[i][1] -= prev_my_position[1]-my_position[1];   // relative y pos
			}
		}*/
		
		update_self_motion(msl,msr);
		
		if(step%30 == 0)
		{
			sprintf(tmp, "my_position: x:%f z:%f theta:%f \n", (double) my_position[0], (double) my_position[1], (double) my_position[2]);
			btcomSendString(tmp);
		}

		// compute_time_step();

		// speed[robot_id][0] = (1000/time_step)*(my_position[0]-prev_my_position[0]);
		// speed[robot_id][1] = (1000/time_step)*(my_position[1]-prev_my_position[1]);
    
		compute_my_speed();

		// Reynold's rules with all previous info (updates the speed[][] table)
		reynolds_rules();
    
		// Compute wheels speed from reynold's speed
		compute_wheel_speeds(&msl, &msr);
    
		// Adapt speed instinct to distance sensor values
		if ((BRAITENBERG==1)&&(sum_sensors > NB_SENSORS*MIN_SENS)) {
			msl -= msl*max_sens/(2*MAX_SENS);
			msr -= msr*max_sens/(2*MAX_SENS);
		}

    
		if(step%30 == 0)
		{
			sprintf(tmp, "max sens: %d \t speed x %d \t speed z %d\n", max_sens, (double) my_speed[0], (double) my_speed[1]);
			btcomSendString(tmp);
		}

		// Add Braitenberg
		
		if (BRAITENBERG==1) {
			msl += BRT_WEIGHT*bmsl;
			msr += BRT_WEIGHT*bmsr;
		}
		
                  
         	limit_duo_proportional(&msl, &msr, MAX_SPEED);
                 

		e_set_speed_left(msl);
		e_set_speed_right(msr);

		//potential problem: odometry will work terribly bad, so we could use the motors get step and set steps functions, and not using any TIME_STEP at all.
 
		//WARNING: RECHECK IF THE REAL TIME STEP IS THE ONE WE THINK; and if we're not spending too much time computing, sending, receiving, etc (besides the waiting step)
	}
        ircomStop();

	return 0;
}


