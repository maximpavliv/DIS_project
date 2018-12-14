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

#define NB_SENSORS	  8	  // Number of distance sensors
#define MIN_SENS          350     // Minimum sensibility value
#define MAX_SENS          4096    // Maximum sensibility value
#define MAX_SPEED         1000     // Maximum speed
#define FLOCK_SIZE	  2	  // Size of flock
#define TIME_STEP	  64	  // [ms] Length of time step
#define M_PI       3.14159265359

#define AXLE_LENGTH 		0.052	// Distance between wheels of robot (meters)
#define SPEED_UNIT_RADS		0.00628	// Conversion factor from speed unit to radian per second
#define WHEEL_RADIUS		0.0205	// Wheel radius (meters)


#define RULE1_THRESHOLD     0.05 // Threshold to activate aggregation rule. default 0.20
#define RULE1_WEIGHT        (1.9/10)	   // Weight of aggregation rule. default 0.6/10

#define RULE2_THRESHOLD     0.05 // Threshold to activate dispersion rule. default 0.15
#define RULE2_WEIGHT        (0.02/10)	   // Weight of dispersion rule. default 0.02/10

#define RULE3_WEIGHT        (0.02/10)   // Weight of consistency rule. default 1.0/10
#define MIGRATION_WEIGHT    (0.05/10)   // Wheight of attraction towards the common goal. default 0.01/10

#define BRT_WEIGHT           10
#define MIGRATORY_URGE 1 // Tells the robots if they should just go forward or move towards a specific migratory direction

#define ABS(x) ((x>=0)?(x):-(x))

#define TRIBE_A 1  //from 0 to 4
#define TRIBE_B 2  // from 5 to 9

int e_puck_matrix[16] = {17,29,34,10,8,-38,-56,-76,-72,-58,-36,8,10,36,28,18}; // Maze
//			float sensorDir[NB_SENSORS] = {0.2967, 0.8727, 1.5708, 2.6180, 3.6652, 4.7124, 5.4105, 5.9865}; probably we gonna reuse this


int robot_id;	// Unique robot ID

float relative_pos[FLOCK_SIZE][3];	// relative X, Z, Theta of all robots of our tribe
float prev_relative_pos[FLOCK_SIZE][3];	// Previous relative  X, Z, Theta values of our tribe
float my_position[3];     		// X, Z, Theta of the current robot
float prev_my_position[3];  		// X, Z, Theta of the current robot in the previous time step
float speed[FLOCK_SIZE][2];		// Speeds calculated with Reynold's rules of our tribe
float relative_speed[FLOCK_SIZE][2];	// Speeds calculated with Reynold's rules of our tribe
//int initialized[FLOCK_SIZE];		// != 0 if initial positions have been received of our tribe
float migr[2] = {0.0,-10.0};	        // Migration vector

//float theta_robots[FLOCK_SIZE];

int my_tribe;



int getselector()
{
    return SELECTOR0 + 2*SELECTOR1 + 4*SELECTOR2 + 8*SELECTOR3;
}

/*
 * Reset the robot's devices and get its ID
 */

static void reset() 
{
	// Initialization functions from the provided 
	e_init_port();
	e_init_ad_scan();
	e_init_uart1();
	e_led_clear();	
	e_init_motors();
	e_start_agendas_processing();

	//e_calibrate_ir(); //DONT KNOW IF GONNA NEED CALIBRATION OR NOT

	ircomStart();
	ircomEnableContinuousListening();
	ircomListen();

	// Reset if Power on (some problem for few robots)
	if (RCONbits.POR) {
		RCONbits.POR = 0;
		__asm__ volatile ("reset");
	}


	int selector_value = getselector();
	if(selector_value>=0 && selector_value<=9)
	{
		robot_id = selector_value;
	}	
	else
	{
		btcomSendString("ERROR: SELECTOR NOT BETWEEN 0 AND 9!!!\n\n");
	}	
	if(robot_id < 5)	//NEED TO CHANGE THIS A LITTLE maybe
               my_tribe = TRIBE_A;
           else
               my_tribe = TRIBE_B;
  
/*	for(i=0; i<FLOCK_SIZE; i++) 
	{
		initialized[i] = 0;		  // Set initialization to 0 (= not yet initialized)
	}
*/	
}

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
 * Updates robot position with wheel speeds
 */
void update_self_motion(int msl, int msr) { 
	float theta = my_position[2];
  
	// Compute deltas of the robot
	float dr = (float)msr * SPEED_UNIT_RADS * WHEEL_RADIUS * TIME_STEP/1000;
	float dl = (float)msl * SPEED_UNIT_RADS * WHEEL_RADIUS * TIME_STEP/1000;
	float du = (dr + dl)/2.0;
	float dtheta = (dr - dl)/AXLE_LENGTH;
  
	// Compute deltas in the environment
	float dx = -du * sinf(theta);
	float dz = -du * cosf(theta);
  
	// Update position
	my_position[0] += dx;
	my_position[1] += dz;
	my_position[2] += dtheta;
  
	// Keep orientation within 0, 2pi
	if (my_position[2] > 2*M_PI) my_position[2] -= 2.0*M_PI;
	if (my_position[2] < 0) my_position[2] += 2.0*M_PI;
	
}

/*
 * Computes wheel speed given a certain X,Z speed
 */
void compute_wheel_speeds(int *msl, int *msr) 
{
	// Compute wanted position from Reynold's speed and current location
	//float x = speed[robot_id][0]*cosf(loc[robot_id][2]) - speed[robot_id][1]*sinf(loc[robot_id][2]); // x in robot coordinates
	//float z = -speed[robot_id][0]*sinf(loc[robot_id][2]) - speed[robot_id][1]*cosf(loc[robot_id][2]); // z in robot coordinates
	
	float x = speed[robot_id][0]*cosf(my_position[2]) + speed[robot_id][1]*sinf(my_position[2]); // x in robot coordinates
	float z = -speed[robot_id][0]*sinf(my_position[2]) + speed[robot_id][1]*cosf(my_position[2]); // z in robot coordinates

	float Ku = 0.2;   // Forward control coefficient 0.2
	float Kw = 1;  // Rotational control coefficient 1
	float range = sqrtf(x*x + z*z);	  // Distance to the wanted position
	float bearing = -atan2(x, z);	  // Orientation of the wanted position
	
	// Compute forward control
	float u = Ku*range*cosf(bearing);
	// Compute rotational control
	float w = Kw*bearing;
	
	// Convert to wheel speeds!
	
	*msl = (u - AXLE_LENGTH*w/2.0) * (1000.0 / WHEEL_RADIUS);
	*msr = (u + AXLE_LENGTH*w/2.0) * (1000.0 / WHEEL_RADIUS);
}


/*
 *  Update speed according to Reynold's rules
 */

void reynolds_rules() {
	int i, j, k;			// Loop counters
	float rel_avg_loc[2] = {0,0};	// Flock average positions
	float rel_avg_speed[2] = {0,0};	// Flock average speeds
	float cohesion[2] = {0,0};
	float dispersion[2] = {0,0};
	float consistency[2] = {0,0};
	
	/* Compute averages over the whole flock */
	for(i=0;i<FLOCK_SIZE;i++){
            if (i==robot_id)
          	continue; // dont consider yourself for average
            for (j=0;j<2;j++) {
              rel_avg_speed[j] += relative_speed[i][j] ;
              rel_avg_loc[j]   += relative_pos[i][j];
             }
  	}
	for (j=0;j<2;j++) {
	rel_avg_speed[j] /= FLOCK_SIZE-1 ;
	rel_avg_loc[j]   /= FLOCK_SIZE-1;
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
	for (j=0;j<2;j++) {
		consistency[j] = rel_avg_speed[j];
         }

         //aggregation of all behaviors with relative influence determined by weights
         for (j=0;j<2;j++) 
	{
                 speed[robot_id][j] = cohesion[j] * RULE1_WEIGHT;
                 speed[robot_id][j] +=  dispersion[j] * RULE2_WEIGHT;
                 speed[robot_id][j] +=  consistency[j] * RULE3_WEIGHT;
         }
        //			speed[robot_id][1] *= -1; //y axis of webots is inverted	WHAT DO WE DO WITH THIS?	ok lets keep it non inverted for now
        
        
        
        //move the robot according to some migration rule
        if(MIGRATORY_URGE == 0){
          speed[robot_id][0] += 0.01*cos(my_position[2] + M_PI/2);
          speed[robot_id][1] += 0.01*sin(my_position[2] + M_PI/2);
        }
        else {
		speed[robot_id][0] += (migr[0]-my_position[0]) * MIGRATION_WEIGHT;
              //			speed[robot_id][1] -= (migr[1]-my_position[1]) * MIGRATION_WEIGHT; //y axis of webots is inverted	WHAT DO WE DO WITH THIS?	ok lets keep it non inverted for now (line below)
		speed[robot_id][1] += (migr[1]-my_position[1]) * MIGRATION_WEIGHT;
        }
}

/*
 *  each robot sends a ping message, so the other robots can measure relative range and bearing to the sender.
 *  the message contains the robot's name
 *  the range and bearing will be measured directly out of message RSSI and direction
*/
void send_ping(void)  
{
//      char out[10];
//	sprintf (out, "robotID%d", robot_id); //NOT SURE WE NEED A STRING
	ircomSend(robot_id);	    
	while (ircomSendDone() == 0);
}

/*
 * processing all the received ping messages, and calculate range and bearing to the other robots
 * the range and bearing are measured directly out of message RSSI and direction
*/
void process_received_ping_messages(void)
{
	
	//OK WHAT WE DO HERE IS: get the data, get the direction,the rssi, and the other robot's ID. Then, check if the other robot is in our tribe, if yes do the following: {compute the theta of the other robot, theta += my_position[2], compute the range, update prev_relative_pos. Compute the new relative_pos (WARNING: there is a minus sign, which we probably dont need to do. RECHECK!). Update the relative_speed.}. Finally, receive next packet


	// ircomData.messagesCount;		//messages dans la cue. 
	IrcomMessage imsg;
	ircomPopMessage(&imsg);
	if (imsg.error == 0)
	{
		//e_set_led(1, 2);
		int other_robot_id = (int) imsg.value;
		if(((other_robot_id<5) && my_tribe==TRIBE_A) || ((other_robot_id>=5) && my_tribe==TRIBE_B))
		{
			double range = imsg.distance;
			double theta = imsg.direction;
			theta += my_position[2];

			prev_relative_pos[other_robot_id%FLOCK_SIZE][0] = relative_pos[other_robot_id%FLOCK_SIZE][0];
			prev_relative_pos[other_robot_id%FLOCK_SIZE][1] = relative_pos[other_robot_id%FLOCK_SIZE][1];

			relative_pos[other_robot_id%FLOCK_SIZE][0] = range*cos(theta);  // relative x pos
			relative_pos[other_robot_id%FLOCK_SIZE][1] = 1.0 * range*sin(theta);   // relative y pos 	//WARNING: there was initially a minus sign here, I deleted it because I think it is there because of the webot's z axis inversion. But I'm not totally sure 
			
		    	relative_speed[other_robot_id%FLOCK_SIZE][0] = relative_speed[other_robot_id%FLOCK_SIZE][0]*0.0 + 1.0*(1000/TIME_STEP)*(relative_pos[other_robot_id%FLOCK_SIZE][0]-prev_relative_pos[other_robot_id%FLOCK_SIZE][0]);
		    	relative_speed[other_robot_id%FLOCK_SIZE][1] = relative_speed[other_robot_id%FLOCK_SIZE][1]*0.0 + 1.0*(1000/TIME_STEP)*(relative_pos[other_robot_id%FLOCK_SIZE][1]-prev_relative_pos[other_robot_id%FLOCK_SIZE][1]);		

			/* Send Value*/		
			//char tmp[128];
			//sprintf(tmp, "Receive successful : %d  - distance=%f \t direction=%f \n", val, (double)imsg.distance, (double)imsg.direction);
			//btcomSendString(tmp);
		}
	}
	else if (imsg.error > 0)
	{
		//btcomSendString("Receive failed \n");		
	}
	// else imsg.error == -1 -> no message available in the queue

}


// the main function
int main(){ 
	int msl, msr;			// Wheel speeds

	int bmsl, bmsr, sum_sensors;	// Braitenberg parameters
	int i;				// Loop counter
	int distances[NB_SENSORS];	// Array for the distance sensor readings
	int max_sens;			// Store highest sensor value
	int step;
	
 	reset();			// Resetting the robot

	msl = 0; msr = 0; 
	max_sens = 0; 
	
	// Forever
	for(step=0;step<2000;step++){ //		RECHECK IF STIL THE SAME	probably yes

		bmsl = 0; bmsr = 0;
                  sum_sensors = 0;
		max_sens = 0;
                
		/* Braitenberg */
		for(i=0;i<NB_SENSORS;i++) 
		{
			distances[i]=e_get_prox(i); //Read sensor values
			sum_sensors += distances[i]; // Add up sensor values
			max_sens = max_sens>distances[i]?max_sens:distances[i]; // Check if new highest sensor value

			// Weighted sum of distance sensor values for Braitenburg vehicle
			bmsr +=  e_puck_matrix[i] *distances[i];
			bmsl +=  e_puck_matrix[i+NB_SENSORS]*distances[i];
                 }

		 // Adapt Braitenberg values (empirical tests)
                 bmsl/=MIN_SENS; bmsr/=MIN_SENS;
                 bmsl+=66; bmsr+=72;
              
		/* Send and get information */
		send_ping();  // sending a ping to other robot, so they can measure their distance to this robot
		//WARNING: here we will maybe get some anormal functioning: how do we ensure that the sending and reading happens synchroniously? (that every robot receives every other robots messages? that when one is reading, others are receiving?) Should we maybe do a background function that sends the message contiously?

		/// Compute self position
		prev_my_position[0] = my_position[0];
		prev_my_position[1] = my_position[1];
		
		update_self_motion(msl,msr);
		
		process_received_ping_messages(); //WARNING: here we will maybe get some anormal functioning: how do we ensure that the sending and reading happens synchroniously? (that every robot receives every other robots messages? that when one is reading, others are receiving?) Should we maybe do a background function that sends the message contiously?

		speed[robot_id][0] = (1000/TIME_STEP)*(my_position[0]-prev_my_position[0]);
		speed[robot_id][1] = (1000/TIME_STEP)*(my_position[1]-prev_my_position[1]);
    
		// Reynold's rules with all previous info (updates the speed[][] table)
		reynolds_rules();
    
		// Compute wheels speed from reynold's speed
		compute_wheel_speeds(&msl, &msr);
    
		// Adapt speed instinct to distance sensor values
		if (sum_sensors > NB_SENSORS*MIN_SENS) {
			msl -= msl*max_sens/(2*MAX_SENS);
			msr -= msr*max_sens/(2*MAX_SENS);
		}
    
		// Add Braitenberg
		
		msl += BRT_WEIGHT*bmsl;
		msr += BRT_WEIGHT*bmsr;
		
                  
         	limit_duo_proportional(&msl, &msr, MAX_SPEED);
                 

		e_set_speed_left(msl);
		e_set_speed_right(msr);
		// DO A TIME STEP (if possible)
		//potential problem: odometry will work terribly bad, so we could use the motors get step and set steps functions, and not using any TIME_STEP at all.
 
		//WARNING: RECHECK IF THE REAL TIME STEP IS THE ONE WE THINK; and if we're not spending too much time computing, sending, receiving, etc (besides the waiting step)
	}
}  
  
