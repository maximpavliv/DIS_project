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
#define FLOCK_SIZE	  5	  // Size of flock
#define TIME_STEP	  64	  // [ms] Length of time step

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
char* robot_name;

//float theta_robots[FLOCK_SIZE];

int my_tribe;

/*
 * Reset the robot's devices and get its ID
 */
//----------------stopped here max jeu 17:39
static void reset() 
{
	//			wb_robot_init();


	//			receiver = wb_robot_get_device("receiver");
	//			emitter = wb_robot_get_device("emitter");
	
	/*Webots 2018b*/
	//get motors
	//			left_motor = wb_robot_get_device("left wheel motor");
         //			right_motor = wb_robot_get_device("right wheel motor");
         //			wb_motor_set_position(left_motor, INFINITY);
         //			wb_motor_set_position(right_motor, INFINITY);
         /*Webots 2018b*/
	
	
	int i;
	char s[4]="ps0";
	/*
				for(i=0; i<NB_SENSORS;i++) {
					ds[i]=wb_robot_get_device(s);	// the device name is specified in the world file
					s[2]++;				// increases the device number
				}
	*/
	//			robot_name=(char*) wb_robot_get_name(); 

	/*
				for(i=0;i<NB_SENSORS;i++)
				  wb_distance_sensor_enable(ds[i],64);
	*/
	//			wb_receiver_enable(receiver,64);

	//Reading the robot's name. Pay attention to name specification when adding robots to the simulation!
	//			sscanf(robot_name,"epuck%d",&robot_id); // read robot id from the robot's name
	if(robot_id < 5)	//NEED TO CHANGE THIS A LITTLE maybe
               my_tribe = TRIBE_A;
           else
               my_tribe = TRIBE_B;
               
  
/*	for(i=0; i<FLOCK_SIZE; i++) 
	{
		initialized[i] = 0;		  // Set initialization to 0 (= not yet initialized)
	}
*/	


        //			printf("Reset: robot %d, tribe %d\n",robot_id, my_tribe);


}
/*
 * Keep given float number within interval {-limit, limit}
 */
/*
			void limitf(float *number, double limit) {
				if (*number > limit)
					*number = limit;
				if (*number < -limit)
					*number = -limit;
			}
*/
/*
 * Keep given int number within interval {-limit, limit}
 */
/*
			void limit(int *number, int limit) {
				if (*number > limit)
					*number = limit;
				if (*number < -limit)
					*number = -limit;
			}
*/

/*
			double gaussianNoise(double mu, double sigma)
			{
			  double U1, U2, W, mult;
			  static double X1, X2;
			  static int call = 0;

			  if (call == 1)
			    {
			      call = !call;
			      return (mu + sigma * (double) X2);
			    }

			  do
			    {
			      U1 = -1 + ((double) rand () / RAND_MAX) * 2;
			      U2 = -1 + ((double) rand () / RAND_MAX) * 2;
			      W = pow (U1, 2) + pow (U2, 2);
			    }
			  while (W >= 1 || W == 0);

			  mult = sqrt ((-2 * log (W)) / W);
			  X1 = U1 * mult;
			  X2 = U2 * mult;

			  call = !call;

			  return (mu + sigma * (double) X1);
			}
*/

/*
			float quanticised_theta(float theta) {
			  float quanticised = 0;
			  if(theta < (sensorDir[0]+sensorDir[1])/2)
			    quanticised = sensorDir[0];
			  else if(theta > (sensorDir[NB_SENSORS-2]+sensorDir[NB_SENSORS-1])/2)
			    quanticised = sensorDir[NB_SENSORS-1];
			  else
			  {
			    for(int i = 0; i<NB_SENSORS-2; i++)
			    {
			      if((theta>(sensorDir[i]+sensorDir[i+1])/2) && (theta<(sensorDir[i+1]+sensorDir[i+2])/2))
				quanticised = sensorDir[i+1];
			    }
			  }
			    return quanticised;
			}
*/

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
	//printf("id = %d, x = %f, y = %f\n", robot_id, x, z);
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
	//printf("bearing = %f, u = %f, w = %f, msl = %d, msr = %d\n", bearing, u, w, msl, msr);
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
        //			speed[robot_id][1] *= -1; //y axis of webots is inverted		 WHAT DO WE DO WITH THIS?
        
        
        
        //move the robot according to some migration rule
        if(MIGRATORY_URGE == 0){
          speed[robot_id][0] += 0.01*cos(my_position[2] + M_PI/2);
          speed[robot_id][1] += 0.01*sin(my_position[2] + M_PI/2);
        }
        else {
              speed[robot_id][0] += (migr[0]-my_position[0]) * MIGRATION_WEIGHT;
              //			speed[robot_id][1] -= (migr[1]-my_position[1]) * MIGRATION_WEIGHT; //y axis of webots is inverted 		WHAT DO WE DO WITH THIS?
        }
}

/*
 *  each robot sends a ping message, so the other robots can measure relative range and bearing to the sender.
 *  the message contains the robot's name
 *  the range and bearing will be measured directly out of message RSSI and direction
*/
void send_ping(void)  
{
        char out[10];
	//			strcpy(out,robot_name);  // in the ping message we send the name of the robot.
	//			wb_emitter_send(emitter,out,strlen(out)+1); 
}

/*
 * processing all the received ping messages, and calculate range and bearing to the other robots
 * the range and bearing are measured directly out of message RSSI and direction
*/
void process_received_ping_messages(void)
{
        const double *message_direction;
        double message_rssi; // Received Signal Strength indicator
	double theta;
	double range;
	char *inbuffer;	// Buffer for the receiver node
        int other_robot_id;
	/*
	while (wb_receiver_get_queue_length(receiver) > 0) {
		inbuffer = (char*) wb_receiver_get_data(receiver);
		message_direction = wb_receiver_get_emitter_direction(receiver);
		message_rssi = wb_receiver_get_signal_strength(receiver);
		double y = message_direction[2] + gaussianNoise(0,0);
		double x = message_direction[0] + gaussianNoise(0,0);

		other_robot_id = (int)(inbuffer[5]-'0');  // since the name of the sender is in the received message. Note: this does not work for robots having id bigger than 9!
                      
                      if(((other_robot_id<5) && my_tribe==TRIBE_A) || ((other_robot_id>=5) && my_tribe==TRIBE_B))
                      {
                          theta = -atan2(y,x);
                          if(theta < 0)
                            theta += 2*3.1416;
                          //Here we quanticise the direction, like its is done with the real epucks
                          theta = quanticised_theta(theta); //We can comment out this line to skip quantisation
                          theta = theta + my_position[2]; // find the relative theta;
            	    range = sqrt((1/message_rssi));
    		
            	    //this is in meters.
            	    //We add some noise to the distance measurement, respecting with the std we measured with real epucks, for a maximal distance between them (around 20cm)
                          //The angle values are discrete, so no std is measured on them. Instead, we quanticise them (higher in the code)
                          range += gaussianNoise(0, STD_NOISE_DISTANCE);//We can comment out this line if we dont want any noise
    		
            	    // Get position update 
    		    //theta += dtheta_g[other_robot_id];
    		    //theta_robots[other_robot_id] = 0.8*theta_robots[other_robot_id] + 0.2*theta;
    		    prev_relative_pos[other_robot_id%FLOCK_SIZE][0] = relative_pos[other_robot_id%FLOCK_SIZE][0];
    		    prev_relative_pos[other_robot_id%FLOCK_SIZE][1] = relative_pos[other_robot_id%FLOCK_SIZE][1];
    
    		    relative_pos[other_robot_id%FLOCK_SIZE][0] = range*cos(theta);  // relative x pos
    		    relative_pos[other_robot_id%FLOCK_SIZE][1] = -1.0 * range*sin(theta);   // relative y pos
    
    		    printf("Robot %s, from robot %d, x: %g, y: %g, theta %g, my theta %g\n",robot_name,other_robot_id,relative_pos[other_robot_id][0],relative_pos[other_robot_id][1],-atan2(y,x),my_position[2]*180.0/3.141592);
    		
        		    relative_speed[other_robot_id%FLOCK_SIZE][0] = relative_speed[other_robot_id%FLOCK_SIZE][0]*0.0 + 1.0*(1000/TIME_STEP)*(relative_pos[other_robot_id%FLOCK_SIZE][0]-prev_relative_pos[other_robot_id%FLOCK_SIZE][0]);
        		    relative_speed[other_robot_id%FLOCK_SIZE][1] = relative_speed[other_robot_id%FLOCK_SIZE][1]*0.0 + 1.0*(1000/TIME_STEP)*(relative_pos[other_robot_id%FLOCK_SIZE][1]-prev_relative_pos[other_robot_id%FLOCK_SIZE][1]);		
    		 }
		 
		wb_receiver_next_packet(receiver);
	}
	*/
}


// the main function
int main(){ 
	int msl, msr;			// Wheel speeds
	/*Webots 2018b*/
	float msl_w, msr_w;
	/*Webots 2018b*/
	int bmsl, bmsr, sum_sensors;	// Braitenberg parameters
	int i;				// Loop counter
	int distances[NB_SENSORS];	// Array for the distance sensor readings
	int max_sens;			// Store highest sensor value
	
 	reset();			// Resetting the robot

	msl = 0; msr = 0; 
	max_sens = 0; 
	
	// Forever
	for(int step=0;step<2000;step++){ //		RECHECK IF STIL THE SAME
           // printf("step %d /n",step);

		bmsl = 0; bmsr = 0;
                sum_sensors = 0;
		max_sens = 0;
                
		/* Braitenberg */
		for(i=0;i<NB_SENSORS;i++) 
		{
			 //			distances[i]=wb_distance_sensor_get_value(ds[i]); //Read sensor values
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

		/// Compute self position
		prev_my_position[0] = my_position[0];
		prev_my_position[1] = my_position[1];
		
		update_self_motion(msl,msr);
//		printf("robot %d position: %f %f %f \n", robot_id, my_position[0], my_position[1], my_position[2]);
		
		process_received_ping_messages();

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
                  
		/*Webots 2018b*/
		// Set speed
		//			msl_w = msl*MAX_SPEED_WEB/1000;
      	        //			msr_w = msr*MAX_SPEED_WEB/1000;
                      
		//			wb_motor_set_velocity(left_motor, msl_w);
                //			wb_motor_set_velocity(right_motor, msr_w);
		//wb_differential_wheels_set_speed(msl,msr);
		/*Webots 2018b*/
    
		// Continue one step
		//			wb_robot_step(TIME_STEP);
	}
}  
  
