
/*
    Copyright 2007 Alexandre Campo, Alvaro Gutierrez, Valentin Longchamp.

    This file is part of libIrcom.

    libIrcom is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License.

    libIrcom is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with libIrcom.  If not, see <http://www.gnu.org/licenses/>.
*/

// simple test :  send or receive numbers, and avoid obstacles in the same time.

#include <ircom/e_ad_conv.h>
#include <epfl/e_init_port.h>
#include <epfl/e_epuck_ports.h>
#include <epfl/e_uart_char.h>
#include <epfl/e_led.h>

#include <epfl/e_led.h>
#include <epfl/e_motors.h>
#include <epfl/e_agenda.h>

#include <stdio.h>
#include <ircom/ircom.h>
#include <btcom/btcom.h>
#include <math.h>
#define FLOCK_SIZE	  2	  // Size of flock
#define TRIBE_A 1  //from 0 to 4
#define TRIBE_B 2  // from 5 to 9
#define NB_SENSORS	  8	  // Number of distance sensors
#define MIN_SENS          350     // Minimum sensibility value
#define MAX_SENS          4096    // Maximum sensibility value
#define MAX_SPEED         1000     // Maximum speed
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

#define BRT_WEIGHT         5
#define MIGRATORY_URGE 1 // Tells the robots if they should just go forward or move towards a specific migratory direction

#define ABS(x) ((x>=0)?(x):-(x))


float sensorDir[NB_IR_SENSORS] = {0.2967, 0.8727, 1.5708, 2.6180, 3.6652, 4.7124, 5.4105, 5.9865};
int e_puck_matrix[16] = {17,29,34,10,8,-38,-56,-76,-72,-58,-36,8,10,36,28,18}; // Maze
//int e_puck_matrix[16] = {17,29,34,10,8,-38,-56,-76,18,28,36,10,8,-36,-58,-72}; // Maze
int my_tribe;
int robot_id;	// Unique robot ID

float relative_pos[FLOCK_SIZE][3];	// relative X, Z, Theta of all robots of our tribe
float prev_relative_pos[FLOCK_SIZE][3];	// Previous relative  X, Z, Theta values of our tribe
float my_position[3];     		// X, Z, Theta of the current robot
float prev_my_position[3];  		// X, Z, Theta of the current robot in the previous time step
float speed[FLOCK_SIZE][2];		// Speeds calculated with Reynold's rules of our tribe
float relative_speed[FLOCK_SIZE][2];	// Speeds calculated with Reynold's rules of our tribe
float time_step;

int sum_sensors;	// Braitenberg parameters
int max_sens;			// Store highest sensor value

float migr[2] = {0.0,-10.0};	        // Migration vector




int getselector()
{
    return SELECTOR0 + 2*SELECTOR1 + 4*SELECTOR2 + 8*SELECTOR3;
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



void compute_time_step() {
	time_step = ircomGetTime()*81.3/1000000;
	ircomResetTime();
}

void update_self_motion() { 
	float theta = my_position[2];
      long int steps_left=e_get_steps_left();
      long int steps_right = e_get_steps_right();
	// Compute deltas of the robot
	float dr = (float)steps_right * WHEEL_RADIUS *2* M_PI/1000;
		float dl = (float)steps_left * WHEEL_RADIUS *2* M_PI/1000;
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
	
	e_set_steps_left(0);
	e_set_steps_right(0);
	
}

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


void obstacleAvoidance(int *bmsl,int *bmsr);
void reynolds_rules();
int main()
{

    // init robot
    e_init_port();
    e_init_ad_scan();
    e_init_uart1();
    e_led_clear();	
    e_init_motors();
    e_start_agendas_processing();
	 // wait for s to start
    //btcomWaitForCommand('s');
    btcomSendString("==== READY - IR TESTING ====\n\n");

    e_calibrate_ir(); 

    // initialize ircom and start reading
    ircomStart();
    ircomEnableContinuousListening();
    ircomListen();
    	int bmsl, bmsr;	// Braitenberg parameters
	    int msl, msr;	// Braitenberg parameters
	    max_sens=0;
for(;;){
// Finish reseting the robot
    // rely on selector to define the role
	int selector = getselector();
	if(selector>=0 && selector<=9)
	{
		robot_id = selector;
	}	
	else
	{
		btcomSendString("ERROR: SELECTOR NOT BETWEEN 0 AND 9!!!\n\n");
	}	
	if(robot_id < 5)	//NEED TO CHANGE THIS A LITTLE maybe
               my_tribe = TRIBE_A;
           else
               my_tribe = TRIBE_B;
               
	// Add Braitenberg
    // activate obstacle avoidance
    //obstacleAvoidance(&bmsl,&bmsr);	
    //e_activate_agenda( obstacleAvoidance, 10000);
    //e_set_speed_left(100);
	//e_set_speed_right(100);


    obstacleAvoidance(&bmsl,&bmsr);
	int i;
	for (i = 0; i < 1; i++)
	{
	    // takes ~15knops for a 32window, avoid putting messages too close...
	    long int j=0;
	    for(j = 0; j < 10000; j++)	asm("nop");
          ircomSend(robot_id);	    
	    while (ircomSendDone() == 0);
	}
	
	/// Compute self position
	prev_my_position[0] = my_position[0];
	prev_my_position[1] = my_position[1];
	
	compute_time_step();
	update_self_motion();
    
    // acting as receiver

	i = 0;
	while (i < 200)
	{
	    // ircomListen();
	    IrcomMessage imsg;
	    ircomPopMessage(&imsg);
	    if (imsg.error == 0)
	    {
			int other_robot_id = (int) imsg.value;
			//if(((other_robot_id<5) && my_tribe==TRIBE_A) || ((other_robot_id>=5) && my_tribe==TRIBE_B))
		    //{
				/* Send Value*/		
				double range = imsg.distance;
				double theta = imsg.direction;
					   theta += my_position[2];

				prev_relative_pos[other_robot_id][0] = relative_pos[other_robot_id][0];
				prev_relative_pos[other_robot_id][1] = relative_pos[other_robot_id][1];
				relative_pos[other_robot_id][0] = range*cos(theta);  // relative x pos
				relative_pos[other_robot_id][1] = range*sin(theta);  // relative x pos

				relative_speed[other_robot_id][0] = relative_speed[other_robot_id][0]*0.0 + 1.0*(1000/time_step)*(relative_pos[other_robot_id][0]-prev_relative_pos[other_robot_id][0]);
				relative_speed[other_robot_id][1] = relative_speed[other_robot_id][1]*0.0 + 1.0*(1000/time_step)*(relative_pos[other_robot_id][1]-prev_relative_pos[other_robot_id][1]);

				//double c=range*cos(theta);
					//	char tmp2[128];
				//sprintf(tmp2, "Relative positon: %f  - distance=%f \t direction=%f \n",(double) c, (double) range, (double) theta);
				//btcomSendString(tmp2);
				//if (i%10==0) btcomSendString("Receive Success \n");
				btcomSendString("Receive Success \n");
			
	    }//}
	    else if (imsg.error > 0)
	    {
              btcomSendString("Receive Failed \n");
	    }
	    // else imsg.error == -1 -> no message available in the queue

	    if (imsg.error == -1) i++;
	}
	
	
	//Reynolds
    
	speed[robot_id][0] = (1000/time_step)*(my_position[0]-prev_my_position[0]);
	speed[robot_id][1] = (1000/time_step)*(my_position[1]-prev_my_position[1]);
    //e_activate_agenda(reynolds_rules, 10000);
    reynolds_rules();
    
    
    
    // Compute wheels speed from reynold's speed

	compute_wheel_speeds(&msl, &msr);
    
	// Adapt speed instinct to distance sensor values
	if (sum_sensors > NB_SENSORS*MIN_SENS) {
		msl -= msl*max_sens/(2*MAX_SENS);
		msr -= msr*max_sens/(2*MAX_SENS);
	}
	
	//Add Braitenberg velociy
    
	msl += BRT_WEIGHT*bmsl;
	msr += BRT_WEIGHT*bmsr;
	

    limit_duo_proportional(&msl, &msr, MAX_SPEED);
   	char tmp2[128];
	sprintf(tmp2, "msl= %f - msr=%f - bmsl= %f - bmsr=%f \n", 5.0 * (double) bmsl, 5.0* (double)bmsr,  5.0 * (double) bmsl, 5.0* (double)bmsr);
	btcomSendString(tmp2);

	e_set_speed_left(msl);
	e_set_speed_right(msr);
	int j;
	for(j = 0; j < 10000; j++)	asm("nop");
		// DO A TIME STEP (if possible)
		//potential problem: odometry will work terribly bad, so we could use the motors get step and set steps functions, and not using any TIME_STEP at all.
 
}	
    ircomStop();
   
    return 0;
    
}

int obstacleAvoidanceThreshold = 50.0;
int obstacleAvoidanceSpeed = 500.0;
void obstacleAvoidance(int *bmsl,int *bmsr)
{    
	
	int n;
	int i;
	double sensorMean[8];
	double sensors[NB_SENSORS];	// Array for the distance sensor readings
	int numberOfSamples=30;
	// check if an obstacle is perceived 
	sum_sensors=0;*bmsl=0;*bmsr=0;	// Braitenberg parameters
	// Braitenberg parameters

		for (i=0;i<8;i++)
			sensorMean[i]=0;	
		//Compute an average value of each sensor on multiple samples to reduce noise	
		for (n=0;n<numberOfSamples;n++)
		{	
			// Get sensor values
			for (i = 0; i < 8; i++) {
				// Use the sensorzero[i] value generated in sensor_calibrate() to zero sensorvalues
				sensors[i] = e_get_calibrated_prox(i);
				//linearize the sensor output and compute the average
				sensorMean[i]+=12.1514*log((double)sensors[i])/(double)numberOfSamples;
			}

		}

		// Add the weighted sensors values
		for (i = 0; i < 8; i++) {
			*bmsr += e_puck_matrix[i] * (int)sensorMean[i];
			*bmsl += e_puck_matrix[i+NB_SENSORS] * (int)sensorMean[i];
			sum_sensors += sensorMean[i]; // Add up sensor values
			max_sens = max_sens>sensorMean[i]?max_sens:sensorMean[i]; // Check if new highest sensor value
		}
		*bmsl/=MIN_SENS; *bmsr/=MIN_SENS;
		*bmsl+=66; *bmsr+=72;


		// Speed bounds, to avoid setting to high speeds to the motor
		if (*bmsl > 1000) {*bmsl = 1000;}
		if (*bmsr > 1000) {*bmsr = 1000;}
		if (*bmsl< -1000) {*bmsl = -1000;}
		if (*bmsr < -1000) {*bmsr= -1000;}

}


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

