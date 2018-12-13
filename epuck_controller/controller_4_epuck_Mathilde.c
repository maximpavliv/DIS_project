
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
#include <string.h>
#include <stdlib.h>

// **************************** FROM Reynolds2

#define NB_SENSORS	  8	  // Number of distance sensors
#define MIN_SENS          350     // Minimum sensibility value
#define MAX_SENS          4096    // Maximum sensibility value
#define MAX_SPEED         1000     // Maximum speed
/*Webots 2018b*/
#define MAX_SPEED_WEB      6.27    // Maximum speed webots
/*Webots 2018b*/
#define FLOCK_SIZE	  5	  // Size of flock
#define TIME_STEP	  64	  // [ms] Length of time step

#define AXLE_LENGTH 		0.052	// Distance between wheels of robot (meters)
#define SPEED_UNIT_RADS		0.00628	// Conversion factor from speed unit to radian per second
#define WHEEL_RADIUS		0.0205	// Wheel radius (meters)
#define DELTA_T			0.064	// Timestep (seconds)


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

/*Webots 2018b*/
// WbDeviceTag left_motor; //handler for left wheel of the robot
// WbDeviceTag right_motor; //handler for the right wheel of the robot
/*Webots 2018b*/
int e_puck_matrix[16] = {17,29,34,10,8,-38,-56,-76,-72,-58,-36,8,10,36,28,18}; // Maze
//int e_puck_matrix[16] = {57,45,30,20,15,15,5,5,5,5,15,15,20,30,45,58}; // for obstacle avoidance
//static double l_weight[NB_SENSORS] = {0.5, 0.25, 0.2, 0, 0, 0, 0, 0};
//static double r_weight[NB_SENSORS] = {0, 0, 0, 0, 0, 0.2, 0.25, 0.5};


// WbDeviceTag ds[NB_SENSORS];	// Handle for the infrared distance sensors
// WbDeviceTag receiver;		// Handle for the receiver node
// WbDeviceTag emitter;		// Handle for the emitter node

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

float theta_robots[FLOCK_SIZE];

int my_tribe;
// End from Reynolds2


float sensorDir[NB_IR_SENSORS] = {0.2967, 0.8727, 1.5708, 2.6180, 3.6652, 4.7124, 5.4105, 5.9865};

int robotsids[] = {136, 108, 54};

int getselector()
{
    return SELECTOR0 + 2*SELECTOR1 + 4*SELECTOR2 + 8*SELECTOR3;
}

int getrobotid()
{
      return robotsids[getselector()];
}

// From Reynolds2
/*
 * Reset the robot's devices and get its ID
 */
static void reset() 
{
	robot_id = getrobotid();
	if(getselector() < 5)
               my_tribe = TRIBE_A;
           else
               my_tribe = TRIBE_B;
               
       char tmp[128];
       sprintf(tmp, "Reset: robot %d, tribe %d\n",robot_id, my_tribe);
       btcomSendString(tmp);
}

// End from reynolds2

void obstacleAvoidance();

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
    btcomWaitForCommand('s');
    btcomSendString("==== READY - IR TESTING ====\n\n");

    reset();

    e_calibrate_ir(); 

    // initialize ircom and start reading
    ircomStart();
    ircomEnableContinuousListening();
    ircomListen();

    // rely on selector to define the role
    int selector = getselector();

    // show selector choosen
    int i;
    long int j;
    for (i = 0; i < selector; i++)
    {
	e_led_clear();
	
	for(j = 0; j < 200000; j++)
	    asm("nop");
	
	e_set_led(i%8, 1);
	
	for(j = 0; j < 300000; j++)
	    asm("nop");

	e_led_clear();

	for(j = 0; j < 300000; j++)
	    asm("nop");
    }

    // activate obstacle avoidance
    //e_activate_agenda(obstacleAvoidance, 10000);

    // acting as sender
    if (selector == 1)
    {    	
	btcomSendString("==== EMITTER ====\n\n");

	int i;
	for (i = 0; i < 10000; i++)
	{
	    // takes ~15knops for a 32window, avoid putting messages too close...
	    for(j = 0; j < 100000; j++)	asm("nop");

	    ircomSend(i % 256);	    
	    while (ircomSendDone() == 0);

	    btcomSendString(".");
	}
    }
    
    // acting as receiver
    else if (selector == 2)
    {
	btcomSendString("==== RECEIVER ====\n\n");

	int i = 0;
	while (i < 200)
	{
	    // ircomListen();
	    IrcomMessage imsg;
	    ircomPopMessage(&imsg);
	    if (imsg.error == 0)
	    {
			e_set_led(1, 2);
		int val = (int) imsg.value;
	    
		/* Send Value*/		
		char tmp[128];
		sprintf(tmp, "Receive successful : %d  - distance=%f \t direction=%f \n", val, (double)imsg.distance, (double)imsg.direction);
		btcomSendString(tmp);
	    }
	    else if (imsg.error > 0)
	    {
		btcomSendString("Receive failed \n");		
	    }
	    // else imsg.error == -1 -> no message available in the queue

	    if (imsg.error != -1) i++;
	}
    }
    // no proper role defined...
    else 
    {
	int i = 0;
	long int j;
	while(1)
	{
	    e_led_clear();
	    
	    for(j = 0; j < 200000; j++)
		asm("nop");
	    
	    e_set_led(i, 1);
	    
	    for(j = 0; j < 300000; j++)
		asm("nop");
	    
	    i++;
	    i = i%8;
	}	
    }    
    
    ircomStop();
    return 0;
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
    e_set_speed_left(leftSpeed);
    e_set_speed_right(rightSpeed);
    
    // advertise obstacle avoidance in progress
    // return 1;
}
