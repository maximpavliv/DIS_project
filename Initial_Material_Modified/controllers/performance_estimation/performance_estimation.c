/*****************************************************************************/
/* File:         performance_estimation.c                                    */
/* Version:      1.0                                                         */
/* Date:         10-Oct-14                                                   */
/* Description:  estimating the performance of a formation 		     */
/*                                                                           */
/* Author: 	 10-Oct-14 by Ali marjovi				     */
/*****************************************************************************/


#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/supervisor.h>

#define FLOCK_SIZE	5		// Number of robots in flock
#define TIME_STEP	64		// [ms] Length of time step
#define MAX_SPEED    0.1287         // in m/s

WbNodeRef robs[FLOCK_SIZE];		// Robots nodes
WbFieldRef robs_trans[FLOCK_SIZE];	// Robots translation fields
WbFieldRef robs_rotation[FLOCK_SIZE];	// Robots rotation fields

float loc[FLOCK_SIZE][3];		// Location of everybody in the flock

#define RULE1_THRESHOLD 0.1
#define fit_cluster_ref 0.03
#define fit_orient_ref 1.0


int offset;				// Offset of robots number
float migrx, migrz;			// Migration vector
float orient_migr; 			// Migration orientation
int t;

/*
 * Initialize flock position and devices
 */
void reset(void) {
	wb_robot_init();

	char rob[7] = "epuck0";
	int i;
	for (i=0;i<FLOCK_SIZE;i++) {
		sprintf(rob,"epuck%d",i+offset);
		robs[i] = wb_supervisor_node_get_from_def(rob);
		robs_trans[i] = wb_supervisor_node_get_field(robs[i],"translation");
		robs_rotation[i] = wb_supervisor_node_get_field(robs[i],"rotation");
	}
}


/*
 * Compute performance metric.
 */
void compute_fitness(float* fit_c, float* fit_o) {
	*fit_c = 0; *fit_o = 0;
	// Compute performance indices
	// Based on distance of the robots compared to the threshold and the deviation from the perfect angle towards
	// the migration goal
	float angle_diff;
	int i; int j;
	float avg_loc[2] = {0,0};	// Flock average positions
       
       //float avg_speed[2] = {0,0};	// Flock average speeds
       
	for(i=0;i<FLOCK_SIZE;i++){

       for (j=0;j<2;j++) {
            //avg_speed[j] += speed[i][j] ;
            avg_loc[j]   += loc[i][j];
           }
  	}
	
       for (j=0;j<2;j++) {
            //avg_speed[j] /= FLOCK_SIZE-1 ;
            avg_loc[j]   /= FLOCK_SIZE;
          }
          
	for (i=0;i<FLOCK_SIZE;i++) {
         // for (j=i+1;j<FLOCK_SIZE;j++) 
		//{	
			// Distance measure for each pair ob robots
			//*fit_c += fabs(sqrtf(powf(loc[i][0]-loc[j][0],2)+powf(loc[i][1]-loc[j][1],2))-RULE1_THRESHOLD*2);
		//}
		*fit_c += fabs(sqrtf(powf(loc[i][0]-avg_loc[0],2)+powf(loc[i][1]-avg_loc[1],2)));

		// Angle measure for each robot
		angle_diff = fabsf(loc[i][2]-orient_migr);
		*fit_o += angle_diff > M_PI ? 2*M_PI-angle_diff : angle_diff;
	}
	//*fit_c /= FLOCK_SIZE*(FLOCK_SIZE+1)/2;
	*fit_c /= FLOCK_SIZE;
	*fit_o /= FLOCK_SIZE;
}



/*
 * Main function.
 */
 
int main(int argc, char *args[]) {
	int i;			// Index
  
	reset();

	
	// Compute reference fitness values
	
	float fit_cluster;			// Performance metric for aggregation
	float fit_orient;			// Performance metric for orientation
	float fit_velocity=0;			// Performance metric for velocity
	float inst_overall=0;	       // Instant Overall Performance metric 
	float fin_overall=0;	       // Final Overall Performance metric 	
		
	for(;;) {
		wb_robot_step(TIME_STEP);
		
		if (t % 10 == 0) {
			for (i=0;i<FLOCK_SIZE;i++) {
				// Get data
				loc[i][0] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[0]; // X
				loc[i][1] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[2]; // Z
				loc[i][2] = wb_supervisor_field_get_sf_rotation(robs_rotation[i])[3]; // THETA
				
    			}
			//Compute and normalize fitness values
			compute_fitness(&fit_cluster, &fit_orient);
			//fit_cluster = fit_cluster_ref/fit_cluster;
			fit_cluster = 1/(1+fit_cluster);
			fit_orient = 1-fit_orient/M_PI;
			fit_velocity=1;
			inst_overall=fit_cluster*fit_orient*fit_velocity;	       
        	       fin_overall+=inst_overall/(t+1);	   
			printf("time:%d, Distance : %f,Orientation: %f,Velocity: %f,Instant overall:%f,Final overall:%f \n", t, fit_cluster, fit_orient,fit_velocity,inst_overall,fin_overall);			
			
		}
		
		t += TIME_STEP;
	}

}