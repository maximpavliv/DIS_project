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
#define MAX_SPEED    0.1287         // in m/s because max velocity is 6.27 rad / sec  0.1287

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
float avg_loc[2]  ;	// Flock average positions at time t
float prev_avg_loc[2] ;	// previous Flock average positions
float mig_urge[2] = {1,0};	// previous Flock average positions
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



float max(float a,float b){
  if (a>b)
    return a;
  else
    return b;
}


/*
 * Compute performance metric.
 */
void update_avg_location() {
  //update la average location 
  int i,j;
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
}

void compute_fitness(float* fit_c, float* fit_o,float* fit_vel, float* dt) {
	*fit_c = 0; *fit_o = 0;
	// Compute performance indices
	// Based on distance of the robots compared to the threshold and the deviation from the perfect angle towards
	// the migration goal
	float delta_avg_loc[2];	// previous Flock average positions
	float avg_vel[2];	// previous Flock average positions
            float projection;
	int i;
           float orientation_sum_real=0;	
           float orientation_sum_imaginary=0;
	          
	for (i=0;i<FLOCK_SIZE;i++) {
         // for (j=i+1;j<FLOCK_SIZE;j++) 
		//{	
			// Distance measure for each pair ob robots
			//*fit_c += fabs(sqrtf(powf(loc[i][0]-loc[j][0],2)+powf(loc[i][1]-loc[j][1],2))-RULE1_THRESHOLD*2);
		//}
		*fit_c += fabs(sqrtf(powf(loc[i][0]-avg_loc[0],2)+powf(loc[i][1]-avg_loc[1],2)));

		orientation_sum_real += cos(loc[i][2]);
		orientation_sum_imaginary += sin(loc[i][2]);


	}
	//*fit_c /= FLOCK_SIZE*(FLOCK_SIZE+1)/2;
	*fit_c /= FLOCK_SIZE;
	//fit_cluster = fit_cluster_ref/fit_cluster;
	*fit_c = 1/(1+*fit_c);
	//orientation metric
	*fit_o = (sqrt(powf(orientation_sum_real,2) + powf(orientation_sum_imaginary,2)))/FLOCK_SIZE;
	// velocity metric
	 delta_avg_loc[0] = avg_loc[0]-prev_avg_loc[0];
        delta_avg_loc[1] = avg_loc[1]-prev_avg_loc[1];
        
        avg_vel[0] = delta_avg_loc[0] / *dt;
        avg_vel[1] = delta_avg_loc[1] / *dt;
        
        projection=(avg_vel[0]*mig_urge[0]+avg_vel[1]*mig_urge[1])/sqrt(powf(mig_urge[0],2)+powf(mig_urge[1],2));
        //projection=sqrt(powf(delta_avg_loc[0],2)+powf(delta_avg_loc[1],2))* mig_urge[0];
        *fit_vel=max(0.,projection)/(MAX_SPEED);

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
	float fit_velocity;			// Performance metric for velocity
	float inst_perf;	       // Instant Performance metric 
       float inter_sum=0;             //intermediary sum
	float overall_perf;	       // Final Overall Performance metric 
	int   k=0;
	float delta_time;

		
	for(;;) {
		wb_robot_step(TIME_STEP);
		k++;
		
		if (t % 10 == 0) {
		
			for (i=0;i<FLOCK_SIZE;i++) {
				// Get data
				loc[i][0] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[0]; // X
				loc[i][1] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[2]; // Z
				loc[i][2] = wb_supervisor_field_get_sf_rotation(robs_rotation[i])[3]; // THETA
				
    			}
    			
    			update_avg_location();
    			delta_time = k * (float)TIME_STEP/1000  ;

			//Compute and normalize fitness values
			compute_fitness(&fit_cluster, &fit_orient, &fit_velocity, &delta_time);

                                  prev_avg_loc[0] = avg_loc[0];	
                                  prev_avg_loc[1] = avg_loc[1];


			
			inst_perf=fit_cluster*fit_orient*fit_velocity;
			
				       
                    	inter_sum+=inst_perf;	   
                      overall_perf=(float) k*TIME_STEP*inter_sum/t;
			printf("time:%d, Cohesion : %f,Orientation: %f,Velocity: %f,Instant performance:%f,Overall performance:%f \n", t, fit_cluster, fit_orient,fit_velocity,inst_perf,overall_perf);			
			k=0;
		}
		
		t += TIME_STEP;
	}

}
