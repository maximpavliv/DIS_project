//******************************************************************************
//  Name:   e-puck.c
//  Author: Jim Pugh & Yvan Bourquin
//  Date:   October 2nd, 2007
//  Last modified: October 3rd, 2014 by M. Vasic
//  Rev:    September 28, 2015 by Florian Maushart
//******************************************************************************

#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <math.h>
#include <stdio.h>
#include <webots/motor.h>

#define NB_SENSORS 8
#define TIME_STEP 64
#define MAX_SPEED 1000
#define MAX_SENS 4095
#define MAX_SPEED_WEB      6.28    // Maximum speed webots

WbDeviceTag ps[NB_SENSORS]; // list of distance sensor handles
// note: the handles are != the sensor values. Sensor values
// are saved in ds_value[NB_SENSORS], which is defined in the main loop
WbDeviceTag left_motor; //handler for left wheel of the robot
WbDeviceTag right_motor; //handler for the right wheel of the robot

struct Vector {
   double left_speed;
   double right_speed;   
};


//-------------------fitness evaluation from braitenberg solution (check if we use the same for the project)-------------
// The fitness evaluation functions are provided to help you testing the
// performance of your controller.

// fitness variables
int step_count;
double fit_speed;
double fit_diff;
double sens_val[NB_SENSORS];

// reset fitness
void fitness_reset() {
  step_count = 0;
  fit_speed = 0.0;
  fit_diff = 0.0;
  int i;
  for (i = 0; i < NB_SENSORS; i++)
    sens_val[i] = 0.0;
}

// update fitness variables
void fitness_step(float left_speed, float right_speed, const double ds_value[NB_SENSORS]) {
  // average speed
  fit_speed += (fabs(left_speed) + fabs(right_speed)) / (2.0 * MAX_SPEED);
  // difference in speed
  fit_diff += fabs(left_speed - right_speed) / (2.0 * MAX_SPEED);
  // sensor values
  int i;
  for (i = 0; i < NB_SENSORS; i++)
    sens_val[i] += ds_value[i] / MAX_SENS;

  step_count++;
}

// compute current fitness
// the higher the number the better is the obstacle avoidance
// a good controller is expected to give a performance above 0.8
// note: this solution only performs at around 0.45 because the
// idle speed is 500 instead of 1000 (MAX_SPEED)
double fitness_compute() {
  double fit_sens = 0.0;
  int i;  
  for (i = 0; i < NB_SENSORS; i++)
    if (sens_val[i] > fit_sens)
      fit_sens = sens_val[i];

  fit_sens /= step_count;

  return fit_speed / step_count * (1.0 - sqrt(fit_diff / step_count)) * (1.0 - fit_sens);
}
//------------------------------------------------------------------------------



// controller initialization
static void reset(void) {
  fitness_reset();
  int i;
  char name[] = "ps0";
  for(i = 0; i < NB_SENSORS; i++) {
    ps[i]=wb_robot_get_device(name); // get sensor handle
    // perform distance measurements every TIME_STEP millisecond
    wb_distance_sensor_enable(ps[i], TIME_STEP);
    name[2]++; // increase the device name to "ps1", "ps2", etc.
  }
  //get motors
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
}

// controller main loop
static int run(int ms) {
  float msl_w, msr_w;
  int duration;
  
  Vector speeds = braitenberg_speeds(); // Here we will add all the other components, with respective weights (or FSM)
  
  // update fitness
  fitness_step(speeds.left_speed, speeds.right_speed, ds_value);

  // actuate wheel motors
  // sets the e-pucks wheel speeds to left_speed (left wheel) and right_speed (right wheel)
  // max speed is 1000 == 2 turns per second
  // Set speed
  msl_w = speeds.left_speed*MAX_SPEED_WEB/1000;
  msr_w = speeds.right_speed*MAX_SPEED_WEB/1000;
  wb_motor_set_velocity(left_motor, msl_w);
  wb_motor_set_velocity(right_motor, msr_w);

  // compute and display fitness every 1000 controller steps
  if (step_count % 1000 == 999) {
    double fitness = fitness_compute();
    printf("fitness = %f\n", fitness);
    fitness_reset();
  }

  return duration;
}

int main() {
  int duration = TIME_STEP;
  duration = TIME_STEP;

  wb_robot_init(); // controller initialization
  reset();
  
  
  while (wb_robot_step(duration) != -1) {
    duration = run(TIME_STEP);
  }
  
  wb_robot_cleanup();
  return 0;
}

Vector braitenberg_speeds() {
    // coefficients
  static double l_weight[NB_SENSORS] = {0.5, 0.25, 0.2, 0, 0, 0, 0, 0};
  static double r_weight[NB_SENSORS] = {0, 0, 0, 0, 0, 0.2, 0.25, 0.5};

  static double ds_value[NB_SENSORS];
  int i;
  Vector braitenberg;
  
    
  for (i = 0; i < NB_SENSORS; i++)
    // read sensor values
    // reads the handle in ps[i] and saves its value in ds_value[i]
    ds_value[i] = wb_distance_sensor_get_value(ps[i]); // range: 0 (far) to 4095 (0 distance (in theory))

  // choose behavior


  braitenberg.left_speed = 0;
  braitenberg.right_speed = 0;

  
  // define speed with respect to the sensory feedback
  for (i = 0; i < NB_SENSORS; i++)
  {
    braitenberg.left_speed += (-l_weight[i]) * ds_value[i];
    braitenberg.right_speed += (-r_weight[i]) * ds_value[i];
  }

  
  return braitenberg_speeds;
}

