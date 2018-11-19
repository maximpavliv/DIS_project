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
  double x;
  double y;  
};


// controller initialization
static void reset(void) {
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


struct Vector braitenberg_speeds(){

  struct Vector speeds;//x for left wheel, y for right wheel
  
  // coefficients
  static double l_weight[NB_SENSORS] = {0.5, 0.25, 0.2, 0, 0, 0, 0, 0};
  static double r_weight[NB_SENSORS] = {0, 0, 0, 0, 0, 0.2, 0.25, 0.5};

  static double ds_value[NB_SENSORS];
  int i;

    
  for (i = 0; i < NB_SENSORS; i++)
    // read sensor values
    // reads the handle in ps[i] and saves its value in ds_value[i]
    ds_value[i] = wb_distance_sensor_get_value(ps[i]); // range: 0 (far) to 4095 (0 distance (in theory))


  speeds.x = 0;
  speeds.y = 0;
  
  // define speed with respect to the sensory feedback
  for (i = 0; i < NB_SENSORS; i++)
  {
    speeds.x += (-l_weight[i]) * ds_value[i];
    speeds.y += (-r_weight[i]) * ds_value[i];
  }
  return speeds;
}



// controller main loop
static int run(int ms) {
  int duration = TIME_STEP;
  float msl_w, msr_w;  
  
  struct Vector speeds;

  speeds = braitenberg_speeds();
  
  // actuate wheel motors
  // sets the e-pucks wheel speeds to left_speed (left wheel) and right_speed (right wheel)
  // max speed is 1000 == 2 turns per second
  // Set speed
  msl_w = speeds.x*MAX_SPEED_WEB/1000;
  msr_w = speeds.y*MAX_SPEED_WEB/1000;
  wb_motor_set_velocity(left_motor, msl_w);
  wb_motor_set_velocity(right_motor, msr_w);

  return duration;
}

int main() {
  int duration = TIME_STEP;

  wb_robot_init(); // controller initialization
  reset();
  
  
  while (wb_robot_step(duration) != -1) {
    duration = run(TIME_STEP);
  }
  
  wb_robot_cleanup();
  return 0;
}
