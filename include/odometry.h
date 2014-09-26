/*
 * odometry.h
 *
 *  Created on: 2 avr. 2014
 *      Author: ldo
 */

#ifndef ODOMETRY_H_
#define ODOMETRY_H_

#include <math.h>
#include <avr/io.h>

#define SEGMENT 0
#define ARC 1

typedef struct
{
  double x; /* x-position [pulse] */
  double y; /* y-position [pulse] */
  double O; /* 0-orientation [pulse] */
} pose; /* cartesian pose */

typedef struct
{
  double d; /* distance position [pulse] */
  double a; /* angle position [pulse] */
  double O; /* 0-orientation [pulse] */
} polar_pose; /* polar pose */

/*
 * @param *p : robot pose
 * @param d : distance between wheels [pulse]
 * @return none
 * */
void
odometry_setup(pose *p, double d);

/* update new robot pose (x, y, O)
 * @param distance : delta value for distance [pulse]
 * @param delta_angle : delta value for angle [pulse]
 * @param approximation : SEGMENT (default) or ARC
 * @return new pose */
void
odometry_update(double distance, double angle, uint8_t approximation);

#endif /* ODOMETRY_H_ */
