/*
 * odometry.c
 *
 *  Created on: 2 avr. 2014
 *      Author: ldo
 */

#include "odometry.h"

pose *robot_pose;
double wheels_distance; /* [pulse] */

/*
 * @param *p : robot pose
 * @param d : distance between wheels [pulse]
 * @return none
 * */
void
odometry_setup(pose *p, double d)
{
  robot_pose = p;
  wheels_distance = d;
}

/* limit angle from -pi to +pi */
void
limit_angle(void)
{
  /* limit angle from -pi to +pi */
  if (robot_pose->O > (M_PI * wheels_distance)) /* > +pi */
    {
      robot_pose->O -= 2.0 * (M_PI * wheels_distance);
    }
  if (robot_pose->O < -(M_PI * wheels_distance)) /* < -pi */
    {
      robot_pose->O += 2.0 * (M_PI * wheels_distance);
    }
}

/* update new robot pose (x, y, O) approximated by straight line segments
 * @param distance : delta value for distance [pulse]
 * @param delta_angle : delta value for angle [pulse]
 * @return new pose */
void
odometry_by_segment(double distance, double angle)
{
  double O_rad = robot_pose->O / wheels_distance; /* [radian] */

  robot_pose->x += distance * cos(O_rad);
  robot_pose->y += distance * sin(O_rad);
  robot_pose->O += angle;

  limit_angle();
}

/* update new robot pose (x, y, O) approximated by an arc
 * @param distance : delta value for distance [pulse]
 * @param delta_angle : delta value for angle [pulse]
 * @return new pose */
void
odometry_by_arc(double distance, double angle)
{
  double O_rad = robot_pose->O / wheels_distance; /* [radian] */

  if (angle == 0)
    {
      /* robot pose */
      robot_pose->x += distance * cos(O_rad);
      robot_pose->y += distance * sin(O_rad);
    }
  else
    {
      /* radius and angle oh the arc */
      double a = angle / wheels_distance;
      double r = distance / a;

      /* coordinates of the center oh the arc */
      double xo = robot_pose->x - r * sin(O_rad);
      double yo = robot_pose->y + r * cos(O_rad);

      /* robot pose */
      robot_pose->O += a;
      robot_pose->x = xo + r * sin(O_rad);
      robot_pose->y = yo - r * cos(O_rad);

      limit_angle();
    }
}

/* update new robot pose (x, y, O)
 * @param distance : delta value for distance [pulse]
 * @param delta_angle : delta value for angle [pulse]
 * @param approximation : SEGMENT (default) or ARC
 * @return new pose */
void
odometry_update(double distance, double angle, uint8_t approximation)
{
  if (approximation == ARC)
    {
      odometry_by_arc(distance, angle);
    }
  else
    {
      odometry_by_segment(distance, angle);
    }
}
