/**
 * @file bresenham.h
 * @brief http://moveit.ros.org/bresenham_8h_source.html
 * @author M. Likhachev (modified by achq)
 * @date 2012/09/04
 */

#ifndef __BRESENHAM_3D_H__
#define __BRESENHAM_3D_H__

#include <vector>
#include <iostream>
#include <math.h>

/**
 * @struct bresenham3d_param_t
 */
typedef struct {

  int x1; int y1; int z1;
  int x2; int y2; int z2;
  int x_index; int y_index; int z_index;

  int using_xyz_index;
  int inc_x; int inc_y; int inc_z;
  
  int dx; int dy; int dz;
  int dx2; int dy2; int dz2;

  int err1; int err2;
} bresenham3d_param_t;


void get_bresenham3d_parameters( int _p1x, int _p1y, int _p1z,
				 int _p2x, int _p2y, int _p2z,
				 bresenham3d_param_t* _params );
void get_current_point3d( bresenham3d_param_t* _params,
			  int* _x, int* _y, int* _z );
bool get_next_point3d( bresenham3d_param_t* _params );

std::vector<std::vector<int> > getLine( int _p1x, int _p1y, int _p1z,
					int _p2x, int _p2y, int _p2z );

#endif /** __BRESENHAM_3D_H__ */
