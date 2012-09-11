/**
 * @file bresenham.cpp
 * @author Maxim Likhachev (modified by achq)
 * @date 2012/09/04
 * @brief http://moveit.ros.org/bresenham_8cpp_source.html
 */

#include "bresenham.h"
#include <stdio.h>

/**
 * @function get_bresenham3d_parameters
 */
void get_bresenham3d_parameters( int _p1x, int _p1y, int _p1z,
				 int _p2x, int _p2y, int _p2z,
				 bresenham3d_param_t* _params ){

  _params->x1 = _p1x;
  _params->y1 = _p1y;
  _params->z1 = _p1z;

  _params->x2 = _p2x;
  _params->y2 = _p2y;
  _params->z2 = _p2z;

  _params->x_index = _params->x1;
  _params->y_index = _params->y1;
  _params->z_index = _params->z1;

  _params->dx = fabs( (double) (_p2x - _p1x) );
  _params->dy = fabs( (double) (_p2y - _p1y) );
  _params->dz = fabs( (double) (_p2z - _p1z) );

  // (dx/y/z) / 2
  _params->dx2 = _params->dx << 1;
  _params->dy2 = _params->dy << 1;
  _params->dz2 = _params->dz << 1;  

  //-- Get slope direction

  // Slope x
  if( (double)( _p2x - _p1x ) < 0 ) {
    _params->inc_x = -1;
  } else {
    _params->inc_x = 1;
  }
  // Slope y
  if( (double)( _p2y - _p1y ) < 0 ) {
    _params->inc_y = -1;
  } else {
    _params->inc_y = 1;
  }
  // Slope z
  if( (double)( _p2z - _p1z ) < 0 ) {
    _params->inc_z = -1;
  } else {
    _params->inc_z = 1;
  }

  //-- Choose which axis to use as the index

  // Axis X
  if( _params->dx >= _params->dy &&
      _params->dx >= _params->dz ) {

    _params->using_xyz_index = 0;
    _params->err1 = _params->dy2 - _params->dx;
    _params->err2 = _params->dz2 - _params->dx;    

  }
  // Axis Y
  else if( _params->dy >= _params->dx &&
	   _params->dy >= _params->dz ) {

    _params->using_xyz_index = 1;
    _params->err1 = _params->dx2 - _params->dy;
    _params->err2 = _params->dz2 - _params->dy;    

  } 

  // Axis Z
  else {

    _params->using_xyz_index = 2;
    _params->err1 = _params->dy2 - _params->dz;
    _params->err2 = _params->dx2 - _params->dz;    

  }

}

/**
 * @function get_current_point3d
 */
void get_current_point3d( bresenham3d_param_t* _params,
			  int* _x, int* _y, int* _z ) {

  *_x = _params->x_index;
  *_y = _params->y_index;
  *_z = _params->z_index;
}

/**
 * @function get_next_point3d
 */
bool get_next_point3d( bresenham3d_param_t* _params ) {
  
  // Check to see if at end of line
  if( _params->x_index == _params->x2 &&
      _params->y_index == _params->y2 &&
      _params->z_index == _params->z2 ) {
    return false;
  }
  // Axis X
  if( _params->using_xyz_index == 0 ) {

    if( _params->err1 > 0 ) {
      _params->y_index += _params->inc_y;
      _params->err1 -= _params->dx2;
    }
    if( _params->err2 > 0 ) {
      _params->z_index += _params->inc_z;
      _params->err2 -= _params->dx2;      
    }

    _params->err1 += _params->dy2;
    _params->err2 += _params->dz2;
    _params->x_index += _params->inc_x;

  }

  // Axis Y
  else if( _params->using_xyz_index == 1 ) {

    if( _params->err1 > 0 ) {
      _params->x_index += _params->inc_x;
      _params->err1 -= _params->dy2;
    }
    if( _params->err2 > 0 ) {
      _params->z_index += _params->inc_z;
      _params->err2 -= _params->dy2;      
    }

    _params->err1 += _params->dx2;
    _params->err2 += _params->dz2;
    _params->y_index += _params->inc_y;

  }

  // Axis Z
  else {

    if( _params->err1 > 0 ) {
      _params->y_index += _params->inc_y;
      _params->err1 -= _params->dz2;
    }
    if( _params->err2 > 0 ) {
      _params->x_index += _params->inc_x;
      _params->err2 -= _params->dz2;      
    }

    _params->err1 += _params->dy2;
    _params->err2 += _params->dx2;
    _params->z_index += _params->inc_z;
  }

  return true;
}

/**
 * @function getLine
 */
std::vector<std::vector<int> > getLine( int _p1x, int _p1y, int _p1z,
					int _p2x, int _p2y, int _p2z ) {
  //-- 1. Initialize a bresenham structure
  bresenham3d_param_t b3d;
  get_bresenham3d_parameters( _p1x, _p1y, _p1z,
			      _p2x, _p2y, _p2z,
			      &b3d );
  //-- 2. Get line
  bool thereIsNextPoint = true;
  std::vector<std::vector<int> > line;
  std::vector<int> p(3);
  int x; int y; int z;
 
  while( thereIsNextPoint ) {
    get_current_point3d( &b3d, &x, &y, &z );
    p[0] = x; p[1] = y; p[2] = z;
    line.push_back( p );
    thereIsNextPoint = get_next_point3d( &b3d );
  } 

  return line;
}
