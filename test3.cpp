/**
 * @file test1.cpp
 * @author A. Huaman Q.
 * @date 2012-08-21
 * @brief Test case 3: Dos cajas largas, delgadas y paralelas
 */

#include <stdio.h>
#include <ctime>
#include <boost/thread/thread.hpp>
#include "PCL_Tools/PCL_Tools.h"
#include <distance_field/pf_distance_field.h>
#include "DiversePaths.h"

/**
 * @function main
 */
int main( int argc, char* argv[] ) {
  
  double sx; double sy; double sz;
  double ox; double oy; double oz;
  double resolution;

  sx = 1.0; sy = 1.0; sz = 1.0;
  ox = 0.0; oy = 0.0; oz = 0.0;
  resolution = 0.0125;
  
  // Distance Field
  printf("Create PF Distance Field \n");
  PFDistanceField pf( sx, sy, sz, resolution, ox, oy, oz );
  pf.reset();

  // Geometry
  double bx; double by; double bz;
  bx = 0.2; by = 0.2; bz = 0.2;

  pf.addBox( 0.1, 0.6, 0.6, bx, by, bz );
  pf.addBox( 0.1, 0.6, 0.6, bx + 0.5, by + 0.0, bz + 0.0 );

  // Settings parameters
  int cost = 1; int radius = 3; int numPaths = 5;

  DiversePaths dp( &pf, radius, cost );

  // Set goal and start
  std::vector<double> goal(3);
  goal[0] = 0.5; goal[1] = 0.9; goal[2] = 0.5;
  std::vector<double> start(3);
  start[0] = 0.5; start[1] = 0.1; start[2] = 0.5;

  // Paths
  std::vector<std::vector<std::vector<double> > > paths;
  std::vector<std::vector<double> > midPoints;  
  paths = dp.getDiversePaths( start, goal, numPaths, midPoints );

  printf("End of program \n");
  return(0);
}

