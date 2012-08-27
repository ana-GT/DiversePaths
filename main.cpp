/**
 * @file main.cpp
 * @author A. Huaman Q.
 * @date 2012-08-21
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
  
  printf("Create PF Distance Field \n");
  PFDistanceField pf( sx, sy, sz, resolution, ox, oy, oz );
  pf.reset();
  pf.addBox( 0.2, 0.2, 0.3, 0.24, 0.24, 0.3 );
  pf.addBox( 0.1, 0.1, 0.2, 0.4, 0.5, 0.4 );

  // Settings parameters
  int cost = 1; int radius = 3;
  int numPaths = 10;

  DiversePaths dp( &pf, radius, cost );

  // Set goal and start
  std::vector<double> goal(3);
  goal[0] = 0.65; goal[1] = 0.4; goal[2] = 0.6;
  std::vector<double> start(3);
  start[0] = 0.16; start[1] = 0.1; start[2] = 0.1;

  // Paths
  std::vector<std::vector<std::vector<double> > > paths;

  paths = dp.getDiversePaths( start, goal, numPaths );

  // Create viewer
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = createViewer();

  // View the path
  reset_PCL_Tools_counters();
  dp.visualizePaths( viewer, paths, true );
  // viewPoints( points, viewer, 255, 0, 255 );


  // Loop
  while( !viewer->wasStopped() ) {
    viewer->spinOnce(100);
    boost::this_thread::sleep( boost::posix_time::microseconds(100000));
    }
 
}

