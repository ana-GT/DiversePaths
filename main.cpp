/**
 * @file main.cpp
 * @author A. Huaman Q.
 * @date 2012-08-21
 */

#include <distance_field/pf_distance_field.h>

#include "DiversePaths.h"
#include <bfs_3d.h>
#include <stdio.h>
#include <ctime>

#include <boost/thread/thread.hpp>
#include "PCL_Tools/PCL_Tools.h"

/**
 * @function main
 */
int main( int argc, char* argv[] ) {
  
  double sx; double sy; double sz;
  double ox; double oy; double oz;
  double resolution;

  sx = 1.0; sy = 1.0; sz = 1.0;
  ox = 0.0; oy = 0.0; oz = 0.0;
  resolution = 0.01;
  
  printf("Test PF \n");
  PFDistanceField pf( sx, sy, sz, resolution, ox, oy, oz );
  pf.reset();
  pf.addBox( 0.2, 0.2, 0.3, 0.24, 0.24, 0.3 );
  pf.addBox( 0.1, 0.1, 0.2, 0.4, 0.5, 0.4 );
  printf( "End test \n" );


  // Settings BFS
  int cost = 1; int radius = 5;
  int numPaths = 3;
  double alpha = 0.1;

  DiversePaths dp( &pf, radius, cost );
  printf("End search \n");
  // Search
  std::vector<std::vector<std::vector<double> > > paths;
  dp.Search( 0.16, 0.1, 0.1, 0.65, 0.4, 0.6, numPaths, alpha, paths );
 
  // Create viewer
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = createViewer();

  // View your thing
  dp.visualizePaths( viewer, true );

  // Loop
  while( !viewer->wasStopped() ) {
    viewer->spinOnce(100);
    boost::this_thread::sleep( boost::posix_time::microseconds(100000));
    }
 
}

