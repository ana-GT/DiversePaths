/**
 * @file test2.cpp
 * @author A. Huaman Q.
 * @date 2012-08-21
 * @brief Test case 2: Caja en forma de U
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
  bx = 0.2; by = 0.25; bz = 0.2;

  pf.addBox( 0.6, 0.5, 0.1, bx, by, bz );
  pf.addBox( 0.1, 0.5, 0.4, bx + 0.0, by + 0.0, bz + 0.1 );
  pf.addBox( 0.1, 0.5, 0.4, bx + 0.5, by + 0.0, bz + 0.1 );

  // Settings parameters
  int cost = 10; int radius = 2; int numPaths = 5;

  DiversePaths dp( &pf, radius, cost );

  // Set goal and start
  std::vector<double> goal(3);
  goal[0] = 0.5; goal[1] = 0.9; goal[2] = 0.5;
  std::vector<double> start(3);
  start[0] = 0.5; start[1] = 0.1; start[2] = 0.5;

  // Paths
  std::vector<std::vector<double> > midPoints;
  std::vector<std::vector<std::vector<double> > > paths;

  // Get paths
  float boundFactor = 2.0;
  int numCheckPoints = 10;
  time_t ts = clock();
  paths = dp.getDiversePaths2( start, goal, numPaths, midPoints, boundFactor, numCheckPoints );
  time_t tf = clock();
  double dt = (double) ( tf - ts ) / CLOCKS_PER_SEC;
  printf( "** getDiversePaths2 time: %f \n", dt );

  printf( "Num points: %d \n", midPoints.size() );

  // Create viewer
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = createViewer();

  // View the path
  reset_PCL_Tools_counters();

  // MidPoints: View pointcloud (NOT BOTH Balls and pointcloud)
  //viewPoints( midPoints, viewer, 255,0,255 );

  // Get checkPoint Lines
  /*
  int ind = 3;
  std::vector<std::vector<std::vector<double> > > checkLines;
   if( paths.size() > 1 ) {
     checkLines = dp.getCheckPointLines( paths[0], paths[ind], numCheckPoints );
   } else {
     printf( " !! Only one path, no drawing checkPoint lines! \n" );
   }
   
   for( int i = 0; i < checkLines.size(); ++i ) {
       dp.visualizePath( viewer, checkLines[i], false, 0, 200, 0, 0.008, 0, 0, 200 );
   }
  */
   // dp.visualizePath( viewer, paths[0], false, 0, 100, 0 );
   dp.visualizePaths( viewer, paths, true, 255, 69, 0 );

  // Loop
  while( !viewer->wasStopped() ) {
    viewer->spinOnce(100);
    boost::this_thread::sleep( boost::posix_time::microseconds(100000));
    } 

  printf("End of program \n");
  return(0);

}

