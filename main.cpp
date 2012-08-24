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

  // Search
  printf("Start search \n" );
  std::vector<double> goal(3);
  goal[0] = 0.65; goal[1] = 0.4; goal[2] = 0.6;
  printf( "Set goal \n" );
  dp.setGoal( goal );
  printf(" Run Dijkstra \n");
  
  time_t ts; time_t tf; double dt;
  ts = clock();
  dp.runDijkstra();
  tf = clock();
  dt = (double)(tf - ts) / CLOCKS_PER_SEC;
  printf("End Dijkstra in %f seconds \n", dt );

  // Get one path
  std::vector<std::vector<double> > pathA;
  std::vector<std::vector<double> > pathD;
  std::vector<double> start(3);
  start[0] = 0.16; start[1] = 0.1; start[2] = 0.1;
  printf("Get shortest path start \n");
  dp.getShortestPath( start, pathD );
  printf("End of run. Path D size: %d \n", pathD.size() );
  printf("Run Astar \n");
  dp.runAstar( start, pathA );
  printf("End of run. Path A size: %d \n", pathA.size() );
  // Get DF from path
  printf("Create DF to Path Set \n");
  PFDistanceField* dpp;
  dpp = dp.createDfToPathSet( pathA );
  printf("End of Create DF to Path Set \n");

  // Get points as far as
  std::vector<std::vector<double> > points;
  double thresh = 0.1; double tol = 0.005;
  points = dp.getPointsAsFarAs( dpp, 0.4, tol );
  printf("Got %d points as far as : %f with tol: %f \n", points.size(), thresh, tol );
  
  std::vector<std::vector<double> > pointsObst;
  pointsObst = dp.getPointsAsFarAs( &pf, thresh, tol );
  printf("Got %d points as far as : %f with tol: %f from obstacle \n", pointsObst.size(), thresh, tol );

  // Create viewer
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = createViewer();

  // View the path
  reset_PCL_Tools_counters();
  dp.visualizePath( viewer, pathD, true, 0, 0, 255 );
  dp.visualizePath( viewer, pathA, true, 255, 255, 0 );
  viewPoints( pointsObst, viewer, 0, 0, 255 );
  viewPoints( points, viewer, 255, 0, 255 );


  // Loop
  while( !viewer->wasStopped() ) {
    viewer->spinOnce(100);
    boost::this_thread::sleep( boost::posix_time::microseconds(100000));
    }
 
}

