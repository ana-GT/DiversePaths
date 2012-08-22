/**
 * @file main.cpp
 * @author A. Huaman Q.
 * @date 2012-08-21
 */

#include <distance_field/pf_distance_field.h>
#include <distance_field/voxel_grid.h>
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
  pf.addBox( 0.2, 0.2, 0.3, 0.05, 0.05, 0.3 );
  pf.addBox( 0.1, 0.1, 0.2, 0.4, 0.5, 0.4 );
  printf( "End test \n" );
  
  // Settings BFS
  int cost = 1;
  int radius = 1;
  BFS3D bfs( pf.getNumCells( PFDistanceField::DIM_X ), 
	     pf.getNumCells( PFDistanceField::DIM_Y ),
	     pf.getNumCells( PFDistanceField::DIM_Z ), 
	     radius, cost );
  bfs.configDistanceField( true, &pf );
  
  //-- Set goal
  std::vector<int> goal(3);
  int gx; int gy; int gz;

  if( ! pf.worldToGrid( 0.95, 0.95, 0.95, gx, gy, gz ) ) {
    printf("Goal no valid exiting! \n");
    return 0;
  }

  goal[0] = gx; goal[1] = gy; goal[2] = gz;
  bfs.setGoal( goal );

  //-- Dijkstra 
  printf( " Start BFS \n" );
  time_t ts, tf; double dt;
  ts = clock();
  bfs.runBFS();
  tf = clock();
  dt = (double) (tf - ts) / CLOCKS_PER_SEC;
  printf( " End BFS: time: %f \n", dt );

  //-- Get a path
  int px; int py; int pz;
  pf.worldToGrid(0.02, 0.02, 0.02, px, py, pz );
  std::vector<std::vector<double> > wPath;
  printf("Get shortest path \n");
  if( bfs.getShortestPath( px, py, pz, wPath) ) {
    printf("Got a path of length: %d \n", wPath.size() );
  }

  /////// VISUALIZATION /////////////////////

  //-- Create a viewer
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = createViewer();

  //-- Get obstacles
  std::vector<Eigen::Vector3d> obstacles;
  pf.getPointsFromField( obstacles );

  //-- Put them in PCD
  pcl::PointCloud<pcl::PointXYZ>::Ptr obstacleCloud = writePCD( obstacles );
  pcl::PolygonMesh triangles;
  getMesh( obstacleCloud, triangles );
  viewMesh( &triangles, viewer );

  //-- View PCD
  viewPCD( obstacleCloud, viewer );
  viewPath( wPath, viewer );
  viewBall( wPath[0][0], wPath[0][1], wPath[0][2], "startBall", 0.02, viewer );
  viewBall( wPath[wPath.size() - 1][0], wPath[wPath.size() - 1][1], wPath[wPath.size() - 1][2], "endBall", 0.02, viewer );

  //-- Loop
  while( !viewer->wasStopped() ) {
    viewer->spinOnce(100);
    boost::this_thread::sleep( boost::posix_time::microseconds(100000));
  }
  return 0;
}

