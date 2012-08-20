/**
 * @file main.cpp
 * @author A. Huaman
 **/

#include "PCL_Tools/PCL_Tools.h"
#include "GraphSearch/Graph.h"
#include "GraphSearch/Dijkstra.h"
#include <stdio.h>
#include <ctime>

/**
 * @function main 
 */
int main ( int argc, char** argv ) {

  printf("Create new graph \n");
  Graph* g;
  g = new Graph( 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0.01, FREE );

  g->createBox( 0.2,0.2,0.2, 0.1, 0.2, 0.3, OBSTACLE );
  g->createBox( 0.6, 0.5, 0.4, 0.1, 0.3, 0.2, OBSTACLE );
  printf("Num vertices graph: %d \n", g->getNumV() );

  printf("Start Dijkstra \n");
  Dijkstra d( g );
  
  int x = 15; int y = 10; int z = 27;
  if( g->getState(x,y,z) == FREE ) {
    printf("Start is FREE. Search Dijkstra \n");
    time_t ts, tf; double dt;
    ts =  clock();
    d.search( x,y,z );
    tf = clock();
    dt = (double)(tf - ts)/CLOCKS_PER_SEC;
    printf("Search time: %f \n", dt );

  } else {
    printf("Start is NOT FREE! \n");
  }

  printf("End! \n");
  std::vector<Eigen::Vector3d> path;
  path = d.getPath( 0.95, 0.95, 0.95 );
  printf("Path size: %d \n", path.size() );
  d.printInfo();

 
  pcl::PointCloud<pcl::PointXYZ>::Ptr obstacleCloud;
  obstacleCloud = g->getPCD( OBSTACLE );

  printf("Creating a viewer \n");
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  viewer = createViewer();
  
  printf("View PCD \n");
  
  /*
  pcl::PolygonMesh mesh;
  printf("Creating a mesh \n");
  getMesh( obstacleCloud, mesh );
  viewMesh( &mesh, viewer);
  */
  
  printf("Size of path: %d \n", path.size());
  viewPath( path, viewer );
  
  //viewPCD( obstacleCloud, viewer );
  printf("Starting the viewer \n");
  while( !viewer->wasStopped() ) {
    viewer->spinOnce(100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
  

  return 0;
  
}
