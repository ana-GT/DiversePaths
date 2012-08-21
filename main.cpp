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
  printf( "End test \n" );

  // Path
  int cost = 1;
  int radius = 1;
  BFS3D bfs( pf.getNumCells( PFDistanceField::DIM_X ), 
	     pf.getNumCells( PFDistanceField::DIM_Y ),
	     pf.getNumCells( PFDistanceField::DIM_Z ), 
	     radius, cost );
  bfs.configDistanceField( true, &pf );
  
  std::vector<int> goal(3);
  int gx; int gy; int gz;

  if( ! pf.worldToGrid( 0.95, 0.95, 0.95, gx, gy, gz ) ) {
    printf("Goal no valid exiting! \n");
    return 0;
  }

  printf( "gx: %d gy: %d gz: %d \n", gx, gy, gz );
  goal[0] = gx; goal[1] = gy; goal[2] = gz;
  bfs.setGoal( goal );
  printf( " Start BFS \n" );
  time_t ts, tf; double dt;
  ts = clock();
  bfs.runBFS();
  tf = clock();
  dt = (double) (tf - ts) / CLOCKS_PER_SEC;
  printf( " End BFS: time: %f \n", dt );
  
  // View


  return 0;
}

