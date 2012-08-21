/**
 * @file main.cpp
 * @author A. Huaman Q.
 * @date 2012-08-21
 */


#include <distance_field/voxel_grid.h>
#include <stdio.h>

/**
 * @function main
 */
int main( int argc, char* argv[] ) {
  
  int i;
  int def = -1;
  VoxelGrid<int> vg( 1.0, 0.8, 0.7, 
		     0.01, 
		     0.0, 0.0, 0.0, 
		     def );

  int freeCell = 36;
  vg.reset( freeCell );
  int numX = vg.getNumCells( VoxelGrid<int>::DIM_X );
  int numY = vg.getNumCells( VoxelGrid<int>::DIM_Y );
  int numZ = vg.getNumCells( VoxelGrid<int>::DIM_Z );
  printf("Get cell 3: %d \n", vg.getCell(3,3,3) );

  int whatever = 25;
  vg.setCell(4,5,6, whatever);

  printf("Get cell 4-5-6: %d \n", vg.getCell(4,5,6) );
  printf("Num cells: X: %d Y: %d Z: %d \n", numX, numY, numZ );

  return 0;

}

