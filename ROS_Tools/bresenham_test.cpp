/**
 * @file bresenham_test.cpp
 */

#include "bresenham.h"
#include <stdio.h>
#include <ctime>
/**
 * @function main
 */
int main( int argc, char* argv[] ) {
  
  std::vector<int> p1(3);
  std::vector<int> p2(3);
  p1[0] = 0; p1[1] = 3; p1[2] = 9;
  p2[0] = 4; p2[1] = 8; p2[2] = 15;
  std::vector<std::vector<int> > line;
  time_t ts = clock();
  for( int i = 0; i < 10000; ++i ) {
    line = getLine( 0, 3, 9, 40, 80, 23 );
  }
  time_t tf = clock();
  double dt = (double)(tf - ts) / CLOCKS_PER_SEC;

  printf("Done in %f seconds! \n", dt );
  return 0;
}
