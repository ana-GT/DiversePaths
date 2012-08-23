/**
 * @file DiversePaths.h
 * @author A. Huaman Q. 
 */

#ifndef __DIVERSE_PATHS_H__
#define __DIVERSE_PATHS_H__

#include <boost/thread/thread.hpp>
#include "PCL_Tools/PCL_Tools.h"

#include <iostream>
#include <vector>
#include <string>
#include <time.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <queue>
#include <distance_field/pf_distance_field.h>


#define SMALL_NUM 0.00000001
#define INFINITE_COST 1000000000

#define DIRECTIONS3D 26
#define GOAL_TOLERANCE 0

/**
 * @struct State3D
 */
typedef struct {
  unsigned int g;
  int iterationclosed;
  int x;
  int y;
  int z;
} State3D;

/**
 * @class DiversePaths
 * @brief
 * @author A. Huaman
 * @date 2012/08/23
 */
class DiversePaths {
  
  /**< @class Cell3D */
  class Cell3D {
  public:
    int x;
    int y;
    int z;
  };

 public:
  
  DiversePaths( PFDistanceField *_df, 
		int _radius,
		int _cost_per_cell );
  ~DiversePaths();

  bool setGoal( std::vector<double> _goal );
  bool setGoal( std::vector<int> _goal );
  bool setGoals( std::vector<std::vector<int> > _goals );
  bool runDijkstra();
  void searchOneSourceAllPaths( State3D*** _stateSpace );
  bool getShortestPath( std::vector<int> _start,
			std::vector< std::vector<int> > &_path );
  bool getShortestPath( std::vector<double> _start,
			std::vector< std::vector<double> > &_path );
  void setRadius( double _r );
  int getRadiusCells();
  void reInitializeState3D( State3D* _state );
  void initializeState3D( State3D* _state, int _x, int _y, int _z );
  void create3DStateSpace( State3D**** _stateSpace3D );
  void delete3DStateSpace( State3D**** _stateSpace3D );
  inline int xyzToIndex( int _x, int _y, int _z );
  bool isGoal( const std::vector<int> &_state );
  bool isValidCell( const int _x, const int _y, const int _z );

  // Visualization
  void visualizePath( boost::shared_ptr<pcl::visualization::PCLVisualizer> _viewer,
		      std::vector<std::vector<double> > _path,
		      bool _viewObstacles = false );

 private:

  int mDimX;
  int mDimY;
  int mDimZ;
  
  int mRadius;
  double mRadiusM;
  
  std::vector< std::vector<int> > mGoal;
  
  int mCost;
  int mCost1Move;
  int mCost2Move;
  int mCost3Move;

  PFDistanceField* mDf;
  
  int mDistLength;
  std::vector<int> mDist;
};

////////////////// INLINE FUNCTIONS ////////////////////

/**
 * @function xyzToIndex
 */
inline int DiversePaths::xyzToIndex( int _x, int _y, int _z ) {
  int ret = _x + _y*mDimX + _z*mDimX*mDimY;

  if( ret < mDistLength ) {
    return ret;
  }
  else {
    printf( "[DP- xyzToIndex] Out of bounds (%d, %d, %d) (index: %d size: %d) \n", _x, _y, _z, ret, mDistLength );
    return 0;
  }
}

#endif /** __DIVERSE_PATHS_H__ */
