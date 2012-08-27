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
  unsigned int f;
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

  // Diverse Paths
  std::vector<std::vector<std::vector<double> > > getDiversePaths( std::vector<double> _start,
								   std::vector<double> _goal,
								   int _numPaths,
								   std::vector<std::vector<double> > &_midPoints );

  // Search functions
  bool setGoal( std::vector<double> _goal );
  bool setGoal( std::vector<int> _goal );
  bool setGoals( std::vector<std::vector<int> > _goals );

  bool runDijkstra( std::vector<double> _goal, 
		    std::vector<int> &_dist );
  void searchOneSourceAllPaths( State3D*** _stateSpace,
				std::vector<int> &_dist );
  bool getShortestPath( std::vector<int> _start,
			std::vector< std::vector<int> > &_path,
			std::vector<int> _dist );
  bool getShortestPath( std::vector<double> _start,
			std::vector<double> _goal,
			std::vector< std::vector<double> > &_path,
			std::vector<int> _dist,
			bool _invert = false );

  bool runAstar(std::vector<double> _start,
		std::vector<double> _goal,
		std::vector<std::vector<double> > &_path );
  std::vector<std::vector<double> > runAstar(std::vector<double> _start );

  bool searchOneToOnePath( std::vector<int> _start,
			   std::vector<int> _goal,
			   std::vector<std::vector<double> > &_path );

  void setRadius( double _r );
  int getRadiusCells();
  void reInitializeState3D( State3D* _state );
  void initializeState3D( State3D* _state, int _x, int _y, int _z );
  void create3DStateSpace( State3D**** _stateSpace3D );
  void delete3DStateSpace( State3D**** _stateSpace3D );
  inline int xyzToIndex( int _x, int _y, int _z );
  bool isGoal( const std::vector<int> &_state );
  bool isGoal( const int &_x, const int &_y, const int &_z );
  bool isValidCell( const int _x, const int _y, const int _z );

  // Heap functions
  int HeuristicCost( int _sx, int _sy, int _sz, 
		     int _gx, int _gy, int _gz );
  void PushOpenSet( State3D* _u );
  State3D* PopOpenSet();
  bool TracePath( std::vector<int> _start,
			      std::vector<int> _goal,
			      std::vector< std::vector<double> > &_path );
  void UpdateLowerOpenSet( State3D* _u ); 

  // Distance Field to paths
  PFDistanceField* createDfToPathSet( std::vector< std::vector<double> > _path  );

  // Distance Field utilities
  void joinPaths( std::vector<std::vector<double> > &_origPath,
		  std::vector<std::vector<double> > _addedPath );

  std::vector<std::vector<double> > getPointsAtLeastAsFarAs( PFDistanceField* _df, double _thresh );

  std::vector<std::vector<double> > getNearestPointFromSet( PFDistanceField* _df,
							    std::vector<std::vector<double> > _set,
							    std::vector<double> &point,
							    double _thresh ); 

  std::vector<std::vector<double> > getPointsAsFarAs( PFDistanceField* _df, 
						      double _thresh, 
						      double _tol = 0.005 );

  std::vector<std::vector<double> > getPointsAsFarAsFromSet( PFDistanceField* _df,
							     std::vector<std::vector<double> > _set,
							     double _thresh, 
							     double _tol );
  
  // Visualization
  void visualizePath( boost::shared_ptr<pcl::visualization::PCLVisualizer> _viewer,
		      std::vector<std::vector<double> > _path,
		      bool _viewObstacles = false,
		      int _r = 0, int _g = 0, int _b = 255 );

  void visualizePaths( boost::shared_ptr<pcl::visualization::PCLVisualizer> _viewer,
		       std::vector<std::vector<std::vector<double> > > _path,
		       bool _viewObstacles = false );

 private:

  static const int NX[];
  static const int NY[];
  static const int NZ[];

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

  // A star variables
  State3D ***mS3D;
  int *mHT;
  std::vector<State3D*> mOpenSet;

  // Diverse paths variables
  int mNumPaths;
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
