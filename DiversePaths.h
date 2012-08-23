/**
 * @file DiversePaths.h
 * @author A. Huaman Q. 
 */

#ifndef __DIVERSE_PATHS_H__
#define __DIVERSE_PATHS_H__

#include <boost/thread/thread.hpp>
#include "PCL_Tools/PCL_Tools.h"

#include <distance_field/pf_distance_field.h>
#include <bfs_3d.h>

/**
 * @class DiversePaths
 * @brief
 * @author A. Huaman
 * @date 2012/08/21
 */
class DiversePaths {

 public:
  DiversePaths( PFDistanceField *_df, 
		int _radius,
		int _cost_per_cell );
  ~DiversePaths();
  bool Search( double _sx, 
	       double _sy, 
	       double _sz,
	       double _gx, 
	       double _gy, 
	       double _gz,
	       int _numPaths,
	       double _alpha,
	       std::vector<std::vector<std::vector<double> > > &_paths );

  void visualizePaths( boost::shared_ptr<pcl::visualization::PCLVisualizer> _viewer,
		       bool _viewObstacles = false );
  

 private:

  void InitSearch();

  std::vector<std::vector<std::vector<double> > > mPaths;

  PFDistanceField* mDf;
  int mRadius;
  int mCost;
  BFS3D* mBfs;

  int mNumPaths;
  double mAlpha;

  
};


#endif /** __DIVERSE_PATHS_H__ */
