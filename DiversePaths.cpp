/**
 * @file DiversePaths.cpp
 * @brief A. Huaman Quispe
 * @date 2012/08/21
 */
#include "DiversePaths.h"
#include <ctime>

/**
 * @function DiversePaths
 * @brief Constructor
 */
DiversePaths::DiversePaths( PFDistanceField *_df, 
			    int _radius,
			    int _cost_per_cell ) {

  mDf = _df;
  mRadius = _radius;
  mCost = _cost_per_cell;

  mDimX = mDf->getNumCells(PFDistanceField::DIM_X);
  mDimY = mDf->getNumCells(PFDistanceField::DIM_Y);
  mDimZ = mDf->getNumCells(PFDistanceField::DIM_Z);

  mCost1Move = mCost;
  mCost2Move = mCost*sqrt(2.0);
  mCost3Move = mCost*sqrt(3.0);

  mRadiusM = double( mRadius*(mDf->getResolution(PFDistanceField::DIM_X))  );
}


/**
 * @function ~DiversePaths
 * @brief Destructor
 */
DiversePaths::~DiversePaths() {

}

/**
 * @function setGoal
 * @brief Set world goal
 */
bool DiversePaths::setGoal( std::vector<double> _goal ) {

  if( _goal.empty() || _goal.size() < 3 ) {
    return false; 
  }
  
  std::vector<int> goal(3);  
  mDf->worldToGrid( _goal[0], _goal[1], _goal[2],
		    goal[0], goal[1], goal[2] );
  
  return setGoal( goal );
}

/**
 * @function setGoal
 * @brief set cell goal
 */
bool DiversePaths::setGoal( std::vector<int> _goal ) {
  
  if( _goal.empty() || _goal.size() < 3 ) {
    return false;
  }

  mGoal.clear();
  
  if( _goal[0] < mDimX && _goal[1] < mDimY && _goal[2] < mDimZ ) {
    if( isValidCell( _goal[0], _goal[1], _goal[2] ) ) {
      mGoal.push_back( _goal );
    } else {
      printf( " [setGoal] Goal no valid. Exiting! \n" );
    }
  }

  if( mGoal.empty() ) {
    printf(" [setGoal] Error: No valid goals were received \n");
    return false;
  }

  return true;
}

/**
 * @function setGoals
 * @brief
 */
bool DiversePaths::setGoals( std::vector<std::vector<int> > _goals ) {

  if( _goals.size() <= 0 ) {
    printf( "[setGoals] No goal cell received. Exiting! \n" );
    return false;
  }
  
  mGoal.clear();
  
  // Check validity of goals
  for( unsigned int i = 0; i < _goals.size(); ++ i ) {

    if( _goals[i].size() < 3 ) {
      continue;
    }

    if( _goals[i][0] < mDimX && _goals[i][1] < mDimY && _goals[i][2] < mDimZ ) {
      mGoal.push_back( _goals[i] );
    } else {
      printf( " [setGoals] Goal: %d %d %d is invalid \n", _goals[i][0], _goals[i][1], _goals[i][2] );
    }
  }

  if( mGoal.empty() ) {
    printf(" Error: No valid goals were received \n" );
    return false;
  }

  return true;

}

/**
 * @function runDijkstra
 * @brief
 */
bool DiversePaths::runDijkstra() {

  if( mGoal.empty() ) {
    printf( " [runDijkstra] Goal location is not set. Exiting \n" );
    return false;
  }

  printf("Running Dijkstra with goal: %d %d %d \n", mGoal[0][0], mGoal[0][1], mGoal[0][2]);

  mDistLength = (mDimX - 1) + (mDimY - 1)*(mDimX) + (mDimZ - 1)*(mDimX)*(mDimY) + 1;
  mDist.resize( mDistLength );
  State3D*** stateSpace3D;
  create3DStateSpace( &stateSpace3D );
  searchOneSourceAllPaths( stateSpace3D );
  delete3DStateSpace( &stateSpace3D );

  return true;
}

/**
 * @function searchOneSourceAllPaths
 */
void DiversePaths::searchOneSourceAllPaths( State3D*** _stateSpace ) {
  
  State3D* u;
  int newx; int newy; int newz;
  int x; int y; int z;
  unsigned int g_temp;

  // these are added here temporarily. should be in the class
  int dx[DIRECTIONS3D] = { 1,  1,  1,  0,  0,  0, -1, -1, -1,    1,  1,  1,  0,  0, -1, -1, -1,    1,  1,  1,  0,  0,  0, -1, -1, -1};
  int dy[DIRECTIONS3D] = { 1,  0, -1,  1,  0, -1, -1,  0,  1,    1,  0, -1,  1, -1, -1,  0,  1,    1,  0, -1,  1,  0, -1, -1,  0,  1};
  int dz[DIRECTIONS3D] = {-1, -1, -1, -1, -1, -1, -1, -1, -1,    0,  0,  0,  0,  0,  0,  0,  0,    1,  1,  1,  1,  1,  1,  1,  1,  1};
  
  //-- Create a queue
  std::queue< State3D* > mQ;

  //-- Initialize all states to INF
  for( int x = 0; x < mDimX; ++x ) {
    for( int y = 0; y < mDimY; ++y ) {
      for( int z = 0; z < mDimZ; ++z ) {
	mDist[ xyzToIndex( x, y, z )  ] = INFINITE_COST;
	reInitializeState3D( &_stateSpace[x][y][z] );
      }
    }
  }
  
  //-- Initialize goals to zero cost
  for( unsigned int i = 0; i < mGoal.size(); ++i ) {
    _stateSpace[ mGoal[i][0] ][ mGoal[i][1] ][ mGoal[i][2] ].g = 0;
    mQ.push( &_stateSpace[ mGoal[i][0] ][ mGoal[i][1] ][ mGoal[i][2] ] );
  }

  //-- Expand all of the states
  while( (int) mQ.size() > 0 ) {
    
    // Get the state to expand
    u = mQ.front();

    mQ.pop();

    // If the state is already closed
    if( u->iterationclosed == 1 ) {
      continue;
    }

    // Mark it as closed
    u->iterationclosed = 1;

    // Set the  corresponding distances to the goal
    mDist[ xyzToIndex( u->x, u->y, u->z ) ] = u->g;

    // Iterate through neighbors
    for( int d = 0; d < DIRECTIONS3D; ++d ) {
      newx = u->x + dx[d];
      newy = u->y + dy[d];
      newz = u->z + dz[d];
      
      // Check neighbor is no obstacle and inside the map
      if( newx < 0 || newx >= mDimX ||
	  newy < 0 || newy >= mDimY ||
	  newz < 0 || newz >= mDimZ ) {
	continue;
      }
      if( !isValidCell( newx, newy, newz ) ) {
	continue;
      }
     
      if( _stateSpace[newx][newy][newz].iterationclosed == 0 ) {
	// Insert into the stack
	mQ.push( &_stateSpace[newx][newy][newz] );
	// Set the g-value
	if( u->x != newx && u->y != newy && u->z != newz ) {
	  g_temp = u->g + mCost3Move;
	}
	else if( ( u->y != newy && u->z != newz ) ||
		 ( u->x != newx && u->z != newz ) ||
		 ( u->x != newx && u->y != newy ) ){
	  g_temp = u->g + mCost2Move;
	}
	else {
	  g_temp = u->g + mCost1Move;
	}

	if( _stateSpace[newx][newy][newz].g > g_temp ) {
	  _stateSpace[newx][newy][newz].g = g_temp;
	}
      }
 
    } // end for DIRECTIONS3D

  } // end while

}



/**
 * @function getShortestPaths
 * @brief
 */
bool DiversePaths::getShortestPath( std::vector<int> _start,
				    std::vector< std::vector<int> > &_path ) {

  int val = 0;
  int counter = 0;
  int min_val = INFINITE_COST;

  std::vector<int> state(3,0);
  std::vector<int> next_state(3,0);
  int newx; int newy; int newz;

  // Make sure the while loop eventually stops
  int max_path_length = mDimX*mDimY;

  int dx[DIRECTIONS3D] = { 1,  1,  1,  0,  0,  0, -1, -1, -1,    1,  1,  1,  0,  0, -1, -1, -1,    1,  1,  1,  0,  0,  0, -1, -1, -1};
  int dy[DIRECTIONS3D] = { 1,  0, -1,  1,  0, -1, -1,  0,  1,    1,  0, -1,  1, -1, -1,  0,  1,    1,  0, -1,  1,  0, -1, -1,  0,  1};
  int dz[DIRECTIONS3D] = {-1, -1, -1, -1, -1, -1, -1, -1, -1,    0,  0,  0,  0,  0,  0,  0,  0,    1,  1,  1,  1,  1,  1,  1,  1,  1};


  _path.resize(0);
  next_state[0] = _start[0];
  next_state[1] = _start[1];
  next_state[2] = _start[2];

  while( !isGoal( next_state ) || counter > max_path_length ) {
    state = next_state;
    min_val = INFINITE_COST;

    // Iterate through neighbors
    for( int d = 0; d < DIRECTIONS3D; ++d ) {
      newx = state[0] + dx[d];
      newy = state[1] + dy[d];
      newz = state[2] + dz[d];

      // Check cell is inside the map and with no obstacles
      if( newx < 0 || newx >= mDimX ||
	  newy < 0 || newy >= mDimY ||
	  newz < 0 || newz >= mDimZ ) {
	continue;
      }

      val = mDist[ xyzToIndex( newx, newy, newz ) ];
      if( val >= INFINITE_COST ) {
	continue;
      }

      if( state[0] != newx && state[1] != newy && state[2] != newz ) {
	val = val + mCost3Move;
      }
      else if( ( state[1] != newy && state[2] != newz ) ||
	       ( state[0] != newx && state[2] != newz ) ||
	       ( state[0] != newx && state[1] != newy ) ) {
	val = val + mCost2Move;
      }
      else {
	val = val + mCost1Move;
      }

      if( val < min_val ) {
	min_val = val;
	next_state[0] = newx;
	next_state[1] = newy;
	next_state[2] = newz;
      }
    } // for
    //printf("New state: %d %d %d \n", next_state[0], next_state[1], next_state[2]);
    _path.push_back( next_state );
    counter++;
  } // while

  // Unable to find paths
  if( counter > max_path_length ) {
    printf( "[getShortestPath] Unable to find path to goal. Exiting! \n" );
    _path.clear();
    return false;
  }
  
  return true;
}

/**
 * @function getShortestPath
 * @brief
 */
bool DiversePaths::getShortestPath( std::vector<double> _start,
				    std::vector< std::vector<double> > &_path ) {


  std::vector<int> start(3);
  std::vector<std::vector<int> > cellPath;
  std::vector<double> p(3);

  _path.clear();
  mDf->worldToGrid( _start[0], _start[1], _start[2], start[0], start[1], start[2] );

  if( !getShortestPath( start, cellPath ) ) {
    printf( "[getShortestPath] Did not find a path. Exiting! \n" );
    return false;
  }

  for( int i = 0; i < cellPath.size(); ++i ) {
    mDf->gridToWorld( cellPath[i][0], cellPath[i][1], cellPath[i][2],
		      p[0], p[1], p[2] );
    _path.push_back( p );
  }

  return true;
}

/**
 * @function DiversePaths
 * @brief
 */
void DiversePaths::setRadius( double _r ) {

  mRadiusM = _r;
  mRadius = _r / ( mDf->getResolution( PFDistanceField::DIM_X) ) + 0.5;
}

/**
 * @function DiversePaths
 * @brief
 */
int DiversePaths::getRadiusCells() {
  return int(mRadius);
}

/**
 * @function DiversePaths
 * @brief
 */
void DiversePaths::reInitializeState3D( State3D* _state ) {

  _state->g = INFINITE_COST;
  _state->iterationclosed = 0;
}

/**
 * @function DiversePaths
 * @brief
 */
void DiversePaths::initializeState3D( State3D* _state, int _x, int _y, int _z ) {

  _state->g = INFINITE_COST;
  _state->iterationclosed = 0;
  _state->x = _x;
  _state->y = _y;
  _state->z = _z;

}

/**
 * @function DiversePaths
 * @brief
 */
void DiversePaths::create3DStateSpace( State3D**** _stateSpace3D ) {
  
  *_stateSpace3D = new State3D** [mDimX];

  for( int x = 0; x < mDimX; ++x ) {    
  
    (*_stateSpace3D)[x] = new State3D* [mDimY];
  
    for( int y = 0; y < mDimY; ++y ) {
      
      (*_stateSpace3D)[x][y] = new State3D[mDimZ];
      
      for( int z = 0; z < mDimZ; ++z ) {
	initializeState3D( &(*_stateSpace3D)[x][y][z], x, y, z );
      }
    }
  }
}

/**
 * @function DiversePaths
 * @brief
 */
void DiversePaths::delete3DStateSpace( State3D**** _stateSpace3D ) {

  int x; int y;
  if( (*_stateSpace3D) != NULL ) {

    for( int x = 0; x < mDimX; ++x ) {
      for( int y = 0; y < mDimY; ++y ) {
	delete [] (*_stateSpace3D)[x][y];
      }
      delete [] (* _stateSpace3D)[x];
    }

    delete[] (*_stateSpace3D);
    ( *_stateSpace3D ) = NULL;
    
  } // end of if
  
}


/**
 * @function DiversePaths
 * @brief
 */
bool DiversePaths::isGoal( const std::vector<int> &_state ) {

  for( unsigned int i = 0; i < mGoal.size(); ++i ) {
    if( ( _state[0] <= mGoal[i][0] + GOAL_TOLERANCE && 
	  _state[0] >= mGoal[i][0] - GOAL_TOLERANCE ) &&
	( _state[1] <= mGoal[i][1] + GOAL_TOLERANCE &&
	  _state[1] >= mGoal[i][1] - GOAL_TOLERANCE ) &&
	( _state[2] <= mGoal[i][2] + GOAL_TOLERANCE &&
	  _state[2] >= mGoal[i][2] - GOAL_TOLERANCE ) ) {
      return true;
    }
  }
  
  return false;
}

/**
 * @function DiversePaths
 * @brief
 */
bool DiversePaths::isValidCell( const int _x, const int _y, const int _z ) {

  if( mDf->getDistanceFromCell( _x, _y, _z ) <= mRadiusM ) {
    return false;
  }

  return true;
  
}

/////////////////////////// VISUALIZATION FUNCTIONS //////////////////////////////////////////


/**
 * @function visualizePath
 */
void DiversePaths::visualizePath( boost::shared_ptr<pcl::visualization::PCLVisualizer> _viewer,
				  std::vector<std::vector<double> > _path,
				  bool _viewObstacles ) {


  //-- Get counters ready
  reset_PCL_Tools_counters();
  
  //-- Get obstacles
  std::vector<Eigen::Vector3d> obstacles;
  mDf->getPointsFromField( obstacles );

  //-- Put them in PCD
  pcl::PointCloud<pcl::PointXYZ>::Ptr obstacleCloud = writePCD( obstacles );

  //-- View paths
    viewPath( _path, _viewer );
    viewBall( _path[0][0], _path[0][1], _path[0][2], 0.02, _viewer );
    viewBall( _path[_path.size() - 1][0], 
	      _path[_path.size() - 1][1], 
	      _path[_path.size() - 1][2], 
	      0.02, _viewer );    

  // Visualize obstacles (or not)
  int obsR; int obsG; int obsB;
  obsR = 0; obsG = 255; obsB = 0;

  if( _viewObstacles ) {
    viewPCD( obstacleCloud, _viewer, obsR, obsG, obsB );
  }

}
