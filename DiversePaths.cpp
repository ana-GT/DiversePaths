/**
 * @file DiversePaths.cpp
 * @brief A. Huaman Quispe
 * @date 2012/08/21
 */
#include "DiversePaths.h"
#include <ctime>

const int DiversePaths::NX[DIRECTIONS3D] = {  0, 0,  0, 1, 0, -1,  1,  0, -1,  0, 1, 0, -1,  0,  1, 1, -1, -1,  -1,-1,1, 1,-1,-1, 1, 1 };
const int DiversePaths::NY[DIRECTIONS3D] = {  0, 0, -1, 0, 1,  0,  0,  1,  0, -1, 0, 1,  0, -1, -1, 1,  1, -1,  -1, 1,1,-1,-1, 1, 1,-1 };
const int DiversePaths::NZ[DIRECTIONS3D] = { -1, 1,  0, 0, 0,  0, -1, -1, -1, -1, 1, 1,  1,  1,  0, 0,  0,  0,   1, 1,1, 1,-1,-1,-1,-1 };


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
  printf( " [*] Costs: %d %d %d \n", mCost1Move, mCost2Move, mCost3Move );
  mRadiusM = double( mRadius*(mDf->getResolution(PFDistanceField::DIM_X))  );
}


/**
 * @function ~DiversePaths
 * @brief Destructor
 */
DiversePaths::~DiversePaths() {

}

/**
 * @functino getDiversePaths2
 */
std::vector<std::vector<std::vector<double> > > DiversePaths::getDiversePaths2( std::vector<double> _start,
										std::vector<double> _goal,
										int _numPaths,
										std::vector<std::vector<double> > &_midPoints ) {
  printf("getDiversePaths2 \n");
  mNumPaths = _numPaths;

  std::vector<std::vector<std::vector<double> > > paths;
  std::vector<std::vector<int> > cellPath;
  std::vector<std::vector<double> > jointPaths;
  std::vector<std::vector<int> > cellMidPoints;
   
  int* dGoal;
  int* dStart;

  // Get ready
  _midPoints.resize(0);

  // Get start and goal cells
  std::vector<int> start(3);
  std::vector<int> goal(3);
  mDf->worldToGrid( _start[0], _start[1], _start[2], start[0], start[1], start[2] );
  mDf->worldToGrid( _goal[0], _goal[1], _goal[2], goal[0], goal[1], goal[2] );

  //-- 2. Get Dijkstra
  printf("Run Dijkstra goal \n");
  runDijkstra( _goal, dGoal );

  printf("Run Dijkstra start \n");
  runDijkstra( _start, dStart );

  //-- 1. Get the first path ( cells )
  printf("Get Shortest Path \n");
  getShortestPath( start, goal, cellPath, dGoal, false );

  printf("[0] Saving path of size %d \n", cellPath.size() );
  paths.push_back( getWorldPoints( cellPath ) );
  
  //-- 2. Get the midpoints ( cells )
  cellMidPoints = getMidCells(_start, _goal, dStart, dGoal, cellPath.size() );
  printf( "Initial number of mid points: %d \n", cellMidPoints.size() );

  //-- Find the paths for all the midPoints
  std::vector<std::vector<std::vector<int> > > midPaths;
  std::vector<std::vector<int> > tempPath;
  std::vector<std::vector<int> > tempFirst;
  std::vector<std::vector<int> > tempLast;

  printf( "--> Start cellMidPoint paths \n" );
  time_t ts = clock();
  for( int i = 0; i < cellMidPoints.size(); ++i ) {
   
    getShortestPath( cellMidPoints[i], start, tempFirst, dStart, true );
    getShortestPath( cellMidPoints[i], goal, tempLast, dGoal, false );
  
    // Make them together properly
    tempPath.resize(0);
    tempFirst.pop_back();
    joinPaths( tempPath, tempFirst );
    joinPaths( tempPath, tempLast );

    midPaths.push_back( tempPath );
  }
  time_t tf = clock();
  double dt = (double) (tf - ts) / CLOCKS_PER_SEC;

  printf( "--> End %d cellMidPoint paths: %f only for for  \n", cellMidPoints.size(), dt );

  std::vector<std::vector<int> > notConnectedCells;
  std::vector<int> midPathCell(3);
  
  // Get the remaining paths
  std::vector<int> thePoint;
  std::vector<std::vector<int> > pathSM; // Start - Middle
  std::vector<std::vector<int> > pathMG; // Middle - Goal

  for( int i = 1; i < mNumPaths; ++i ) {

    // Get the latest middle point
    midPathCell = cellPath[ cellPath.size() / 2 ];
    cellMidPoints = getNoFreeLineCells( cellMidPoints, midPathCell );
    printf("[%d] Trimmed Mid Points: %d \n", i, cellMidPoints.size() );  

    // Get a new middle point
    thePoint = cellMidPoints[ cellMidPoints.size() / 2 ];

    //-- Find path with thePoint in the middle
    pathSM.resize(0);  pathMG.resize(0);

    getShortestPath( thePoint, start, pathSM, dStart, true );
    getShortestPath( thePoint, goal, pathMG, dGoal, false );
  
    // Make them together properly
    cellPath.resize(0);
    pathSM.pop_back();
    joinPaths( cellPath, pathSM );
    joinPaths( cellPath, pathMG );
    
    paths.push_back( getWorldPoints(cellPath) );
  }

  for( int i = 0; i < mNumPaths; ++i ) {
    _midPoints.push_back( paths[i][ paths[i].size() / 2 ] );
  }

  return paths;
}

/**
 * @function getNoFreeLineCells
 */
std::vector<std::vector<int> > DiversePaths::getNoFreeLineCells( std::vector<std::vector<int> > _points, 
								 std::vector<int> _evalPoint ) {

  std::vector<std::vector<int> > noFree;
  std::vector<std::vector<int> > testLine;

  for( int i = 0; i < _points.size(); ++i ) {
    testLine = getLine( _points[i][0], _points[i][1], _points[i][2],
			_evalPoint[0], _evalPoint[1], _evalPoint[2] );
    for( int j = 0; j < testLine.size(); ++j ) {
      if( !isValidCell( testLine[j][0], testLine[j][1], testLine[j][2] ) ) {
	noFree.push_back( _points[i] );
	break;
      }
    }
  }
  return noFree;
}

/**
 * @function getDiversePaths
 */
std::vector<std::vector<std::vector<double> > > DiversePaths::getDiversePaths( std::vector<double> _start,
									       std::vector<double> _goal,
									       int _numPaths,
									       std::vector<std::vector<double> > &_midPoints ) {
  mNumPaths = _numPaths;
  std::vector<std::vector<std::vector<double> > > paths;
  std::vector<std::vector<double> > path;
  std::vector<std::vector<double> > jointPaths;

  int* dGoal;
  int* dStart;

  // Get ready
  _midPoints.resize(0);

  //-- 2. Get Dijkstra
  runDijkstra( _goal, dGoal );
  runDijkstra( _start, dStart );

  //-- 1. Get the first path
  getShortestPath( _start, _goal, path, dGoal );
  // Save it
  printf("[0] Saving path of size %d \n", path.size() );
  paths.push_back( path );

  // Join it
  joinPaths( jointPaths, path );


  std::vector<std::vector<double> > pointsPath;
  std::vector<double> thePoint;

  //-- Repeat this:
  for( int i = 1; i < mNumPaths; ++i ) {

    //-- 2. Create DF to joint paths
    PFDistanceField* dpp;
    dpp = createDfToPathSet( jointPaths );

    //-- 3. Get points that are both far from obstacles and paths
    std::vector<std::vector<double> > pointsObst;
    double obstDist = 0.1;
    double obstTol = 0.005;
    //pointsObst = getPointsAtLeastAsFarAs( mDf, obstDist );
    pointsObst = getPointsAsFarAs( mDf, obstDist, obstTol );

    double pathDist = 0.2;
    double pathTol = 0.005;
    pointsPath = getNearestPointFromSet( dpp, pointsObst, thePoint, pathDist );
    //pointsPath = getPointsAsFarAsFromSet( dpp, pointsObst, pathDist, pathTol );    

    thePoint = pointsPath[ pointsPath.size() / 2  ];

    //-- 4. Find path with thePoint in the middle
    std::vector<std::vector<double> > pathSM; // Start - Middle
    std::vector<std::vector<double> > pathMG; // Middle - Goal
    getShortestPath( thePoint, _start, pathSM, dStart, true );
    getShortestPath( thePoint, _goal, pathMG, dGoal, false );

    // Save thePoint
    _midPoints.push_back( thePoint );
    
    // Make them together properly
    path.resize(0);
    pathSM.pop_back();
    joinPaths( path, pathSM );
    joinPaths( path, pathMG );

    // Join with previous path
    joinPaths( jointPaths, path );
    
    // Save it
    paths.push_back( path );
    printf(" [%d] Adding a path of %d \n points \n", i, path.size() );
  }

  /////////// PUT VISUALIZATION INSIDE TO DEBUG ///////////////////////////
  // Create viewer
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = createViewer();

  // View the path
  reset_PCL_Tools_counters();
  visualizePaths( viewer, paths, true );

  // View balls midpoints
  for( int i = 0; i < _midPoints.size(); ++i ) {
    viewBall( _midPoints[i][0], _midPoints[i][1], _midPoints[i][2],
	      0.025, viewer, 0, 0, 255 );
  }

  // Visualize points obst + paths away
  viewPoints( pointsPath, viewer, 255, 0 , 255 ); 

  // Loop
  while( !viewer->wasStopped() ) {
    viewer->spinOnce(100);
    boost::this_thread::sleep( boost::posix_time::microseconds(100000));
    }

  //////////// END VISUALIZATION INSIDE /////////////////////////////////////
  return paths;
}

/**
 * @function getMidPoints
 */
std::vector<std::vector<double> > DiversePaths::getMidPoints( std::vector<double> _start,
							      std::vector<double> _goal,
							      int* _dStart,
							      int* _dGoal,
							      int _length ) {
  
  std::vector<std::vector<int> > cellMidPoints;
  std::vector<std::vector<double> > midPoints;
  std::vector<double> p(3);

  cellMidPoints = getMidCells( _start, _goal, _dStart, _dGoal, _length );

  for( int i = 0; i < cellMidPoints.size(); ++i ) {
    mDf->gridToWorld( cellMidPoints[i][0], cellMidPoints[i][1], cellMidPoints[i][2], p[0], p[1], p[2] );
    midPoints.push_back( p );    
  }

  return midPoints;
}


/**
 * @function getMidCells
 */
std::vector<std::vector<int> > DiversePaths::getMidCells( std::vector<double> _start,
							  std::vector<double> _goal,
							  int* _dStart,
							  int* _dGoal,
							  int _length ) {
  
  std::vector<std::vector<int> > midPoints;
  std::vector<int> p(3);

  for( int x = 0; x < mDimX; ++x ) {
    for( int y = 0; y < mDimY; ++y ) {
      for( int z = 0; z < mDimZ; ++z ) {

	int ind = xyzToIndex( x, y, z );
	if( _dStart[ind] != INFINITE_COST ) {
	  if( _dGoal[ind] != INFINITE_COST ) {	    
	    int ds = _dStart[ind];
	    int dg = _dGoal[ind];
	    if( ds < dg + 2*mCost1Move && 
		ds > dg - 2*mCost1Move && 
		ds + dg < (2*_length*mCost1Move) ) {
	      p[0] = x; p[1] = y; p[2] = z;
	      midPoints.push_back( p );
	    }
	  } // if _dGoal[ind] != INF
	} // if _dStart[ind]  != INF

      } // for z
    } // for y
  } // for x

  return midPoints;
}


/**
 * @function runDijkstra
 * @brief
 */
bool DiversePaths::runDijkstra( std::vector<double> _goal,
				int* &_dist ) {

  if( !setGoal(_goal) ) {
    return false;
  }

  mDistLength = (mDimX - 1) + (mDimY - 1)*(mDimX) + (mDimZ - 1)*(mDimX)*(mDimY) + 1;

  _dist = new int[mDistLength];

  State3D*** stateSpace3D;
  create3DStateSpace( &stateSpace3D );
  searchOneSourceAllPaths( stateSpace3D, _dist );
  delete3DStateSpace( &stateSpace3D );

  return true;
}

/**
 * @function searchOneSourceAllPaths
 */
void DiversePaths::searchOneSourceAllPaths( State3D*** _stateSpace,
					    int* _dist ) {
  
  State3D* u;
  int newx; int newy; int newz;
  int x; int y; int z;
  unsigned int g_temp;
  
  //-- Create a queue
  std::queue< State3D* > mQ;

  //-- Initialize all states to INF
  for( int x = 0; x < mDimX; ++x ) {
    for( int y = 0; y < mDimY; ++y ) {
      for( int z = 0; z < mDimZ; ++z ) {
	_dist[ xyzToIndex( x, y, z )  ] = INFINITE_COST;
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
    _dist[ xyzToIndex( u->x, u->y, u->z ) ] = u->g;

    // Iterate through neighbors
    for( int d = 0; d < DIRECTIONS3D; ++d ) {
      newx = u->x + NX[d];
      newy = u->y + NY[d];
      newz = u->z + NZ[d];
      
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
 * @function getShortestPath
 * @brief
 */
bool DiversePaths::getShortestPath( std::vector<double> _start,
				    std::vector<double> _goal,
				    std::vector< std::vector<double> > &_path,
				    int* _dist,
				    bool _invert ) {

  if( !setGoal( _goal ) ) {
    return false;
  }

  std::vector<int> start(3);
  std::vector<int> goal(3);

  std::vector<std::vector<int> > cellPath;
  std::vector<double> p(3);

  _path.clear();

  mDf->worldToGrid( _start[0], _start[1], _start[2], start[0], start[1], start[2] );
  mDf->worldToGrid( _goal[0], _goal[1], _goal[2], goal[0], goal[1], goal[2] );

  if( !getShortestPath( start, goal, cellPath, _dist, _invert ) ) {
    printf( "[getShortestPath] Did not find a path. Exiting! \n" );
    return false;
  }

  // Pass to world coordinates
  for( int i = 0; i < cellPath.size(); ++i ) {
    mDf->gridToWorld( cellPath[i][0], cellPath[i][1], cellPath[i][2],
		      p[0], p[1], p[2] );
    _path.push_back( p );
  }

  return true;
}

/**
 * @function getShortestPath
 */
bool DiversePaths::getShortestPath( std::vector<double> _start,
				    std::vector<double> _goal,
				    std::vector<std::vector<int> > &_path,
				    int* _dist,
				    bool _invert ) {

  if( !setGoal( _goal ) ) {
    return false;
  }

  std::vector<int> start(3);
  std::vector<int> goal(3);
  _path.clear();

  mDf->worldToGrid( _start[0], _start[1], _start[2], start[0], start[1], start[2] );
  mDf->worldToGrid( _goal[0], _goal[1], _goal[2], goal[0], goal[1], goal[2] );

  if( !getShortestPath( start, goal,  _path, _dist, _invert ) ) {
    printf( "[getShortestPath] Did not find a path. Exiting! \n" );
    return false;
  }

  return true;
}


/**
 * @function getShortestPaths
 * @brief
 */
bool DiversePaths::getShortestPath( std::vector<int> _start,
				    std::vector<int> _goal,
				    std::vector< std::vector<int> > &_path,
				    int* _dist,
				    bool _invert ) {

  // Set goal - important
  if( !setGoal( _goal ) ) {
    return false;
  }

  int val = 0;
  int counter = 0;
  int min_val = INFINITE_COST;

  std::vector<int> state(3);
  std::vector<int> next_state(3);
  std::vector<std::vector<int> > temp_path;
  int newx; int newy; int newz;

  // Make sure the while loop eventually stops
  int max_path_length = mDimX*mDimY;

  temp_path.resize(0);
  next_state = _start;

  // Push first element
  temp_path.push_back( next_state );

  // Iterate until you get to the goal
  int diff;
  int dx; int dy; int dz;
  const int* dpx; const int* dpy; const int* dpz;
  int* pVal;
  int min_ind;

  while( !isGoal( next_state ) && counter < max_path_length ) {
    state = next_state;
    min_val = INFINITE_COST;
    min_ind = 0;
    // Iterate through neighbors
    pVal = & _dist[ xyzToIndex( state[0], state[1], state[2] ) ];

    dpx = &NX[0]; dpy = &NY[0]; dpz = &NZ[0];
    for( int d = 0; d < DIRECTIONS3D; ++d ) {
      dx = *dpx; dy = *dpy; dz = *dpz;
      newx = state[0] + dx;
      newy = state[1] + dy;
      newz = state[2] + dz;

      dpx++; dpy++; dpz++;

      // Check cell is inside the map and with no obstacles
      if( newx < 0 || newx >= mDimX ||
	  newy < 0 || newy >= mDimY ||
	  newz < 0 || newz >= mDimZ ) {
	continue;
      }

      //val = _dist[ xyzToIndex( newx, newy, newz ) ];
      val = *( pVal + dx + dy*mDimX + dz*mDimX*mDimY );

      if( val >= INFINITE_COST ) {
	continue;
      }
      diff = abs( dx ) + abs( dy ) + abs( dz );
      if( diff == 1 ) {
	val += mCost1Move;
      }
      else if( diff == 2 ) {
	val += mCost2Move;
      }
      else {
	val += mCost3Move;
      }
      

      if( val < min_val ) {
	min_val = val;
	min_ind = d;
      }
    } // end for


    next_state[0] = state[0] + NX[min_ind];
    next_state[1] = state[1] + NY[min_ind];
    next_state[2] = state[2] + NZ[min_ind];

    temp_path.push_back( next_state );
    counter++;
  } // end while

  // Unable to find paths
  if( counter > max_path_length ) {
    printf( "[getShortestPath] Unable to find path to goal. Exiting! \n" );
    _path.clear();
    return false;
  }
  
  // If it found a path, check if it should be inverted
  int n = temp_path.size();
  _path.resize(0);

  if( _invert == true ) {
    for( int i = 0; i < n; ++i ) {
      _path.push_back( temp_path[n-1-i] );
    }
  }
  else {
    _path = temp_path;
  }

  return true;
}

//////////////////////// A STAR SECTION /////////////////////////////


/**
 * @function runAstar
 */
bool DiversePaths::runAstar(std::vector<double> _start, 
			    std::vector<double> _goal,
			    std::vector<std::vector<double> > &_path ) {

  // Check start
  std::vector<int> start(3);
  if( _start.size() < 3 ) {
    printf( "[runAstar] Error in start, need 3-element vector \n " );
    return false;
  }

  if( !mDf->worldToGrid( _start[0], _start[1], _start[2], start[0], start[1], start[2] ) ) {
    printf( "[runAstar] No valid start position  \n" );
  }

  // Check goal
  std::vector<int> goal(3);
  if( _goal.size() < 3 ) {
    printf( "[runAstar] Error in goal, need 3-element vector \n " );
    return false;
  }

  if( !mDf->worldToGrid( _goal[0], _goal[1], _goal[2], goal[0], goal[1], goal[2] ) ) {
    printf( "[runAstar] No valid goal position  \n" );
  }

  // Run
  bool b;

  mDistLength = (mDimX - 1) + (mDimY - 1)*(mDimX) + (mDimZ - 1)*(mDimX)*(mDimY) + 1;

  // Start configuration
  mHT = new int[mDistLength];
  std::fill( mHT, mHT + mDistLength, -1 );
  mOpenSet.resize(0);

  create3DStateSpace( &mS3D );

  // Search
  b = searchOneToOnePath( start, goal, _path );

  // Cleanup before going out
  delete3DStateSpace( &mS3D );
  delete [] mHT;
  mOpenSet.resize(0);

  return b;
}

/**
 * @function searchOneToOnePath
 */
bool DiversePaths::searchOneToOnePath( std::vector<int> _start,
				       std::vector<int> _goal,
				       std::vector<std::vector<double> > &_path ) {
  
  _path.resize(0);
  State3D* u;
  int newx; int newy; int newz;
  int x; int y; int z;
  unsigned int g_temp;


  //-- Initialize all states to INF and Not Visited
  //-- 0: No visited 1: OpenSet 2: Closed
  for( int x = 0; x < mDimX; ++x ) {
    for( int y = 0; y < mDimY; ++y ) {
      for( int z = 0; z < mDimZ; ++z ) {
	reInitializeState3D( &mS3D[x][y][z] );
      }
    }
  }

  //-- Initialize start to zero cost
  u = &mS3D[_start[0]][_start[1]][_start[2]];
  u->g = 0;
  u->f = HeuristicCost( _start[0], _start[1], _start[2], _goal[0], _goal[1], _goal[2] );

  //-- Push start to heap
  PushOpenSet( u );

  //-- Loop
  int count = 0;
  int maxIter = mDimX*mDimY*mDimZ;

  State3D* v;

  while( count < maxIter ) {
    // Remove top node in OpenSet
    u = PopOpenSet();
    if( u == NULL ) {
      printf( " [searchOneToOnePath] Error PopOpenSet, exiting! \n" );
      return false;
    }
    //-- Check if it is the goal
    if( u->x == _goal[0] && u->y == _goal[1] && u->z == _goal[2] ) {
      TracePath( _start, _goal, _path );
      break;
    }
    
    // Add node to closed set
    u->iterationclosed = 2;
    
    // Iterate through neighbors
    for( int d = 0; d < DIRECTIONS3D; ++d ) {
      newx = u->x + NX[d];
      newy = u->y + NY[d];
      newz = u->z + NZ[d];
      
      // Check neighbor is no obstacle and inside the map
      if( newx < 0 || newx >= mDimX ||
	  newy < 0 || newy >= mDimY ||
	  newz < 0 || newz >= mDimZ ) {
	continue;
      }
      if( !isValidCell( newx, newy, newz ) ) {
	continue;
      }

     
      v = &mS3D[newx][newy][newz];

      // If it is in closed list go on
      if( v->iterationclosed == 2 ) {
	continue;
      }

      // Get tentative value
      if( u->x != v->x && u->y != v->y && u->z != v->z ) {
	g_temp = u->g + mCost3Move;
      }
      else if( ( u->y != v->y && u->z != v->z ) ||
	       ( u->x != v->x && u->z != v->z ) ||
	       ( u->x != v->x && u->y != v->y ) ){
	g_temp = u->g + mCost2Move;
      }
      else {
	g_temp = u->g + mCost1Move;
      }
      
      // If v is not in OpenSet (it is not visited yet)
      if( v->iterationclosed == 0 ) {
	// Insert into the heap
	v->g = g_temp;
	v->f = g_temp + HeuristicCost( v->x, v->y, v->z, _goal[0], _goal[1], _goal[2] );
	PushOpenSet( v );
      }
      // If it is in OpenSet
      else if( v->iterationclosed == 1 ) {
	if( v->g > g_temp ) {
	  v->g = g_temp;
	  v->f = g_temp + HeuristicCost( v->x, v->y, v->z, _goal[0], _goal[1], _goal[2] );
	  UpdateLowerOpenSet( v );
	}	
      }

    } // end for neighbors
    
    count++;
  } // end for while
  
  return true;
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
    printf(" [setGoals] Error: No valid goals were received \n" );
    return false;
  }

  return true;

}


/**
 * @function HeuristicCost
 */
int DiversePaths::HeuristicCost( int _sx, int _sy, int _sz, 
				 int _gx, int _gy, int _gz ) {

  return (int) ( sqrt( (_sx - _gx)*(_sx - _gx) + 
		       (_sy - _gy)*(_sy - _gy) + 
		       (_sz - _gz)*(_sz - _gz) ) )*mCost1Move;  
}

/**
 * @function PushOpenSet
 */
void DiversePaths::PushOpenSet( State3D* _u ) {

  int n;
  int node; int parent;
  int temp;

  //-- Sign the flag
  _u->iterationclosed = 1;

  //-- Add
  mOpenSet.push_back(_u );
  n = mOpenSet.size() - 1;

  mHT[ xyzToIndex(_u->x, _u->y, _u->z) ] = n;  
  // If this is the first element added
  if( n == 0 ) {
    return;
  }

  // If not, start on the bottom and go up
  node = n;
  State3D* qp; 
  State3D* qn;
  State3D* qtemp;

  while( node != 0 ) {

    parent = floor( (node - 1) / 2 );
    qp = mOpenSet[parent];
    qn = mOpenSet[node];
    // Always try to put new nodes up
    if( qp->f > qn->f ) {
      qtemp = qp;
      mOpenSet[parent] = qn; mHT[ xyzToIndex(qn->x, qn->y, qn->z) ] = parent;
      mOpenSet[node] = qtemp; mHT[ xyzToIndex(qtemp->x, qtemp->y, qtemp->z) ] = node;
      node = parent;
    }
    else {
      break;
    }
  }

}

/**
 * @function setRadius
 * @brief
 */
void DiversePaths::setRadius( double _r ) {

  mRadiusM = _r;
  mRadius = _r / ( mDf->getResolution( PFDistanceField::DIM_X) ) + 0.5;
}

/**
 * @function getRadiusCells
 * @brief
 */
int DiversePaths::getRadiusCells() {
  return int(mRadius);
}

/**
 * @function reInitializeState3D
 * @brief
 */
void DiversePaths::reInitializeState3D( State3D* _state ) {

  _state->g = INFINITE_COST;
  _state->iterationclosed = 0;
}

/**
 * @function initializeState3D
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
 * @function create3DStateSpace
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
 * @function isGoal
 */
bool DiversePaths::isGoal( const int &_x, const int &_y, const int &_z ) {

  std::vector<int> goal(3);
  goal[0] = _x; goal[1] = _y; goal[2] = _z;
  return isGoal( goal );
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

////////////////////////////// HEAP UTILITIES /////////////////////////////////////////////////
/**
 * @function PopOpenSet
 */
State3D* DiversePaths::PopOpenSet() {

  State3D* first; 
  int bottom;

  State3D* v;
  int node; int u;
  int child_1; int child_2;
  int n;
  int temp;

  if( mOpenSet.size() == 0 ) {
    return NULL;
  }

  // Save the pop out element
  first = mOpenSet[0];

  // Reorder your binary heap
  bottom = mOpenSet.size() - 1;
  v = mOpenSet[bottom];
  mOpenSet[0] = v; mHT[ xyzToIndex(v->x, v->y, v->z) ] = 0;
  mOpenSet.pop_back();
  n = mOpenSet.size();

  u = 0;
  State3D* qu; State3D* qtemp;

  while( true ) {
    node = u;
    child_1 = 2*node + 1;
    child_2 = 2*node + 2;

    if( child_2 < n ) {
      if( mOpenSet[node]->f > mOpenSet[child_1]->f ) {
	u = child_1;
      }
      if( mOpenSet[u]->f > mOpenSet[child_2]->f ) {
	u = child_2;
      }
    }
    else if( child_1 < n ) {
      if( mOpenSet[node]->f > mOpenSet[child_1]->f ) {
	u = child_1;
      }
    }
    
    qu = mOpenSet[u];
    if( node != u ) {
      qtemp = mOpenSet[node];
      mOpenSet[node] = qu; mHT[xyzToIndex(qu->x, qu->y, qu->z)] = node;
      mOpenSet[u] = qtemp; mHT[ xyzToIndex(qtemp->x, qtemp->y, qtemp->z)] = u;
    }

    else {
      break;
    }

  }
  return first;

}

/**
 * @function TracePath
 */
bool DiversePaths::TracePath( std::vector<int> _start,
			      std::vector<int> _goal,
			      std::vector< std::vector<double> > &_path ) {

  int val = 0;
  int counter = 0;
  int min_val = INFINITE_COST;

  std::vector<int> state(3,0);
  std::vector<int> next_state(3,0);
  int newx; int newy; int newz;

  std::vector<std::vector<double> > backPath;

  // Make sure the while loop eventually stops
  int max_path_length = mDimX*mDimY;
  
  _path.resize(0);
  backPath.resize(0);
  next_state[0] = _goal[0];
  next_state[1] = _goal[1];
  next_state[2] = _goal[2];

  // Push the very first
  std::vector<double> p(3);
  mDf->gridToWorld( next_state[0], next_state[1], next_state[2], p[0], p[1], p[2] );
  backPath.push_back( p );
  
  while(  (next_state[0] != _start[0] || next_state[1] != _start[1] || next_state[2] != _start[2] )
	  && counter < max_path_length ) {
    state = next_state;
    min_val = INFINITE_COST;

    // Iterate through neighbors
    for( int d = 0; d < DIRECTIONS3D; ++d ) {
      newx = state[0] + NX[d];
      newy = state[1] + NY[d];
      newz = state[2] + NZ[d];

      // Check cell is inside the map and with no obstacles
      if( newx < 0 || newx >= mDimX ||
	  newy < 0 || newy >= mDimY ||
	  newz < 0 || newz >= mDimZ ) {
	continue;
      }

      val = mS3D[newx][newy][newz].g;
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

      if( val < min_val ) {  // Added equal sign
	min_val = val;
	next_state[0] = newx;
	next_state[1] = newy;
	next_state[2] = newz;
      }
    } // for
    mDf->gridToWorld( next_state[0], next_state[1], next_state[2], p[0], p[1], p[2] );
    backPath.push_back( p );
    counter++;
  } // while

  // Unable to find paths
  if( counter > max_path_length ) {
    printf( "[getShortestPath] Unable to find path to goal. Exiting! \n" );
    _path.clear();
    return false;
  }

  // Put path in correct order
  int n = backPath.size();
  for( int i = 0; i < n; ++i ) {
    _path.push_back( backPath[n - 1 - i] );
  }

  return true;


}

/**
 * @function UpdateLowerOpenSet
 */
void DiversePaths::UpdateLowerOpenSet( State3D* _u ) {
  
  int n; 
  int node; int parent;
  int temp;

  //-- Find your guy
  n = mHT[ xyzToIndex(_u->x, _u->y, _u->z) ];

  if( n == -1 ) {
    printf("[UpdateLowerOpenSet] ERROR Trying to lower key from a non-open-set element. Exiting! \n");
    return;
  }

  //-- If it happens to be the first element
  if( n == 0 ) { 
    return; 
  } //-- mHT[ID] = 0; // the same

  //-- If not, start on the bottom and go up
  node = n;

  State3D* qp; 
  State3D* qn;
  State3D* qtemp;

  while( node != 0 )
  { 
    parent = floor( (node - 1)/2 );

    qp = mOpenSet[parent]; 
    qn = mOpenSet[node];
    // Always try to put new nodes up
    if( qp->f > qn->f )
      {
        qtemp = qp;
        mOpenSet[parent] = qn; mHT[xyzToIndex(qn->x, qn->y, qn->z)] = parent;
        mOpenSet[node] = qtemp; mHT[xyzToIndex(qtemp->x, qtemp->y, qtemp->z)] = node; 
        node = parent; 
      }  
    else {  
      break; 
    }
  }   
  
}

/////////////////////////// DF TO PATH FUNCTIONS /////////////////////////////////////////////
PFDistanceField* DiversePaths::createDfToPathSet( std::vector< std::vector<double> > _path  ) {

  //-- Create Distance Field for Path(s)
  PFDistanceField* pathDf;
  pathDf = new PFDistanceField( mDf->getSize( PFDistanceField::DIM_X),
				mDf->getSize( PFDistanceField::DIM_Y),
				mDf->getSize( PFDistanceField::DIM_Z),
				mDf->getResolution( PFDistanceField::DIM_X ),
				mDf->getOrigin( PFDistanceField::DIM_X ),
				mDf->getOrigin( PFDistanceField::DIM_Y ),
				mDf->getOrigin( PFDistanceField::DIM_Z ) );
  pathDf->reset();

  //-- Add path objects
  std::vector<Eigen::Vector3d> pathPoints;
  Eigen::Vector3d point;

  for( int i = 0; i < _path.size(); ++i ) {
    point << _path[i][0], _path[i][1], _path[i][2];
    pathPoints.push_back( point );
  }

  pathDf->addPointsToField( pathPoints );
  
  return pathDf; 
}

/**
 * @function joinPaths
 */
void DiversePaths::joinPaths( std::vector<std::vector<double> > &_origPath,
			      std::vector<std::vector<double> > _addedPath ) {
  
  for( int i = 0; i < _addedPath.size(); ++i ) {
    _origPath.push_back( _addedPath[i] );
  }

}

/**
 * @function joinPaths
 */
void DiversePaths::joinPaths( std::vector<std::vector<int> > &_origPath,
			      std::vector<std::vector<int> > _addedPath ) {
  
  for( int i = 0; i < _addedPath.size(); ++i ) {
    _origPath.push_back( _addedPath[i] );
  }

}

/**
 * @function getPoinsAtLeastAsFarAs
 * @brief Get points that are at least _thresh away from the set point in _df
 */
std::vector<std::vector<double> > DiversePaths::getPointsAtLeastAsFarAs( PFDistanceField* _df, 
									 double _thresh ) {

  double dist;
  std::vector<std::vector<double> > points;
  std::vector<double> p(3);
  
  for( int x = 0; x < mDf->getNumCells( PFDistanceField::DIM_X ); ++x ) {
    for( int y = 0; y < mDf->getNumCells( PFDistanceField::DIM_Y ); ++y ) {
      for( int z = 0; z < mDf->getNumCells( PFDistanceField::DIM_Z); ++z ) {
	dist = _df->getDistanceFromCell( x, y, z );
	
	if( dist >= _thresh ) {
	  _df->gridToWorld( x, y, z, p[0], p[1], p[2] );
	  points.push_back( p );
	}

      }
    }
  }
  
  return points;
}

/**
 * @function getNearestPointFromSet
 */
std::vector<std::vector<double> > DiversePaths::getNearestPointFromSet( PFDistanceField* _df,
									std::vector<std::vector<double> > _set,
									std::vector<double> &_point,
									double _thresh ) {
  double dist;
  std::vector<std::vector<double> > points;
  std::vector<double> p(3);
  int x; int y; int z;

  double minDist = INFINITE_COST;

  for( int i = 0; i < _set.size(); ++i ) {
    _df->worldToGrid( _set[i][0], _set[i][1], _set[i][2], x, y, z );
    dist = _df->getDistanceFromCell( x, y, z );
    
    if( dist >= _thresh  ) {
      points.push_back( _set[i] );

      if( dist < minDist ) {
	minDist = dist;
	_point = _set[i];
      }
    }
    
  }

  return points;
}

/**
 * @function getPointsAsFarAs
 * @brief Get points _thresh + _tol < d < _thresh - _tol away from "object" ( path / obstacle )
 */
std::vector<std::vector<double> > DiversePaths::getPointsAsFarAs( PFDistanceField* _df, 
								  double _thresh, 
								  double _tol ) {
  double dist;
  std::vector<std::vector<double> > points;
  std::vector<double> p(3);

  for( int x = 0; x < mDf->getNumCells( PFDistanceField::DIM_X ); ++x ) {
    for( int y = 0; y < mDf->getNumCells( PFDistanceField::DIM_Y ); ++y ) {
      for( int z = 0; z < mDf->getNumCells( PFDistanceField::DIM_Z); ++z ) {
	dist = _df->getDistanceFromCell( x, y, z );

	if( dist >= _thresh - _tol && dist <= _thresh + _tol ) {
	  _df->gridToWorld( x, y, z, p[0], p[1], p[2] );
	  points.push_back( p );
	}

      }
    }
  }

  return points;
}


/**
 * @function getPointsAsFarAsFromSet
 * @brief Get points _thresh + _tol < d < _thresh - _tol away from "object" ( path / obstacle )
 */
std::vector<std::vector<double> > DiversePaths::getPointsAsFarAsFromSet( PFDistanceField* _df,
									 std::vector<std::vector<double> > _set,
									 double _thresh, 
									 double _tol ) {
  double dist;
  std::vector<std::vector<double> > points;
  std::vector<double> p(3);
  int x; int y; int z;

  for( int i = 0; i < _set.size(); ++i ) {
    _df->worldToGrid( _set[i][0], _set[i][1], _set[i][2], x, y, z );
    dist = _df->getDistanceFromCell( x, y, z );
    
    if( dist >= _thresh - _tol && dist <= _thresh + _tol ) {
      points.push_back( _set[i] );
    }
    
  }

  return points;
}

///////////////////////// UTILITIES /////////////////////////////////

/**
 * @function getWorldPoints
 */
std::vector<std::vector<double> > DiversePaths::getWorldPoints( std::vector<std::vector<int> > _cellPath ) {

  std::vector<std::vector<double> > path;
  std::vector<double> p(3);

  for( int i = 0; i < _cellPath.size(); ++i ) {
    mDf->gridToWorld( _cellPath[i][0], _cellPath[i][1], _cellPath[i][2],
		      p[0], p[1], p[2] );
    path.push_back( p );
  }

  return path;
}

/////////////////////////// VISUALIZATION FUNCTIONS //////////////////////////////////////////

/**
 * @function visualizePath
 */
void DiversePaths::visualizePath( boost::shared_ptr<pcl::visualization::PCLVisualizer> _viewer,
				  std::vector<std::vector<double> > _path,
				  bool _viewObstacles,
				  int _r, int _g, int _b ) {

  
  //-- View paths
  printf("Viewing path from (%f %f %f) to (%f %f %f ) \n", _path[0][0], _path[0][1], _path[0][2],
	 _path[_path.size() - 1][0], _path[_path.size() - 1][1], _path[_path.size() - 1][2]);
  viewPath( _path, _viewer, _r, _g, _b );
  viewBall( _path[0][0], _path[0][1], _path[0][2], 0.02, _viewer );
  viewBall( _path[_path.size() - 1][0], 
	    _path[_path.size() - 1][1], 
	    _path[_path.size() - 1][2], 
	    0.02, _viewer );    
  
  if( _viewObstacles ) {
    
    // Visualize obstacles (or not)
    int obsR; int obsG; int obsB;
    obsR = 0; obsG = 255; obsB = 0;
    
    //-- Get obstacles
    std::vector<Eigen::Vector3d> obstacles;
    mDf->getPointsFromField( obstacles );
    
    //-- Put them in PCD
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacleCloud = writePCD( obstacles );
    
    
    viewPCD( obstacleCloud, _viewer, obsR, obsG, obsB );
  }

}


/**
 * @function visualizePaths
 */
void DiversePaths::visualizePaths( boost::shared_ptr<pcl::visualization::PCLVisualizer> _viewer,
				   std::vector<std::vector<std::vector<double> > > _paths,
				   bool _viewObstacles ) {


  //-- Get counters ready
  //reset_PCL_Tools_counters();
  srand( time(NULL) );
  int r; int g; int b;


  for( int i = 0; i < _paths.size(); ++i ) {
    r = (int) ( rand() % 255 );
    g = (int) ( rand() % 255 );
    b = (int) ( rand() % 255 );
    visualizePath( _viewer, _paths[i], false, r, g, b ); 
  }

  if( _viewObstacles ) {
    
    // Visualize obstacles (or not)
    int obsR; int obsG; int obsB;
    obsR = 0; obsG = 255; obsB = 0;
    
    //-- Get obstacles
    std::vector<Eigen::Vector3d> obstacles;
    mDf->getPointsFromField( obstacles );
    
    //-- Put them in PCD
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacleCloud = writePCD( obstacles );
    
    
    viewPCD( obstacleCloud, _viewer, obsR, obsG, obsB );
  } 
}
