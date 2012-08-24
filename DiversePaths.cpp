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
 * @function runAstar
 */
bool DiversePaths::runAstar(std::vector<double> _start, 
			    std::vector<std::vector<double> > &_path ) {

  // Check goal
  if( mGoal.empty() ) {
    return false;
  }

  // Check start
  std::vector<int> start(3);
  if( _start.size() < 3 ) {
    printf( "[runAstar] Error in start, need 3-element vector \n " );
    return false;
  }

  if( !mDf->worldToGrid( _start[0], _start[1], _start[2], start[0], start[1], start[2] ) ) {
    printf( "[runAstar] No valid start position  \n" );
  }

  // Run
  bool b;
  printf(" [runAstar] Running A* with goal: (%d %d %d) and start (%d %d %d) \n", 
	 mGoal[0][0], mGoal[0][1], mGoal[0][2],
	 start[0], start[1], start[2] );

  mDistLength = (mDimX - 1) + (mDimY - 1)*(mDimX) + (mDimZ - 1)*(mDimX)*(mDimY) + 1;

  // Start configuration
  mHT = new int[mDistLength];
  std::fill( mHT, mHT + mDistLength, -1 );
  mOpenSet.resize(0);

  create3DStateSpace( &mS3D );

  // Search
  b = searchOneToOnePath( start, mGoal[0], _path );

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

  // these are added here temporarily. should be in the class
  int dx[DIRECTIONS3D] = { 1,  1,  1,  0,  0,  0, -1, -1, -1,    1,  1,  1,  0,  0, -1, -1, -1,    1,  1,  1,  0,  0,  0, -1, -1, -1};
  int dy[DIRECTIONS3D] = { 1,  0, -1,  1,  0, -1, -1,  0,  1,    1,  0, -1,  1, -1, -1,  0,  1,    1,  0, -1,  1,  0, -1, -1,  0,  1};
  int dz[DIRECTIONS3D] = {-1, -1, -1, -1, -1, -1, -1, -1, -1,    0,  0,  0,  0,  0,  0,  0,  0,    1,  1,  1,  1,  1,  1,  1,  1,  1};

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
      printf("Found a path. Tracing it \n");
      TracePath( _start, _goal, _path );
      break;
    }
    
    // Add node to closed set
    u->iterationclosed = 2;
    
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
  
  // If this is the first element added
  if( n == 0 ) {
    mHT[ xyzToIndex(_u->x, _u->y, _u->z) ] = n;
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
    if( qp->f >= qn->f ) {
      qtemp = qp;
      mOpenSet[parent] = qn; mHT[ xyzToIndex(qn->x, qn->y, qn->z) ] = parent;
      mOpenSet[node] = qtemp; mHT[ xyzToIndex(qtemp->x, qtemp->y, qtemp->z) ] = node;
      node = parent;
    }
    else {
      break;
    }
  }

  mHT[ xyzToIndex(_u->x, _u->y, _u->z) ] = node;
}

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
      if( mOpenSet[node]->f >= mOpenSet[child_1]->f ) {
	u = child_1;
      }
      if( mOpenSet[u]->f >= mOpenSet[child_2]->f ) {
	u = child_2;
      }
    }
    else if( child_1 < n ) {
      if( mOpenSet[node]->f >= mOpenSet[child_1]->f ) {
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

  // Make sure the while loop eventually stops
  int max_path_length = mDimX*mDimY;

  int dx[DIRECTIONS3D] = { 1,  1,  1,  0,  0,  0, -1, -1, -1,    1,  1,  1,  0,  0, -1, -1, -1,    1,  1,  1,  0,  0,  0, -1, -1, -1};
  int dy[DIRECTIONS3D] = { 1,  0, -1,  1,  0, -1, -1,  0,  1,    1,  0, -1,  1, -1, -1,  0,  1,    1,  0, -1,  1,  0, -1, -1,  0,  1};
  int dz[DIRECTIONS3D] = {-1, -1, -1, -1, -1, -1, -1, -1, -1,    0,  0,  0,  0,  0,  0,  0,  0,    1,  1,  1,  1,  1,  1,  1,  1,  1};


  _path.resize(0);
  next_state[0] = _goal[0];
  next_state[1] = _goal[1];
  next_state[2] = _goal[2];
  printf("G of goal: %d \n", mS3D[_goal[0]][_goal[1]][_goal[2]].g);
  while(  (next_state[0] != _start[0] || next_state[1] != _start[1] || next_state[2] != _start[2] )
	  && counter < max_path_length ) {
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

      if( val < min_val ) {
	min_val = val;
	next_state[0] = newx;
	next_state[1] = newy;
	next_state[2] = newz;
      }
    } // for
    //printf("New state: %d %d %d \n", next_state[0], next_state[1], next_state[2]);
    std::vector<double> p(3);
    mDf->gridToWorld( next_state[0], next_state[1], next_state[2], p[0], p[1], p[2] );
    printf("[A] Min val: %d \n", min_val );
    _path.push_back( p );
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
 * @function UpdateLowerOpenSet
 */
void DiversePaths::UpdateLowerOpenSet( State3D* _u ) {
  
  int n; 
  int node; int parent;
  int temp;

  //-- Find your guy
  n = mHT[ xyzToIndex(_u->x, _u->y, _u->z) ];

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
    printf("Min val: %d \n", min_val);
    _path.push_back( next_state );
    counter++;
  } // while

  printf("Dijkstra G is: %d \n", mDist[xyzToIndex(next_state[0], next_state[1], next_state[2])]);

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

/////////////////////////// VISUALIZATION FUNCTIONS //////////////////////////////////////////


/**
 * @function visualizePath
 */
void DiversePaths::visualizePath( boost::shared_ptr<pcl::visualization::PCLVisualizer> _viewer,
				  std::vector<std::vector<double> > _path,
				  bool _viewObstacles,
				  int _r, int _g, int _b ) {


  //-- Get counters ready
  //reset_PCL_Tools_counters();
  
  //-- View paths
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
