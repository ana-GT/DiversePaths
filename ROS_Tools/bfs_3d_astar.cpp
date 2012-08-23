/**
 * @file bfs_3d_astar.cpp
 * @author A. Huaman
 * @brief A star 
 */

#include "bfs_3d.h"

/**
 * @function searchPath3D
 */
bool BFS3D::searchPath3D( State3D*** _statespace, 
			  double wx, double wy, double wz ) {
  State3D* ExpState;
  int newx, newy, newz;
  int x,y,z;
  unsigned int g_temp;
  
  // these are added here temporarily. should be in the class
  int dx[DIRECTIONS3D] = { 1,  1,  1,  0,  0,  0, -1, -1, -1,    1,  1,  1,  0,  0, -1, -1, -1,    1,  1,  1,  0,  0,  0, -1, -1, -1};
  int dy[DIRECTIONS3D] = { 1,  0, -1,  1,  0, -1, -1,  0,  1,    1,  0, -1,  1, -1, -1,  0,  1,    1,  0, -1,  1,  0, -1, -1,  0,  1};
  int dz[DIRECTIONS3D] = {-1, -1, -1, -1, -1, -1, -1, -1, -1,    0,  0,  0,  0,  0,  0,  0,  0,    1,  1,  1,  1,  1,  1,  1,  1,  1};
  
  // create a queue
  std::queue<State3D*> Queue;

  // initialize to infinity all
  for (x = 0; x < dimX_; x++) {
    for (y = 0; y < dimY_; y++) {
      for (z = 0; z < dimZ_; z++) {
        dist_[xyzToIndex(x,y,z)] = INFINITE_COST;
        reInitializeState3D( &_statespace[x][y][z] );
      }
    }
  }
  
  // initialization - throw starting states on queue with g cost = 0
  for( unsigned int i = 0; i < goal_.size(); ++i ) {
    _statespace[ goal_[i][0] ][ goal_[i][1] ][ goal_[i][2] ].g = 0;
    Queue.push( &_statespace[ goal_[i][0] ][ goal_[i][1] ][ goal_[i][2] ] );
  }

  // expand all of the states
  while( (int)Queue.size() > 0 ) {
    
    // get the state to expand
    ExpState = Queue.front();

    Queue.pop();
    
    // it may be that the state is already closed
    if( ExpState->iterationclosed == 1 ) {
      continue;
    }

    //close it
    ExpState->iterationclosed = 1;

    //set the corresponding distances to the goal
    dist_[ xyzToIndex(ExpState->x, ExpState->y, ExpState->z) ] = ExpState->g;

    //iterate through neighbors
    for( int d = 0; d < DIRECTIONS3D; d++ ) {
      newx = ExpState->x + dx[d];
      newy = ExpState->y + dy[d];
      newz = ExpState->z + dz[d];
      
      //make sure it is inside the map and has no obstacle
      if(0 > newx || newx >= dimX_ || 0 > newy || newy >= dimY_ || 0 > newz || newz >= dimZ_) {
        continue;
      }

      if( !isValidCell(newx, newy, newz) ) {
        continue;
      }

      if( _statespace[newx][newy][newz].iterationclosed == 0 ) {
	//insert into the stack
        Queue.push( &_statespace[newx][newy][newz] );

        //set the g-value
        if ( ExpState->x != newx && ExpState->y != newy && ExpState->z != newz ) {
          g_temp = ExpState->g + cost_sqrt3_move_;
	}
        else if ( (ExpState->y != newy && ExpState->z != newz) ||
		  (ExpState->x != newx && ExpState->z != newz) ||
		  (ExpState->x != newx && ExpState->y != newy) ) {
          g_temp = ExpState->g + cost_sqrt2_move_;
	}
        else {
          g_temp = ExpState->g + cost_1_move_;
	}

        if( _statespace[newx][newy][newz].g > g_temp ) {
          _statespace[newx][newy][newz].g = g_temp;
	}
      }

    } // end for DIRECTIONS3D
  } // end while Q.size() > 0
}
