/**
 * @file Dijkstra.cpp
 */

#include "Dijkstra.h"

const int Dijkstra::mNX[26] = { -1,-1,-1, -1,-1,-1, -1,-1,-1,   0, 0, 0,  0,   0,  0, 0, 0,   1, 1, 1, 1, 1, 1, 1, 1, 1 };
const int Dijkstra::mNY[26] = { -1, 0, 1, -1, 0, 1, -1, 0, 1,  -1, 0, 1, -1,   1, -1, 0, 1,  -1, 0, 1,-1, 0, 1,-1, 0, 1 };
const int Dijkstra::mNZ[26] = { -1,-1,-1,  0, 0, 0,  1, 1, 1,  -1,-1,-1,  0,   0,  1, 1, 1,  -1,-1,-1, 0, 0, 0, 1, 1, 1 };

/**
 * @function Dijkstra
 * @brief Constructor
 */
Dijkstra::Dijkstra( Graph* _G ) {
  
  // Initialize a few values
  mV = NULL;
  mH = NULL;
  mHLength = 0;
  mHSize = 0;

  // Save a pointer to vertice array
  mG = _G;
  mNumV = mG->getNumV();
  mNumVx = mG->getNumVx();
  mNumVy = mG->getNumVy();
  mNumVz = mG->getNumVz();

  // Store vertices
  mV = new Vertex[mNumV];
  Vertex* v = &mV[0];

  for( int i = 0; i < mG->getNumVx(); ++i ) {
    for( int j = 0; j < mG->getNumVy(); ++j ) {
      for( int k = 0; k < mG->getNumVz(); ++k ) {
	int ind = mG->ref(i,j,k);
	mV[ind].init( INF, NIL, i, j, k,
		       ind,mG->getState(ind), NIL );
      }
    }
  }
  
}

/**
 * @function ~Dijkstra
 * @brief Destructor
 */
Dijkstra::~Dijkstra() {

  if( mV != NULL ) {
    delete [] mV;
  }

  if( mH != NULL ) {
    delete [] mH;
  }

}

/**
 * @function search
 * @brief Search
 */
void Dijkstra::search( int _vx, int _vy, int _vz ) {

  // Save Info
  mStartVx = _vx; mStartVy = _vy; mStartVz = _vz;

  // 1. Initialize
  time_t ts, tf; double dt;
  
  ts = clock();
  initFromSource();
  tf = clock();
  dt = (double)(tf - ts)/CLOCKS_PER_SEC;
  printf("initFromSource time: %f \n", dt );
  
  // 2. Enqueue the vertices
  ts = clock();
  enqueue();
  tf = clock();
  dt = (double)(tf - ts)/CLOCKS_PER_SEC;
  printf("enqueue time: %f \n", dt );

  // 3. Loop

  ts = clock();
  int u;
  Vertex* vu;

  while( mHSize != 0 ) {
    u = extractMin();
    vu = &mV[u];

    std::vector<int> adj = getAdj(u);
    for( int i = 0; i < adj.size(); ++i ) {
      relax( vu, &mV[ adj[i] ] );
    }
  }
  tf = clock();
  dt = (double)(tf - ts)/CLOCKS_PER_SEC;
  printf("loop time: %f \n", dt );

}

/**
 * @function getAdj
 */
std::vector<int> Dijkstra::getAdj( int _u ) {

  Vertex* v = &mV[_u];

  int x; int y; int z;
  int ax; int ay; int az;
  std::vector<int> adj;

  x = v->x; y = v->y; z = v->z;

  for( int i = 0; i < 26; ++ i ) {

    ax = x + mNX[i];
    ay = y + mNY[i];
    az = z + mNZ[i];

    //    if( mG->isValid( ax, ay, az ) == true ) {
    if( ax >= 0 && ax < mNumVx && ay >= 0 && ay < mNumVy && az >= 0 && az < mNumVz ) {
	int ref = mG->ref(ax, ay, az);
      if( mG->getState(ref ) == FREE ) {
	adj.push_back(ref);
      }
    }
  }
      

  return adj;
}

/**
 * @function relax
 */
bool Dijkstra::relax( Vertex* _u, Vertex* _v ) {

  int cost = _u->d + _u->edgeCost(_v);

  if( _v->d > cost ) {
    _v->d = cost;
    _v->parent = _u->key;
    lowerKey( _v );
  } 
}

/**
 * @function initFromSource
 */
void Dijkstra::initFromSource() {
  
  Vertex* v = &mV[0];
  for( int i = 0; i < mNumV; ++i ) {
    v->parent = NIL;
    v->d = INF;
    v++;
  }

  // Start
  mV[ mG->ref(mStartVx, mStartVy, mStartVz) ].d = 0;
}

/**
 * @function printInfo
 */
void Dijkstra::printInfo() {

  Vertex* v = &mV[0];
  
  int free_counter = 0;
  int obstacle_counter = 0;
  int path_counter = 0;

  for( int i = 0; i < mNumV; ++i ) {
    // State
    if( v->state == FREE ) {
      free_counter++;
    } 
    else if( v->state == OBSTACLE ) {
      obstacle_counter++;
    }
    // Path
    if( v->parent != NIL ) {
      path_counter++;
    }

    v++;  
  }

  // Print
  printf( " Dijkstra print info \n " );
  printf( " =================== \n " );
  printf( " [+] Number of free vertices : %d \n", free_counter );
  printf( " [+] Number of obst vertices : %d \n", obstacle_counter );
  printf( " [+] Number of vertices with parent : %d \n", path_counter );
}


/**
 * @function enqueue
 */
int Dijkstra::enqueue() {

  // Create the heap
  mHLength = mNumV;
  mHSize = 0;
  mH = new int[mHLength];
  std::fill( mH, mH + mHLength, NIL );

  Vertex* v = &mV[0];

  int count = 0;
  for( int i = 0; i < mNumV; ++i ) {
    if( v->state == OBSTACLE ) {
      v->heap = NIL; // Not on the heap
    } else {
      insert( v ); count++;
    }
    v++;
  }

  return count;
}

/**
 * @function insert
 */
int Dijkstra::insert( Vertex* _v ) {

  int u; int parent;
  int temp;

  mH[ mHSize ] = _v->key; _v->heap = mHSize;
  mHSize = mHSize + 1;

  if( mHSize == 1 ) {
    return _v->heap;
  }

  u = mHSize - 1;

  while( u > 0 ) {
    parent = Parent(u);

    if( mV[ mH[u] ].d < mV[ mH[parent] ].d ) {
      temp = mH[u];
      mH[u] = mH[parent]; mV[ mH[u] ].heap = u; 
      mH[parent] = temp; mV[ mH[parent] ].heap = parent;
      u = parent;
    }
    else {
      break;
    }
  }
  
  return _v->heap;
}


/**
 * @function extractMin
 */
int Dijkstra::extractMin() {

  int u; int l; int r; int temp;
  int smallest;

  if( mHSize == 0 ) {
    printf("[extractMin] ERROR: Underflow \n");
    return NIL;
  }

  // Save first element to extract
  int top = mH[0];
  mV[ top ].heap = NIL;

  // Put last element on top
  mH[0] = mH[mHSize-1]; mV[ mH[mHSize-1] ].heap = 0;
  // Decrease Heap size in one
  mHSize = mHSize -1;

  // Check if heap property is preserved
  u = 0;

  while( true ) {
    
    l = Left(u);
    r = Right(u);
    smallest = u; // starting

    if( r < mHSize ) {
      if( mV[ mH[l] ].d < mV[ mH[u] ].d ) {
	smallest = l;
      }
      if( mV[ mH[r] ].d < mV[ mH[smallest] ].d ) {
	smallest = r;
      }
    }
    else if( l < mHSize ){
      if( mV[ mH[l] ].d < mV[ mH[u] ].d ) {
	smallest = l;
      }      
    }

    if( smallest != u ) {
      temp = mH[ u ];
      mH[u] = mH[smallest]; mV[ mH[u] ].heap = u;
      mH[smallest] = temp; mV[temp].heap = smallest;
      u = smallest;
    }
    else {
      break;
    }
  }
  
  return top;
}

/**
 * @function lowerKey
 */
int Dijkstra::lowerKey( Vertex* _v ) {
  
  int u; int parent; int temp;

  // Check its location
  u = _v->heap;

  if( u == NIL ) {
    printf( "[LowerKey] No in Heap - something is wrong! \n" );
    return NIL;
  }

  if( u == 0 ) {
    return 0;
  }

  int hu; int hp;
  Vertex* vu; Vertex* vp;

  while( u > 0 ) {

    parent = Parent(u);

    hu = mH[u]; vu = &mV[hu];
    hp = mH[parent]; vp = &mV[hp];

    if( vu->d < vp->d ) {
      temp = hu;
      mH[u] = hp; vp->heap = u;
      mH[parent] = temp; vu->heap = parent;
      u = parent;
    }
    else {
      break;
    }
  }
  
  return u;
}

/**
 * @function getPath
 */
std::vector<Eigen::Vector3d> Dijkstra::getPath( float _wx, float _wy, float _wz ) {
  int ind = mG->WorldToIndex( _wx, _wy, _wz );
  return getPath(ind);
}

/**
 * @function getPath
 */
std::vector<Eigen::Vector3d> Dijkstra::getPath( int _i ) {
  
  std::vector<Eigen::Vector3d> path;
  Eigen::Vector3d p;
  float wx; float wy; float wz;
  int parent; int count;

  Vertex *_v = &mV[_i];
  
  if( _v->parent == NIL ) {
    printf( "[getPath] Error: No parent to end vertex \n" );
    return path;
  }

  else {
    count = 0;
    while( count <= mNumV ) {
      mG->VertexToWorld( _v->x, _v->y, _v->z, wx, wy, wz );
      p << wx, wy, wz;
      path.push_back(p);
      parent = _v->parent;
      if( parent != NIL ) {
	_v = &mV[parent];
      }
      else {
	break;
      }
      count++;
    }
  }

  if( count > mNumV ) {
    printf("[getPath] Error: Got out of infinite loop \n");
  }
  return path;
}
