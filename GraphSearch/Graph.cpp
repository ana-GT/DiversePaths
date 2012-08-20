/**
 * @file Graph.cpp
 * @author A. Huaman Q.
 * @date 2012 / 08/ 16
 */
#include "Graph.h"
#include <iostream>

/**
 * @function Graph
 */
Graph::Graph( float _OX, float _OY, float _OZ,
	      float _LX, float _LY, float _LZ,
	      float _resolution, 
	      int _default ) {
 
  mOX = _OX; mOY = _OY;   mOZ = _OZ;

  mMaxX = mOX + _LX;
  mMaxY = mOY + _LY;
  mMaxZ = mOZ + _LZ;

  mRes = _resolution;

  mNumVx = ( ( mMaxX - mOX ) / mRes );
  mNumVy = ( ( mMaxY - mOY ) / mRes );
  mNumVz = ( ( mMaxZ - mOZ ) / mRes );

  mStepYZ = mNumVy*mNumVz;
  mStepZ = mNumVz;

  mNumV = mNumVx*mNumVy*mNumVz;
  mV = new int[mNumV];

  std::fill( mV, mV + mNumV, _default );

}

/**
 * @function ~Graph
 */
Graph::~Graph() {

  if( mV != NULL ) {
    delete [] mV;
  }
}

/**
 * @function WorldToVertex
 */
void Graph::WorldToVertex( const float &_wx, const float &_wy, const float &_wz,
			   int &_vx, int &_vy, int &_vz ) {

  // mOX <= _wx <= mMaxX && mOX <= _wx <= mMaxY && mOY <= _wz <= mMaxZ
  // floor effect expected: Round to minimum integer
  _vx = (int) ( ( _wx - mOX ) / mRes );
  _vy = (int) ( ( _wy - mOY ) / mRes );
  _vz = (int) ( ( _wz - mOZ ) / mRes );
}

/**
 * @function WorldToIndex
 */
int Graph::WorldToIndex( const float &_wx, 
			 const float &_wy, 
			 const float &_wz ) {
  
  int vx; int vy; int vz;
  WorldToVertex( _wx, _wy, _wz, vx, vy, vz );
  return ref( vx, vy, vz );
}

/**
 * @function VertexToWorld
 */
void Graph::VertexToWorld( const int &_vx, const int &_vy, const int &_vz,  
			   float &_wx, float &_wy, float &_wz ) {

  // 0 <= _vx < numVx && 0 <= _vy < numVy && 0 <= _vz < numVz
  _wx = mOX + mRes*_vx;
  _wy = mOY + mRes*_vy;
  _wz = mOZ + mRes*_vz;
}

/**
 * @function IndexToVertex
 */
void Graph::IndexToVertex( const int &_ind,
			   int &_x, int &_y, int &_z ) {
 
  int remaining;
  _x = _ind / mStepYZ;
  remaining = _ind % mStepYZ;
  _y = remaining / mStepZ;
  _z = remaining % mStepZ;
}

/**
 * @function createBox
  */
void Graph::createBox( float _ox, float _oy, float _oz,
		       float _lx, float _ly, float _lz,
		       int _type ) {
  
  int b1x; int b1y; int b1z;
  int b2x; int b2y; int b2z;

  WorldToVertex( _ox, _oy, _oz, b1x, b1y, b1z );
  WorldToVertex( (_lx + _ox), (_ly +_oy), (_lz + _oz),
		 b2x, b2y, b2z );

  for( int i = b1x; i < b2x; i++ ) {
    for( int j = b1y; j < b2y; j++ ) {
      for( int k = b1z; k < b2z; k++ ) {
	mV[ref(i,j,k)] = _type;
      }
    }
  }
}

/**
 * @function getPCD
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr Graph::getPCD( int _type ) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ> );

  int *v;
  v = &mV[0];
  
  // Get how many obstacle vertices there are in the graph
  int count = 0;
  for( int i = 0; i < mNumV; ++i ) {
    if( *v == _type ) {
      count++;
    }
    v++;
  }
  printf("Num V: %d \n", mNumV );
  printf("Num obst: %d \n", count );

  cloud->width = count;
  cloud->height = 1;
  cloud->is_dense = false;
  cloud->points.resize( cloud->width*cloud->height );


  // Enter vertices in the graph
  int vx; int vy; int vz;
  float wx; float wy; float wz;
  v = &mV[0];
  count = 0;

  for( int i = 0; i < mNumV; ++i ) {
    if( *v == _type ) {
      IndexToVertex( i, vx, vy, vz );
      VertexToWorld( vx, vy, vz, wx, wy, wz );
      cloud->points[count].x = wx;
      cloud->points[count].y = wy;
      cloud->points[count].z = wz;      
      count++;
    }
    v++;
  }
 
  return cloud;
}
