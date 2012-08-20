/**
 * @file Graph.h 
 * @author A. Huaman Quispe
 * @date 2012-08-16
 */

#ifndef __GRAPH_H__
#define __GRAPH_H__

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

enum VERTEX_TYPE {
  OBSTACLE = 3,
  FREE,
  INFLATED
};

/**
 * @class Graph
 */
class Graph {

 public:
  Graph( float _OX, float _OY, float _OZ,
	 float _LX, float _LY, float _LZ,
	 float _resolution, 
	 int _default = FREE );
  ~Graph();

  void WorldToVertex( const float &_wx, const float &_wy, const float &_wz,
		      int &_vx, int &_vy, int &_vz );
  
  int WorldToIndex( const float &_wx, const float &_wy, const float &_wz );

  void VertexToWorld( const int &_vx, const int &_vy, const int &_vz, 
		      float &_wx, float &_wy, float &_wz ); 

  void IndexToVertex( const int &_ind,
		      int &_x, int &_y, int &_z );

  void createBox( float _ox, float _oy, float _oz,
		  float _lx, float _ly, float _lz,
		  int _type = OBSTACLE );

  pcl::PointCloud<pcl::PointXYZ>::Ptr getPCD( int _type );

  inline int ref( int _vx, int _vy, int _vz );
  inline int getNumV();
  inline int getNumVx();
  inline int getNumVy();
  inline int getNumVz();
  inline bool isValid( int _vx, int _vy, int _vz );
  inline int getState( int _ind );
  inline int getState( int _vx, int _vy, int _vz );

 private:

  int mNumV; //< Number of total vertices
  int* mV;  //< Vertices
  float mOX; float mOY; float mOZ; //< Origin coordinates for each direction
  float mMaxX; float mMaxY; float mMaxZ; //< Max value for each direction
  float mRes;  //< Resolution
  int mNumVx; int mNumVy; int mNumVz; //< Num vertices at each direction
  int mStepYZ;
  int mStepZ;
};

/**
 * @function getNumV
 */
inline int Graph::getNumV() {
  return mNumV;
}

/**
 * @function getNumVx
 */
inline int Graph::getNumVx() {
  return mNumVx;
}

/**
 * @function getNumVy
 */
inline int Graph::getNumVy() {
  return mNumVy;
}

/**
 * @function getNumVz
 */
inline int Graph::getNumVz() {
  return mNumVz;
}
/**
 * @function ref
 */
inline int Graph::ref( int _vx, int _vy, int _vz ) {
  return _vx*mStepYZ + _vy*mStepZ + _vz;
}


/**
 * @function isValid
 */
inline bool Graph::isValid( int _vx, int _vy, int _vz ) {
  if( _vx >= 0 && _vx < mNumVx &&
      _vy >= 0 && _vy < mNumVy && 
      _vz >= 0 && _vz < mNumVz ) {
    return true;
  }
  return false;
}

/**
 * @function getState
 */
inline int Graph::getState( int _ind ) {
  return mV[ _ind ];
}

/** 
 * @function getState
 */
inline int Graph::getState( int _vx, int _vy, int _vz ) {
  return mV[ ref(_vx, _vy, _vz) ];
}


#endif /** __GRAPH_H__ */
