/**
 * @file Dijkstra.h
 * @brief 
 * @author A. Huaman
 * @date 2012-8-19 
 */

#ifndef __DIJKSTRA_H__
#define __DIJKSTRA_H__

#include "Graph.h"
#include "Vertex.h"

/**
 * @class Dijkstra
 */
class Dijkstra {

 public:
  Dijkstra( Graph* _G );
  ~Dijkstra();
  void search( int _vx, int _vy, int _vz );
  void printInfo();

  // Heap functions
  int enqueue();
  int insert( Vertex* _v );
  void initFromSource();
  int extractMin();
  int lowerKey( Vertex* _v );
  bool relax( Vertex* _u, Vertex* _v );
  std::vector<int> getAdj( int _u );
  std::vector<Eigen::Vector3d> getPath( int _i );
  std::vector<Eigen::Vector3d> getPath( float _wx, float _wy, float _wz );
  inline int Parent( int _u );
  inline int Left( int _u );
  inline int Right( int _u );

 private:
  int mNumV;
  Vertex* mV;
  int mStartVx; int mStartVy; int mStartVz;
  int mStartIndex;

  Graph* mG;

  int* mH;
  int mHSize;
  int mHLength;
  
};

//////////////// INLINE FUNCTIONS /////////////////

/**
 * @function Parent
 */
inline int Dijkstra::Parent( int _u ) {
  return (_u - 1 ) / 2;
}

/**
 * @function Left
 */
inline int Dijkstra::Left( int _u ) {
  return 2*_u+1;
}

/**
 * @function Right
 */
inline int Dijkstra::Right( int _u ) {
  return 2*_u+2;
}

#endif /**  __DIJKSTRA_H__ */
