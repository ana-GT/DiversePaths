/**
 * @file Vertex.h
 * @author A. Huaman
 * @date 2012-08-16
 */
#ifndef __VERTEX_H__
#define __VERTEX_H__

#include <climits>
#include <stdio.h>

#define INF INT_MAX 
#define NIL (-1)
#define COST_3 17
#define COST_2 14
#define COST_1 10

/**
 * @class Vertex
 */
class Vertex {
 public:
  Vertex();
  ~Vertex();
  void init( int _d, 
	     int _parent, 
	     int _x, int _y, int _z, 
	     int _key, int _state, 
	     int _heap = NIL );
  int edgeCost( Vertex* _v );

  int d;
  int parent;
  int x; 
  int y;
  int z;
  int heap;
  int state;
  int key;

};


#endif /** __VERTEX_H__ */
