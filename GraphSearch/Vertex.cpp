/**
 * @file Vertex.cpp
 * @author A. Huaman
 * @date 2012-08-16
 */

#include "Vertex.h"
#include <cmath>


/**
 * @function Vertex
 * @brief Vertex
 */
Vertex::Vertex() {
}

/**
 * @function ~Vertex
 * @brief Destructor
 */
Vertex::~Vertex() {

}

/**
 * @function init
 * @brief Initialize member variables
 */
void Vertex::init( int _d, int _parent, int _x, int _y, int _z, int _key, int _state, int _heap ) {

  d = _d;
  parent = _parent;
  x = _x; y = _y; z = _z;
  state = _state;
  key = _key;
  heap = _heap;
}

/**
 * @function edgeCost
 */
int Vertex::edgeCost( Vertex* _v ) {

  int sum = std::abs(_v->x - x) + std::abs(_v->y - y) + std::abs(_v->z - z);

  if( sum == 3 ) {
    return COST_3;
  }
  if( sum == 2) {
    return COST_2;
  }

  if( sum == 1) {
    return COST_1;
  }
  else {
    printf("[edgeCost] COST NO FOUND - sum: %d  \n", sum);
    return 0;
  }
}
