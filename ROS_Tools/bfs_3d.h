/*
 * Copyright (c) 2009, Maxim Likhachev
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Pennsylvania nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/** @author: Benjamin Cohen /bcohen@willowgarage.com **/
/** Modified by A.C.H.Q. to suit her needs :) */

#ifndef _BFS3D_CUSTOMIZED_
#define _BFS3D_CUSTOMIZED_

#include <iostream>
#include <vector>
#include <string>
#include <time.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <queue>
#include <distance_field/pf_distance_field.h>

#define SMALL_NUM  0.00000001  
#define INFINITE_COST 1000000000 
 
#define DEBUG_TIME 0 
#define DIRECTIONS3D 26 
#define GOAL_TOLERANCE 0 


/**
 * @struct State3D
 */
typedef struct
{
  unsigned int g;
  int iterationclosed;
  int x;
  int y;
  int z;
} State3D;


/**
 * @class BFS3D
 */
class BFS3D
{
  /**< @class Cell3D*/
  class Cell3D
  {
  public:
    int x;
    int y;
    int z;
  };
  
  /**< @class FIFO */
  class FIFO
  {
  public:
    FIFO( int length );
    ~FIFO();
    bool empty();
    void clear();
    void insert( int x, int y, int z );
    void remove( int* x, int* y, int* z );
    
  private:
    int head_;
    int tail_;
    int size_;
    Cell3D *q_;
  };
  
 public:

  BFS3D( int dim_x, int dim_y, int dim_z, 
	 int radius, int cost_per_cell );
  ~BFS3D();

  void init();
  bool setGoal( std::vector<int> goal ); /**< goal is a 3x1 vector {x,y,z}  */
  bool setGoals( std::vector<std::vector<int> > goals ); /**< goal is a nx3 vector */
  bool runBFS();
  int getDist( int x, int y, int z );
  bool getShortestPath( std::vector<int> start, 
			std::vector<std::vector<int> > &path );
  void configDistanceField( bool enable, 
  			    const PFDistanceField* df );

  void printGrid(); /**< debugging */
  void printCostToGoal(); /**< debugging */
  void printConfig(FILE* fOut); /**< debugging */

  void setRadius( double r );
  int getRadiusCells() { return int(radius_); };
  
  /* for two arm planner - no roll,pitch allowed */
  void setZInflation( int cells_above, int cells_below );
  void useResearchGrid( bool use_xygrid ) { use_research_grid_ = use_xygrid; };
  void initializeXYGrid();
  void printXYPlane( int z );
  void printInflatedXYPlane( int z );
  void inflateXYGrid();
  void inflateXYGrid( int z_min, int z_max );
  void computeDistancestoNonfreeAreas( std::vector<std::vector<unsigned char> > &Grid2D, 
				       int width_x, int height_y, unsigned char obsthresh, 
				       std::vector<std::vector<double> > &disttoObs_incells );
  bool setGoal(int x, int y, int z);
  bool getShortestPath(int x, int y, int z, std::vector<std::vector<int> > &path);
  bool getShortestPath(int x, int y, int z, std::vector<std::vector<double> > &path);
  
 private:
  
  int dimX_;
  int dimY_;
  int dimZ_;
  
  int radius_;
  double radius_m_;

  std::vector<std::vector<int> > goal_;
  
  int cost_1_move_;
  int cost_sqrt2_move_;
  int cost_sqrt3_move_;
  
  bool enable_df_;
  bool use_research_grid_;
  const PFDistanceField* df_;
  
  unsigned char*** grid3D_;
  
  int dist_length_;
  std::vector<int> dist_;
  
  void reInitializeState3D( State3D* state );
  void initializeState3D( State3D* state, int x, int y, int z );
  void create3DStateSpace( State3D**** statespace3D );
  void delete3DStateSpace( State3D**** statespace3D );
  inline int xyzToIndex( int x, int y, int z );
  void search3DwithFifo();
  void search3DwithQueue( State3D*** statespace );
  bool isGoal( const std::vector<int> &state );
  bool isValidCell( const int x, const int y, const int z );
  
  FIFO *q_;
  
  /* for two arm planner - no roll,pitch allowed */
  short unsigned int z_cells_above_;
  short unsigned int z_cells_below_;
  std::vector<std::vector<std::vector<unsigned char> > > xy_grid_;
  std::vector<std::vector<std::vector<double> > > inflatedxy_grid_; //in cells
  
};

/////////////////////// INLINE FUNCTIONS /////////////////////////

/**
 * @function xyzToIndex
 */
inline int BFS3D::xyzToIndex( int x, int y, int z )
{
  int ret = x + y*dimX_ + z*dimX_*dimY_;
  if(ret < dist_length_)
    return ret;
  else
  {
    printf("[bfs3d] out of bounds (%d %d %d) (index: %d  size: %d)\n", x,y,z,ret,dist_length_ );
    return 0;
  }
}


#endif /** _BFS3D_CUSTOMIZED_ */


