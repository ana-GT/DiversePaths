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

  // Create a BFS object
  mBfs = new BFS3D( mDf->getNumCells(PFDistanceField::DIM_X),
		    mDf->getNumCells(PFDistanceField::DIM_Y),
		    mDf->getNumCells(PFDistanceField::DIM_Z),
		    mRadius,
		    mCost );

  mBfs->configDistanceField( true, mDf );

}

/**
 * @function ~DiversePaths
 * @brief Constructor
 */
DiversePaths::~DiversePaths() {
  
  if( mBfs != NULL ) {
    delete mBfs;
  }

}

/**
 * @function FindDiversePaths
 */
bool DiversePaths::Search( double _sx, double _sy, double _sz,
			   double _gx, double _gy, double _gz,
			   int _numPaths,  
			   double _alpha,
			   std::vector<std::vector<std::vector<double> > > &_paths ) {
  
  mNumPaths = _numPaths;
  mAlpha = _alpha;
  std::vector<int> start(3);
  std::vector<int> goal(3);

  // Output
    _paths.clear();

  // Verify that start and goal locations are valid
  if( !mDf->worldToGrid( _sx, _sy, _sz, start[0], start[1], start[2] ) ) {
    printf( " * Start position no valid. Exiting \n" );
    return false;
  }

  if( !mDf->worldToGrid( _gx, _gy, _gz, goal[0], goal[1], goal[2] ) ) {
    printf( " * Goal position no valid. Exiting \n" );
    return false;
  }

  // Set goal
  printf("Set goal \n");
  mBfs->setGoal( goal );

  // Apply Dijkstra
  printf("Run BFS  \n");
  time_t ts; time_t tf; double dt;
  ts = clock();
  mBfs->runBFS();
  tf = clock();
  dt = (double) ( (tf - ts) / CLOCKS_PER_SEC );
  printf( "[*] Run BFS time: %f \n", dt );


  //-- Get a path
  printf("path: %d %d %d to %d %d %d \n", 
	 start[0], start[1], start[2],
	 goal[0], goal[1], goal[2] );

  std::vector<std::vector<double> > path;
  if(mBfs->getShortestPath( start, path ) == true ) {
    _paths.push_back( path );
    printf("Got  path \n");
  }

  else {
    printf("No path, damn! \n");
    return false;
  }
  
  mPaths = _paths; 
  return true;
  /*
  
  //std::vector< std::vector<double> > wPath; /**< World path */
  //std::vector< std::vector<int> > vPath; /**< Vertex path */
  /*
  InitSearch();

  for( size_t i = 0; i < _mNumPaths; ++i ) {
    wPath.resize(0);
    path = FindPath( _sx, _sy, _sz );
    printf("Found path of size: %d \n", path.size() );
    wPaths.push_back( wPath );
    
    // Update the values
    ResetSearch();
    JointPaths();
    UpdateVertexValues();
  }
  

  return wPaths; */
}

/**
 * @function InitSearch
 */
void DiversePaths::InitSearch() {

}

/**
 * @function visualizePaths
 */
void DiversePaths::visualizePaths( boost::shared_ptr<pcl::visualization::PCLVisualizer> _viewer,
				   bool _viewObstacles ) {

  //-- Get counters ready
  reset_PCL_Tools_counters();
  
  //-- Get obstacles
  std::vector<Eigen::Vector3d> obstacles;
  mDf->getPointsFromField( obstacles );

  //-- Put them in PCD
  pcl::PointCloud<pcl::PointXYZ>::Ptr obstacleCloud = writePCD( obstacles );
  //pcl::PolygonMesh triangles;
  //getMesh( obstacleCloud, triangles );
  //viewMesh( &triangles, viewer );

  //-- View paths
  for( int i = 0; i < mPaths.size(); ++i ) {
    viewPath( mPaths[i], _viewer );
    viewBall( mPaths[i][0][0], mPaths[i][0][1], mPaths[i][0][2], 0.02, _viewer );
    viewBall( mPaths[i][mPaths[i].size() - 1][0], 
	      mPaths[i][mPaths[i].size() - 1][1], 
	      mPaths[i][mPaths[i].size() - 1][2], 
	      0.02, _viewer );    
  }

  // Visualize obstacles (or not)
  int obsR; int obsG; int obsB;
  obsR = 0; obsG = 255; obsB = 0;

  if( _viewObstacles ) {
    viewPCD( obstacleCloud, _viewer, obsR, obsG, obsB );
  }

}
