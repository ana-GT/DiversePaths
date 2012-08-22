/**
 * @file PCL_Tools
 * @brief
 * @author A. Huaman
 * @date 2012-08-15
 */
#include "PCL_Tools.h"

/**
 * @function createViewer
 */
boost::shared_ptr<pcl::visualization::PCLVisualizer> createViewer( int _r, int _g, int _b ) {

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer( new pcl::visualization::PCLVisualizer("3D viewer") );
  viewer->setBackgroundColor( _r/255.0, _g/255.0, _b/255.0 );
  return viewer;
}

/**
 * @function viewPCD
 **/
void viewPCD( pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud,
	      boost::shared_ptr<pcl::visualization::PCLVisualizer> _viewer,
	      int _r, int _g, int _b ) {

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorCloud( _cloud, _r, _g, _b );
  _viewer->addPointCloud<pcl::PointXYZ> ( _cloud, colorCloud, "Pointcloud" );
  _viewer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Pointcloud" );

  _viewer->addCoordinateSystem(1.0);
  _viewer->initCameraParameters();
}

/**
 * @function viewPath
 */
void viewPath( std::vector<Eigen::Vector3d> _path, 
	       boost::shared_ptr<pcl::visualization::PCLVisualizer> _viewer,
	       int _r, int _g, int _b ) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  cloud = writePCD( _path );
  viewPCD( cloud, _viewer, _r, _g, _b );
  
}

/**
 * @function viewPath
 */
void viewPath( std::vector< std::vector<double> > _path,
	       boost::shared_ptr<pcl::visualization::PCLVisualizer> _viewer,
	       int _r, int _g, int _b ) {
  
  // Create PCD
  pcl::PointCloud<pcl::PointXYZ>::Ptr pathCloud;
  pathCloud = writePCD( _path );
  
  // Color
  double r; double g; double b;

  r = ( _r % 256 )/255.0;
  g = ( _g % 256 )/255.0;
  b = ( _b % 256 )/255.0;

  int mCountPaths = 0;
  for( int j = 0; j < pathCloud->points.size() - 1; ++j ) {
    char linename[15];
    sprintf( linename, "path%d-%d", mCountPaths, j );
    std::string id(linename);
    _viewer->addLine<pcl::PointXYZ>( pathCloud->points[j], pathCloud->points[j + 1], r, g, b, id );
    _viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, id );
  }
}

/**
 * @function viewBall
 */
void viewBall( double _x, double _y, double _z,
	       std::string _name, double _radius,
	       boost::shared_ptr<pcl::visualization::PCLVisualizer> _viewer,
	       int _r, int _g, int _b ) {

  pcl::PointXYZ pos;
  pos.x = _x;
  pos.y = _y;
  pos.z = _z;
  _viewer->addSphere ( pos, _radius, _r/255.0, _g/255.0, _b/255.0, _name );
  _viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, _name );

}

/**
 * @function viewMesh
 **/
void viewMesh( pcl::PolygonMesh *_triangles,
	       boost::shared_ptr<pcl::visualization::PCLVisualizer> _viewer,
	       int _r, int _g, int _b ) {

  _viewer->addPolygonMesh( *_triangles );
} 


/**
 * @function readPCDFile
 */
void readPCDFile( char* _filename, 
		  pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud ) {
  
  if( pcl::io::loadPCDFile<pcl::PointXYZ>( _filename, *_cloud ) == -1 ) {
    printf("[readPCDFile] Could not read the file. Exiting! \n");
    return;
  }
  printf("[readPCDFile] Correctly loaded! \n");
  return;
}

/**
 * @function writePCD
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr writePCD( std::vector<Eigen::Vector3d> _points ) {
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ> );
  // Fill cloud
  cloud->width = _points.size();
  cloud->height = 1;
  cloud->is_dense = false;
  cloud->points.resize( cloud->width*cloud->height );

  for( size_t i = 0; i < cloud->points.size(); ++i ) {
    cloud->points[i].x = _points[i](0);
    cloud->points[i].y = _points[i](1);
    cloud->points[i].z = _points[i](2);
  }

  return cloud;
}

/**
 * @function writePCD
 */ 
pcl::PointCloud<pcl::PointXYZ>::Ptr writePCD( std::vector<std::vector< double > > _points ) {
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ> );
  
  // Fill cloud
  cloud->width = _points.size();
  cloud->height = 1;
  cloud->is_dense = false;
  cloud->points.resize( cloud->width*cloud->height );
  
  for( size_t i = 0; i < cloud->points.size(); ++i ) {
    cloud->points[i].x = _points[i][0];
    cloud->points[i].y = _points[i][1];
    cloud->points[i].z = _points[i][2];
  }
  
  return cloud;
} 


/**
 * @function getMesh
 **/
void getMesh( pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud, 
	      pcl::PolygonMesh &_triangles,
	      int _numNeighbors,
	      float _searchRadius ) {

  // Normal estimation *
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals( new pcl::PointCloud<pcl::Normal> );
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree( new pcl::search::KdTree<pcl::PointXYZ> );
  tree->setInputCloud( _cloud );
  n.setInputCloud( _cloud );
  n.setSearchMethod( tree );
  n.setKSearch( _numNeighbors );
  n.compute( *normals );
  //* normals should not contain the point normals  + surface curvatures
  
  // Concatenate the XYZ and normal fields*
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals( new pcl::PointCloud<pcl::PointNormal> );
  pcl::concatenateFields( *_cloud, *normals, *cloud_with_normals );
  //* cloud_with_normals = cloud + normals

  // Create search tree*
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2( new pcl::search::KdTree<pcl::PointNormal> );
  tree2->setInputCloud( cloud_with_normals );
  
  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;

  // Set the maximum distance between connected points
  gp3.setSearchRadius( _searchRadius );

  // Set typical values for the parameters 
  gp3.setMu(2.5);
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3);  
  gp3.setNormalConsistency(false);


  // Get result
  gp3.setInputCloud ( cloud_with_normals );
  gp3.setSearchMethod( tree2 );
  gp3.reconstruct( _triangles );

}

