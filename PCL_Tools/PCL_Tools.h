
/**
 * @file PCL_Tools.h
 * @author A. Huaman
 * @date 2012-08-15
 **/

#ifndef __PCL_TOOLS_H__
#define __PCL_TOOLS_H__

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/parse.h>
#include <pcl/surface/gp3.h>

#include <boost/thread/thread.hpp>
#include <iostream>

#include <Eigen/Core>


boost::shared_ptr<pcl::visualization::PCLVisualizer> createViewer( int _r = 125, int _g = 125, int _b = 125 );

void viewPCD( pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud,
	      boost::shared_ptr<pcl::visualization::PCLVisualizer> _viewer,
	      int _r = 0, int _g = 255, int _b = 0 );

void viewPath( std::vector<Eigen::Vector3d> _path, 
	       boost::shared_ptr<pcl::visualization::PCLVisualizer> _viewer,
	       int _r = 0, int _g = 0, int _b = 255 );

void viewPath( std::vector< std::vector<double> > _path,
	       boost::shared_ptr<pcl::visualization::PCLVisualizer> _viewer,
	       int _r = 0, int _g = 0, int _b = 255 );

void viewMesh( pcl::PolygonMesh *_triangles,
	       boost::shared_ptr<pcl::visualization::PCLVisualizer> _viewer,
	       int _r = 0, int _g = 255, int _b = 0 );

void viewBall( double _x, double _y, double _z,
	       std::string _name, double _radius,
	       boost::shared_ptr<pcl::visualization::PCLVisualizer> _viewer,
	       int _r = 255, int _g = 0, int _b = 0 );

void readPCDFile( char* _filename, 
		  pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud );

pcl::PointCloud<pcl::PointXYZ>::Ptr writePCD( std::vector<Eigen::Vector3d> _points );
pcl::PointCloud<pcl::PointXYZ>::Ptr writePCD( std::vector<std::vector<double> > _points );

void getMesh( pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud, 
	      pcl::PolygonMesh &_triangles,
	      int _numNeighbors = 50,
	      float _searchRadius = 0.015 ); // Usually 20 neighbors and 0.025 radius

#endif /** __PCL_TOOLS_H__  */
