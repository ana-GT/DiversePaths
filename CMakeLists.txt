# Project

cmake_minimum_required( VERSION 2.8 FATAL_ERROR )
project( Test1 )
find_package( PCL 1.3 REQUIRED )
include_directories( ${PCL_INCLUDE_DIRS} . RosFx )
link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

set( RosFx_srcs RosFx/distance_field/pf_distance_field.cpp RosFx/bfs_3d.cpp )
set( src_files main.cpp )
set( hdr_files )

add_executable( main ${src_files} ${hdr_files} ${RosFx_srcs} )
target_link_libraries( main ${PCL_LIBRARIES} )