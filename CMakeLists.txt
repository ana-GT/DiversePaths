# Project

cmake_minimum_required( VERSION 2.8 FATAL_ERROR )
project( Test1 )
find_package( PCL 1.3 REQUIRED )
include_directories( ${PCL_INCLUDE_DIRS} . ROS_Tools )
link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

set( ROS_tDf  ROS_Tools/distance_field )
set( ROS_t ROS_Tools)


set( RosFx_srcs ${ROS_tDf}/pf_distance_field.cpp DiversePaths.cpp  ${ROS_t}/bresenham.cpp )
set( PCL_srcs PCL_Tools/PCL_Tools.cpp )
set( src_files main.cpp DiversePaths.cpp )
set( hdr_files )

add_executable( main ${src_files} ${hdr_files} ${RosFx_srcs} ${PCL_srcs} )
target_link_libraries( main ${PCL_LIBRARIES} )

# Test 1
add_executable( test1 test1.cpp ${hdr_files} ${RosFx_srcs} ${PCL_srcs} )
target_link_libraries( test1 ${PCL_LIBRARIES} )

# Test 2
#add_executable( test2 test2.cpp ${hdr_files} ${RosFx_srcs} ${PCL_srcs} )
#target_link_libraries( test2 ${PCL_LIBRARIES} )

# Test 3
#add_executable( test3 test3.cpp ${hdr_files} ${RosFx_srcs} ${PCL_srcs} )
#target_link_libraries( test3 ${PCL_LIBRARIES} )

# Test 4
#add_executable( test4 test4.cpp ${hdr_files} ${RosFx_srcs} ${PCL_srcs} )
#target_link_libraries( test4 ${PCL_LIBRARIES} )

# Test 5
#add_executable( test5 test5.cpp ${hdr_files} ${RosFx_srcs} ${PCL_srcs} )
#target_link_libraries( test5 ${PCL_LIBRARIES} )

# Test 6
#add_executable( test6 test6.cpp ${hdr_files} ${RosFx_srcs} ${PCL_srcs} )
#target_link_libraries( test6 ${PCL_LIBRARIES} )

# Test 7
#add_executable( test7 test7.cpp ${hdr_files} ${RosFx_srcs} ${PCL_srcs} )
#target_link_libraries( test7 ${PCL_LIBRARIES} )