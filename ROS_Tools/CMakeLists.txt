# Project

cmake_minimum_required( VERSION 2.8 )
project( bresenham_test )


set( b3d_srcs bresenham.cpp )
set( b3d_hdrs bresenham.h )

add_executable( test_b3d bresenham_test ${b3d_srcs} ${b3d_hdrs} )
