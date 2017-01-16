cmake_minimum_required(VERSION 2.8.9)
project (sandbox)
add_definitions(-std=c++11)

find_package(PCL 1.8 REQUIRED)
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(include)

add_library(${PROJECT_NAME}
			src/utils/surface_segmentation.cpp
			src/pcl/pcl_greedy_projection.cpp)

add_executable (hello src/hello/hello_world.cpp)
add_executable (ensenso_util src/utils/ensenso_util.cpp)
add_executable (pcl-greedy-projection src/pcl/pcl_greedy_projection.cpp)
add_executable (pcd-to-vtp src/utils/PCDtoVTP.cpp)
add_executable (vtk-delaunay src/vtk/vtk_delaunay_projection.cpp)
target_link_libraries (ensenso_util ${PCL_LIBRARIES} ncurses)
target_link_libraries (pcl-greedy-projection ${PCL_LIBRARIES} ${PROJECT_NAME})
target_link_libraries (pcd-to-vtp ${PCL_LIBRARIES} ${PROJECT_NAME})

if(VTK_LIBRARIES)
  target_link_libraries(vtk-delaunay ${VTK_LIBRARIES})
else()
  target_link_libraries(vtk-delaunay vtkHybrid vtkWidgets)
endif()