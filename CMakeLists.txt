cmake_minimum_required(VERSION 2.8.9)
project (sandbox)
add_definitions(-std=c++11)

find_package(PCL 1.8 REQUIRED)
find_package(VTK REQUIRED)
find_package(OpenMP REQUIRED)

if(OPENMP_FOUND)
message("OPENMP FOUND")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

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
add_executable (moi src/pcl/moment_of_inertia.cpp)
add_executable (checkVTKVersion src/vtk/checkVTKVersion.cpp)

target_link_libraries (ensenso_util ${PCL_LIBRARIES} ncurses)
target_link_libraries (pcl-greedy-projection ${PCL_LIBRARIES} ${PROJECT_NAME})
target_link_libraries (pcd-to-vtp ${PCL_LIBRARIES} ${PROJECT_NAME})
target_link_libraries (moi ${PCL_LIBRARIES})
target_link_libraries (checkVTKVersion ${VTK_LIBRARIES})

if(VTK_LIBRARIES)
  target_link_libraries(vtk-delaunay ${VTK_LIBRARIES})
else()
  target_link_libraries(vtk-delaunay vtkHybrid vtkWidgets)
endif()