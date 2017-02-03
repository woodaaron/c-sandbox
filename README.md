Sandbox for testing C++ Scripts

Currently includes:
  - Hello World
  - PCL
  	+ Moment of Inertia: Computes a boundary box for a point cloud
  	+ Greedy Projection: A greedy meshing algorithm using PCL libraries
  - VTK
  	+ Check Version: Reports the major and minor versions of the installed VTK library
  	+ Delaunay Projection: Meshing algorithm using VTK libraries. Heavily dependent on alpha parameter.
    + Smooth Poly Data Filter
  - Utils
  	+ Ensenso Util: minimal utility to capture single frames from an Ensenso camera and save them as pcd files
  	+ PCD to VTP: tool to convert pcl PCD files to vtk VTP files
  	+ Surface Segmentation: godel region growing tool
 