#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkCellArray.h>
#include <vtkCleanPolyData.h>
#include <vtkDataSetMapper.h>
#include <vtkDataSetWriter.h>
#include <vtkDelaunay3D.h>
#include <vtkGeometryFilter.h>
#include <vtkMath.h>
#include <vtkPoints.h>
#include <vtkPointSource.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyDataNormals.h>
#include <vtkPolygon.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>
#include <vtkSmoothPolyDataFilter.h>
#include <vtkUnstructuredGrid.h>
#include <vtkVersion.h>
#include <vtkXMLPolyDataReader.h>


int main ( int argc, char *argv[] )
{
  if(argc < 2)
    {
    std::cout << "Usage: " << argv[0] << "alpha input.vtp output.vtk" << std::endl;
    return EXIT_FAILURE;
    }
 
  //Read the file
  vtkSmartPointer<vtkXMLPolyDataReader> reader = vtkSmartPointer<vtkXMLPolyDataReader>::New();
  reader->SetFileName(argv[2]);
 
  vtkSmartPointer<vtkDataSetMapper> originalMapper = vtkSmartPointer<vtkDataSetMapper>::New();
  originalMapper->SetInputConnection(reader->GetOutputPort());
 
  vtkSmartPointer<vtkActor> originalActor = vtkSmartPointer<vtkActor>::New();
  originalActor->SetMapper(originalMapper);
  originalActor->GetProperty()->SetColor(1,0,0);
 
  // Clean the polydata. This will remove duplicate points that may be
  // present in the input data.
  vtkSmartPointer<vtkCleanPolyData> cleaner = vtkSmartPointer<vtkCleanPolyData>::New();
  cleaner->SetInputConnection (reader->GetOutputPort());
 
  // Generate a tetrahedral mesh from the input points. By
  // default, the generated volume is the convex hull of the points.
  vtkSmartPointer<vtkDelaunay3D> delaunay3D = vtkSmartPointer<vtkDelaunay3D>::New();
  delaunay3D->SetInputConnection (cleaner->GetOutputPort());
 
  vtkSmartPointer<vtkDataSetMapper> delaunayMapper = vtkSmartPointer<vtkDataSetMapper>::New();
  delaunayMapper->SetInputConnection(delaunay3D->GetOutputPort());
 
  vtkSmartPointer<vtkActor> delaunayActor = vtkSmartPointer<vtkActor>::New();
  delaunayActor->SetMapper(delaunayMapper);
  delaunayActor->GetProperty()->SetColor(1,0,0);
 
  // Generate a mesh from the input points. If Alpha is non-zero, then
  // tetrahedra, triangles, edges and vertices that lie within the
  // alpha radius are output.
  vtkSmartPointer<vtkDelaunay3D> delaunay3DAlpha = vtkSmartPointer<vtkDelaunay3D>::New();
  delaunay3DAlpha->SetInputConnection (cleaner->GetOutputPort());
  delaunay3DAlpha->SetAlpha(std::atof(argv[1]));
 
  vtkSmartPointer<vtkDataSetMapper> delaunayAlphaMapper = vtkSmartPointer<vtkDataSetMapper>::New();
  delaunayAlphaMapper->SetInputConnection(delaunay3DAlpha->GetOutputPort());
 
  vtkSmartPointer<vtkActor> delaunayAlphaActor = vtkSmartPointer<vtkActor>::New();
  delaunayAlphaActor->SetMapper(delaunayAlphaMapper);
  delaunayAlphaActor->GetProperty()->SetColor(1,0,0);

  //***************************************************************************************************************************/
  // Smoothing??? Doesn't seem to do much...

  /*vtkSmartPointer<vtkGeometryFilter> geometryFilter = vtkSmartPointer<vtkGeometryFilter>::New();
  geometryFilter->SetInputConnection(delaunay3DAlpha->GetOutputPort());
  geometryFilter->Update(); 

  vtkSmartPointer<vtkSmoothPolyDataFilter> smoothFilter = vtkSmartPointer<vtkSmoothPolyDataFilter>::New();
  smoothFilter->SetInputConnection(geometryFilter->GetOutputPort());
  smoothFilter->SetNumberOfIterations(15);
  smoothFilter->SetRelaxationFactor(0.1);
  smoothFilter->FeatureEdgeSmoothingOff();
  smoothFilter->BoundarySmoothingOn();
  smoothFilter->Update();

  // Update normals on newly smoothed polydata
  vtkSmartPointer<vtkPolyDataNormals> normalGenerator = vtkSmartPointer<vtkPolyDataNormals>::New();
  normalGenerator->SetInputConnection(smoothFilter->GetOutputPort());
  normalGenerator->ComputePointNormalsOn();
  normalGenerator->ComputeCellNormalsOn();
  normalGenerator->Update(); 


  vtkSmartPointer<vtkPolyDataMapper> smoothedMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  smoothedMapper->SetInputConnection(geometryFilter->GetOutputPort());
  vtkSmartPointer<vtkActor> smoothedActor = vtkSmartPointer<vtkActor>::New();
  smoothedActor->SetMapper(smoothedMapper);*/
  
  //***************************************************************************************************************************/
  // Output the mesh

  vtkSmartPointer<vtkDataSetWriter> writer = vtkSmartPointer<vtkDataSetWriter>::New();
  writer->SetFileName ( argv[3] );
  writer->SetInputConnection ( delaunay3DAlpha->GetOutputPort() );
  writer->Write();

  //***************************************************************************************************************************/
 
  // Visualize
 
  // Define viewport ranges
  // (xmin, ymin, xmax, ymax)
  double leftViewport[4] = {0.0, 0.0, 0.33, 1.0};
  double centerViewport[4] = {0.33, 0.0, 0.66, 1.0};
  double rightViewport[4] = {0.66, 0.0, 1.0, 1.0};
 
  // Create a renderer, render window, and interactor
  vtkSmartPointer<vtkRenderer> originalRenderer = vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderer> delaunayRenderer = vtkSmartPointer<vtkRenderer>::New();
  //vtkSmartPointer<vtkRenderer> smoothedRenderer = vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderer> delaunayAlphaRenderer = vtkSmartPointer<vtkRenderer>::New();
 
  vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->SetSize(900,300);
 
  renderWindow->AddRenderer(originalRenderer);
  originalRenderer->SetViewport(leftViewport);
  renderWindow->AddRenderer(delaunayRenderer);
  delaunayRenderer->SetViewport(centerViewport);
  //renderWindow->AddRenderer(smoothedRenderer);
  //smoothedRenderer->SetViewport(centerViewport);
  renderWindow->AddRenderer(delaunayAlphaRenderer);
  delaunayAlphaRenderer->SetViewport(rightViewport);
 
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);
 
  originalRenderer->AddActor(originalActor);
  delaunayRenderer->AddActor(delaunayActor);
  //smoothedRenderer->AddActor(smoothedActor);
  delaunayAlphaRenderer->AddActor(delaunayAlphaActor);
 
  originalRenderer->SetBackground(.3, .6, .3);
  delaunayRenderer->SetBackground(.4, .6, .3);
  //smoothedRenderer->SetBackground(.4, .6, .3);
  delaunayAlphaRenderer->SetBackground(.5, .6, .3);
 
  // Render and interact
  renderWindow->Render();
  renderWindowInteractor->Start();
  return EXIT_SUCCESS;
}