#include <iostream>
#include <csignal>
#include <curses.h>
#include <pcl/io/ensenso_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <sstream>

const std::string FDIR = "/home/ros-industrial/Pictures/";

/** @brief PCL Ensenso object pointer */
pcl::EnsensoGrabber::Ptr ensenso_ptr;
bool is_open;
int i;

void cleanup( int signum )
{
  // Close curses terminal
  endwin(); 

  // If ensesno is open, close it before exiting
  if(is_open)
    ensenso_ptr->closeDevice();

  // Close application
  exit(signum);  
}

int main (int argc, char** argv)
{

  signal(SIGINT, cleanup);

  is_open = false;
  i = 0;
  ensenso_ptr.reset (new pcl::EnsensoGrabber);
  ensenso_ptr->openTcpPort ();
  is_open = ensenso_ptr->openDevice ();

  // Set up curses
  initscr();
  cbreak();
  addstr("Device Open\nPress 's' to capture the point cloud or 'q' to quit.\n");

  while(true)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    noecho();
    int input = getch();
    if(input != -1)
    {
      switch(input)
      {
        case 's':
        {
          // Grab a single frame
          ensenso_ptr->grabSingleCloud(cloud);
          if(cloud.points.size() > 0)
          {
            std::string fname = FDIR + std::to_string(++i) + ".pcd";
            pcl::io::savePCDFileASCII (fname, cloud);
            std::string msg = fname + " - saved to file\n";
            const char* c = msg.c_str();
            addstr(c);
          }
          else
          {
            addstr("Cloud was empty - no file saved. Did you remember to switch the network configuration?\n");
          }
          break;
        }

        case 'q':
        {
          cleanup(0);
        }

        case 'c':
        {
          clear();
          addstr("Device Open\nPress 's' to capture the point cloud or 'q' to quit.\n");
          break;
        }
      }

    }
  }

  cleanup(0);

  return (0);
}
