#include <iostream>
#include <math.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/point_types.h>
#include <pcl_ros/io/pcd_io.h>
#include <boost/filesystem.hpp>

using Eigen::Matrix3f;
using Eigen::Vector3f;
using Eigen::AngleAxisf;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
using namespace std;

int main(int argc, char** argv)
{
  if (argc != 3) {
    cerr << "Requires 2 arguments: pcd input and pcd output path" << endl;
    return 1;
  }

  Matrix3f M;
  M = AngleAxisf(M_PI / 4, Vector3f::UnitZ())
      * AngleAxisf(M_PI / 4, Vector3f::UnitY()); 
   
  PointCloud::Ptr inputCloud(new PointCloud);
  PointCloud::Ptr outputCloud(new PointCloud);
  pcl::io::loadPCDFile(argv[1], *inputCloud);

  for (auto point: *inputCloud) {
    Vector3f p(point.x, point.y, point.z);
    Vector3f rotated_p = M * p;
    pcl::PointXYZ newPoint(rotated_p[0], rotated_p[1], rotated_p[2]);
    outputCloud->push_back(newPoint);
  } 

  pcl::io::savePCDFile(argv[2], *outputCloud);

  return 0;
}	
