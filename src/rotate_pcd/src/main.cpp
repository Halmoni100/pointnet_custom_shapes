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
using boost::filesystem::path;

int main(int argc, char** argv)
{
  if (argc != 3) {
    cerr << "Requires 2 arguments: pcd input and pcd output path" << endl;
    return 1;
  }
  path inputFilepath = path(argv[1]);
  path outputDir = path(argv[2]);
  string inputFilenameStem = inputFilepath.stem().string();
   
  PointCloud::Ptr inputCloud(new PointCloud);
  pcl::io::loadPCDFile(inputFilepath.string(), *inputCloud);

  float startAngle = 0;
  float endAngle = M_PI;
  float numIntervals = 20;
  float intervalAngle = (endAngle - startAngle) / numIntervals;

  float currAngle = startAngle;
  int i = 0;
  while (currAngle < endAngle) {
    PointCloud::Ptr outputCloud(new PointCloud);

    Matrix3f M;
    M = AngleAxisf(currAngle, Vector3f::UnitZ());
    //    * AngleAxisf(M_PI / 4, Vector3f::UnitY()); 
    for (auto point: *inputCloud) {
      Vector3f p(point.x, point.y, point.z);
      Vector3f rotated_p = M * p;
      pcl::PointXYZ newPoint(rotated_p[0], rotated_p[1], rotated_p[2]);
      outputCloud->push_back(newPoint);
    } 

    string outputFilename = inputFilenameStem + "_rot" + to_string(i) + ".pcd";
    path outputFilepath = outputDir / outputFilename;
    pcl::io::savePCDFile(outputFilepath.string(), *outputCloud);

    currAngle += intervalAngle;
    i += 1;
  }



  return 0;
}	
