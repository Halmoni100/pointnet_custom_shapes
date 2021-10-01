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
using namespace std;
using boost::filesystem::path;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

vector<vector<float>> getRotationAngles(float startAngle, float endAngle, int numIntervals)
{
  float intervalAngle = (endAngle - startAngle) / numIntervals;
  vector<float> intervals;
  float currAngle = startAngle;
  while (currAngle < endAngle) {
    intervals.push_back(currAngle);
    currAngle += intervalAngle;
  }
  
  vector<vector<float>> rotationAngles;
  for (float zAngle: intervals) {
    for (float yAngle: intervals) {
      for (float xAngle: intervals) {
        rotationAngles.push_back({zAngle, yAngle, xAngle});
      }
    }
  }
  return rotationAngles;
}

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
  vector<vector<float>> rotationAngles = getRotationAngles(startAngle, endAngle, numIntervals);
  int i = 0;
  for (vector<float> angles: rotationAngles) {
    PointCloud::Ptr outputCloud(new PointCloud);

    Matrix3f M;
    M = AngleAxisf(angles[0], Vector3f::UnitZ())
        * AngleAxisf(angles[1], Vector3f::UnitY()) 
        * AngleAxisf(angles[2], Vector3f::UnitX());
    for (auto point: *inputCloud) {
      Vector3f p(point.x, point.y, point.z);
      Vector3f rotated_p = M * p;
      pcl::PointXYZ newPoint(rotated_p[0], rotated_p[1], rotated_p[2]);
      outputCloud->push_back(newPoint);
    } 

    string outputFilename = inputFilenameStem + "_rot" + to_string(i) + ".pcd";
    path outputFilepath = outputDir / outputFilename;
    pcl::io::savePCDFile(outputFilepath.string(), *outputCloud);

    i += 1;
  }
}	
