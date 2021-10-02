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

vector<vector<float>> getRotationAngles(float intervalAngle)
{
  vector<float> intervals_0_to_2pi;
  float currAngle = 0;
  while (currAngle < 2*M_PI) {
    intervals_0_to_2pi.push_back(currAngle);
    currAngle += intervalAngle;
  }
  vector<float> intervals_npihalf_to_pihalf;
  currAngle = -M_PI/2;
  while (currAngle < M_PI/2) {
    intervals_npihalf_to_pihalf.push_back(currAngle);
    currAngle += intervalAngle;
  }
  
  vector<vector<float>> rotationAngles;
  for (float zAngle: intervals_0_to_2pi) {
    for (float yAngle: intervals_npihalf_to_pihalf) {
      for (float xAngle: intervals_0_to_2pi) {
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
  srand(1234);
  
  path inputFilepath = path(argv[1]);
  path outputDir = path(argv[2]);
  string inputFilenameStem = inputFilepath.stem().string();
   
  PointCloud::Ptr inputCloud(new PointCloud);
  pcl::io::loadPCDFile(inputFilepath.string(), *inputCloud);

  float numIntervalsIn2PI = 10;
  float intervalAngle = 2*M_PI / numIntervalsIn2PI;

  float randomIntervalAnglePertubationRatio = 0.25;
  float maxPertubation = randomIntervalAnglePertubationRatio * intervalAngle;
  auto get_pertubation = [maxPertubation]()
  {
    float x = (float) rand() / (float) RAND_MAX;
    return maxPertubation * (2*x - 1);
  };

  vector<vector<float>> rotationAngles = getRotationAngles(intervalAngle);
  int i = 0;
  cout << "ran1" << endl;
  for (vector<float> angles: rotationAngles) {
    PointCloud::Ptr outputCloud(new PointCloud);

    Matrix3f M;
    float angleZ = angles[0] + get_pertubation();
    float angleY = angles[1] + get_pertubation();
    float angleX = angles[2] + get_pertubation();
    M = AngleAxisf(angleZ, Vector3f::UnitZ())
        * AngleAxisf(angleY, Vector3f::UnitY()) 
        * AngleAxisf(angleX, Vector3f::UnitX());
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
