#include <boost/program_options/options_description.hpp>
#include <boost/program_options/variables_map.hpp>
#include <iostream>
#include <math.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/point_types.h>
#include <pcl_ros/io/pcd_io.h>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

using Eigen::Matrix3f;
using Eigen::Vector3f;
using Eigen::AngleAxisf;
using namespace std;
using boost::filesystem::path;

namespace po = boost::program_options;

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

vector<float> getScales(int numIntervals, float maxScale)
{
  if (numIntervals == 1)
  {
    return {maxScale};
  }

  vector<float> scales;
  float intervalScale = maxScale / numIntervals;
  float currScale = 0.0;
  while (currScale < maxScale)
  {
    scales.push_back(currScale);
    currScale += intervalScale;
  }
  return scales;
}

struct ScaleIntervalParams
{
  string axis;
  int numIntervals;
  float maxScaling;
};

pair<po::variables_map, vector<ScaleIntervalParams>> parse_arguments(int argc, char** argv)
{
  po::options_description desc{"Allowed options"};
  desc.add_options()
    ("help,h", "Transform pcd file by scaling then rotation")
    ("input,i", po::value<string>()->required(), "Path to .pcd input file")
    ("output,o", po::value<string>()->required(), "Path to output directory of transformed .pcd files")
    ("rotate,r", po::value<int>(), "Number of rotation intervals in 2pi")
    ("x", po::value<int>(), "Number of intervals for scaling in x direction")
    ("X", po::value<float>(), "Max scaling value in x direction")
    ("y", po::value<int>(), "Number of intervals for scaling in y direction")
    ("Y", po::value<float>(), "Max scaling value in y direction")
    ("z", po::value<int>(), "Number of intervals for scaling in z direction")
    ("Z", po::value<float>(), "Max scaling value in z direction")
  ;
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  vector<pair<string, string>> scalingOptions = {{"x","X"}, {"y","Y"}, {"z","Z"}}; 
  vector<ScaleIntervalParams> scalingArgs;
  for (auto options: scalingOptions)
  {
    if ( (vm.count(options.first) && !vm.count(options.second)) ||
         (!vm.count(options.first) && vm.count(options.second))) {
      cerr << "Both number of intervals and max scaling value must be given for a direction" << endl;
      exit(1);
    }
    if (vm.count(options.first))
    {
      scalingArgs.push_back({options.first, vm[options.first].as<int>(), vm[options.second].as<float>()});
    }
  }
  return {vm, scalingArgs};
}

int main(int argc, char** argv)
{
  pair<po::variables_map, vector<ScaleIntervalParams>> parse_args_return = parse_arguments(argc, argv);
  po::variables_map vm = parse_args_return.first;
  vector<ScaleIntervalParams> scalingArgs = parse_args_return.second;

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
