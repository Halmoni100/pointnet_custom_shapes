#include <iostream>
#include <math.h>
#include <random>

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

template <class T>
vector<T> getRandomSubset(vector<T> input, float ratio)
{
  vector<T> subset(input);
  random_shuffle(subset.begin(), subset.end());
  size_t subsetSize = input.size() * ratio;
  size_t numElementsToRemove = input.size() - subsetSize;
  for (int i = 0; i < numElementsToRemove; i++)
  {
    subset.pop_back();
  }
  return subset;
}


vector<string> scalingAxes = {"X","Y","Z"}; 

struct ScaleIntervalParams
{
  int numIntervals;
  float minScale;
  float maxScale;
};

vector<float> getScales(ScaleIntervalParams params)
{
  if (params.numIntervals == 1)
  {
    return {params.minScale};
  }

  vector<float> scales;
  float intervalScale = (params.maxScale - params.minScale) / (params.numIntervals - 1);
  float currScale = params.minScale;
  while (currScale <= params.maxScale)
  {
    scales.push_back(currScale);
    currScale += intervalScale;
  }
  return scales;
}

struct TranslateParams
{
  path inputFilepath;
  path outputPath;
  int numIntervalsIn2PI;
  float rotationsRatio;
  float pertubationRatio;
  map<string, ScaleIntervalParams> scalingArgs;
};

vector<vector<float>> getXYZScales(map<string, ScaleIntervalParams> scalingArgs)
{
  map<string, vector<float>> scalesByAxis;
  for (string axis: scalingAxes)
  {
    if (scalingArgs.find(axis) == scalingArgs.end())
    {
      scalesByAxis[axis] = {1.0};
    } else {
      scalesByAxis[axis] = getScales(scalingArgs[axis]);
    }
  } 

  vector<vector<float>> xyzScales;
  for (float zScale: scalesByAxis["Z"]) {
    for (float yScale: scalesByAxis["Y"]) {
      for (float xScale: scalesByAxis["X"]) {
        xyzScales.push_back({xScale, yScale, zScale});
      }
    }
  }
  return xyzScales;
}

TranslateParams parse_arguments(int argc, char** argv)
{
  po::options_description desc{"Allowed options"};
  desc.add_options()
    ("help,h", "Transform pcd file by scaling then rotation")
    ("input,i", po::value<string>(), "Path to .pcd input file")
    ("output,o", po::value<string>(), "Path to output directory of transformed .pcd files")
    ("rotate,r", po::value<int>()->default_value(1), "Number of rotation intervals in 2pi")
    ("rotationsRatio,R", po::value<float>()->default_value(1.0), "Ratio b/w (0,1] of rotations to take from euler angle set.  Rotations will be chosen at random")
    ("pertubationRatio,P", po::value<float>()->default_value(0.0), "Size of rotation pertubation relative to interval angle")
  ;
  for (string axis: scalingAxes)
  {
    desc.add_options()
      (("num" + axis).c_str(), po::value<int>(), ("Number of intervals for scaling in " + axis + " direction").c_str())
      (("min" + axis).c_str(), po::value<float>(), ("Min scaling value in " + axis + " direction").c_str())
      (("max" + axis).c_str(), po::value<float>(), ("Max scaling value in " + axis + " direction").c_str())
    ;
  }
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help")) {
    cout << desc << endl;
    exit(0);
  }

  map<string, ScaleIntervalParams> scalingArgs;
  for (string axis: scalingAxes)
  {
    string numArg = "num" + axis;
    string minArg = "min" + axis;
    string maxArg = "max" + axis;
    bool valuesGiven = true;
    if (vm.count(numArg) != (vm.count(minArg) || vm.count(maxArg))) { 
      cerr << "Both number of intervals and max scaling value must be given for a direction" << endl;
      exit(1);
    }
    if (vm.count(numArg))
    {
      scalingArgs[axis] = {vm[numArg].as<int>(), vm[minArg].as<float>(), vm[maxArg].as<float>()};
    }
  }

  float rotationsRatio = vm["rotationsRatio"].as<float>();
  if (rotationsRatio > 1 || rotationsRatio <= 0)
  {
    cerr << "rotationsRatio must be b/w (0,1]" << endl;
    exit(1);
  }

  TranslateParams params = {vm["input"].as<string>(), vm["output"].as<string>(), 
                            vm["rotate"].as<int>(), rotationsRatio, vm["pertubationRatio"].as<float>(),
                            scalingArgs};
  return params;
}

PointCloud::Ptr getTransformedCloud(PointCloud::Ptr inputCloud, Matrix3f transform)
{
  PointCloud::Ptr transformedCloud(new PointCloud);
  for (auto point: *inputCloud) {
    Vector3f p(point.x, point.y, point.z);
    Vector3f rotated_p = transform * p;
    pcl::PointXYZ newPoint(rotated_p[0], rotated_p[1], rotated_p[2]);
    transformedCloud->push_back(newPoint);
  } 
  return transformedCloud; 
}

int main(int argc, char** argv)
{
  TranslateParams params = parse_arguments(argc, argv);
  
  string inputFilenameStem = params.inputFilepath.stem().string();
   
  PointCloud::Ptr inputCloud(new PointCloud);
  pcl::io::loadPCDFile(params.inputFilepath.string(), *inputCloud);

  // Scaling
  
  vector<PointCloud::Ptr> scaledClouds;
  
  vector<vector<float>> xyzScales = getXYZScales(params.scalingArgs);
  for (vector<float> scales: xyzScales)
  {
    Matrix3f M;
    M << scales[0], 0, 0,
         0, scales[1], 0,
         0, 0, scales[2];
    scaledClouds.push_back(getTransformedCloud(inputCloud, M));
  }

  // Rotation (combined with scaling)

  srand(1234);

  float intervalAngle = 2*M_PI / params.numIntervalsIn2PI;

  float maxPertubation = params.pertubationRatio * intervalAngle;
  auto get_pertubation = [maxPertubation]()
  {
    float x = (float) rand() / ((float) RAND_MAX + 1.0);
    return maxPertubation * (2*x - 1);
  };

  vector<PointCloud::Ptr> rotAndScaledClouds;

  vector<vector<float>> rotationAngles = getRandomSubset(getRotationAngles(intervalAngle), params.rotationsRatio);
  for (vector<float> angles: rotationAngles) {
    Matrix3f M;
    float angleZ = angles[0] + get_pertubation();
    float angleY = angles[1] + get_pertubation();
    float angleX = angles[2] + get_pertubation();
    M = AngleAxisf(angleZ, Vector3f::UnitZ())
        * AngleAxisf(angleY, Vector3f::UnitY()) 
        * AngleAxisf(angleX, Vector3f::UnitX());
    for (PointCloud::Ptr scaledCloud: scaledClouds) {
      rotAndScaledClouds.push_back(getTransformedCloud(scaledCloud, M));
    }
  }

  // Write transformed clouds

  for (size_t i = 0; i < rotAndScaledClouds.size(); i++) {
    PointCloud::Ptr transformedCloud = rotAndScaledClouds[i];
    string outputFilename = inputFilenameStem + "_transformed" + to_string(i) + ".pcd";
    path outputFilepath = params.outputPath / outputFilename;
    pcl::io::savePCDFile(outputFilepath.string(), *transformedCloud);
  }
}	
