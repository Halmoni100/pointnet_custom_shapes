// https://answers.ros.org/question/273046/pcd-visualization-in-rviz-closed/

#include <iostream>
#include <string>

#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/io/pcd_io.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
using namespace std;

int main(int argc, char** argv)
{
  if (argc != 2) {
    cerr << "Requires 1 argument for pcd file path" << endl;
    return 1; 
  }

  double x_cloud; double y_cloud; double z_cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<PointCloud> ("points2", 1);
  pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud);

  PointCloud::Ptr msg (new PointCloud);
  msg->header.frame_id = "frame1";
  msg->height = cloud->height;
  msg->width = cloud->width;

  for (size_t i = 0; i < cloud->size (); i++) {
  x_cloud = cloud->points[i].x;
  y_cloud = cloud->points[i].y;
  z_cloud = cloud->points[i].z;
  msg->points.push_back (pcl::PointXYZ (x_cloud, y_cloud, z_cloud));
  }

  ros::Rate loop_rate(4);


  while (nh.ok())
  {
    //msg->header.stamp = ros::Time::now().toNSec();
    pub.publish (msg);
    ros::spinOnce ();
    loop_rate.sleep ();
  }
}
