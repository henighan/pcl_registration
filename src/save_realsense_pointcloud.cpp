

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <pcl/io/pcd_io.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointXYZ PointXYZ;

class SavePublishedCloud
{
  public:
    SavePublishedCloud()
    {
      std::cout << "Made Object" << std::endl;
      sub = nh.subscribe<PointCloud>("/camera/depth_registered/points", 1, &SavePublishedCloud::callback, this);
      this->called_flag = false;
    }

    void callback(const PointCloud::ConstPtr& msg)
    {
      // std::cout << "Callback Called" << std::endl;
      if(!this->called_flag)
      {
        PointCloud::Ptr cloud (new PointCloud);
        *cloud = *msg;
        std::cout << "Got the Point Cloud!" << std::endl;
        this->called_flag = true;
        pcl::io::savePCDFileASCII("test_pcd.pcd", *cloud);
        std::cout << "Saved point cloud!" << std::endl;
      }
    }

  private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    bool called_flag;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "save_published_cloud");
  SavePublishedCloud f = SavePublishedCloud();
  ros::spin();
  return(0);
}
