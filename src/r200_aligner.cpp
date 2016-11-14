
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <iostream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <tf/transform_broadcaster.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointXYZ PointXYZ;

class AlignCameras
{
  public:
    boost::shared_ptr<pcl::visualization::PCLVisualizer>
      viewer_final;
    AlignCameras():
    target_cloud(new PointCloud),
    input_cloud(new PointCloud),
    viewer_final(new pcl::visualization::PCLVisualizer),
    pose1{-0.7, 0.0, 0.41, 0, 0.5, 0},
    pose2{0.362, -0.597, 0.418, 0.1, 0.503, 2.066}
    {

      cloud1_received = false;
      cloud2_received = false;
      transformed_made = false;
      cloud1_sub = nh.subscribe<PointCloud>("/camera1/depth_registered/points", 1, &AlignCameras::SetCloud1, this);
      cloud2_sub = nh.subscribe<PointCloud>("/camera2/depth_registered/points", 1, &AlignCameras::SetCloud2, this);

      // transform1.setOrigin( tf::Vector3(-0.7, 0.0, 0.41) );
      // q.setRPY(0, 0.5, 0);
      // transform1.setRotation(q);
      // double pose1[] = {-0.7, 0.0, 0.41, 0, 0.5, 0};
      // transform1 = GetTf(pose1);

      // transform2.setOrigin( tf::Vector3(0.362, -0.597, 0.418) );
      // q.setRPY(0.1, 0.503, 2.066);
      // transform2.setRotation(q);
      // transform2 = GetTf(pose2);
    
      zlim = 1.0;
      viewer_final->setBackgroundColor(0,0,0);
      std::cout << "Made Object" << std::endl;
    }

    void Broadcast()
    {
      br.sendTransform(tf::StampedTransform(GetTf(pose1), ros::Time::now(),
        "world", "camera1_link"));
      br.sendTransform(tf::StampedTransform(GetTf(pose2), ros::Time::now(),
        "world", "camera2_link"));
    }

    void UpdateVis()
    {
      if(cloud1_received)
      {
        // Coloring and visualizing target cloud
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
          target_color(target_cloud, 255, 0, 0);
        viewer_final->addPointCloud<pcl::PointXYZ>(target_cloud, target_color, "target_cloud");
        viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
            1, "target_cloud");
      }
    }

    void IcpAlign()
    {
      PointCloud::Ptr target_cloud (new PointCloud);
      PointCloud::Ptr input_cloud (new PointCloud);
      std::vector<int> indicies;

      pcl::removeNaNFromPointCloud(*cloud1, *target_cloud, indicies);
      zfilter(target_cloud, zlim);
      PositionCloud(target_cloud, pose1);
      pcl::removeNaNFromPointCloud(*cloud2, *input_cloud, indicies);
      zfilter(input_cloud, zlim);
      PositionCloud(input_cloud, pose2);
      
      pcl::IterativeClosestPoint<PointXYZ, PointXYZ> icp;
      icp.setTransformationEpsilon(1e-7);
      icp.setMaxCorrespondenceDistance(0.03);
      icp.setInputSource(input_cloud);
      icp.setInputTarget(target_cloud);
      // icp.setUseReciprocalCorrespondences(true);
      icp.align(*transformed_cloud);
      PointCloud::Ptr transformed_cloud (new PointCloud);
      
    }
 
  private:
    tf::TransformBroadcaster br;
    double pose1[6];
    double pose2[6];
    ros::NodeHandle nh;
    PointCloud::Ptr target_cloud;
    PointCloud::Ptr input_cloud;
    PointCloud::Ptr cloud1;
    PointCloud::Ptr cloud2;
    ros::Subscriber cloud1_sub;
    ros::Subscriber cloud2_sub;
    bool cloud1_received;
    bool cloud2_received;
    bool transformed_made;
    float zlim;
 
    void SetCloud1(const PointCloud::ConstPtr& msg)
    {
      //  pcl::removeNaNFromPointCloud(*msg, *target_cloud, indicies);
      //  zfilter(target_cloud, zlim);
      if(!cloud1_received)
      {
        *cloud1 = *msg;
        cloud1_received = true;
        std::cout << "cloud1 received" << std::endl;
        // UpdateVis();
      }
    }

    void SetCloud2(const PointCloud::ConstPtr& msg)
    {
      //  pcl::removeNaNFromPointCloud(*msg, *input_cloud, indicies);
      //  zfilter(input_cloud, zlim);
      if(!cloud2_received)
      {
        *cloud2 = *msg;
        cloud2_received = true;
        std::cout << "cloud2 received" << std::endl;
      }
    }

    // function to filter out points past a certain distance
    void zfilter(PointCloud::Ptr cloud, float upper_limit)
    {
      pcl::PassThrough<PointXYZ> pass;
      pass.setInputCloud(cloud);
      pass.setFilterFieldName("z");
      pass.setFilterLimits(0.0, upper_limit);
      pass.filter(*cloud);
    }

    tf::Transform GetTf(double pose[6])
    {
      tf::Quaternion q;
      tf::Transform camtf;
      camtf.setOrigin( tf::Vector3(pose[0], pose[1], pose[2]) );
      q.setRPY(pose[3], pose[4], pose[5]);
      camtf.setRotation(q);
      return camtf;
    }

    void PositionCloud(PointCloud::Ptr cloud, double pose[6])
    {
      Eigen::Affine3f camera_tranform;
      camera_tranform = pcl::getTransformation(pose[1], -pose[2], 
          pose[0], pose[4], pose[5], pose[3]);
      pcl::transformPointCloud (*cloud, *cloud, camera_tranform);
    }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "align_cameras");
  AlignCameras ac = AlignCameras();
  ros::Rate loop_rate(50);

  while (ros::ok())
  {
    ac.Broadcast();
    ac.viewer_final->spinOnce(20);
    ros::spinOnce();
    loop_rate.sleep();    
  }
}
