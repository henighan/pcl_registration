
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/transforms.h>
#include <pcl/search/kdtree.h>
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
    cloud1(new PointCloud),
    cloud2(new PointCloud),
    target_cloud(new PointCloud),
    input_cloud(new PointCloud),
    transformed_cloud(new PointCloud),
    viewer_final(new pcl::visualization::PCLVisualizer),
    pose1{-0.7, 0.0, 0.41, 0, 0.5, 0},
    pose2{0.362, -0.597, 0.418, 2.066, 0.503, 0.1}
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
      // Coloring and visualizing target cloud
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
        target_color(target_cloud, 255, 0, 0);
      viewer_final->addPointCloud<pcl::PointXYZ>(target_cloud, target_color, "target_cloud");
      viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
          1, "target_cloud");

      // Coloring and visualizing input cloud
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
        input_color(input_cloud, 0, 0, 255);
      viewer_final->addPointCloud<pcl::PointXYZ>(input_cloud, input_color, "input_cloud");
      viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
          1, "input_cloud");

      // Coloring and visualizing transformed cloud
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
        transformed_color(transformed_cloud, 0, 255, 0);
      viewer_final->addPointCloud<pcl::PointXYZ>(transformed_cloud, transformed_color, "transformed_cloud");
      viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
          1, "transformed_cloud");
      
      viewer_final->addCoordinateSystem(1.0, "global");

    }

    void IcpAlign()
    {
      // PointCloud::Ptr target_cloud (new PointCloud);
      // PointCloud::Ptr input_cloud (new PointCloud);
      std::vector<int> indicies;
      Eigen::Affine3f camera2_transform;
      // camera2_transform = pcl::getTransformation(-0.61, -0.41, 0.35, -0.5, 2.09, 0.0);
      // camera2_transform = pcl::getTransformation(-0.597, -0.418, 0.362, -0.503, 2.066, 0.1);
      camera2_transform = pcl::getTransformation(-0.8, -0.418, 0.3, -0.503, 2.066, 0.0);
      Eigen::Affine3f camera1_transform;
      camera1_transform = pcl::getTransformation(0.0, -0.41, -0.7, -0.5, 0.0, 0.0);

      pcl::removeNaNFromPointCloud(*cloud1, *target_cloud, indicies);
      zfilter(target_cloud, zlim);
      // pcl::transformPointCloud(*target_cloud, *target_cloud, GetTransMatrix(pose1));
      pcl::transformPointCloud(*target_cloud, *target_cloud, camera2_transform);
      // PositionCloud(target_cloud, pose1);
      pcl::removeNaNFromPointCloud(*cloud2, *input_cloud, indicies);
      zfilter(input_cloud, zlim);
      pcl::transformPointCloud(*input_cloud, *input_cloud, camera1_transform);
      // pcl::transformPointCloud(*input_cloud, *input_cloud, 
      //     GetTransMatrix(pose2));
      // PositionCloud(input_cloud, pose2);
      
      std::cout << " inside IcpAlign" << std::endl;
      pcl::IterativeClosestPoint<PointXYZ, PointXYZ> icp;
      icp.setTransformationEpsilon(1e-5);
      icp.setMaxCorrespondenceDistance(1e-2);
      icp.setInputSource(input_cloud);
      icp.setInputTarget(target_cloud);
      icp.setUseReciprocalCorrespondences(true);
      icp.align(*transformed_cloud);

      // PointCloud::Ptr input_filtered (new PointCloud);
      // PointCloud::Ptr target_filtered (new PointCloud);
      // PointCloud::Ptr output_cloud (new PointCloud);
      // pcl::ApproximateVoxelGrid<PointXYZ> avg;
      // avg.setLeafSize(0.02, 0.02, 0.02);
      // avg.setInputCloud(input_cloud);
      // avg.filter(*input_filtered);
      // avg.setInputCloud(target_cloud);
      // avg.filter(*target_filtered);
      // 
      // // initial alignment estimate
      // Eigen::AngleAxisf init_rotation(0.0, Eigen::Vector3f::UnitZ());
      // Eigen::Translation3f init_translation (0.0, 0.0, 0.0);
      // Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();


      // pcl::NormalDistributionsTransform<PointXYZ, PointXYZ> ndt;
      // ndt.setTransformationEpsilon(0.001);
      // ndt.setStepSize(0.1);
      // ndt.setResolution(1.0);
      // ndt.setMaximumIterations(35);
      // ndt.setInputSource(input_filtered);
      // ndt.setInputTarget(target_cloud);
      // ndt.align(*transformed_cloud, init_guess);
      // std::cout << "Normal Distributions Transform has converged:" << 
      //   ndt.hasConverged() << " score:" << ndt.getFitnessScore() << std::endl;

      // Eigen::Affine3f camera2_transform = 
      //   pcl::getTransformation(-0.61, -0.41, 0.35, -0.5, 2.09, 0.0);
      Eigen::Matrix4f cam2_aligned_trans = camera2_transform * icp.getFinalTransformation();
      // Eigen::Matrix4f cam2_aligned_trans = camera2_transform * ndt.getFinalTransformation();
      Eigen::Matrix3f cam2alignedrot = cam2_aligned_trans.matrix().block(0,0,3,3);
      Eigen::Vector3f cam2alignedrpy;
      cam2alignedrpy = cam2alignedrot.eulerAngles(1,0,2);
      std::cout << "camera 2 aligned transform: " << std::endl;
      std::cout << cam2_aligned_trans.matrix() << std::endl;
      std::cout << "aligned camera2 roll pitch yaw: " << 
        cam2alignedrpy << std::endl;

      UpdateVis();

    }
 
  private:
    tf::TransformBroadcaster br;
    double pose1[6];
    double pose2[6];
    ros::NodeHandle nh;
    PointCloud::Ptr target_cloud;
    PointCloud::Ptr input_cloud;
    PointCloud::Ptr transformed_cloud;
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
      // std::cout << "in setcloud1" << std::endl;
      if(!cloud1_received)
      {
        *cloud1 = *msg;
        cloud1_received = true;
        std::cout << "cloud1 received" << std::endl;
        if(cloud2_received)
        {
          IcpAlign();
        }
      }
    }

    void SetCloud2(const PointCloud::ConstPtr& msg)
    {
      //  pcl::removeNaNFromPointCloud(*msg, *input_cloud, indicies);
      //  zfilter(input_cloud, zlim);
      // std::cout << "in setcloud2" << std::endl;
      if(!cloud2_received)
      {
        *cloud2 = *msg;
        cloud2_received = true;
        std::cout << "cloud2 received" << std::endl;
        if(cloud1_received)
        {
          IcpAlign();
        }
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

    Eigen::Affine3f GetTransMatrix(double pose[6])
    {
      Eigen::Affine3f camera_tranform;
      camera_tranform = pcl::getTransformation(pose[1], -pose[2], pose[0], 
          pose[4], -pose[3], -pose[5]);
      // pcl::transformPointCloud (*cloud, *cloud, camera_tranform);
      return camera_tranform;
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
