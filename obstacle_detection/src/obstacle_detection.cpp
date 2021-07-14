#include <iostream>
#include <boost/shared_ptr.hpp>
#include <ros/console.h>

// PCL specific includes
#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/features/normal_3d.h>

//Filters and Downsampling
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/radius_outlier_removal.h>

//Clustering
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/features/normal_3d.h>

// ROS
#include <ros/ros.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
// Type Defs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> CloudT;

using namespace std;

double downsample_leaf_size;
double ground_min, ground_max, cluster_tolerance, min_cluster_size, max_cluster_size, distance_threshould;

ros::Publisher pub;
ros::Publisher pcl_pub;
ros::Publisher pub_cluster;

void filterGround(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_filtered)
{
  if (cloud->size() > 0)
  {
    //Create the model coefficient objects required for segmentation, coefficients and the point index collection object inliers that store the inner points
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // Create segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional configuration, setting model coefficients need to be optimized
    seg.setOptimizeCoefficients(true);
    // Necessary configuration, set the segmentation model type, random parameter estimation method used, distance threshold, input point cloud
    seg.setModelType(pcl::SACMODEL_PLANE); //Set model type
    seg.setMethodType(pcl::SAC_RANSAC);    //Set random sampling consistency method type
    // you can modify the parameter below
    seg.setMaxIterations(10000);                   //Indicates the maximum distance from the point to the estimated model,
    seg.setDistanceThreshold(distance_threshould); // 0.15 Set the distance threshold. The distance threshold determines the condition that the point is considered to be an inside point.
    seg.setInputCloud(cloud);
    //Initiate segmentation, store segmentation results to point geometric inliers and store plane model coefficients
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0)
      cout << "error! Could not found any inliers!" << endl;
    // extract ground
    // Extract the segmented point set on the plane from the point cloud
    pcl::ExtractIndices<pcl::PointXYZ> extractor; //Point extraction object
    extractor.setInputCloud(cloud);
    extractor.setIndices(inliers);
    extractor.setNegative(true);
    extractor.filter(*cloud_filtered);
    // vise-versa, remove the ground not just extract the ground
    // just setNegative to be true
  }
  else
  {
    cout << "no data!" << endl;
  }
}

void clusterPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud)
{
  //pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr clustered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  /* Creating the KdTree from input point cloud*/
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(input_cloud);

  /* Here we are creating a vector of PointIndices, which contains the actual index
  * information in a vector<int>. The indices of each detected cluster are saved here.
  * Cluster_indices is a vector containing one instance of PointIndices for each detected 
  * cluster. Cluster_indices[0] contain all indices of the first cluster in input point cloud.
  */
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(cluster_tolerance);
  ec.setMinClusterSize(min_cluster_size);
  ec.setMaxClusterSize(max_cluster_size);
  ec.setSearchMethod(tree);
  ec.setInputCloud(input_cloud);
  /* Extract the clusters out of pc and save indices in cluster_indices.*/
  ec.extract(cluster_indices);

  /* To separate each cluster out of the vector<PointIndices> we have to 
  * iterate through cluster_indices, create a new PointCloud for each 
  * entry and write all points of the current cluster in the PointCloud. 
  */
  std::vector<pcl::PointIndices>::const_iterator it;
  std::vector<int>::const_iterator pit;
  int cluster_size = 0;
  for (it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZI>);
    for (pit = it->indices.begin(); pit != it->indices.end(); pit++)
    {
      pcl::PointXYZI cloud_xyzi;
      cloud_xyzi.x = input_cloud->points[*pit].x;
      cloud_xyzi.y = input_cloud->points[*pit].y;
      cloud_xyzi.z = input_cloud->points[*pit].z;
      cloud_xyzi.intensity = cluster_size;
      cloud_cluster->points.push_back(cloud_xyzi);
    }

    //Merge current clusters to whole point cloud
    *clustered_cloud += *cloud_cluster;
    cluster_size++;
  }

  sensor_msgs::PointCloud2 clusters;
  pcl::toROSMsg(*clustered_cloud, clusters);
  clusters.header.frame_id = "/os1_lidar";
  clusters.header.stamp = ros::Time::now();
  pub_cluster.publish(clusters);
}

void pointCloudCb(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr current_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*in_cloud_ptr, *current_pc_ptr);
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(current_pc_ptr);
  vg.setLeafSize(downsample_leaf_size, downsample_leaf_size, downsample_leaf_size);
  vg.filter(*filtered_pc_ptr);

  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(filtered_pc_ptr);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(ground_min, ground_max);
  pcl::PointCloud<pcl::PointXYZ>::Ptr floorRemoved(new pcl::PointCloud<pcl::PointXYZ>);
  pass.filter(*floorRemoved);

  sensor_msgs::PointCloud2 pub_pc;
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  //filterGround(floorRemoved, cloud_filtered);

  //pcl::toROSMsg(*noiseRemoved, pub_pc);
  pcl::toROSMsg(*floorRemoved, pub_pc);

  //pcl::toROSMsg(*cloud_filtered, pub_pc);
  pub_pc.header = in_cloud_ptr->header;
  pub_pc.header.stamp = ros::Time::now();
  pcl_pub.publish(pub_pc);

  // clusterPoints(cloud_filtered);
  //clusterPoints(floorRemoved);
}

int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "obstacle_detection");
  ros::NodeHandle nh, nh_priv("~");

  ROS_INFO("obstacle detection started start");
  downsample_leaf_size = 0.05; // 1cm
  nh_priv.getParam("downsample_leaf_size", downsample_leaf_size);
  nh_priv.getParam("ground_min", ground_min);
  nh_priv.getParam("ground_max", ground_max);
  nh_priv.getParam("cluster_tolerance", cluster_tolerance);
  nh_priv.getParam("min_cluster_size", min_cluster_size);
  nh_priv.getParam("max_cluster_size", max_cluster_size);
  nh_priv.getParam("distance_threshould", distance_threshould);

  ROS_INFO_STREAM("ground_min: " << ground_min << "; ground_max: " << ground_max);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/os_cloud_node/points", 1, pointCloudCb);

  // Publishers
  pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("keypoints", 1);
  pub_cluster = nh.advertise<sensor_msgs::PointCloud2>("clusters", 1);

  // Spin
  ros::Rate spin_rate(5);
  while (ros::ok())
  {
    spin_rate.sleep();
    ros::spinOnce();
  }
}
