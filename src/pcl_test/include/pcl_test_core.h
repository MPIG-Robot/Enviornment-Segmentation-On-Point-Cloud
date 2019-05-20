#pragma once
#include<ros/ros.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/point_types.h>
#include<pcl/conversions.h>
#include<pcl_ros/transforms.h>
#include<pcl/filters/extract_indices.h>
#include<pcl/filters/voxel_grid.h>
#include<sensor_msgs/PointCloud2.h>

#define CLIP_HEIGHT 0.2
#define MIN_DISTANCE 2.4 
#define RADIAL_DIVIDER_ANGLE 0.18
#define SENSOR_HEIGHT 1.78

#define concentric_divider_distance_ 0.01
#define min_height_threshold_ 0.05
#define local_max_slope_ 8
#define general_max_slope_ 5
#define reclass_distance_threshold_ 0.2

class PclTestCore
{
	private:
	  ros::Subscriber sub_point_cloud_;
	  ros::Publisher pub_ground_,pub_no_ground_;
	  struct PointXYZIRTColor
	  {
	    pcl::PointXYZI point;
	    float radius;
	    float theta;
	    size_t radial_div;
	    size_t concentric_div;
	    size_t original_index;	
          };
	  typedef std::vector<PointXYZIRTColor>PointCloudXYZIRTColor;
	  size_t radial_dividers_num_;
	  size_t concentric_dividers_;
	  //ros::Publisher pub_filtered_points_;
	  void point_cb(const sensor_msgs::PointCloud2ConstPtr& in_cloud);
	  void clip_above(double clip_height,const pcl::PointCloud<pcl::PointXYZI>::Ptr in,const pcl::PointCloud<pcl::PointXYZI>::Ptr out);
	  void remove_close_pt(double min_distance,const pcl::PointCloud<pcl::PointXYZI>::Ptr in,const pcl::PointCloud<pcl::PointXYZI>::Ptr out);
	  void XYZI_to_RTZColor(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud,PointCloudXYZIRTColor &out_organized_points,std::vector<pcl::PointIndices> &out_radial_divided_indices,std::vector<PointCloudXYZIRTColor> &out_radial_ordered_clouds);
	  void classify_pc(std::vector<PointCloudXYZIRTColor> &in_radial_ordered_clouds,pcl::PointIndices &out_ground_indies,pcl::PointIndices &out_no_ground_indices);
	  void publish_cloud(const ros::Publisher &in_publisher,const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_to_publish_ptr,const std_msgs::Header &in_header);
	  
	public:
	  PclTestCore(ros::NodeHandle &nh);
	  ~PclTestCore();
	  void Spin();
};

