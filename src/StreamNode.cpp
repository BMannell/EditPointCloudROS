
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "stream_pcl");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<PointCloud>("/clean_points", 1);
  ros::Subscriber sub = nh.subscribe<PointCloud>("/os1_cloud_node/points", 1,
                                                 [&](const PointCloud::ConstPtr &cloud) {
                                                   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
                                                   // build the condition
                                                   pcl::ConditionOr<pcl::PointXYZ>::Ptr range_cond(new pcl::ConditionOr<pcl::PointXYZ>());
                                                   range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::GT, 3.5)));
                                                   range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::LT, -3.5)));
                                                   range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::GT, 3.5)));
                                                   range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::LT, -3.5)));
                                                   // build the filter
                                                   pcl::ConditionalRemoval<pcl::PointXYZ> range_filt;
                                                   range_filt.setCondition(range_cond);
                                                   range_filt.setInputCloud(cloud);
                                                   range_filt.setKeepOrganized(false);
                                                   // apply filter
                                                   range_filt.filter(*cloud_filtered);

                                                   sensor_msgs::PointCloud2 newMsg{};
                                                   pcl::toROSMsg(*cloud_filtered, newMsg);

                                                   pub.publish(newMsg);
                                                 });

  ros::spin();

  return (0);
}