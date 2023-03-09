#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
 
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
 
class PclSub : public rclcpp::Node
{
public:
    PclSub(std::string name)
        : Node(name)
    {
        using std::placeholders::_1;
        sub_novel = this->create_subscription<sensor_msgs::msg::PointCloud2>("//此处为所需订阅的点云话题", 1, std::bind(&PclSub::topic_callback, this, std::placeholders::_1));
    }
 
private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_novel;
    
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
       //此处为需要对点云进行处理的操作
    }
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
 
    auto node = std::make_shared<PclSub>("pclsub");
 
    rclcpp::spin(node);
 
    //rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
