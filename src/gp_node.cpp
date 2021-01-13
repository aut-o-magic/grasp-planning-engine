#include <cstdio>
#include <chrono>
#include <octomap/octomap.h>
#include <octomap_msgs/msg/octomap.hpp>
#include "rclcpp/rclcpp.hpp"

#include <Eigen/StdVector>
#include <Eigen/Geometry>

using namespace std::chrono_literals;

class gpNode : public rclcpp::Node
{
public:
    gpNode()
        : Node("gpNode"), count_(0)
    {
        //pub_ = this->create_publisher<octomap_msgs::msg::Octomap>("octomap", 10); // TODO create octree publisher
        timer_ = this->create_wall_timer(10s, std::bind(&gpNode::timer_callback, this));
        sub_ = this->create_subscription<octomap_msgs::msg::Octomap>("input_octomap", 10, std::bind(&gpNode::topic_callback, this, std::placeholders::_1));
    }

private:
    void timer_callback()
    {
        /*
        sensor_msgs::msg::PointCoctomap::OcTreeloud2::SharedPtr msg{world_pcl.get_ros_cloud()};

        RCLCPP_INFO(this->get_logger(), "Publish stamp: '%s'", msg->header.stamp);
        std::flush(std::cout);

        pub_->publish(std::move(*msg));
        */
    }

    void topic_callback(const octomap_msgs::msg::Octomap::SharedPtr octomap)
    {
        
    }

    size_t count_;
    //rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    Eigen::Affine3f prev_Ti_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<gpNode>());
    rclcpp::shutdown();
    return 0;
}