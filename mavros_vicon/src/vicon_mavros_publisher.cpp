#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node{
  public:
    MinimalPublisher(): Node("mavros_pose_node"){
      // Publisher for pose estimator (MAVROS vision_pose -> VISION_POSITION_ESTIMATE)
      estimator_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/vision_pose/pose", 10);

      // Create subscription to the position of the UAV (posestamped) 
      vicon_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>(
				"/vicon/HexaDanyy/HexaDanyy", 10, std::bind(&MinimalPublisher::vicon_callback, this, std::placeholders::_1));
      // Create timer to publish the pose of the UAV
      timer = this->create_wall_timer(20ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
   
    void timer_callback(){
      estimator_publisher->publish(pose);
    }

    void vicon_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
      geometry_msgs::msg::PoseStamped pose_out;
      pose_out.header = msg->header;
      pose_out.header.frame_id = "map";
      // Same frame mapping as viconpub: position [0]=y, [1]=x, [2]=-z (NED)
      pose_out.pose.position.x = msg->pose.position.y;
      pose_out.pose.position.y = msg->pose.position.x;
      pose_out.pose.position.z = -1 * msg->pose.position.z;
      // Same quaternion mapping as viconpub: q[0]=w, q[1]=y, q[2]=x, q[3]=-z -> (x,y,z,w)
      pose_out.pose.orientation.w = msg->pose.orientation.w;
      pose_out.pose.orientation.x = msg->pose.orientation.x;
      pose_out.pose.orientation.y = msg->pose.orientation.y;
      pose_out.pose.orientation.z = -1 * msg->pose.orientation.z;

      this->pose = pose_out;
    }

    geometry_msgs::msg::PoseStamped pose;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr estimator_publisher;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vicon_subscriber;

    rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
