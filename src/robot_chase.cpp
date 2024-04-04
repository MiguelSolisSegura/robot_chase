#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "geometry_msgs/msg/detail/transform_stamped__struct.hpp"
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/utilities.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;
using Twist = geometry_msgs::msg::Twist;
using TransformStamped = geometry_msgs::msg::TransformStamped;

class RobotChase : public rclcpp::Node {
public:
    RobotChase() : rclcpp::Node::Node("robot_chase") {
        // Declare and acquire target frame
        target_frame_ = this->declare_parameter<std::string>("target_frame", "morty/base_link");
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        // Create a velocity publisher
        publisher_ = this->create_publisher<Twist>("rick/cmd_vel", 1);
        // Create a callback every second
        timer_ = this->create_wall_timer(250ms, [this](){return this->on_timer();});
    }
private:
    // Attributes
    std::string target_frame_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    rclcpp::Publisher<Twist>::SharedPtr publisher_{nullptr};
    rclcpp::TimerBase::SharedPtr timer_{nullptr};
    const float kp_yaw_ = 1.5;
    const float kp_distance_ = 0.2; 
    
    // Methods
    void on_timer() {
        // Store frame names
        std::string fromFrame = target_frame_.c_str();
        std::string toFrame = "rick/base_link";

        // Look up transformations between frames
        TransformStamped t;
        try {
            t = tf_buffer_->lookupTransform(toFrame, fromFrame, tf2::TimePointZero);
        } 
        catch (const tf2::TransformException & ex) {
            RCLCPP_ERROR(this->get_logger(), "Error in transformation: %s", ex.what());
            return;
        }
        
        // Compute velocities
        auto x = t.transform.translation.x;
        auto y = t.transform.translation.y;
        RCLCPP_DEBUG(this->get_logger(), "X: %.2f, Y: %.2f, Z: %.2f.", x, y, t.transform.translation.z);

        float error_yaw = kp_yaw_ * std::atan2(y, x);
        float error_distance = kp_distance_ * std::sqrt(x*x + y*y);
        RCLCPP_DEBUG(this->get_logger(), "Error distance: %.4f", error_distance);
        Twist vel_msg;
        if (error_distance > 0.08) {
            vel_msg.angular.z = error_yaw;
            vel_msg.linear.x = error_distance;
        }
        else {
            vel_msg.angular.z = 0;
            vel_msg.linear.x = 0;
        }
        // Pubish velocity
        publisher_->publish(vel_msg);
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotChase>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}