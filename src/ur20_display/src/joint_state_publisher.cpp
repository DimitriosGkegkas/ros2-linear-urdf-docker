#include <vector>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class JointStatePublisher : public rclcpp::Node
{
public:
    JointStatePublisher()
        : Node("joint_state_publisher")
    {
        // Define Parameters
        this->declare_parameter<std::vector<std::string>>("joint_names", {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"});
        this->declare_parameter<std::vector<double>>("joint_positions", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

        // Create Publisher
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        _timer = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&JointStatePublisher::publish_joint_states, this));
        RCLCPP_INFO(this->get_logger(), "Joint State Publisher Node has been started.");
    }

private:
    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    
    void publish_joint_states()
    {
        auto message = sensor_msgs::msg::JointState();
        message.header.stamp = this->get_clock()->now();
        this->get_parameter("joint_names", message.name);
        this->get_parameter("joint_positions", message.position);
        publisher_->publish(message);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointStatePublisher>());
    rclcpp::shutdown();
    return 0;
}