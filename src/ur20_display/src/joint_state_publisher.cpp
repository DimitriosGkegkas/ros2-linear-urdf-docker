#include <vector>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <Eigen/Geometry>
#include <rviz_visual_tools/rviz_visual_tools.hpp>
#include <Eigen/Dense>

#define PI 3.14159265358979323846

class JointStatePublisher : public rclcpp::Node
{
public:
    JointStatePublisher()
        : Node("joint_state_publisher")
    {
        // Define Parameters
        this->declare_parameter<std::vector<std::string>>("joint_names", {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"});
        this->declare_parameter<std::vector<double>>("joint_positions", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        this->declare_parameter<double>("period", 5.0); // Default period of 5 seconds

        period_ = this->get_parameter("period").as_double();

        // Initialize joint positions
        std::vector<double> joint_position_start;
        this->get_parameter("joint_positions", joint_position_start);
        joint_position_start_ = Eigen::VectorXd::Map(joint_position_start.data(), joint_position_start.size());
        // Random goal positions for demonstration
        // Is it with in safe limits? I should check first the joint limit through /robot_description
        joint_position_goal_ = Eigen::VectorXd::Random(joint_position_start_.size()) * PI; 
        joint_position_current_ = joint_position_start_;

        visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("base_link", "/rviz_visual_markers", this));

        // Create Publisher
        // sleep for a while to ensure the system is ready
        rclcpp::sleep_for(std::chrono::seconds(1));
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        _publish_timer = this->create_wall_timer(
            std::chrono::milliseconds(1),
            std::bind(&JointStatePublisher::publish_joint_states, this));
        RCLCPP_INFO(this->get_logger(), "Joint State Publisher Node has been started.");

        // Initialize tf listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        _transform_timer = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&JointStatePublisher::check_transform, this));
    }

private:
    rclcpp::TimerBase::SharedPtr _publish_timer;
    rclcpp::TimerBase::SharedPtr _transform_timer;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    Eigen::VectorXd joint_position_start_;
    Eigen::VectorXd joint_position_goal_;
    Eigen::VectorXd joint_position_current_;
    double start_time_;
    double period_;
    // For visualizing things in rviz
    rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

    void publish_joint_states()
    {
        if (!start_time_)
        {
            // Initialize start time on first call
            start_time_ = this->get_clock()->now().seconds();
        }
        auto t = (this->get_clock()->now().seconds() - start_time_);
        if (t <= period_ * 1.5)
        {
            joint_position_current_ = (joint_position_goal_ - joint_position_start_) * std::sin(2 * PI * t / period_ - PI / 2) / 2. + (joint_position_goal_ + joint_position_start_) / 2.;
        }

        auto message = sensor_msgs::msg::JointState();
        message.header.stamp = this->get_clock()->now();
        this->get_parameter("joint_names", message.name);
        message.position = std::vector<double>(joint_position_current_.data(), joint_position_current_.data() + joint_position_current_.size());
        publisher_->publish(message);
    }

    void check_transform()
    {
        try
        {
            // I guess there is an easier way to transform ros2 transforms to Eigen, (found package tf2_eigen)
            geometry_msgs::msg::TransformStamped transformStamped;
            transformStamped = tf_buffer_->lookupTransform("world", "gripper_link", tf2::TimePointZero);
            Eigen::Isometry3d T_world_to_gripper = Eigen::Isometry3d::Identity();
            T_world_to_gripper.translate(Eigen::Vector3d(transformStamped.transform.translation.x,
                                                         transformStamped.transform.translation.y,
                                                         transformStamped.transform.translation.z));
            T_world_to_gripper.rotate(Eigen::Quaterniond(transformStamped.transform.rotation.w,
                                                         transformStamped.transform.rotation.x,
                                                         transformStamped.transform.rotation.y,
                                                         transformStamped.transform.rotation.z));

            transformStamped = tf_buffer_->lookupTransform("forearm_link", "gripper_link", tf2::TimePointZero);
            Eigen::Isometry3d T_elbow_to_gripper = Eigen::Isometry3d::Identity();
            T_elbow_to_gripper.translate(Eigen::Vector3d(transformStamped.transform.translation.x,
                                                         transformStamped.transform.translation.y,
                                                         transformStamped.transform.translation.z));
            T_elbow_to_gripper.rotate(Eigen::Quaterniond(transformStamped.transform.rotation.w,
                                                         transformStamped.transform.rotation.x,
                                                         transformStamped.transform.rotation.y,
                                                         transformStamped.transform.rotation.z));

            transformStamped = tf_buffer_->lookupTransform("world", "forearm_link", tf2::TimePointZero);
            Eigen::Isometry3d T_world_to_elbow = Eigen::Isometry3d::Identity();
            T_world_to_elbow.translate(Eigen::Vector3d(transformStamped.transform.translation.x,
                                                       transformStamped.transform.translation.y,
                                                       transformStamped.transform.translation.z));
            T_world_to_elbow.rotate(Eigen::Quaterniond(transformStamped.transform.rotation.w,
                                                       transformStamped.transform.rotation.x,
                                                       transformStamped.transform.rotation.y,
                                                       transformStamped.transform.rotation.z));

            if ((T_world_to_elbow * T_elbow_to_gripper).isApprox(T_world_to_gripper))
            {
                visual_tools_->publishAxis(T_world_to_gripper, 2.0, 1.0, "Tf_elbow_gripper");
                RCLCPP_INFO(this->get_logger(), "Transforms are consistent");
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Transform mismatch detected");
            }
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not find transform: %s", ex.what());
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointStatePublisher>());
    rclcpp::shutdown();
    return 0;
}