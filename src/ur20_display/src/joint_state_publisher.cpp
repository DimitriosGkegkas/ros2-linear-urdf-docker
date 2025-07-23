#include <vector>
#include <string>
#include <memory>
#include <chrono>
#include <ctime>
#include <cstdlib>
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
        this->declare_parameter<double>("period", 10.0); // Default period of 5 seconds
        period_ = this->get_parameter("period").as_double();

        // Create Publisher
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

        // Initialize tf listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Visual tools for RViz
        visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("base_link", "/rviz_visual_markers", this));
        visual_tools_->loadMarkerPub();
        visual_tools_->setLifetime(2.0);

        // Create a timer to check transforms
        _transform_timer = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&JointStatePublisher::check_transform, this));

        // Initialize joint positions
        joint_position_start_ = Eigen::VectorXd::Map(
            this->get_parameter("joint_positions").as_double_array().data(),
            this->get_parameter("joint_positions").as_double_array().size());
        joint_position_current_ = joint_position_start_;
        this->set_joint_states(joint_position_start_);
        // Random goal positions for demonstration
        // Is it with in safe limits? I should check first the joint limit through /robot_description
        joint_position_goal_ = Eigen::VectorXd::Random(joint_position_start_.size()) * PI;

        // Start the trajectory loop timer
        _publish_timer = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(dt * 1000)),
            std::bind(&JointStatePublisher::trajectory_loop, this));
        RCLCPP_INFO(this->get_logger(), "Joint State Publisher Node has been started.");
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
    double current_time_ = 0.0;
    double dt = 0.01;
    double period_;
    // For visualizing things in rviz
    rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

    void trajectory_loop()
    {
        current_time_ += dt; // Not using rclcpp::Clock cause issues with the timer
        if (current_time_ <= period_ * 1.5)
        {
            joint_position_current_ = (joint_position_goal_ - joint_position_start_) * std::sin(2 * PI * current_time_ / period_ - PI / 2) / 2. + (joint_position_goal_ + joint_position_start_) / 2.;
        }
        this->set_joint_states(joint_position_current_);
    }

    void set_joint_states(Eigen::VectorXd joint_position)
    {
        auto message = sensor_msgs::msg::JointState();
        message.header.stamp = this->get_clock()->now();
        this->get_parameter("joint_names", message.name);
        message.position = std::vector<double>(joint_position.data(), joint_position.data() + joint_position.size());
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
                // Visualize the transforms in RViz
                visual_tools_->publishAxis(T_world_to_gripper, 0.5, 0.02, "Tf_elbow_gripper");
                T_world_to_gripper.translate(Eigen::Vector3d(0.5, 0.0, 0.));
                visual_tools_->publishText(
                    T_world_to_gripper,
                    "Tf_elbow_gripper",
                    rviz_visual_tools::WHITE,
                    rviz_visual_tools::XLARGE,
                    false);
                visual_tools_->trigger();
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
    std::srand(static_cast<unsigned>(std::time(nullptr)));
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointStatePublisher>());
    rclcpp::shutdown();
    return 0;
}