#include <vector>
#include <Eigen/Dense>

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "linear_algebra_interfaces/srv/get_least_squares.hpp"

using namespace std;

class Client : public rclcpp::Node
{
public:
  Client()
      : Node("client")
  {
    // Parameter declaration
    this->declare_parameter("b", vector<double>{1.0, 2.0, 3.0});
    this->declare_parameter("A", vector<double>{
                                     1.0, 2.0, 3.0,
                                     4.0, 5.0, 6.0,
                                     7.0, 8.0, 9.0});

    auto b = this->get_parameter("b").as_double_array();
    auto A = this->get_parameter("A").as_double_array();

    client_ = this->create_client<linear_algebra_interfaces::srv::GetLeastSquares>("get_least_squares");
    publisher_ = this->create_publisher<geometry_msgs::msg::Point>("least_squares_solution", 10);

    auto leastSquareSolution = this->getLeastSquares(A, b);
    if (leastSquareSolution)
    {
      RCLCPP_INFO(this->get_logger(), "Least squares solution: [%f, %f, %f]",
                  leastSquareSolution->x, leastSquareSolution->y, leastSquareSolution->z);
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Least squares failed");
    }
  }

private:
  rclcpp::Client<linear_algebra_interfaces::srv::GetLeastSquares>::SharedPtr client_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;

  optional<geometry_msgs::msg::Point> getLeastSquares(const vector<double> &A, const vector<double> &b)
  {

    this->logInfo(A, b);

    // Creating request
    auto request = std::make_shared<linear_algebra_interfaces::srv::GetLeastSquares::Request>();
    request->a = A;
    request->b = b;

    // Wait for the service to be available
    while (!client_->wait_for_service(1s))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        return std::nullopt;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
    }

    // Call the service asynchronously
    auto result = client_->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      // Process the response
      auto response = result.get();
      Eigen::Vector3f x_t(
          response->x_t.x,
          response->x_t.y,
          response->x_t.z);

      Eigen::Vector3f t(
          response->t.translation.x,
          response->t.translation.y,
          response->t.translation.z);

      Eigen::Quaternionf q(
          response->t.rotation.w, // Note: Eigen constructor order is (w, x, y, z)
          response->t.rotation.x,
          response->t.rotation.y,
          response->t.rotation.z);

      // Apply the inverse rotation and translation to the point to get x out of x_t
      Eigen::Vector3f x = q.inverse() * (x_t - t);

      // Create the Point message
      geometry_msgs::msg::Point point;
      point.x = x.x();
      point.y = x.y();
      point.z = x.z();
      return point;
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service get_least_squares");
      return std::nullopt;
    }
  }

  void logInfo(const std::vector<double> &A, const std::vector<double> &b)
  {
    std::stringstream ss;

    ss << "Requesting Least Squares with parameters:\n";
    ss << "Matrix A:\n";
    for (size_t i = 0; i < A.size(); i += 3)
    {
      if (i + 2 < A.size())
      {
        ss << A[i] << " " << A[i + 1] << " " << A[i + 2] << "\n";
      }
      else
      {
        ss << "Incomplete row at end of A vector!\n";
        break;
      }
    }

    ss << "Vector b: ";
    for (const auto &value : b)
    {
      ss << value << " ";
    }
    ss << "\n";

    RCLCPP_INFO_STREAM(this->get_logger(), ss.str());
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(make_shared<Client>());
  rclcpp::shutdown();
  return 0;
}