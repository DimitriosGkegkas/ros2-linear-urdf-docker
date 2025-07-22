#include <vector>
#include <cinttypes>
#include <memory>
#include <cstdlib>
#include <ctime>
#include <random>
#include <Eigen/Dense>
#include "geometry_msgs/msg/point.hpp"

#include <rclcpp/rclcpp.hpp>
#include <linear_algebra_interfaces/srv/get_least_squares.hpp>

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

using namespace std;

class Server : public rclcpp::Node
{
public:
  Server() : Node("server")
  {
    service_ = this->create_service<linear_algebra_interfaces::srv::GetLeastSquares>(
        "get_least_squares", std::bind(&Server::handleGetLeastSquares, this, _1, _2, _3));

    subscriber_ = this->create_subscription<geometry_msgs::msg::Point>(
        "least_squares_solution", 10, std::bind(&Server::handleSubscription, this, _1));

    RCLCPP_INFO(this->get_logger(), "Service 'get_least_squares' is ready.");
  }

private:
  rclcpp::Service<linear_algebra_interfaces::srv::GetLeastSquares>::SharedPtr service_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscriber_;

  void
  handleGetLeastSquares(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<linear_algebra_interfaces::srv::GetLeastSquares::Request> request,
      std::shared_ptr<linear_algebra_interfaces::srv::GetLeastSquares::Response> response)
  {
    (void)request_header;

    Eigen::MatrixXf A(request->b.size(), 3);
    Eigen::VectorXf b(request->b.size());

    for (size_t i = 0; i < request->b.size(); ++i)
    {
      b(i) = request->b[i];
      for (size_t j = 0; j < 3; ++j)
      {
        A(i, j) = request->a[i * 3 + j];
      }
    }

    RCLCPP_INFO(this->get_logger(), "Received request for least squares with A size [%ld, %ld] and b size [%ld]",
                A.rows(), A.cols(), b.size());

    Eigen::VectorXf sol = (A.transpose() * A).ldlt().solve(A.transpose() * b);
    std::stringstream ssol;
    ssol << sol.transpose();
    RCLCPP_INFO_STREAM(this->get_logger(), "Least Square Solution: " << ssol.str());

    Eigen::Vector3f translation = Eigen::Vector3f::Random(3);
    response->t.translation.x = translation[0];
    response->t.translation.y = translation[1];
    response->t.translation.z = translation[2];

    Eigen::Quaternionf q = Eigen::Quaternionf::UnitRandom();
    response->t.rotation.x = q.x();
    response->t.rotation.y = q.y();
    response->t.rotation.z = q.z();
    response->t.rotation.w = q.w();

    // Apply translation and rotation to the solution
    auto x_t = q * sol + translation;

    response->x_t.x = x_t[0];
    response->x_t.y = x_t[1];
    response->x_t.z = x_t[2];

    std::stringstream sx_t;
    sx_t << x_t.transpose();
    RCLCPP_INFO_STREAM(this->get_logger(), "Transformed Least Square Solution: " << sx_t.str());
  }

  void handleSubscription(const geometry_msgs::msg::Point::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Received least squares solution: [%f, %f, %f]",
                msg->x, msg->y, msg->z);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(make_shared<Server>());
  rclcpp::shutdown();
  return 0;
}