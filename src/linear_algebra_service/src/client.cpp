#include <chrono>
#include <functional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

using namespace std;

class Client : public rclcpp::Node
{
public:
  Client()
      : Node("client")
  {
    this->declare_parameter("b", vector<double>{1.0, 2.0, 3.0});
    this->declare_parameter("A", vector<double>{
                                     1.0, 2.0, 3.0,
                                     4.0, 5.0, 6.0,
                                     7.0, 8.0, 9.0});
    this->declare_parameter("debug", false);

    auto b = this->get_parameter("b").as_double_array();
    auto A = this->parseMatrix(this->get_parameter("A").as_double_array(), b.size(), b.size());

    if (this->get_parameter("debug").as_bool()) {
      this->debugLog(A, b);
    }
  }

private:
  vector<vector<double>> parseMatrix(const vector<double> & matrix, unsigned rows, unsigned cols)
  {
    if (matrix.size() != rows * cols) {
      throw runtime_error("Matrix size does not match specified dimensions.");
    }
    vector<vector<double>> parsed_matrix(rows, vector<double>(cols));
    for (unsigned i = 0; i < rows; ++i) {
      for (unsigned j = 0; j < cols; ++j) {
        parsed_matrix[i][j] = matrix[i * cols + j];
      }
    }
    return parsed_matrix;
  }

  void debugLog(vector<vector<double>>  A, const vector<double> & b)
  {
    printf("Debug Log:\n");
    printf("Matrix A:\n");
    for (const auto & row : A) {
      for (const auto & value : row) {
        printf("%f ", value);
      }
      printf("\n");
    }
    printf("Vector b: ");
    for (const auto & value : b) {
      printf("%f ", value);
    }
    printf("\n");
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(make_shared<Client>());
  rclcpp::shutdown();
  return 0;
}