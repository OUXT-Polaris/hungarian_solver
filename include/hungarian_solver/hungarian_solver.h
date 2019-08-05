// Headers in Eigen
#define EIGEN_MPL2_ONLY
#include <Eigen/Core>

// Headers in ROS
#include <ros/ros.h>

namespace hungarian_solver
{
    void solve(Eigen::MatrixXd cost_matrix);
    void solve(Eigen::MatrixXd cost_matrix,double cost_of_non_assignment);
}