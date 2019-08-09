// Headers in Eigen
#define EIGEN_MPL2_ONLY
#include <Eigen/Core>

// Headers in ROS
#include <ros/ros.h>

// Headers in STL
#include <float.h>

//headers in Google Test
#include <gtest/gtest.h>

namespace hungarian_solver
{
    class Solver
    {
    public:
        Solver();
        ~Solver();
        void solve(Eigen::MatrixXd cost_matrix);
        void solve(Eigen::MatrixXd cost_matrix,double cost_of_non_assignment);
        void test(double t);
    private:
        Eigen::MatrixXd getInitialCostMatrix(Eigen::MatrixXd cost_matrix);
        friend class SolverTestSuite;
        FRIEND_TEST(SolverTestSuite, getInitialCostMatrixTestCase1);
    };
    void test(){}
}