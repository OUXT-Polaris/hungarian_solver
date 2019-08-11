// Headers in Eigen
#define EIGEN_MPL2_ONLY
#include <Eigen/Core>

// Headers in ROS
#include <ros/ros.h>

// Headers in STL
#include <float.h>

// Headers in Google Test
#include <gtest/gtest.h>

// Headers in Boost
#include <boost/optional.hpp>

namespace hungarian_solver
{
    class Solver
    {
    public:
        Solver();
        ~Solver();
        void solve(Eigen::MatrixXd cost_matrix);
        void solve(Eigen::MatrixXd cost_matrix,double cost_of_non_assignment);
    private:
        Eigen::MatrixXd subtractColMinima(Eigen::MatrixXd mat);
        Eigen::MatrixXd subtractRawMinima(Eigen::MatrixXd mat);
        std::vector<bool> getNonZeroColFlags(Eigen::MatrixXd mat);
        Eigen::MatrixXd getInitialCostMatrix(Eigen::MatrixXd cost_matrix);
        Eigen::MatrixXd getPaddCostMatrix(Eigen::MatrixXd cost_matrix,double cost_of_non_assignment);
        boost::optional<std::vector<std::pair<int,int> > > getAssignment(Eigen::MatrixXd cost_matrix);
        std::vector<std::pair<int,int> > getZeroIndex(Eigen::MatrixXd mat);
        // macros for Rostest
        friend class SolverTestSuite;
        FRIEND_TEST(SolverTestSuite, getInitialCostMatrixTestCase1);
        FRIEND_TEST(SolverTestSuite, getInitialCostMatrixTestCase2);
        FRIEND_TEST(SolverTestSuite, getNonZeroColFlagsTestCase1);
        FRIEND_TEST(SolverTestSuite, getNonZeroColFlagsTestCase2);
        FRIEND_TEST(SolverTestSuite, getPaddCostMatrixTestCase1);
        FRIEND_TEST(SolverTestSuite, subtractRawMinimaTestCase1);
        FRIEND_TEST(SolverTestSuite, subtractColMinimaTestCase1);
        FRIEND_TEST(SolverTestSuite, getAssignmentTestCase1);
        FRIEND_TEST(SolverTestSuite, getAssignmentTestCase2);
        FRIEND_TEST(SolverTestSuite, getZeroIndexTestCase1);
        FRIEND_TEST(SolverTestSuite, getZeroIndexTestCase2);
    };
}