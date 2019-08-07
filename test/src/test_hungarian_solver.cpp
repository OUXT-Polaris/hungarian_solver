// Headers in Gtest
#include <gtest/gtest.h>

// Headers in ROS
#include <ros/ros.h>

// Headers in this package
#include <hungarian_solver/hungarian_solver.h>

// Headers in STL
#include <float.h>

namespace hungarian_solver
{
    class SolverTestSuite : public ::testing::Test
    {
    protected:
        SolverTestSuite()
        {

        }

        virtual ~SolverTestSuite()
        {

        }

        virtual void SetUp()
        {

        }
        virtual void TearDown()
        {
            
        }
    public:
        hungarian_solver::Solver obj_;
        Eigen::MatrixXd getInitialCostMatrix(Eigen::MatrixXd cost_matrix)
        {
            return obj_.getInitialCostMatrix(cost_matrix);
        }
    };

    TEST(SolverTestSuite, getInitialCostMatrixTestCase1)
    {
        Eigen::MatrixXd cost_matrix(6,6);
        cost_matrix << 
            0,       0,       0,       0,       DBL_MAX, DBL_MAX,
            0,       0,       0,       DBL_MAX, 0,       DBL_MAX,
            0,       0,       0,       DBL_MAX, DBL_MAX, 0,
            0,       DBL_MAX, DBL_MAX, 0,       0,       0,
            DBL_MAX, 0,       DBL_MAX, 0,       0,       0,
            DBL_MAX, DBL_MAX, 0,       0,       0,       0;
        //hungarian_solver::Solver solver;
        //solver.solve(cost_matrix);
        //solver.getInitialCostMatrix(cost_matrix);
    }
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}