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
        std::vector<bool> getNonZeroColFlags(Eigen::MatrixXd mat)
        {
            return obj_.getNonZeroColFlags(mat);
        }
    };

    TEST(SolverTestSuite, getInitialCostMatrixTestCase1)
    {
        Eigen::MatrixXd cost_matrix(6,6);
        cost_matrix << 
            0,       0,       0,       3,       DBL_MAX, DBL_MAX,
            0,       1,       2,       DBL_MAX, 2,       DBL_MAX,
            1,       2,       0,       DBL_MAX, DBL_MAX, 2,
            3,       DBL_MAX, DBL_MAX, 0,       0,       0,
            DBL_MAX, 3,       DBL_MAX, 0,       0,       0,
            DBL_MAX, DBL_MAX, 3,       0,       0,       0;
        hungarian_solver::Solver solver;
        Eigen::MatrixXd ret = solver.getInitialCostMatrix(cost_matrix);
        EXPECT_FLOAT_EQ(ret(0,0),1);
        EXPECT_FLOAT_EQ(ret(0,1),0);
        EXPECT_FLOAT_EQ(ret(0,2),0);
        EXPECT_FLOAT_EQ(ret(0,3),0);
        EXPECT_FLOAT_EQ(ret(0,4),0);
        EXPECT_FLOAT_EQ(ret(0,5),0);

        EXPECT_FLOAT_EQ(ret(1,0),0);
        EXPECT_FLOAT_EQ(ret(1,1),0);
        EXPECT_FLOAT_EQ(ret(1,2),0);
        EXPECT_FLOAT_EQ(ret(1,3),0);
        EXPECT_FLOAT_EQ(ret(1,4),0);
        EXPECT_FLOAT_EQ(ret(1,5),0);

        EXPECT_FLOAT_EQ(ret(2,0),0);
        EXPECT_FLOAT_EQ(ret(2,1),0);
        EXPECT_FLOAT_EQ(ret(2,2),1);
        EXPECT_FLOAT_EQ(ret(2,3),0);
        EXPECT_FLOAT_EQ(ret(2,4),0);
        EXPECT_FLOAT_EQ(ret(2,5),0);

        EXPECT_FLOAT_EQ(ret(3,0),0);
        EXPECT_FLOAT_EQ(ret(3,1),0);
        EXPECT_FLOAT_EQ(ret(3,2),0);
        EXPECT_FLOAT_EQ(ret(3,3),1);
        EXPECT_FLOAT_EQ(ret(3,4),0);
        EXPECT_FLOAT_EQ(ret(3,5),0);

        EXPECT_FLOAT_EQ(ret(4,0),0);
        EXPECT_FLOAT_EQ(ret(4,1),0);
        EXPECT_FLOAT_EQ(ret(4,2),0);
        EXPECT_FLOAT_EQ(ret(4,3),0);
        EXPECT_FLOAT_EQ(ret(4,4),1);
        EXPECT_FLOAT_EQ(ret(4,5),0);

        EXPECT_FLOAT_EQ(ret(5,0),0);
        EXPECT_FLOAT_EQ(ret(5,1),0);
        EXPECT_FLOAT_EQ(ret(5,2),0);
        EXPECT_FLOAT_EQ(ret(5,3),0);
        EXPECT_FLOAT_EQ(ret(5,4),0);
        EXPECT_FLOAT_EQ(ret(5,5),1);
    }

    TEST(SolverTestSuite, getInitialCostMatrixTestCase2)
    {
        Eigen::MatrixXd cost_matrix(4,4);
        cost_matrix << 
                  0,       0,       3, DBL_MAX,
                  0,       1, DBL_MAX,       2,
                  3, DBL_MAX,       0,       0,
            DBL_MAX,       3,       0,       0;
        hungarian_solver::Solver solver;
        Eigen::MatrixXd ret = solver.getInitialCostMatrix(cost_matrix);
        EXPECT_FLOAT_EQ(ret(0,0),1);
        EXPECT_FLOAT_EQ(ret(0,1),0);
        EXPECT_FLOAT_EQ(ret(0,2),0);
        EXPECT_FLOAT_EQ(ret(0,3),0);

        EXPECT_FLOAT_EQ(ret(1,0),0);
        EXPECT_FLOAT_EQ(ret(1,1),0);
        EXPECT_FLOAT_EQ(ret(1,2),0);
        EXPECT_FLOAT_EQ(ret(1,3),0);

        EXPECT_FLOAT_EQ(ret(2,0),0);
        EXPECT_FLOAT_EQ(ret(2,1),0);
        EXPECT_FLOAT_EQ(ret(2,2),1);
        EXPECT_FLOAT_EQ(ret(2,3),0);

        EXPECT_FLOAT_EQ(ret(3,0),0);
        EXPECT_FLOAT_EQ(ret(3,1),0);
        EXPECT_FLOAT_EQ(ret(3,2),0);
        EXPECT_FLOAT_EQ(ret(3,3),1);
    }

    TEST(SolverTestSuite, getNonZeroColFlagsTestCase1)
    {
        Eigen::MatrixXd cost_matrix(3,3);
        cost_matrix <<
            0, 0, 3,
            0, 0, 3,
            0, 0, 3;
        hungarian_solver::Solver solver;
        std::vector<bool> flags = solver.getNonZeroColFlags(cost_matrix);
        EXPECT_EQ(flags[0],false);
        EXPECT_EQ(flags[1],false);
        EXPECT_EQ(flags[2],true);
    }
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    hungarian_solver::test();
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}