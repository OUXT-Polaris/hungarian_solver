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
        Eigen::MatrixXd getPaddCostMatrix(Eigen::MatrixXd cost_matrix,double cost_of_non_assignment)
        {
            return obj_.getPaddCostMatrix(cost_matrix,cost_of_non_assignment);
        }
        Eigen::MatrixXd subtractRawMinima(Eigen::MatrixXd mat)
        {
            return obj_.subtractRawMinima(mat);
        }
        Eigen::MatrixXd subtractColMinima(Eigen::MatrixXd mat)
        {
            return obj_.subtractColMinima(mat);
        }
        boost::optional<std::vector<std::pair<int,int> > > getAssignment(Eigen::MatrixXd mat)
        {
            return obj_.getAssignment(mat);
        }
        std::vector<std::pair<int,int> > getZeroIndex(Eigen::MatrixXd mat)
        {
            return obj_.getZeroIndex(mat);
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

    TEST(SolverTestSuite, getNonZeroColFlagsTestCase2)
    {
        Eigen::MatrixXd cost_matrix(3,3);
        cost_matrix <<
            3, 0, 3,
            0, 0, 3,
            0, 0, 3;
        hungarian_solver::Solver solver;
        std::vector<bool> flags = solver.getNonZeroColFlags(cost_matrix);
        EXPECT_EQ(flags[0],true);
        EXPECT_EQ(flags[1],false);
        EXPECT_EQ(flags[2],true);
    }

    TEST(SolverTestSuite, getPaddCostMatrixTestCase1)
    {
        Eigen::MatrixXd cost_matrix(2,2);
        cost_matrix <<
            0, 0,
            1, 2;
        hungarian_solver::Solver solver;
        Eigen::MatrixXd padded_cost_mat = solver.getPaddCostMatrix(cost_matrix,3);
        EXPECT_EQ(padded_cost_mat.rows(),padded_cost_mat.cols());

        EXPECT_FLOAT_EQ(padded_cost_mat(0,0),0);
        EXPECT_FLOAT_EQ(padded_cost_mat(0,1),0);
        EXPECT_FLOAT_EQ(padded_cost_mat(0,2),3);
        EXPECT_FLOAT_EQ(padded_cost_mat(0,3),DBL_MAX);

        EXPECT_FLOAT_EQ(padded_cost_mat(1,0),1);
        EXPECT_FLOAT_EQ(padded_cost_mat(1,1),2);
        EXPECT_FLOAT_EQ(padded_cost_mat(1,2),DBL_MAX);
        EXPECT_FLOAT_EQ(padded_cost_mat(1,3),3);

        EXPECT_FLOAT_EQ(padded_cost_mat(2,0),3);
        EXPECT_FLOAT_EQ(padded_cost_mat(2,1),DBL_MAX);
        EXPECT_FLOAT_EQ(padded_cost_mat(2,2),0);
        EXPECT_FLOAT_EQ(padded_cost_mat(2,3),0);

        EXPECT_FLOAT_EQ(padded_cost_mat(3,0),DBL_MAX);
        EXPECT_FLOAT_EQ(padded_cost_mat(3,1),3);
        EXPECT_FLOAT_EQ(padded_cost_mat(3,2),0);
        EXPECT_FLOAT_EQ(padded_cost_mat(3,3),0);
    }

    TEST(SolverTestSuite, subtractRawMinimaTestCase1)
    {
        Eigen::MatrixXd cost_matrix(2,2);
        cost_matrix <<
            0, 2,
            1, 2;
        hungarian_solver::Solver solver;
        Eigen::MatrixXd ret = solver.subtractRawMinima(cost_matrix);
        EXPECT_FLOAT_EQ(ret(0,0),0);
        EXPECT_FLOAT_EQ(ret(0,1),2);
        EXPECT_FLOAT_EQ(ret(1,0),0);
        EXPECT_FLOAT_EQ(ret(1,1),1);
    }

    TEST(SolverTestSuite, subtractColMinimaTestCase1)
    {
        Eigen::MatrixXd cost_matrix(2,3);
        cost_matrix <<
            0, 1, 2,
            1, 2, 2;
        hungarian_solver::Solver solver;
        Eigen::MatrixXd ret = solver.subtractColMinima(cost_matrix);
        EXPECT_FLOAT_EQ(ret(0,0),0);
        EXPECT_FLOAT_EQ(ret(0,1),0);
        EXPECT_FLOAT_EQ(ret(0,2),0);
        EXPECT_FLOAT_EQ(ret(1,0),1);
        EXPECT_FLOAT_EQ(ret(1,1),1);
        EXPECT_FLOAT_EQ(ret(1,2),0);
    }

    TEST(SolverTestSuite, solveTestCase1)
    {
        Eigen::MatrixXd cost_matrix(2,2);
        cost_matrix <<
            0, 2,
            1, 2;
        hungarian_solver::Solver solver;
        solver.solve(cost_matrix,3);
    }

    TEST(SolverTestSuite, getAssignmentTestCase1)
    {
        Eigen::MatrixXd mat(4,4);
        mat <<
            0, 0, 5, 3,
            2, 4, 2, 0,
            3, 7, 0, 2,
            0, 0, 0, 0;
        hungarian_solver::Solver solver;
        boost::optional<std::vector<std::pair<int,int> > > ret = solver.getAssignment(mat);
        EXPECT_EQ(ret->size(),4);
        EXPECT_EQ(ret.get()[0].first,0);
        EXPECT_EQ(ret.get()[0].second,0);
        EXPECT_EQ(ret.get()[1].first,1);
        EXPECT_EQ(ret.get()[1].second,3);
        EXPECT_EQ(ret.get()[2].first,2);
        EXPECT_EQ(ret.get()[2].second,2);
        EXPECT_EQ(ret.get()[3].first,3);
        EXPECT_EQ(ret.get()[3].second,1);
    }

    TEST(SolverTestSuite, getZeroIndexTestCase1)
    {
        Eigen::MatrixXd mat(2,2);
        mat <<
            0, 2,
            1, 2;
        hungarian_solver::Solver solver;
        std::vector<std::pair<int,int> > ret = solver.getZeroIndex(mat);
        EXPECT_EQ(ret.size(),1);
        EXPECT_EQ(ret[0].first,0);
        EXPECT_EQ(ret[0].second,0);
    }

    TEST(SolverTestSuite, getZeroIndexTestCase2)
    {
        Eigen::MatrixXd mat(2,2);
        mat <<
            0, 2,
            0, 0;
        hungarian_solver::Solver solver;
        std::vector<std::pair<int,int> > ret = solver.getZeroIndex(mat);
        EXPECT_EQ(ret.size(),3);
        EXPECT_EQ(ret[0].first,0);
        EXPECT_EQ(ret[0].second,0);
        EXPECT_EQ(ret[1].first,1);
        EXPECT_EQ(ret[1].second,0);
        EXPECT_EQ(ret[2].first,1);
        EXPECT_EQ(ret[2].second,1);
    }
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}