#include <hungarian_solver/hungarian_solver.h>

namespace hungarian_solver
{
    Solver::Solver()
    {

    }

    Solver::~Solver()
    {

    }

    Eigen::MatrixXd getInitialCostMatrix(Eigen::MatrixXd cost_matrix)
    {
        std::vector<std::pair<int,int> > zero_index;
        for(int i=0; i<cost_matrix.rows(); i++)
        {
            for(int j=0; j<cost_matrix.cols(); j++)
            {
                double a = cost_matrix(i,j);
                double b = 0.0;
                if (fabs(a - b) <= DBL_EPSILON * fmax(1, fmax(fabs(a), fabs(b))))
                {
                    zero_index.push_back(std::make_pair(i,j));
                }
            }
        }
        int size = cost_matrix.rows() + cost_matrix.cols();
        std::vector<bool> row_flags = std::vector<bool>(size);
        std::vector<bool> col_flags = std::vector<bool>(size);
        Eigen::MatrixXd ret = Eigen::MatrixXd::Zero(size, size);
        for(int i=0; i<size; i++)
        {
            if(row_flags[zero_index[i].first] == false && col_flags[zero_index[i].second] == false)
            {
                ret(zero_index[i].first,zero_index[i].second) = 1.0;
                row_flags[zero_index[i].first] = true;
                col_flags[zero_index[i].second] = true;
            }
        }
        return ret;
    }

    void Solver::solve(Eigen::MatrixXd cost_matrix)
    {
        ROS_ASSERT(cost_matrix.rows() == cost_matrix.cols());
    }

    void Solver::solve(Eigen::MatrixXd cost_matrix,double cost_of_non_assignment)
    {
        int size = cost_matrix.rows() + cost_matrix.cols();
        Eigen::MatrixXd padded_cost_mat = Eigen::MatrixXd::Zero(size, size);
    }
}