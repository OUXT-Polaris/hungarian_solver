#include <hungarian_solver/hungarian_solver.h>

namespace hungarian_solver
{
    Solver::Solver()
    {

    }

    Solver::~Solver()
    {

    }

    std::vector<bool> Solver::getNonZeroColFlags(Eigen::MatrixXd mat)
    {
        std::vector<bool> flags(mat.cols());
        for(auto itr = flags.begin(); itr != flags.end(); itr++)
        {
            *itr = false;
        }
        for(int j=0; j<mat.cols(); j++)
        {
            for(int i=0;i<mat.rows(); i++)
            {
                double a = mat(i,j);
                double b = 0.0;
                if (fabs(a - b) > DBL_EPSILON * fmax(1, fmax(fabs(a), fabs(b))))
                {
                    flags[j] = true;
                }
            }
        }
        return flags;
    }

    Eigen::MatrixXd Solver::getInitialCostMatrix(Eigen::MatrixXd cost_matrix)
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
        int size = cost_matrix.rows();
        std::vector<bool> row_flags = std::vector<bool>(size);
        std::vector<bool> col_flags = std::vector<bool>(size);
        Eigen::MatrixXd ret = Eigen::MatrixXd::Zero(size, size);
        for(int i=0; i<zero_index.size(); i++)
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
        cost_matrix = subtractRawMinima(cost_matrix);
        Eigen::MatrixXd initial_cost_mat = getInitialCostMatrix(cost_matrix);
        std::vector<bool> non_zero_col_flags = getNonZeroColFlags(initial_cost_mat);
        //ROS_ERROR_STREAM(initial_cost_mat);
        return;
    }

    void Solver::solve(Eigen::MatrixXd cost_matrix,double cost_of_non_assignment)
    {
        Eigen::MatrixXd padded_cost_mat = getPaddCostMatrix(cost_matrix,cost_of_non_assignment);
        solve(padded_cost_mat);
        return;
    }

    Eigen::MatrixXd Solver::subtractRawMinima(Eigen::MatrixXd mat)
    {
        for(int i=0; i<mat.rows(); i++)
        {
            Eigen::MatrixXd block = mat.block(i,0,1,mat.cols());
            double min_value = block.minCoeff();
            for(int m=0; m<mat.cols(); m++)
            {
                block(0,m) = block(0,m) - min_value;
            }
            mat.block(i,0,1,mat.cols()) = block;
        }
        return mat;
    }

    Eigen::MatrixXd Solver::subtractColMinima(Eigen::MatrixXd mat)
    {
        for(int i=0; i<mat.cols(); i++)
        {
            Eigen::MatrixXd block = mat.block(0,i,mat.rows(),1);
            double min_value = block.minCoeff();
            for(int m=0; m<mat.rows(); m++)
            {
                block(m,0) = block(m,0) - min_value;
            }
            mat.block(0,i,mat.rows(),1) = block;
        }
        return mat;
    }

    Eigen::MatrixXd Solver::getPaddCostMatrix(Eigen::MatrixXd cost_matrix,double cost_of_non_assignment)
    {
        int size = cost_matrix.rows() + cost_matrix.cols();
        Eigen::MatrixXd padded_cost_mat = Eigen::MatrixXd::Ones(size, size);
        double max_val = DBL_MAX;
        padded_cost_mat = padded_cost_mat*max_val;
        padded_cost_mat.block(0,0,cost_matrix.rows(),cost_matrix.cols()) = cost_matrix;
        for(int i=0; i<cost_matrix.rows(); i++)
        {
            padded_cost_mat(i,i+cost_matrix.cols()) = cost_of_non_assignment;
        }
        for(int i=0; i<cost_matrix.cols(); i++)
        {
            padded_cost_mat(i+cost_matrix.rows(),i) = cost_of_non_assignment;
        }
        padded_cost_mat.block(cost_matrix.rows(),cost_matrix.cols(),cost_matrix.rows(),cost_matrix.cols()) 
            = Eigen::MatrixXd::Zero(cost_matrix.rows(),cost_matrix.cols());
        return padded_cost_mat;
    }
}