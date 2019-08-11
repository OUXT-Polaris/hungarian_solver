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
        cost_matrix = subtractRowMinima(cost_matrix);
        cost_matrix = subtractColMinima(cost_matrix);
        boost::optional<std::vector<std::pair<int,int> > > assignment = getAssignment(cost_matrix);
        return;
    }

    std::vector<std::pair<int,int> > Solver::getZeroIndex(Eigen::MatrixXd mat)
    {
        std::vector<std::pair<int,int> > ret;
        for(int i=0; i<mat.rows(); i++)
        {
            for(int m=0; m<mat.cols(); m++)
            {
                double a = mat(i,m);
                double b = 0.0;
                if (fabs(a - b) <= DBL_EPSILON * fmax(1, fmax(fabs(a), fabs(b))))
                {
                    std::pair<int,int> pair = std::make_pair(i,m);
                    ret.push_back(pair);
                }
            }
        }
        return ret;
    }

    boost::optional<std::vector<std::pair<int,int> > > Solver::getAssignment(Eigen::MatrixXd mat)
    {
        ROS_ASSERT(mat.rows() == mat.cols());
        std::vector<std::pair<int,int> > ret;
        std::vector<std::pair<int,int> > zero_index = getZeroIndex(mat);
        std::vector<bool> col_flags = std::vector<bool>(mat.cols(),false);
        std::vector<bool> row_flags = std::vector<bool>(mat.rows(),false);
        for(auto itr = zero_index.begin(); itr != zero_index.end(); itr++)
        {
            if(col_flags[itr->first] == false && row_flags[itr->second] == false)
            {
                col_flags[itr->first] = true;
                row_flags[itr->second] = true;
                std::pair<int,int> pair = std::make_pair(itr->first,itr->second);
                ret.push_back(pair);
            }
        }
        if(ret.size() == mat.rows())
        {
            return ret;
        }
        return boost::none;
    }

    void Solver::solve(Eigen::MatrixXd cost_matrix,double cost_of_non_assignment)
    {
        Eigen::MatrixXd padded_cost_mat = getPaddCostMatrix(cost_matrix,cost_of_non_assignment);
        solve(padded_cost_mat);
        return;
    }

    Eigen::MatrixXd Solver::subtractRowMinima(Eigen::MatrixXd mat)
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