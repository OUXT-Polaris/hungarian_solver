/**
 * @file hungarian_solver.cpp
 * @author Masaya Kataoka (ms.kataoka@gmail.com)
 * @brief Implimentation of Solver Class
 * @version 0.1
 * @date 2019-08-27
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include <hungarian_solver/hungarian_solver.h>

namespace hungarian_solver
{
    Solver::Solver()
    {

    }

    Solver::~Solver()
    {

    }

    Eigen::MatrixXd Solver::updateCostMatrix(Eigen::MatrixXd mat,std::vector<int> delete_rows_index,std::vector<int> delete_cols_index)
    {
        std::vector<int> min_candidate_lists;
        for(int i=0;i<mat.rows();++i)
        {
            for(int j=0;j<mat.cols();++j) 
            {
                auto result_row = std::find(delete_rows_index.begin(), delete_rows_index.end(), i);
                auto result_col = std::find(delete_cols_index.begin(), delete_cols_index.end(), j);
                if(result_row  == delete_rows_index.end() && result_col == delete_cols_index.end())
                {
                    min_candidate_lists.push_back(mat(i,j));
                }
            }
        }
        std::vector<int>::iterator min_iter = std::min_element(min_candidate_lists.begin(), min_candidate_lists.end());
        int min_value = min_candidate_lists[std::distance(min_candidate_lists.begin(), min_iter)];
        for(int i=0;i<mat.rows();++i)
        {
            for(int j=0;j<mat.cols();++j) 
            {
                auto result_row = std::find(delete_rows_index.begin(), delete_rows_index.end(), i);
                auto result_col = std::find(delete_cols_index.begin(), delete_cols_index.end(), j);
                if(result_row  == delete_rows_index.end() && result_col == delete_cols_index.end())
                {
                    mat(i,j) = mat(i,j) - min_value;
                }
                if(result_row  != delete_rows_index.end() && result_col != delete_cols_index.end())
                {
                    mat(i,j) = mat(i,j) + min_value;
                }
            }
        }
        return mat;
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
        if(!assignment)
        {

        }
        return;
    }

    std::pair<std::vector<int>,std::vector<int> > Solver::getDeleteLinesIndex(Eigen::MatrixXd mat)
    {
        std::pair<std::vector<int>,std::vector<int> > ret;
        std::vector<std::pair<int,int> > zero_index = getZeroIndex(mat);
        std::vector<std::pair<int,int> > zero_index_list;
        std::copy(zero_index.begin(), zero_index.end(), back_inserter(zero_index_list));
        std::vector<int> col_lists;
        std::vector<int> row_lists;
        std::vector<std::string> rc_lists;
        std::vector<int> index_lists;
        while(zero_index_list.size() > 0)
        {
            std::vector<int> col_counts = std::vector<int>(mat.cols(),0);
            std::vector<int> row_counts = std::vector<int>(mat.rows(),0);
            for(auto itr = zero_index_list.begin(); itr != zero_index_list.end(); itr++)
            {
                col_counts[itr->second] = col_counts[itr->second] + 1;
                row_counts[itr->first] = row_counts[itr->first] + 1;
            }
            std::vector<int>::iterator max_col_itr = std::max_element(col_counts.begin(), col_counts.end());
            std::vector<int>::iterator max_row_itr = std::max_element(row_counts.begin(), row_counts.end());
            size_t max_col = std::distance(col_counts.begin(), max_col_itr);
            size_t max_row = std::distance(row_counts.begin(), max_row_itr);
            std::vector<std::pair<int,int> > tmp_zero_index_list;
            if(col_counts[max_col]>=row_counts[max_row])
            {
                col_lists.push_back((int)max_col);
                for(auto itr = zero_index_list.begin(); itr != zero_index_list.end(); itr++)
                {
                    if(itr->second != max_col)
                    {
                        tmp_zero_index_list.push_back(std::make_pair(itr->first,itr->second));
                    }
                }
            }
            else
            {
                row_lists.push_back((int)max_row);
                for(auto itr = zero_index_list.begin(); itr != zero_index_list.end(); itr++)
                {
                    if(itr->first != max_row)
                    {
                        tmp_zero_index_list.push_back(std::make_pair(itr->first,itr->second));
                    }
                }
            }
            zero_index_list = tmp_zero_index_list;
            ret.first = row_lists;
            ret.second = col_lists;
        }
        return ret;
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