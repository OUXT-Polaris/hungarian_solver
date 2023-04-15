// Copyright (c) 2021 OUXT Polaris
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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

#include <hungarian_solver/hungarian_solver.hpp>

namespace hungarian_solver
{
Solver::Solver() {}

Solver::~Solver() {}

Eigen::MatrixXd Solver::updateCostMatrix(
  const Eigen::MatrixXd & matrix, const std::vector<int> & delete_rows_index,
  const std::vector<int> & delete_cols_index)
{
  Eigen::MatrixXd mat = matrix;
  std::vector<double> min_candidate_lists;
  for (int i = 0; i < mat.rows(); ++i) {
    for (int j = 0; j < mat.cols(); ++j) {
      auto result_row = std::find(delete_rows_index.begin(), delete_rows_index.end(), i);
      auto result_col = std::find(delete_cols_index.begin(), delete_cols_index.end(), j);
      if (result_row == delete_rows_index.end() && result_col == delete_cols_index.end()) {
        min_candidate_lists.push_back(mat(i, j));
      }
    }
  }
  std::vector<double>::iterator min_iter =
    std::min_element(min_candidate_lists.begin(), min_candidate_lists.end());
  double min_value = min_candidate_lists[std::distance(min_candidate_lists.begin(), min_iter)];
  for (int i = 0; i < mat.rows(); ++i) {
    for (int j = 0; j < mat.cols(); ++j) {
      auto result_row = std::find(delete_rows_index.begin(), delete_rows_index.end(), i);
      auto result_col = std::find(delete_cols_index.begin(), delete_cols_index.end(), j);
      if (result_row == delete_rows_index.end() && result_col == delete_cols_index.end()) {
        mat(i, j) = mat(i, j) - min_value;
      }
      if (result_row != delete_rows_index.end() && result_col != delete_cols_index.end()) {
        mat(i, j) = mat(i, j) + min_value;
      }
    }
  }
  return mat;
}

std::vector<bool> Solver::getNonZeroColFlags(const Eigen::MatrixXd & mat)
{
  std::vector<bool> flags(mat.cols());
  for (auto itr = flags.begin(); itr != flags.end(); itr++) {
    *itr = false;
  }
  for (int j = 0; j < mat.cols(); j++) {
    for (int i = 0; i < mat.rows(); i++) {
      double a = mat(i, j);
      double b = 0.0;
      if (fabs(a - b) > DBL_EPSILON * fmax(1, fmax(fabs(a), fabs(b)))) {
        flags[j] = true;
      }
    }
  }
  return flags;
}

Eigen::MatrixXd Solver::getInitialCostMatrix(const Eigen::MatrixXd & cost_matrix)
{
  std::vector<std::pair<int, int> > zero_index;
  for (int i = 0; i < cost_matrix.rows(); i++) {
    for (int j = 0; j < cost_matrix.cols(); j++) {
      double a = cost_matrix(i, j);
      double b = 0.0;
      if (fabs(a - b) <= DBL_EPSILON * fmax(1, fmax(fabs(a), fabs(b)))) {
        zero_index.push_back(std::make_pair(i, j));
      }
    }
  }
  int size = cost_matrix.rows();
  std::vector<bool> row_flags = std::vector<bool>(size);
  std::vector<bool> col_flags = std::vector<bool>(size);
  Eigen::MatrixXd ret = Eigen::MatrixXd::Zero(size, size);
  for (size_t i = 0; i < zero_index.size(); i++) {
    if (row_flags[zero_index[i].first] == false && col_flags[zero_index[i].second] == false) {
      ret(zero_index[i].first, zero_index[i].second) = 1.0;
      row_flags[zero_index[i].first] = true;
      col_flags[zero_index[i].second] = true;
    }
  }
  return ret;
}

std::optional<std::vector<std::pair<int, int> > > Solver::solve(Eigen::MatrixXd cost_matrix)
{
  std::optional<std::vector<std::pair<int, int> > > assignment;
  assert(cost_matrix.rows() == cost_matrix.cols());
  subtractRowMinima(cost_matrix);
  subtractColMinima(cost_matrix);
  while (true) {
    assignment = getAssignment(cost_matrix);
    if (assignment) {
      break;
    }
    std::pair<std::vector<int>, std::vector<int> > delete_lines = getDeleteLinesIndex(cost_matrix);
    cost_matrix = updateCostMatrix(cost_matrix, delete_lines.first, delete_lines.second);
  }
  return assignment;
}

std::pair<std::vector<int>, std::vector<int> > Solver::getDeleteLinesIndex(
  const Eigen::MatrixXd & mat)
{
  std::random_device seed_gen;
  std::default_random_engine engine(seed_gen());
  std::uniform_int_distribution<> dist(0, 1);
  std::pair<std::vector<int>, std::vector<int> > ret;
  std::vector<int> col_lists;
  std::vector<int> row_lists;
  do {
    std::vector<std::pair<int, int> > zero_index = getZeroIndex(mat);
    std::vector<std::pair<int, int> > zero_index_list;
    std::copy(zero_index.begin(), zero_index.end(), back_inserter(zero_index_list));
    col_lists = std::vector<int>();
    row_lists = std::vector<int>();
    std::vector<std::string> rc_lists;
    std::vector<int> index_lists;
    while (zero_index_list.size() > 0) {
      std::vector<int> col_counts = std::vector<int>(mat.cols(), 0);
      std::vector<int> row_counts = std::vector<int>(mat.rows(), 0);
      for (auto itr = zero_index_list.begin(); itr != zero_index_list.end(); itr++) {
        col_counts[itr->second] = col_counts[itr->second] + 1;
        row_counts[itr->first] = row_counts[itr->first] + 1;
      }
      std::vector<int>::iterator max_col_itr =
        std::max_element(col_counts.begin(), col_counts.end());
      std::vector<int>::iterator max_row_itr =
        std::max_element(row_counts.begin(), row_counts.end());
      size_t max_col = std::distance(col_counts.begin(), max_col_itr);
      size_t max_row = std::distance(row_counts.begin(), max_row_itr);
      std::vector<std::pair<int, int> > tmp_zero_index_list;
      if (dist(engine) == 0) {
        if (col_counts[max_col] >= row_counts[max_row]) {
          col_lists.push_back((int)max_col);
          for (auto itr = zero_index_list.begin(); itr != zero_index_list.end(); itr++) {
            if (itr->second != (int)max_col) {
              tmp_zero_index_list.push_back(std::make_pair(itr->first, itr->second));
            }
          }
        } else {
          row_lists.push_back((int)max_row);
          for (auto itr = zero_index_list.begin(); itr != zero_index_list.end(); itr++) {
            if (itr->first != (int)max_row) {
              tmp_zero_index_list.push_back(std::make_pair(itr->first, itr->second));
            }
          }
        }
      } else {
        if (col_counts[max_col] > row_counts[max_row]) {
          col_lists.push_back((int)max_col);
          for (auto itr = zero_index_list.begin(); itr != zero_index_list.end(); itr++) {
            if (itr->second != (int)max_col) {
              tmp_zero_index_list.push_back(std::make_pair(itr->first, itr->second));
            }
          }
        } else {
          row_lists.push_back((int)max_row);
          for (auto itr = zero_index_list.begin(); itr != zero_index_list.end(); itr++) {
            if (itr->first != (int)max_row) {
              tmp_zero_index_list.push_back(std::make_pair(itr->first, itr->second));
            }
          }
        }
      }

      zero_index_list = tmp_zero_index_list;
      ret.first = row_lists;
      ret.second = col_lists;
    }
  } while (row_lists.size() == 0 || col_lists.size() == 0);
  return ret;
}

std::vector<std::pair<int, int> > Solver::getZeroIndex(const Eigen::MatrixXd & mat)
{
  std::vector<std::pair<int, int> > ret;
  for (int i = 0; i < mat.rows(); i++) {
    for (int m = 0; m < mat.cols(); m++) {
      double a = mat(i, m);
      double b = 0.0;
      if (fabs(a - b) <= DBL_EPSILON * fmax(1, fmax(fabs(a), fabs(b)))) {
        std::pair<int, int> pair = std::make_pair(i, m);
        ret.push_back(pair);
      }
    }
  }
  return ret;
}

std::optional<std::vector<std::pair<int, int> > > Solver::getAssignment(const Eigen::MatrixXd & mat)
{
  assert(mat.rows() == mat.cols());
  std::vector<std::pair<int, int> > ret;
  std::vector<std::pair<int, int> > zero_index = getZeroIndex(mat);
  std::vector<bool> col_flags = std::vector<bool>(mat.cols(), false);
  std::vector<bool> row_flags = std::vector<bool>(mat.rows(), false);
  for (auto itr = zero_index.begin(); itr != zero_index.end(); itr++) {
    if (col_flags[itr->first] == false && row_flags[itr->second] == false) {
      col_flags[itr->first] = true;
      row_flags[itr->second] = true;
      std::pair<int, int> pair = std::make_pair(itr->first, itr->second);
      ret.push_back(pair);
    }
  }
  if (ret.size() == (size_t)mat.rows()) {
    return ret;
  }
  return std::nullopt;
}

std::optional<std::vector<std::pair<int, int> > > Solver::solve(
  const Eigen::MatrixXd & cost_matrix, double cost_of_non_assignment)
{
  std::vector<std::pair<int, int> > assignment;
  Eigen::MatrixXd padded_cost_mat = getPaddCostMatrix(cost_matrix, cost_of_non_assignment);
  std::optional<std::vector<std::pair<int, int> > > result = solve(padded_cost_mat);
  if (!result) {
    return std::nullopt;
  }
  for (const auto & val : result.value()) {
    if (val.first < cost_matrix.rows() && val.second < cost_matrix.cols()) {
      assignment.emplace_back(std::make_pair(val.first, val.second));
    }
  }
  return assignment;
}

void Solver::subtractRowMinima(Eigen::MatrixXd & mat)
{
  for (int i = 0; i < mat.rows(); i++) {
    Eigen::MatrixXd block = mat.block(i, 0, 1, mat.cols());
    double min_value = block.minCoeff();
    for (int m = 0; m < mat.cols(); m++) {
      block(0, m) = block(0, m) - min_value;
    }
    mat.block(i, 0, 1, mat.cols()) = block;
  }
}

void Solver::subtractColMinima(Eigen::MatrixXd & mat)
{
  for (int i = 0; i < mat.cols(); i++) {
    Eigen::MatrixXd block = mat.block(0, i, mat.rows(), 1);
    double min_value = block.minCoeff();
    for (int m = 0; m < mat.rows(); m++) {
      block(m, 0) = block(m, 0) - min_value;
    }
    mat.block(0, i, mat.rows(), 1) = block;
  }
}

Eigen::MatrixXd Solver::getPaddCostMatrix(
  const Eigen::MatrixXd & cost_matrix, double cost_of_non_assignment)
{
  int size = cost_matrix.rows() + cost_matrix.cols();
  Eigen::MatrixXd padded_cost_mat = Eigen::MatrixXd::Ones(size, size);
  double max_val = DBL_MAX;
  padded_cost_mat = padded_cost_mat * max_val;
  padded_cost_mat.block(0, 0, cost_matrix.rows(), cost_matrix.cols()) = cost_matrix;
  for (int i = 0; i < cost_matrix.rows(); i++) {
    padded_cost_mat(i, i + cost_matrix.cols()) = cost_of_non_assignment;
  }
  for (int i = 0; i < cost_matrix.cols(); i++) {
    padded_cost_mat(i + cost_matrix.rows(), i) = cost_of_non_assignment;
  }
  padded_cost_mat.block(
    cost_matrix.rows(), cost_matrix.cols(), cost_matrix.cols(), cost_matrix.rows()) =
    Eigen::MatrixXd::Zero(cost_matrix.cols(), cost_matrix.rows());
  return padded_cost_mat;
}
}  // namespace hungarian_solver