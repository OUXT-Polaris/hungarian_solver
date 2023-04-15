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
 * @file hungarian_solver.h
 * @author Masaya Kataoka (ms.kataoka@gmail.com)
 * @brief Definition of the Hungarian Solver Class
 * @version 0.1
 * @date 2019-08-11
 * 
 * @copyright Copyright (c) 2019
 * 
 */

// Headers in Eigen
#define EIGEN_MPL2_ONLY
#include <Eigen/Core>

// Headers in ROS
#include <rclcpp/rclcpp.hpp>

// Headers in STL
#include <float.h>

#include <random>

// Headers in Google Test
#include <gtest/gtest.h>

// Headers in Boost
#include <optional>

namespace hungarian_solver
{
/**
     * @brief Hungarian Solver Class
     */
class Solver
{
public:
  Solver();
  ~Solver();
  /**
         * @brief solve hungarian algorithum by using padding matrix
         */
  std::optional<std::vector<std::pair<int, int> > > solve(Eigen::MatrixXd cost_matrix);
  /**
         * @brief solve hungarian algorithum by using cost matrix
         */
  std::optional<std::vector<std::pair<int, int> > > solve(
    Eigen::MatrixXd cost_matrix, double cost_of_non_assignment);

private:
  /**
         * @brief update cost matrix by using delete line index
         */
  Eigen::MatrixXd updateCostMatrix(
    Eigen::MatrixXd mat, std::vector<int> delete_rows_index, std::vector<int> delete_cols_index);
  /**
         * @brief subtract col values by minimam value of col
         */
  Eigen::MatrixXd subtractColMinima(Eigen::MatrixXd mat);
  /**
         * @brief subtract row values by minimam value of row
         */
  Eigen::MatrixXd subtractRowMinima(Eigen::MatrixXd mat);
  /**
         * @brief get non zero col flags
         */
  std::vector<bool> getNonZeroColFlags(Eigen::MatrixXd mat);
  /**
         * @brief get initial cost matrix 
         */
  Eigen::MatrixXd getInitialCostMatrix(Eigen::MatrixXd cost_matrix);
  /**
         * @brief get padded cost matrix
         */
  Eigen::MatrixXd getPaddCostMatrix(Eigen::MatrixXd cost_matrix, double cost_of_non_assignment);
  /**
         * @brief get assinment of cost matrix
         */
  std::optional<std::vector<std::pair<int, int> > > getAssignment(Eigen::MatrixXd cost_matrix);
  /**
         * @brief get index with value zero
         */
  std::vector<std::pair<int, int> > getZeroIndex(Eigen::MatrixXd mat);
  /**
         * @brief get delete lines index
         */
  std::pair<std::vector<int>, std::vector<int> > getDeleteLinesIndex(Eigen::MatrixXd mat);
  // macros for Rostest
  friend class SolverTestSuite;
  /**
         * @brief FRIENT TEST macro for getInitialCostMatrix
         * @sa getInitialCostMatrix
         */
  FRIEND_TEST(SolverTestSuite, getInitialCostMatrixTestCase1);
  /**
         * @brief FRIENT TEST macro for getInitialCostMatrix
         * @sa getInitialCostMatrix
         */
  FRIEND_TEST(SolverTestSuite, getInitialCostMatrixTestCase2);
  /**
         * @brief FRIENT TEST macro for getNonZeroColFlags
         * @sa getNonZeroColFlags
         */
  FRIEND_TEST(SolverTestSuite, getNonZeroColFlagsTestCase1);
  /**
         * @brief FRIENT TEST macro for getNonZeroColFlags
         * @sa getNonZeroColFlags
         */
  FRIEND_TEST(SolverTestSuite, getNonZeroColFlagsTestCase2);
  /**
         * @brief FRIENT TEST macro for getPaddCostMatrix
         * @sa getPaddCostMatrix
         */
  FRIEND_TEST(SolverTestSuite, getPaddCostMatrixTestCase1);
  /**
         * @brief FRIENT TEST macro for subtractRowMinima
         * @sa subtractRowMinima
         */
  FRIEND_TEST(SolverTestSuite, subtractRowMinimaTestCase1);
  /**
         * @brief FRIENT TEST macro for subtractColMinima
         * @sa subtractColMinima
         */
  FRIEND_TEST(SolverTestSuite, subtractColMinimaTestCase1);
  /**
         * @brief FRIENT TEST macro for getAssignment
         * @sa getAssignment
         */
  FRIEND_TEST(SolverTestSuite, getAssignmentTestCase1);
  /**
         * @brief FRIENT TEST macro for getAssignment
         * @sa getAssignment
         */
  FRIEND_TEST(SolverTestSuite, getAssignmentTestCase2);
  /**
         * @brief FRIENT TEST macro for getZeroIndex
         * @sa getZeroIndex
         */
  FRIEND_TEST(SolverTestSuite, getZeroIndexTestCase1);
  /**
         * @brief FRIENT TEST macro for getZeroIndex
         * @sa getZeroIndex
         */
  FRIEND_TEST(SolverTestSuite, getZeroIndexTestCase2);
  /**
         * @brief FRIENT TEST macro for getDeleteLinesIndex
         * @sa getDeleteLinesIndex
         */
  FRIEND_TEST(SolverTestSuite, getDeleteLinesIndexTestCase1);
  /**
         * @brief FRIENT TEST macro for updateCostMatrix
         * @sa updateCostMatrix
         */
  FRIEND_TEST(SolverTestSuite, updateCostMatrixTestCase1);
};
}  // namespace hungarian_solver