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
  std::optional<std::vector<std::pair<int, int> > > solve(Eigen::MatrixXd cost_matrix) const;
  /**
   * @brief solve hungarian algorithum by using cost matrix
   */
  std::optional<std::vector<std::pair<int, int> > > solve(
    const Eigen::MatrixXd & cost_matrix, double cost_of_non_assignment) const;

private:
  /**
   * @brief update cost matrix by using delete line index
   */
  Eigen::MatrixXd updateCostMatrix(
    const Eigen::MatrixXd & mat, const std::vector<int> & delete_rows_index,
    const std::vector<int> & delete_cols_index) const;
  /**
   * @brief subtract col values by minimam value of col
   */
  void subtractColMinima(Eigen::MatrixXd & mat) const;
  /**
   * @brief subtract row values by minimam value of row
   */
  void subtractRowMinima(Eigen::MatrixXd & mat) const;
  /**
   * @brief get non zero col flags
   */
  std::vector<bool> getNonZeroColFlags(const Eigen::MatrixXd & mat) const;
  /**
   * @brief get initial cost matrix 
   */
  Eigen::MatrixXd getInitialCostMatrix(const Eigen::MatrixXd & cost_matrix) const;
  /**
   * @brief get padded cost matrix
   */
  Eigen::MatrixXd getPaddCostMatrix(
    const Eigen::MatrixXd & cost_matrix, double cost_of_non_assignment) const;
  /**
   * @brief get assinment of cost matrix
   */
  std::optional<std::vector<std::pair<int, int> > > getAssignment(
    const Eigen::MatrixXd & cost_matrix) const;
  /**
   * @brief get index with value zero
   */
  std::vector<std::pair<int, int> > getZeroIndex(const Eigen::MatrixXd & mat) const;
  /**
   * @brief get delete lines index
   */
  std::pair<std::vector<int>, std::vector<int> > getDeleteLinesIndex(
    const Eigen::MatrixXd & mat) const;
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