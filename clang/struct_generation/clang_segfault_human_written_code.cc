// Copyright 2018 kvedder@umass.edu
// College of Information and Computer Sciences,
// University of Massachusetts Amherst
//
// This software is free: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License Version 3,
// as published by the Free Software Foundation.
//
// This software is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// Version 3 in the file COPYING that came with this distribution.
// If not, see <http://www.gnu.org/licenses/>.
// ========================================================================

#include "search/fourgrid/fourgrid.h"
#include "constants/constants.h"
#include "search/fourgrid/fourgrid_solver.h"

using ::search::fourgrid::FourGrid;
using ::search::fourgrid::JointPosition;
using ::search::fourgrid::solver::expandingastar::FourGridSolver;
using ::search::fourgrid::util::FileToJointPosition;
using ::search::fourgrid::util::CostPathCollides;

static constexpr size_t kRobotCount = 2;

void Setup(char* name) {
  google::InitGoogleLogging(name);
  FLAGS_stderrthreshold = 0;
  FLAGS_colorlogtostderr = 1;
  FLAGS_logtostderr = true;
}

struct Scenario {
  FourGrid<kRobotCount> fourgrid_joint;
  JointPosition<kRobotCount> fourgrid_joint_start;
  JointPosition<kRobotCount> fourgrid_joint_end;
  std::array<FourGrid<1>, kRobotCount> fourgrid_individuals;
  std::array<JointPosition<1>, kRobotCount> fourgrid_individual_starts;
  std::array<JointPosition<1>, kRobotCount> fourgrid_individual_goals;
};

Scenario MakeSplit() {
  return {
      FourGrid<kRobotCount>("fourgrid3_split_joint.txt"),
      FileToJointPosition<kRobotCount>("fourgrid3_split_joint_start.txt"),
      FileToJointPosition<kRobotCount>("fourgrid3_split_joint_goal.txt"),
      {{FourGrid<1>("fourgrid3_split_individual_0.txt"),
        FourGrid<1>("fourgrid3_split_individual_1.txt")}},
      {{FileToJointPosition<1>("fourgrid3_split_individual_0_start.txt"),
        FileToJointPosition<1>("fourgrid3_split_individual_1_start.txt")}},
      {{FileToJointPosition<1>("fourgrid3_split_individual_0_goal.txt"),
        FileToJointPosition<1>("fourgrid3_split_individual_1_goal.txt")}}};
}

int main(int argc, char** argv) {
  Setup(argv[0]);
  const Scenario split = MakeSplit();

  FourGridSolver<kRobotCount> fgs;

  const auto individual_plans = fgs.SolveIndividualAStar(
      split.fourgrid_individuals, split.fourgrid_individual_starts,
      split.fourgrid_individual_goals);
  const auto joint_unrepaired_plan = fgs.ZipIndividualPlans(individual_plans);
  const auto result_collides = CostPathCollides(joint_unrepaired_plan);
  NP_CHECK(result_collides.first);
  const auto& colliding_edge = result_collides.second;
  const JointPosition<kRobotCount> collision_center =
      (::search::fourgrid::util::JointPositionCollides(colliding_edge.to)
           ? colliding_edge.to
           : colliding_edge.from);

  LOG(INFO) << "Collision center: "
            << ::search::fourgrid::util::JointPositionToString(
                   collision_center);

  for (size_t r = 2; r < 5; ++r) {
    const auto window_fourgrid =
        split.fourgrid_joint.RestrictLInfNorm(collision_center, r);
  }

  LOG(INFO) << "Running!";
}

