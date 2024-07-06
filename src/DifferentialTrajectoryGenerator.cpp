#include "trajopt/DifferentialTrajectoryGenerator.hpp"
#include <cstdint>
#include <string>
#include "trajopt/solution/DifferentialSolution.hpp"
#include "trajopt/util/TrajoptUtil.hpp"
#include "trajopt/util/expected"

namespace trajopt {

inline std::vector<double> RowSolutionValue(
    std::vector<sleipnir::Variable>& rowVector) {
  std::vector<double> valueRowVector;
  valueRowVector.reserve(rowVector.size());
  for (auto& expression : rowVector) {
    valueRowVector.push_back(expression.Value());
  }
  return valueRowVector;
}

inline std::vector<std::vector<double>> MatrixSolutionValue(
    std::vector<std::vector<sleipnir::Variable>>& matrix) {
  std::vector<std::vector<double>> valueMatrix;
  valueMatrix.reserve(matrix.size());
  for (auto& row : matrix) {
    valueMatrix.push_back(RowSolutionValue(row));
  }
  return valueMatrix;
}

DifferentialTrajectoryGenerator::DifferentialTrajectoryGenerator(
    DifferentialPathBuilder pathbuilder)
    : path(pathbuilder.GetPath()), N(pathbuilder.GetControlIntervalCounts()) {
  auto initialGuess = pathbuilder.CalculateInitialGuess();

  auto sgmtCnt = N.size();

  for (size_t sgmtIndex = 0; sgmtIndex < sgmtCnt; ++sgmtIndex) {
    for (auto& constraint :
         path.waypoints.at(sgmtIndex + 1).segmentConstraints) {
      size_t startIndex = GetIndex(N, sgmtIndex + 1, 0);
      size_t endIndex = GetIndex(N, sgmtIndex + 2, 0);

      for (size_t index = startIndex; index < endIndex; ++index) {
        Pose2v pose{
            x.at(index), y.at(index), {thetacos.at(index), thetasin.at(index)}};
        Translation2v linearVelocity{vL.at(index), vR.at(index)};
        auto angularVelocity = omega.at(index);
        Translation2v linearAcceleration{ax.at(index), ay.at(index)};
        auto angularAcceleration = alpha.at(index);

        std::visit(
            [&](auto&& arg) {
              arg.Apply(problem, pose, linearVelocity, angularVelocity,
                        linearAcceleration, angularAcceleration);
            },
            constraint);
      }
    }
  }

  ApplyInitialGuess(initialGuess);
}

expected<DifferentialSolution, std::string> DifferentialTrajectoryGenerator::Generate(bool diagnostics) {
}

void DifferentialTrajectoryGenerator::ApplyInitialGuess(
    const DifferentialSolution& solution) {}

DifferentialSolution
DifferentialTrajectoryGenerator::ConstructDifferentialSolution() {
  return DifferentialSolution{};
}

};  // namespace trajopt
