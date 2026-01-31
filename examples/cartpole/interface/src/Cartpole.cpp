/******************************************************************************
 * Cartpole OCS2 Interface - Implementation
 ******************************************************************************/

#include "Cartpole.h"

#include <ocs2_core/augmented_lagrangian/AugmentedLagrangian.h>
#include <ocs2_core/constraint/LinearStateInputConstraint.h>
#include <ocs2_core/cost/QuadraticStateCost.h>
#include <ocs2_core/cost/QuadraticStateInputCost.h>
#include <ocs2_core/initialization/DefaultInitializer.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/penalties/Penalties.h>

#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>
#include <iostream>

namespace ocs2 {
namespace cartpole {

// =============================================================================
// Parameters
// =============================================================================

void Parameters::loadFromYaml(const std::string& taskFile,
                              const std::string& prefix, bool verbose) {
  boost::property_tree::ptree pt;
  loadData::loadPtreeFromFile(taskFile, pt);

  if (verbose) {
    std::cerr << "\n #### Cart-pole Parameters:";
    std::cerr << "\n #### ==========================================================\n";
  }
  loadData::loadPtreeValue(pt, cartMass, prefix + ".cartMass", verbose);
  loadData::loadPtreeValue(pt, poleMass, prefix + ".poleMass", verbose);
  loadData::loadPtreeValue(pt, poleLength, prefix + ".poleLength", verbose);
  loadData::loadPtreeValue(pt, maxInput, prefix + ".maxInput", verbose);
  loadData::loadPtreeValue(pt, gravity, prefix + ".gravity", verbose);

  // Compute derived quantities
  computeDerived();

  if (verbose) {
    std::cerr << " #### ==========================================================\n\n";
  }
}

// =============================================================================
// Interface
// =============================================================================

Interface::Interface(const std::string& taskFile,
                     const std::string& libraryFolder, bool verbose) {
  // Validate paths
  if (!boost::filesystem::exists(taskFile)) {
    throw std::invalid_argument("[CartPole] Task file not found: " + taskFile);
  }
  boost::filesystem::create_directories(libraryFolder);

  if (verbose) {
    std::cerr << "[CartPole] Task file: " << taskFile << "\n"
              << "[CartPole] Library folder: " << libraryFolder << "\n";
  }

  // Load initial and target states from YAML
  loadData::loadEigenMatrix(taskFile, "initialState", initialState_);
  loadData::loadEigenMatrix(taskFile, "x_final", targetState_);

  if (verbose) {
    std::cerr << "[CartPole] Initial state: " << initialState_.transpose() << "\n"
              << "[CartPole] Target state:  " << targetState_.transpose() << "\n";
  }

  // Load solver settings
  ddpSettings_ = ddp::loadSettings(taskFile, "ddp", verbose);
  mpcSettings_ = mpc::loadSettings(taskFile, "mpc", verbose);

  // Reference manager
  referenceManagerPtr_ = std::make_shared<ReferenceManager>();

  // =========================================================================
  // Optimal Control Problem Setup
  // =========================================================================

  // 1. Cost function (from YAML)
  matrix_t Q(STATE_DIM, STATE_DIM);
  matrix_t R(INPUT_DIM, INPUT_DIM);
  matrix_t Qf(STATE_DIM, STATE_DIM);
  loadData::loadEigenMatrix(taskFile, "Q", Q);
  loadData::loadEigenMatrix(taskFile, "R", R);
  loadData::loadEigenMatrix(taskFile, "Q_final", Qf);

  if (verbose) {
    std::cerr << "Q:  \n" << Q << "\n";
    std::cerr << "R:  \n" << R << "\n";
    std::cerr << "Q_final:\n" << Qf << "\n";
  }

  problem_.costPtr->add("cost",
                        std::make_unique<QuadraticStateInputCost>(Q, R));
  problem_.finalCostPtr->add("finalCost",
                             std::make_unique<QuadraticStateCost>(Qf));

  // 2. Dynamics (robot-specific)
  Parameters params;
  params.loadFromYaml(taskFile, "cartpole_parameters", verbose);
  problem_.dynamicsPtr = std::make_unique<Dynamics>(params, libraryFolder, verbose);

  // 3. Rollout
  auto rolloutSettings = rollout::loadSettings(taskFile, "rollout", verbose);
  rolloutPtr_ = std::make_unique<TimeTriggeredRollout>(*problem_.dynamicsPtr,
                                                       rolloutSettings);

  // 4. Input constraints (load penalty config from YAML)
  auto getPenalty = [&taskFile, verbose]() {
    using penalty_type = augmented::SlacknessSquaredHingePenalty;
    penalty_type::Config boundsConfig;
    loadData::loadPenaltyConfig(taskFile, "bounds_penalty_config", boundsConfig, verbose);
    return penalty_type::create(boundsConfig);
  };

  auto getConstraint = [&params]() {
    constexpr size_t numIneqConstraint = 2;
    const vector_t e = (vector_t(numIneqConstraint) << params.maxInput, params.maxInput).finished();
    const vector_t D = (vector_t(numIneqConstraint) << 1.0, -1.0).finished();
    const matrix_t C = matrix_t::Zero(numIneqConstraint, STATE_DIM);
    return std::make_unique<LinearStateInputConstraint>(e, C, D);
  };

  problem_.inequalityLagrangianPtr->add("inputLimits",
                                        create(getConstraint(), getPenalty()));

  // 5. Initialization
  initializerPtr_ = std::make_unique<DefaultInitializer>(INPUT_DIM);
}

}  // namespace cartpole
}  // namespace ocs2
