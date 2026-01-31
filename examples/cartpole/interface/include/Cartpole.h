/******************************************************************************
 * Cartpole OCS2 Interface
 *
 * This is a SIMPLIFIED, SINGLE-HEADER interface for the cartpole example.
 * It demonstrates the minimal code needed to integrate a robot with OCS2.
 *
 * For a NEW ROBOT, you need to change:
 *   1. STATE_DIM and INPUT_DIM
 *   2. systemFlowMap() - the dynamics equations
 *   3. Physical parameters (or load from URDF/YAML)
 *   4. Cost matrices and constraints (in YAML)
 *
 * Everything else is boilerplate that can be copied.
 ******************************************************************************/

#pragma once

#include <ocs2_core/Types.h>
#include <ocs2_core/dynamics/SystemDynamicsBaseAD.h>
#include <ocs2_core/initialization/Initializer.h>
#include <ocs2_ddp/DDP_Settings.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <ocs2_python_interface/PythonInterface.h>
#include <ocs2_robotic_tools/common/RobotInterface.h>

namespace ocs2 {
namespace cartpole {

// =============================================================================
// ROBOT-SPECIFIC: Dimensions and Parameters
// =============================================================================

// State: [theta, x, theta_dot, x_dot] - pole angle, cart position, velocities
constexpr size_t STATE_DIM = 4;
// Input: [F] - force applied to cart
constexpr size_t INPUT_DIM = 1;

/**
 * Physical parameters - can be loaded from YAML or extracted from URDF
 */
struct Parameters {
  scalar_t cartMass = 2.0;      // kg
  scalar_t poleMass = 0.2;      // kg
  scalar_t poleLength = 1.0;    // m
  scalar_t maxInput = 5.0;      // N
  scalar_t gravity = 9.81;      // m/s²

  // Derived quantities
  scalar_t poleHalfLength() const { return poleLength / 2.0; }
  scalar_t poleSteinerMoi() const {
    return poleMass * poleHalfLength() * poleHalfLength() +
           poleMass * gravity * poleHalfLength();
  }

  void loadFromYaml(const std::string& taskFile, const std::string& prefix,
                    bool verbose = false);
};

// =============================================================================
// ROBOT-SPECIFIC: Dynamics Equations
// =============================================================================

/**
 * Cartpole dynamics using Euler-Lagrange equations.
 *
 * For YOUR ROBOT: Override systemFlowMap() with your dynamics.
 *
 * Options:
 *   1. Analytical equations (fast, like this example)
 *   2. Use Pinocchio to compute M, C, G and solve: q_ddot = M^{-1}(τ - C*qdot - G)
 *      (requires wrapping Pinocchio with CppAD types - see ocs2_pinocchio_interface)
 */
class Dynamics final : public SystemDynamicsBaseAD {
public:
  Dynamics(const Parameters& params, const std::string& libraryFolder,
           bool verbose = true)
      : params_(params) {
    initialize(STATE_DIM, INPUT_DIM, "cartpole_dynamics", libraryFolder, true,
               verbose);
  }

  Dynamics* clone() const override { return new Dynamics(*this); }

  /**
   * ROBOT-SPECIFIC: The dynamics equations
   *
   * State: x = [θ, x, θ̇, ẋ]
   * Input: u = [F]
   * Returns: ẋ = [θ̇, ẋ, θ̈, ẍ]
   */
  ad_vector_t systemFlowMap(ad_scalar_t /*time*/, const ad_vector_t& state,
                            const ad_vector_t& input,
                            const ad_vector_t& /*params*/) const override {
    const ad_scalar_t theta = state(0);
    const ad_scalar_t theta_dot = state(2);
    const ad_scalar_t F = input(0);

    const ad_scalar_t cosTheta = cos(theta);
    const ad_scalar_t sinTheta = sin(theta);

    // Inertia matrix [2x2]
    const ad_scalar_t L = static_cast<ad_scalar_t>(params_.poleHalfLength());
    const ad_scalar_t mp = static_cast<ad_scalar_t>(params_.poleMass);
    const ad_scalar_t mc = static_cast<ad_scalar_t>(params_.cartMass);
    const ad_scalar_t g = static_cast<ad_scalar_t>(params_.gravity);

    Eigen::Matrix<ad_scalar_t, 2, 2> M;
    M << mp * L * L + mp * g * L * cosTheta, mp * L * cosTheta,
         mp * L * cosTheta, mc + mp;

    // Right-hand side
    Eigen::Matrix<ad_scalar_t, 2, 1> rhs;
    rhs << mp * g * L * sinTheta,
           F + mp * L * theta_dot * theta_dot * sinTheta;

    // Solve: [theta_ddot, x_ddot]^T = M^{-1} * rhs
    Eigen::Matrix<ad_scalar_t, 2, 1> accel = M.inverse() * rhs;

    ad_vector_t xdot(STATE_DIM);
    xdot << state(2), state(3), accel(0), accel(1);
    return xdot;
  }

private:
  Dynamics(const Dynamics&) = default;
  Parameters params_;
};

// =============================================================================
// GENERIC: Robot Interface (mostly boilerplate)
// =============================================================================

class Interface final : public RobotInterface {
public:
  Interface(const std::string& taskFile, const std::string& libraryFolder,
            bool verbose = true);

  const vector_t& getInitialState() const { return initialState_; }
  const vector_t& getTargetState() const { return targetState_; }

  ddp::Settings& ddpSettings() { return ddpSettings_; }
  mpc::Settings& mpcSettings() { return mpcSettings_; }

  const OptimalControlProblem& getOptimalControlProblem() const override {
    return problem_;
  }
  const RolloutBase& getRollout() const { return *rolloutPtr_; }
  const Initializer& getInitializer() const override { return *initializerPtr_; }
  std::shared_ptr<ReferenceManagerInterface> getReferenceManagerPtr() const override {
    return referenceManagerPtr_;
  }

private:
  ddp::Settings ddpSettings_;
  mpc::Settings mpcSettings_;
  OptimalControlProblem problem_;
  std::shared_ptr<ReferenceManager> referenceManagerPtr_;
  std::unique_ptr<RolloutBase> rolloutPtr_;
  std::unique_ptr<Initializer> initializerPtr_;
  vector_t initialState_{STATE_DIM};
  vector_t targetState_{STATE_DIM};
};

// =============================================================================
// GENERIC: Python Bindings (boilerplate)
// =============================================================================

class PyBindings final : public PythonInterface {
public:
  PyBindings(const std::string& taskFile, const std::string& libraryFolder,
             const std::string& /*urdfFile*/ = "") {
    Interface interface(taskFile, libraryFolder, true);

    // Create MPC solver
    auto mpc = std::make_unique<ocs2::GaussNewtonDDP_MPC>(
        interface.mpcSettings(), interface.ddpSettings(), interface.getRollout(),
        interface.getOptimalControlProblem(), interface.getInitializer());

    // Initialize Python interface
    PythonInterface::init(interface, std::move(mpc));
  }
};

}  // namespace cartpole
}  // namespace ocs2
