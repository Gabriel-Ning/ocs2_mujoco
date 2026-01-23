
#pragma once

#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_python_interface/PythonInterface.h>

#include "ocs2_cartpole/CartPoleInterface.h"
#include "ocs2_cartpole/definitions.h"

namespace ocs2 {
namespace cartpole {

class CartPolePyBindings final : public PythonInterface {
 public:
  /**
   * Constructor
   *
   * @note Creates directory for generated library into if it does not exist.
   * @throw Invalid argument error if input task file does not exist.
   *
   * @param [in] taskFile: The absolute path to the configuration file for the MPC.
   * @param [in] libraryFolder: The absolute path to the directory to generate CppAD library into.
   * @param [in] urdfFile: The absolute path to the URDF of the robot. This is not used for cartpole.
   */
  CartPolePyBindings(const std::string& taskFile, const std::string& libraryFolder, const std::string urdfFile = "") {
    // System dimensions
    stateDim_ = static_cast<int>(STATE_DIM);
    inputDim_ = static_cast<int>(INPUT_DIM);

    // Robot interface
    CartPoleInterface cartPoleInterface(taskFile, libraryFolder, true /* verbose */);

    // MPC
    auto mpcPtr =
        std::make_unique<GaussNewtonDDP_MPC>(cartPoleInterface.mpcSettings(), cartPoleInterface.ddpSettings(), cartPoleInterface.getRollout(),
                                             cartPoleInterface.getOptimalControlProblem(), cartPoleInterface.getInitializer());
    mpcPtr->getSolverPtr()->setReferenceManager(cartPoleInterface.getReferenceManagerPtr());

    // Python interface
    PythonInterface::init(cartPoleInterface, std::move(mpcPtr));
  }
};

}  // namespace cartpole
}  // namespace ocs2
