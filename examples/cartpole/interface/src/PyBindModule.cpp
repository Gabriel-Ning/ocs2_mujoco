/******************************************************************************
 * Python Bindings Module
 *
 * This creates the ocs2_cartpole Python module with CartpolePyBindings class.
 * Usage in Python:
 *   from ocs2_cartpole import CartpolePyBindings
 *   mpc = CartpolePyBindings.mpc_interface(task_file, lib_folder, urdf_file)
 ******************************************************************************/

#include "Cartpole.h"
#include <ocs2_python_interface/PybindMacros.h>

// This macro creates the Python module with standard OCS2 bindings
CREATE_ROBOT_PYTHON_BINDINGS(ocs2::cartpole::PyBindings, CartpolePyBindings)
