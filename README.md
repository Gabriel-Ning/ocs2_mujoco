# OCS2 + MuJoCo Integration Tutorial

This tutorial demonstrates how to integrate [OCS2](https://github.com/leggedrobotics/ocs2) (Optimal Control for Switched Systems) with [MuJoCo](https://mujoco.org/) simulation.

## Key Insight

To use OCS2 with MuJoCo, you only need **three essential files**:

```
model/
├── task.yaml       # OCS2 problem definition (costs, constraints, solver settings)
├── robot.urdf      # Robot description (kinematics, dynamics parameters)
└── robot.xml       # MuJoCo simulation model (physics, visualization)
```

Everything else (C++ interface, Python bindings, auto-generated code) is scaffolding to connect these files.

## Project Structure

```
ocs2_mujoco/
├── CMakeLists.txt              # Build configuration
├── pixi.toml                   # Environment & dependencies
├── external/
│   └── ocs2_lib/               # OCS2 library (git submodule)
└── examples/
    └── cartpole/               # Complete working example
        ├── model/              # ★ THE ESSENTIAL FILES ★
        │   ├── task.yaml       # OCS2 configuration
        │   ├── cartpole.urdf   # Robot description
        │   └── cartpole.xml    # MuJoCo model
        ├── ocs2_cartpole_interface/
        │   ├── include/        # C++ headers
        │   └── src/            # C++ interface & bindings
        ├── script/
        │   └── simulate.py     # Main simulation script
        └── auto_generated/     # CppAD compiled code (runtime)
```

## Quick Start

```bash
# Install pixi if not already installed
curl -fsSL https://pixi.sh/install.sh | bash

# Build and run the cartpole example
pixi run cartpole
```

## Understanding the Three Essential Files

### 1. task.yaml - OCS2 Problem Definition

This file defines the optimal control problem:

```yaml
# Physical parameters
cartpole_parameters:
  cartMass: 2.0
  poleMass: 0.2
  poleLength: 1.0
  maxInput: 5.0

# Cost matrices
Q:       # State cost (intermediate)
Q_final: # Terminal state cost
R:       # Control input cost

# Solver settings
ddp:     # DDP algorithm settings
mpc:     # MPC settings (horizon, frequency)
```

### 2. robot.urdf - Robot Description

Standard URDF format describing:
- Link masses and inertias
- Joint types and limits
- Kinematic chain

For simple systems like cartpole, dynamics can be defined analytically. For complex robots, OCS2 can use Pinocchio to compute dynamics from URDF.

### 3. robot.xml - MuJoCo Model

MuJoCo XML defining:
- Physics simulation parameters
- Visual geometry
- Actuators and sensors

**Important:** Joint ordering in XML must match your state vector convention.

## Optional Components

### Python Bindings

Enable with `BUILD_PYTHON_INTERFACE=ON` (default). Allows calling OCS2 MPC from Python scripts.

### Custom Dynamics

For systems where analytical dynamics provide better performance than URDF-based computation, define a custom dynamics class:

```cpp
class MyDynamics : public SystemDynamicsBaseAD {
  ad_vector_t systemFlowMap(...) override {
    // Your dynamics equations here
  }
};
```

### CppAD Code Generation

OCS2 uses CppAD for automatic differentiation. Generated code is cached in `auto_generated/` for faster subsequent runs.

## Custom Integration Steps

1. **Prepare URDF and XML models**
   - Create `model/robot.urdf` and `model/robot.xml`.
   - Create `model/task.yaml` for OCS2 configuration.

2. **Prepare `ocs2_xxx_interface`**
   - Create a library interface (e.g., `ocs2_myrobot_interface`) that implements the C++ OCS2 interface.
   - If you want to use a Python script for simulation, you **must** provide Python bindings for this interface.
   - Reference the `ocs2_cartpole_interface` as a template.

3. **Simulate**
   - Use a Python script (like `script/simulate.py`) to run the simulation, connecting the OCS2 bindings with the MuJoCo simulation.

## References

- [OCS2 Documentation](https://leggedrobotics.github.io/ocs2/)
- [MuJoCo Documentation](https://mujoco.readthedocs.io/)
- [Pinocchio](https://stack-of-tasks.github.io/pinocchio/) - For URDF-based dynamics
