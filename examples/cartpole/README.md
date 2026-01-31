# Cartpole Example

This example demonstrates OCS2 + MuJoCo integration for the classic cartpole swing-up problem.

## Directory Structure

```
cartpole/
├── models/                     # Robot models
│   ├── control/
│   │   └── cartpole.urdf       # URDF for controller (Pinocchio)
│   └── simulation/
│       └── cartpole.xml        # MuJoCo simulation model
├── configs/
│   └── task.yaml               # OCS2 task definition
├── scripts/
│   └── simulate.py             # Simulation with MPC control
├── interface/                  # C++ interface & Python bindings
│   ├── include/                # Headers
│   └── src/                    # Implementation
└── auto_generated/             # CppAD compiled code (runtime)
```

## Essential Files

To define a new robot control problem, you need:

| File | Purpose |
|------|---------|
| `configs/task.yaml` | Cost matrices, constraints, solver settings, physical parameters |
| `models/control/*.urdf` | Robot kinematics/dynamics for controller |
| `models/simulation/*.xml` | MuJoCo model for physics simulation |

### configs/task.yaml

Defines the optimal control problem:
- **State**: `[theta, x, theta_dot, x_dot]` (pole angle, cart position, velocities)
- **Input**: Force applied to cart
- **Cost**: Terminal cost to reach upright position
- **Solver**: SLQ (Sequential Linear Quadratic) with 5s horizon

### models/control/cartpole.urdf

Standard URDF with:
- Cart: 2.0 kg mass, prismatic joint
- Pole: 0.2 kg mass, 1.0 m length, continuous joint

### models/simulation/cartpole.xml

MuJoCo model for physics simulation:
- RK4 integrator, 0.001s timestep
- Actuator: Force on cart, range [-5, 5] N

## Running the Example

```bash
pixi run cartpole
```

This will:
1. Build the C++ library and Python bindings
2. Launch MuJoCo simulation with OCS2 MPC control
3. The pole starts hanging down and swings up to upright

## Why C++ Code is Needed

OCS2 requires C++ for:

1. **Automatic Differentiation (CppAD)**: OCS2 uses CppAD to compute analytical derivatives of dynamics for efficient optimization. This requires C++ types.

2. **Python Bindings (pybind11)**: The MPC solver runs in C++ for performance; Python bindings expose the interface.

3. **Problem Assembly**: Cost functions, constraints, and dynamics are assembled into an `OptimalControlProblem` in C++.

### Robot-Specific C++ Code

Only these parts need modification for a new robot:

| File | What to Change |
|------|----------------|
| `definitions.h` | `STATE_DIM` and `INPUT_DIM` |
| `dynamics/CartPoleSystemDynamics.h` | `systemFlowMap()` dynamics equations |
| `CartPoleParameters.h` | Physical parameters struct |
| `CartPoleInterface.cpp` | Cost/constraint setup (if needed) |

### Reducing C++ Code with Pinocchio

For robots with URDF models, you can use Pinocchio to automatically compute dynamics instead of writing `systemFlowMap()` manually. This is recommended for:
- Complex multi-joint robots
- Robots where URDF already exists
- When analytical dynamics are tedious to derive

See `ocs2_mobile_manipulator` in ocs2_lib for a Pinocchio-based example.

## Customization

### Tuning the Controller

Edit `configs/task.yaml`:
- Increase `Q_final` weights for tighter terminal tracking
- Decrease `R` for more aggressive control
- Adjust `mpc.timeHorizon` for longer/shorter planning

### Modifying Physics

Edit `models/simulation/cartpole.xml`:
- Change `timestep` for simulation accuracy
- Modify actuator `ctrlrange` for force limits
- Add damping to joints for realistic friction

## State Mapping

MuJoCo and OCS2 use different state orderings:

| Index | OCS2 State | MuJoCo qpos | MuJoCo qvel |
|-------|------------|-------------|-------------|
| 0 | theta | qpos[1] | qvel[1] |
| 1 | x | qpos[0] | qvel[0] |
| 2 | theta_dot | - | - |
| 3 | x_dot | - | - |

The simulation script handles this mapping.

## References

- [OCS2 Documentation](https://leggedrobotics.github.io/ocs2/)
- [MuJoCo Documentation](https://mujoco.readthedocs.io/)
- [Pinocchio](https://stack-of-tasks.github.io/pinocchio/)
