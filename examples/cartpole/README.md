# Cartpole Example

This example demonstrates a complete OCS2 + MuJoCo integration for the classic cartpole swing-up problem.

## The Three Essential Files

Everything you need to define the control problem is in `model/`:

```
model/
├── task.yaml       # OCS2 configuration (costs, constraints, solver)
├── cartpole.urdf   # Robot description
└── cartpole.xml    # MuJoCo simulation model
```

### task.yaml

Defines the optimal control problem:
- **State**: `[theta, x, theta_dot, x_dot]` (pole angle, cart position, velocities)
- **Input**: Force applied to cart
- **Cost**: Terminal cost to reach upright position
- **Solver**: SLQ (Sequential Linear Quadratic) with 5s horizon

### cartpole.urdf

Standard URDF with:
- Cart: 2.0 kg mass, prismatic joint
- Pole: 0.2 kg mass, 1.0 m length, continuous joint

### cartpole.xml

MuJoCo model for physics simulation:
- RK4 integrator, 0.001s timestep
- Actuator: Force on cart, range [-5, 5] N

## Directory Structure

```
cartpole/
├── model/                      # Essential configuration files
├── script/
│   └── simulate.py             # Main simulation script
├── ocs2_cartpole_interface/    # C++ Interface & Python Bindings
│   ├── include/                # Header files
│   └── src/                    # Implementation files
└── auto_generated/             # CppAD compiled code (created at runtime)
```

## Running the Example

From the repository root:

```bash
pixi run cartpole
```

This will:
1. Build the C++ library and Python bindings
2. Launch the MuJoCo simulation with OCS2 MPC control
3. The pole starts hanging down and swings up to upright

## Custom Integration Flow

To integrate a new robot with OCS2 and MuJoCo, follow these three steps:

1.  **Prepare URDF and XML models**
    -   Define the physics and visualization in MuJoCo (`.xml`).
    -   Define the kinematic structure in URDF (`.urdf`) if needed for library dynamics.
    -   Configure the MPC costs and constraints in `task.yaml`.

2.  **Prepare `ocs2_xxx_interface`**
    -   Implement the C++ interface to bridge your models with OCS2 logic.
    -   Generate Python bindings for this interface to enable control orchestration from Python.
    -   Use `ocs2_cartpole_interface` as a template for this structure.

3.  **Simulate**
    -   Write a script (see `script/simulate.py`) to handle the control loop:
        -   Read the current state from MuJoCo.
        -   Compute the optimal control using the OCS2 interface.
        -   Apply the control input back to the MuJoCo simulation.

## Tuning and Customization

### Tuning the Controller

Edit `model/task.yaml`:
- Increase `Q_final` weights for tighter terminal tracking.
- Decrease `R` for more aggressive control.
- Adjust `mpc.timeHorizon` for longer/shorter planning.

### Modifying Physics

Edit `model/cartpole.xml`:
- Change `timestep` for simulation accuracy.
- Modify actuator `ctrlrange` for force limits.
- Add damping to joints for realistic friction.

## References

- [OCS2 Documentation](https://leggedrobotics.github.io/ocs2/)
- [MuJoCo Documentation](https://mujoco.readthedocs.io/)
- [Pinocchio](https://stack-of-tasks.github.io/pinocchio/) - For URDF-based dynamics
