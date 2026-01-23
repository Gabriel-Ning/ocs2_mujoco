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
- RK4 integrator, 0.01s timestep
- Actuator: Force on cart, range [-50, 50] N

## Directory Structure

```
cartpole/
├── model/                      # Essential configuration files
├── scripts/
│   └── simulate.py             # Main simulation script
├── src/
│   ├── CartPoleInterface.cpp   # OCS2 interface implementation
│   └── pyBindModule.cpp        # Python bindings
├── include/ocs2_cartpole/
│   ├── CartPoleInterface.h     # Interface class
│   ├── CartPolePyBindings.h    # Python binding wrapper
│   ├── CartPoleParameters.h    # Physical parameters struct
│   ├── definitions.h           # State/input dimensions
│   └── dynamics/
│       └── CartPoleSystemDynamics.h  # Analytical dynamics
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

## How It Works

1. **Initialization**: `CartPoleInterface` reads `task.yaml` and sets up the OCS2 optimal control problem
2. **Python bindings**: `CartpolePyBindings` exposes MPC to Python
3. **Simulation loop** (`scripts/simulate.py`):
   - Read state from MuJoCo
   - Call `mpc.advanceMpc()` to compute optimal control
   - Apply control to MuJoCo actuator
   - Step physics simulation

## Customization

### Tuning the Controller

Edit `model/task.yaml`:
- Increase `Q_final` weights for tighter terminal tracking
- Decrease `R` for more aggressive control
- Adjust `mpc.timeHorizon` for longer/shorter planning

### Modifying Physics

Edit `model/cartpole.xml`:
- Change `timestep` for simulation accuracy
- Modify actuator `ctrlrange` for force limits
- Add damping to joints for realistic friction

## Files You Can Ignore

These are scaffolding, not essential for understanding the integration:
- `CMakeLists.txt` - Build configuration
- `cmake/` - CMake package config
- `include/ocs2_cartpole/package_path.h.in` - Path resolution template
