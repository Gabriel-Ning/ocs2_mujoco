# Cartpole Example

This example demonstrates OCS2 + MuJoCo integration for the classic cartpole swing-up problem.

## Directory Structure

```
cartpole/
├── models/
│   ├── control/cartpole.urdf      # URDF for controller (Pinocchio)
│   └── simulation/cartpole.xml    # MuJoCo simulation model
├── configs/
│   └── task.yaml                  # OCS2 task definition
├── scripts/
│   └── simulate.py                # Simulation with MPC control
├── interface/
│   ├── include/
│   │   └── Cartpole.h             # Single header: dynamics + interface + bindings
│   └── src/
│       ├── Cartpole.cpp           # Implementation
│       └── PyBindModule.cpp       # Python module creation
└── auto_generated/                # CppAD compiled code (runtime)
```

## Essential Files

| File | Purpose |
|------|---------|
| `configs/task.yaml` | Cost matrices, constraints, solver settings |
| `models/control/*.urdf` | Robot kinematics (optional for simple robots) |
| `models/simulation/*.xml` | MuJoCo physics model |
| `interface/include/Cartpole.h` | **Robot-specific code** |

## Running the Example

```bash
pixi run cartpole
```

## Why C++ Code is Needed

**Short answer:** OCS2 uses CppAD for automatic differentiation, which requires C++ types.

**Detailed explanation:**

1. **OCS2's optimization** requires derivatives of the dynamics equations
2. **CppAD** (a C++ library) computes these derivatives automatically
3. The dynamics must use `ad_scalar_t` types (CppAD's automatic differentiation type)
4. **Python cannot provide this** - pybind11 just wraps the C++ solver

### Can't Pinocchio eliminate the dynamics code?

**Partially.** OCS2's Pinocchio interface provides:
- ✅ URDF loading
- ✅ State/input dimensions from joints
- ✅ Joint limits
- ✅ Forward kinematics
- ❌ **Forward dynamics wrapped with CppAD**

For **velocity-controlled robots** (like `ocs2_mobile_manipulator`), dynamics is trivial:
```cpp
xdot = u;  // velocity input directly becomes state derivative
```

For **force/torque-controlled robots** (like cartpole), you need actual dynamics:
```cpp
// Euler-Lagrange equations: M*q_ddot + C*q_dot + G = τ
q_ddot = M.inverse() * (τ - C*q_dot - G);
```

## What to Change for a New Robot

The simplified `Cartpole.h` shows exactly what's robot-specific:

### 1. Dimensions (2 lines)
```cpp
constexpr size_t STATE_DIM = 4;  // [theta, x, theta_dot, x_dot]
constexpr size_t INPUT_DIM = 1;  // [force]
```

### 2. Parameters (struct)
```cpp
struct Parameters {
  scalar_t cartMass = 2.0;
  scalar_t poleMass = 0.2;
  // ...
};
```

### 3. Dynamics equations (~20 lines)
```cpp
ad_vector_t systemFlowMap(ad_scalar_t time, const ad_vector_t& state,
                          const ad_vector_t& input, ...) const override {
  // Your dynamics here
  // Must use ad_scalar_t types for automatic differentiation
}
```

### 4. YAML configuration
Edit `configs/task.yaml` for costs, constraints, solver settings.

## Comparison with ocs2_mobile_manipulator

| Feature | Cartpole | Mobile Manipulator |
|---------|----------|-------------------|
| Control type | Force/torque | Velocity |
| Dynamics | Euler-Lagrange equations | `xdot = u` |
| Pinocchio use | Optional (for params) | Kinematics, limits |
| Lines of dynamics | ~20 | 1 |

## Future: Full Pinocchio Dynamics

To use Pinocchio for automatic dynamics generation:

1. **CppAD-Pinocchio integration**: Pinocchio supports CppAD, but OCS2 doesn't fully leverage it
2. **Template dynamics class**: Could create a generic class that:
   - Loads URDF via `getPinocchioInterfaceFromUrdfFile()`
   - Computes `M, C, G` using Pinocchio
   - Wraps with CppAD for derivatives

This would require modifying OCS2's pinocchio interface - contributions welcome!

## State Mapping

MuJoCo and OCS2 use different state orderings:

| Index | OCS2 State | MuJoCo |
|-------|------------|--------|
| 0 | theta (angle) | qpos[1] |
| 1 | x (position) | qpos[0] |
| 2 | theta_dot | qvel[1] |
| 3 | x_dot | qvel[0] |

## References

- [OCS2 Documentation](https://leggedrobotics.github.io/ocs2/)
- [MuJoCo Documentation](https://mujoco.readthedocs.io/)
- [Pinocchio](https://stack-of-tasks.github.io/pinocchio/)
- [CppAD](https://coin-or.github.io/CppAD/doc/cppad.htm)
