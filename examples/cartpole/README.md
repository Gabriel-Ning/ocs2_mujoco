# Cartpole Integration Tutorial

This example demonstrates how to control a custom robot (Cartpole) using OCS2 and MuJoCo.
The process is broken down into 5 key steps:

## Step 1: Prepare the Robot Description (URDF)

**File:** `urdf/cartpole.urdf`

- Defines the kinematic chain and inertial properties.
- Used by OCS2 to compute the system dynamics and derivatives.

## Step 2: Prepare the Simulation Model (MuJoCo XML)

**File:** `cartpole.xml`

- Defines the visual simulation, contacts, and physics engine settings.
- **Critical:** The joint names and ordering usually need to match the URDF, or you must handle the mapping manually.

## Step 3: Configure the MPC Task

**File:** `config/mpc/task.info`

- Defines the cost function (matrices Q, R), constraints, and time horizons.
- Tuning these values changes the robot's behavior (e.g., how aggressive it swings up).

## Step 4: Generate Python Bindings (C++)

**Files:** `CMakeLists.txt`, `src/CartPoleInterface.cpp`

- OCS2 is a C++ library. We need a small C++ wrapper to expose the MPC interface to Python.
- This is handled automatically by the build system (`pixi run install`).

## Step 5: Run the Simulation Loop

**File:** `simulate.py`

- Connects everything:
    1. Initializes MPC with Step 1 & 3 files.
    2. Loads MuJoCo simulation from Step 2.
    3. Runs the control loop:
        - Read State (`qpos`, `qvel`)
        - Compute Optimal Control (`mpc.advanceMpc()`)
        - Apply Control (`data.ctrl`)
        - Step Simulation (`mujoco.mj_step`)
